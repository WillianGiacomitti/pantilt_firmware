#include "Eixo.h"

Eixo::Eixo(TMC2209Stepper* drv, AccelStepper* mot, AS5600* enc, 
           uint8_t canalMux, float dentesMotor, float dentesSaida, uint16_t mSteps, String nome, uint16_t rmsCurrent,
           uint8_t pinoOut, ModoLeituraEncoder modoInicial) 
{
  driver = drv;
  motor = mot;
  encoder = enc;
  canalI2C = canalMux;
  current = rmsCurrent;
  microsteps = mSteps;
  nomeEixo = nome;
  pinoAnalogico = pinoOut;
  modoLeitura = modoInicial;
  
  relacaoReducao = dentesSaida / dentesMotor;
  float passosPorVoltaMotor = 200.0 * (microsteps == 0 ? 1 : microsteps);
  passosPorGrauSaida = (passosPorVoltaMotor * relacaoReducao) / 360.0;
  
  velocidadeAlvoDegSec = 0.0f;
  velocidadeComandadaDegSec = 0.0f;
  aceleracaoDegSec2 = 45.0f; 
  tempoUltimaAtualizacaoRampa = millis();
  modoAtual = MODO_POSICAO;
}

bool Eixo::selecionarCanalI2C() {
  Wire.beginTransmission(ENDERECO_PCA9548A);
  Wire.write(1 << canalI2C);
  uint8_t status = Wire.endTransmission();
  delayMicroseconds(20);
  return (status == 0); // 0 = ACK recebido do mux; qualquer outro valor = falha/timeout
}

void Eixo::setModoLeituraEncoder(ModoLeituraEncoder modo) {
  if (modo == ENCODER_MODO_ANALOGICO && pinoAnalogico == PINO_ANALOGICO_INDEFINIDO) {
    return; // não permite trocar pra analógico se nenhum pino foi configurado no construtor
  }
  modoLeitura = modo;
}

ModoLeituraEncoder Eixo::getModoLeituraEncoder() {
  return modoLeitura;
}

void Eixo::begin() {
  driver->begin();
  driver->toff(4);
  driver->blank_time(24);
  driver->mstep_reg_select(true);
  driver->rms_current(current);      
  driver->microsteps(microsteps); 
  driver->en_spreadCycle(false);   

  motor->setMaxSpeed(30.0 * passosPorGrauSaida);     
  motor->setAcceleration(15.0 * passosPorGrauSaida); 

  if (pinoAnalogico != PINO_ANALOGICO_INDEFINIDO) {
    pinMode(pinoAnalogico, INPUT);
    analogSetPinAttenuation(pinoAnalogico, ADC_11db); // permite ler a faixa completa ~0-3.3V
  }

  float anguloInicial = 0.0f;
  if (!lerAnguloAbsolutoEncoder(&anguloInicial)) {
    // Falha na primeira leitura (boot): assume 0 e deixa a task de encoder
    // corrigir/reportar erro nos próximos ciclos.
    anguloInicial = 0.0f;
  }
  ultimoAnguloBruto = anguloInicial;
  anguloAcumuladoEixo = 0.0;

  tempoUltimaLeituraVelocidade = millis();
  anguloUltimaLeituraVelocidade = 0.0;
  velocidadeRealAtual = 0.0;
}

void Eixo::setZero() {
  motor->setCurrentPosition(0); 
  anguloAcumuladoEixo = 0.0;
}

void Eixo::setAceleracao(float grausPorSegundo2) {
  aceleracaoDegSec2 = abs(grausPorSegundo2);
}

void Eixo::setVelocidadeMaxima(float grausPorSegundo) {
  motor->setMaxSpeed(abs(grausPorSegundo) * passosPorGrauSaida);
}

void Eixo::moverParaGrausAbsoluto(float anguloAbsoluto) {
  modoAtual = MODO_POSICAO;
  long passosAlvo = round(anguloAbsoluto * passosPorGrauSaida);
  motor->moveTo(passosAlvo);
}

void Eixo::iniciarMovimentoContinuo(float grausPorSegundo) {
  modoAtual = MODO_VELOCIDADE;
  velocidadeAlvoDegSec = grausPorSegundo;
}

void Eixo::processarRampaVelocidade() {
  if (modoAtual != MODO_VELOCIDADE) return;

  unsigned long tempoAtual = millis();
  float deltaTempo = (tempoAtual - tempoUltimaAtualizacaoRampa) / 1000.0f;

  if (deltaTempo < 0.010f) { 
    return;
  }
  tempoUltimaAtualizacaoRampa = tempoAtual;

  float erroVelocidade = velocidadeAlvoDegSec - velocidadeComandadaDegSec;

  if (abs(erroVelocidade) < 0.01f) {
    velocidadeComandadaDegSec = velocidadeAlvoDegSec;
  } else {
    float maxDeltaVelocidade = aceleracaoDegSec2 * deltaTempo;
    if (erroVelocidade > 0) {
      velocidadeComandadaDegSec += min(erroVelocidade, maxDeltaVelocidade);
    } else {
      velocidadeComandadaDegSec -= min(abs(erroVelocidade), maxDeltaVelocidade);
    }
  }

  if (abs(velocidadeComandadaDegSec) < 0.05f && velocidadeAlvoDegSec == 0.0f) {
    velocidadeComandadaDegSec = 0.0f;
  }

  float passosPorSegundo = velocidadeComandadaDegSec * passosPorGrauSaida;
  motor->setSpeed(passosPorSegundo);
}

void Eixo::parar() {
  velocidadeAlvoDegSec = 0.0f;
}

// ---------------- Leitura via I2C (AS5600 atrás do mux PCA9548A) ----------------
bool Eixo::lerAnguloViaI2C(float* outDeg) {
  if (!selecionarCanalI2C()) {
    return false; // mux não respondeu (ACK ausente / barramento travado)
  }
  uint16_t bruto = encoder->readAngle();
  *outDeg = bruto * (360.0f / 4096.0f);
  return true;
}

// ---------------- Leitura via saída analógica (pino OUT do AS5600) ----------------
bool Eixo::lerAnguloViaAnalogico(float* outDeg) {
  if (pinoAnalogico == PINO_ANALOGICO_INDEFINIDO) {
    return false; // nenhum pino configurado para este eixo
  }

  // Múltiplas amostras para reduzir o ruído típico do ADC do ESP32
  uint32_t soma = 0;
  const uint8_t N_AMOSTRAS = 8;
  for (uint8_t i = 0; i < N_AMOSTRAS; i++) {
    soma += analogRead(pinoAnalogico);
  }
  uint16_t media = soma / N_AMOSTRAS;

  // ESP32 ADC: 12 bits (0-4095) ~ 0-3.3V, mapeado linearmente para 0-360°.
  // Se a saída OUT do seu AS5600 não cobrir a faixa completa 0-VCC (depende
  // da configuração ANGLE/OUT nos registradores do sensor), pode ser necessário
  // calibrar min/max aqui em vez de assumir 0-4095 direto.
  *outDeg = (media / 4095.0f) * 360.0f;
  return true;
}

// ---------------- Dispatcher: escolhe I2C ou analógico conforme o modo ----------------
bool Eixo::lerAnguloAbsolutoEncoder(float* outDeg) {
  if (modoLeitura == ENCODER_MODO_ANALOGICO) {
    return lerAnguloViaAnalogico(outDeg);
  }
  return lerAnguloViaI2C(outDeg);
}

bool Eixo::atualizarPosicaoEncoder() {
  float atualBruto;
  if (!lerAnguloAbsolutoEncoder(&atualBruto)) {
    return false; // mantém último ângulo válido conhecido, não atualiza o acumulado
  }

  float delta = atualBruto - ultimoAnguloBruto;

  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }

  anguloAcumuladoEixo += delta;
  ultimoAnguloBruto = atualBruto;
  return true;
}

float Eixo::getAnguloEixo() {
  return anguloAcumuladoEixo / relacaoReducao;
}

float Eixo::getVelocidadeEixo() {
  unsigned long tempoAtual = millis();
  float deltaTempo = (tempoAtual - tempoUltimaLeituraVelocidade) / 1000.0f;

  if (deltaTempo >= 0.02f) { 
    float anguloAtual = getAnguloEixo();
    velocidadeRealAtual = (anguloAtual - anguloUltimaLeituraVelocidade) / deltaTempo;
    anguloUltimaLeituraVelocidade = anguloAtual;
    tempoUltimaLeituraVelocidade = tempoAtual;
  }
  
  return velocidadeRealAtual;
}

void Eixo::runStep() {
  if (modoAtual == MODO_VELOCIDADE) {
    motor->runSpeed();
  } else {
    motor->run();
  }
}