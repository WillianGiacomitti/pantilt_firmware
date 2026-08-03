#include "Eixo.h"

Eixo::Eixo(TMC2209Stepper* drv, AccelStepper* mot, AS5600* enc, 
           uint8_t canalMux, float dentesMotor, float dentesSaida, uint16_t mSteps, String nome, uint16_t rmsCurrent) 
{
  driver = drv;
  motor = mot;
  encoder = enc;
  canalI2C = canalMux;
  current = rmsCurrent;
  microsteps = mSteps;
  nomeEixo = nome;
  
  relacaoReducao = dentesSaida / dentesMotor;
  float passosPorVoltaMotor = 200.0 * (microsteps == 0 ? 1 : microsteps);
  passosPorGrauSaida = (passosPorVoltaMotor * relacaoReducao) / 360.0;
  
  velocidadeAlvoDegSec = 0.0f;
  velocidadeComandadaDegSec = 0.0f;
  aceleracaoDegSec2 = 45.0f; 
  tempoUltimaAtualizacaoRampa = millis();
  modoAtual = MODO_POSICAO;
}

void Eixo::selecionarCanalI2C() {
  Wire.beginTransmission(ENDERECO_PCA9548A);
  Wire.write(1 << canalI2C);
  Wire.endTransmission();
  delayMicroseconds(20);
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

  ultimoAnguloBruto = lerAnguloAbsolutoEncoder();
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

float Eixo::lerAnguloAbsolutoEncoder() {
  selecionarCanalI2C();
  uint16_t bruto = encoder->readAngle();
  return bruto * (360.0f / 4096.0f);
}

void Eixo::atualizarPosicaoEncoder() {
  float atualBruto = lerAnguloAbsolutoEncoder();
  float delta = atualBruto - ultimoAnguloBruto;

  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }

  anguloAcumuladoEixo += delta;
  ultimoAnguloBruto = atualBruto;
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