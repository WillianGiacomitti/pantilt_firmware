#include "Eixo.h"

void Eixo::selecionarCanalI2C() {
  Wire.beginTransmission(ENDERECO_PCA9548A);
  Wire.write(1 << canalI2C);
  Wire.endTransmission();
  delayMicroseconds(20);
}

// NOVO: Função que roda a cada ciclo para rastrear o movimento real
void Eixo::atualizarPosicaoEncoder() {
  float atualBruto = lerAnguloAbsolutoEncoder();
  float delta = atualBruto - ultimoAnguloBruto;

  // Corrige o "pulo" de 360 para 0 e de 0 para 360
  if (delta > 180.0) {
    delta -= 360.0;
  } else if (delta < -180.0) {
    delta += 360.0;
  }

  anguloAcumuladoEixo += delta;
  ultimoAnguloBruto = atualBruto;
}

Eixo::Eixo(TMC2209Stepper* drv, AccelStepper* mot, AS5600* enc, 
           uint8_t canalMux, float dentesMotor, float dentesSaida, uint16_t mSteps, String nome) 
{
  driver = drv;
  motor = mot;
  encoder = enc;
  canalI2C = canalMux;
  microsteps = mSteps;
  nomeEixo = nome;
  
  relacaoReducao = dentesSaida / dentesMotor;
  float passosPorVoltaMotor = 200.0 * (microsteps == 0 ? 1 : microsteps);
  passosPorGrauSaida = (passosPorVoltaMotor * relacaoReducao) / 360.0;
}

void Eixo::begin() {
  driver->begin();
  driver->toff(4);
  driver->blank_time(24);
  driver->mstep_reg_select(true);
  driver->rms_current(1200);      
  driver->microsteps(microsteps); 
  driver->en_spreadCycle(false);   

  motor->setMaxSpeed(30.0 * passosPorGrauSaida);     
  motor->setAcceleration(15.0 * passosPorGrauSaida); 

  // Inicializa o histórico do encoder
  ultimoAnguloBruto = lerAnguloAbsolutoEncoder();
  anguloAcumuladoEixo = 0.0;
}

void Eixo::setZero() {
  motor->setCurrentPosition(0); 
  // Agora o Zero é exclusivamente zerar o acumulador do encoder
  anguloAcumuladoEixo = 0.0;
  Serial.printf("[DEBUG] %s: Ponto Zero do Eixo Definido via ENCODER (0°).\n", nomeEixo.c_str());
}

void Eixo::setVelocidadeMaxima(float grausPorSegundo) {
  motor->setMaxSpeed(abs(grausPorSegundo) * passosPorGrauSaida);
}

void Eixo::moverParaGrausRelativo(float anguloRelativo) {
  modoAtual = MODO_POSICAO;
  long passosAlvo = round(anguloRelativo * passosPorGrauSaida);
  Serial.printf("[DEBUG] %s: Movendo %ld passos.\n", nomeEixo.c_str(), passosAlvo);
  motor->moveTo(passosAlvo);
}

void Eixo::iniciarMovimentoContinuo(float grausPorSegundo) {
  modoAtual = MODO_VELOCIDADE;
  velocidadeAtualDegSec = grausPorSegundo;
  float passosPorSegundo = grausPorSegundo * passosPorGrauSaida;
  motor->setSpeed(passosPorSegundo);
}

void Eixo::parar() {
  if (modoAtual == MODO_VELOCIDADE) {
    motor->setSpeed(0);
    motor->moveTo(motor->currentPosition());
  } else {
    motor->stop();
  }
  modoAtual = MODO_POSICAO;
}

float Eixo::lerAnguloAbsolutoEncoder() {
  selecionarCanalI2C();
  uint16_t bruto = encoder->readAngle();
  return bruto * (360.0 / 4096.0);
}

float Eixo::lerAnguloRelativoEncoder() {
  // Retorna o valor 0-360° em relação ao zero atual, se ainda for útil
  float rel = lerAnguloAbsolutoEncoder() - (ultimoAnguloBruto - anguloAcumuladoEixo);
  if (rel < 0) rel += 360.0;
  while (rel >= 360.0) rel -= 360.0;
  return rel;
}

// RETORNA A POSIÇÃO REAL DETERMINADA PELO ENCODER
float Eixo::getAnguloEixo() {
  return anguloAcumuladoEixo / relacaoReducao;
}

void Eixo::run() {
  // 1. Atualiza a posição real lendo o sensor I2C
  atualizarPosicaoEncoder();

  // 2. Executa a movimentação do motor (por enquanto ainda em malha aberta)
  if (modoAtual == MODO_POSICAO) {
    motor->run();
  } else {
    motor->runSpeed();
  }
}