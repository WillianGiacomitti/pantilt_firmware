#ifndef EIXO_H
#define EIXO_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <TMCStepper.h>
#include <AS5600.h>
#include <Wire.h>

#define ENDERECO_PCA9548A 0x70

enum ModoOperacao { MODO_POSICAO, MODO_VELOCIDADE };

class Eixo {
private:
  TMC2209Stepper* driver;
  AccelStepper* motor;
  AS5600* encoder;
  String nomeEixo;
  
  uint8_t canalI2C;
  float relacaoReducao;
  uint16_t microsteps;
  float passosPorGrauSaida;
  
  float offsetZeroEncoder = 0.0;
  ModoOperacao modoAtual = MODO_POSICAO;
  float velocidadeAtualDegSec = 0.0;

  float ultimoAnguloBruto = 0.0;
  float anguloAcumuladoEixo = 0.0;

  void selecionarCanalI2C();
  void atualizarPosicaoEncoder();

public:
  Eixo(TMC2209Stepper* drv, AccelStepper* mot, AS5600* enc, 
       uint8_t canalMux, float dentesMotor, float dentesSaida, uint16_t mSteps, String nome);

  void begin();
  void setZero();
  void setVelocidadeMaxima(float grausPorSegundo);
  void moverParaGrausRelativo(float anguloRelativo);
  void iniciarMovimentoContinuo(float grausPorSegundo);
  void parar();
  float lerAnguloAbsolutoEncoder();
  float lerAnguloRelativoEncoder();
  float getAnguloEixo();

  void run();
};

#endif