#ifndef EIXO_H
#define EIXO_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <AS5600.h>
#include <Wire.h>

#define ENDERECO_PCA9548A 0x70
#define PINO_ANALOGICO_INDEFINIDO 255


enum ModoOperacao {
  MODO_POSICAO,
  MODO_VELOCIDADE
};

enum ModoLeituraEncoder : uint8_t {
  ENCODER_MODO_I2C = 0,
  ENCODER_MODO_ANALOGICO = 1
};

class Eixo {
private:
  TMC2209Stepper* driver;
  AccelStepper* motor;
  AS5600* encoder;
  uint8_t canalI2C;
  uint16_t current;
  uint16_t microsteps;
  String nomeEixo;

  float relacaoReducao;
  float passosPorGrauSaida;
  
  // Variáveis de Telemetria e Posição Real
  float ultimoAnguloBruto;
  float anguloAcumuladoEixo;
  unsigned long tempoUltimaLeituraVelocidade;
  float anguloUltimaLeituraVelocidade;
  float velocidadeRealAtual;


  // Variáveis do Limitador de Taxa de Variação (Rampa Iterativa)
  float velocidadeAlvoDegSec;
  float velocidadeComandadaDegSec;
  float aceleracaoDegSec2;
  unsigned long tempoUltimaAtualizacaoRampa;


  ModoOperacao modoAtual;

  // Leitura do encoder: fonte (I2C x analógico) e pino usado no modo analógico
  uint8_t pinoAnalogico;
  ModoLeituraEncoder modoLeitura;

  bool selecionarCanalI2C();
  bool lerAnguloViaI2C(float* outDeg);
  bool lerAnguloViaAnalogico(float* outDeg);

public:
Eixo(TMC2209Stepper* drv, AccelStepper* mot, AS5600* enc,
     uint8_t canalMux, float dentesMotor, float dentesSaida, uint16_t mSteps, String nome, uint16_t rmsCurrent,
     uint8_t pinoOut = PINO_ANALOGICO_INDEFINIDO, ModoLeituraEncoder modoInicial = ENCODER_MODO_I2C);
  void begin();
  
  // Controle de Parâmetros e Posição
  void setZero();
  void setAceleracao(float grausPorSegundo2);
  void setVelocidadeMaxima(float grausPorSegundo);
  void moverParaGrausAbsoluto(float anguloAbsoluto);
  
  // Controle de Velocidade Contínua
  void iniciarMovimentoContinuo(float grausPorSegundo);
  void processarRampaVelocidade();
  void parar();

  // Sensores e Telemetria
  bool lerAnguloAbsolutoEncoder(float* outDeg);
  bool atualizarPosicaoEncoder();
  float getAnguloEixo();
  float getVelocidadeEixo();

  // Seleção da fonte de leitura do encoder (I2C ou analógica)
  void setModoLeituraEncoder(ModoLeituraEncoder modo);
  ModoLeituraEncoder getModoLeituraEncoder();

  // Acionamento Físico
  void runStep();
};

#endif