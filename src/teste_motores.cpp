#include <Arduino.h>
#include <AccelStepper.h>
#include <TMCStepper.h>  // Biblioteca para controle UART

// ================= MOTOR pan =================
#define STEP_PIN_1 19
#define DIR_PIN_1  18
#define DRIVER_1_ADDRESS 0b00  // MS1=GND, MS2=GND

// ================= MOTOR tlt =================
#define STEP_PIN_2 25  // Escolha pinos livres no seu ESP32
#define DIR_PIN_2  26  // Escolha pinos livres no seu ESP32
#define DRIVER_2_ADDRESS 0b01  // MS1=VIO (3.3V), MS2=GND

// Configurações Comuns
#define RXD2 16
#define TXD2 17
#define R_SENSE 0.11f  // Resistor de amostragem padrão do MKS v2.0

// Instancia as bibliotecas para o Motor 1
TMC2209Stepper driver1(&Serial2, R_SENSE, DRIVER_1_ADDRESS);
AccelStepper motor1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);

// Instancia as bibliotecas para o Motor 2
TMC2209Stepper driver2(&Serial2, R_SENSE, DRIVER_2_ADDRESS);
AccelStepper motor2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

void setup() {
  Serial.begin(115200);   
  
  // 1. O TRUQUE DO TEMPO: Estabilização dos pinos após reset
  delay(1000); 

  // Inicializa a Serial2 do ESP32 (Barramento único para ambos os drivers)
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 
  
  // 2. LIMPEZA DE LIXO da linha serial
  while(Serial2.available() > 0) {
    Serial2.read();
  }
  delay(100); 

  // Inicializa ambos os drivers
  driver1.begin();
  driver2.begin();
  
  // 3. DIAGNÓSTICO ATIVO - DRIVER 1
  int statusUART1 = driver1.test_connection();
  Serial.print("--- DRIVER 1 UART: ");
  if (statusUART1 == 0) Serial.println("CONECTADO! ---");
  else { Serial.print("FALHOU (Erro: "); Serial.print(statusUART1); Serial.println(") ---"); }

  // 3. DIAGNÓSTICO ATIVO - DRIVER 2
  int statusUART2 = driver2.test_connection();
  Serial.print("--- DRIVER 2 UART: ");
  if (statusUART2 == 0) Serial.println("CONECTADO! ---");
  else { Serial.print("FALHOU (Erro: "); Serial.print(statusUART2); Serial.println(") ---"); }

  // ================= CONFIGURAÇÕES DRIVER 1 =================
  driver1.toff(4);                 
  driver1.blank_time(24);
  driver1.mstep_reg_select(true);  
  driver1.rms_current(1200);       
  driver1.microsteps(2);           // 1/2 Passo (Meio Passo) -> 400 passos por volta
  driver1.en_spreadCycle(false);   // Modo StealthChop (Silencioso)

  // ================= CONFIGURAÇÕES DRIVER 2 =================
  driver2.toff(4);                 
  driver2.blank_time(24);
  driver2.mstep_reg_select(true);  
  driver2.rms_current(1200);       
  driver2.microsteps(2);           // 1/2 Passo (Meio Passo) -> 400 passos por volta
  driver2.en_spreadCycle(false);   // Modo StealthChop (Silencioso)

  // ================= CONFIGURAÇÃO DOS MOTORES =================
  motor1.setMaxSpeed(300);      
  motor1.setAcceleration(100);  
  motor1.moveTo(400); // 1 volta inicial             

  motor2.setMaxSpeed(300);      
  motor2.setAcceleration(100);  
  motor2.moveTo(400); // 1 volta inicial  
}

void loop() {
  // Lógica de Movimento Assíncrona para o Motor 1
  if (motor1.distanceToGo() == 0) {
    if (motor1.currentPosition() == 400) {
      motor1.moveTo(0);
    } else {
      motor1.moveTo(400);
    }
    Serial.print("M1 Corrente (mA): ");
    Serial.println(driver1.rms_current());
  }

  // Lógica de Movimento Assíncrona para o Motor 2
  if (motor2.distanceToGo() == 0) {
    if (motor2.currentPosition() == 400) {
      motor2.moveTo(0);
    } else {
      motor2.moveTo(400);
    }
    Serial.print("M2 Corrente (mA): ");
    Serial.println(driver2.rms_current());
  }
  
  // Executa os passos de ambos os motores continuamente
  motor1.run();
  motor2.run();
}