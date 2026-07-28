#ifndef WEB_CONTROLLER_H
#define WEB_CONTROLLER_H

#include <Arduino.h>
#include <WebServer.h>
#include "Eixo.h"

class WebController {
private:
  WebServer server;
  Eixo* eixoPan;
  Eixo* eixoTilt;

  // Métodos privados para os handlers
  void handleRoot();
  void handleStatus();
  void handleZero();
  void handleMover();
  void handleJog();
  void handleStop();

public:
  // Construtor recebe ponteiros para os eixos para poder controlá-los
  WebController(Eixo* pan, Eixo* tilt, int port = 80);
  
  void begin();
  void handleClient();
};

#endif