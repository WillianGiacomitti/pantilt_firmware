#include "WebController.h"
#include "PageIndex.h" // Incluímos o HTML aqui agora

WebController::WebController(Eixo* pan, Eixo* tilt, int port) 
  : server(port), eixoPan(pan), eixoTilt(tilt) 
{}

void WebController::begin() {
  // Usamos lambdas para conectar a rota do WebServer ao método da classe
  server.on("/", [this]() { handleRoot(); });
  server.on("/status", [this]() { handleStatus(); });
  server.on("/zero", [this]() { handleZero(); });
  server.on("/mover", [this]() { handleMover(); });
  server.on("/jog", [this]() { handleJog(); });
  server.on("/stop", [this]() { handleStop(); });
  
  server.begin();
  Serial.println("[WEB] Servidor HTTP iniciado.");
}

void WebController::handleClient() {
  server.handleClient();
}

void WebController::handleRoot() {
  server.send(200, "text/html", PAGE_MAIN);
}

void WebController::handleStatus() {
  String json = "{";
  json += "\"pan_real\":" + String(eixoPan->lerAnguloAbsolutoEncoder(), 1) + ",";
  json += "\"pan_rel\":" + String(eixoPan->lerAnguloRelativoEncoder(), 1) + ",";
  json += "\"pan_eixo\":" + String(eixoPan->getAnguloEixo(), 1) + ",";
  json += "\"pan_vel\":" + String(eixoPan->getVelocidadeEixo(), 1) + ",";
  
  json += "\"tilt_real\":" + String(eixoTilt->lerAnguloAbsolutoEncoder(), 1) + ",";
  json += "\"tilt_rel\":" + String(eixoTilt->lerAnguloRelativoEncoder(), 1) + ",";
  json += "\"tilt_eixo\":" + String(eixoTilt->getAnguloEixo(), 1) + ",";
  json += "\"tilt_vel\":" + String(eixoTilt->getVelocidadeEixo(), 1);
  json += "}";
  server.send(200, "application/json", json);
}

void WebController::handleZero() {
  if (server.hasArg("eixo")) {
    String eixo = server.arg("eixo");
    Serial.printf("[WEB] Rota /zero chamada. Eixo: %s\n", eixo.c_str());
    if (eixo == "pan") eixoPan->setZero();
    if (eixo == "tilt") eixoTilt->setZero();
  }
  server.send(200, "text/plain", "OK");
}

void WebController::handleMover() {
  if (server.hasArg("pp") && server.hasArg("pt")) {
    float vp = server.arg("vp").toFloat();
    float vt = server.arg("vt").toFloat();
    float pp = server.arg("pp").toFloat();
    float pt = server.arg("pt").toFloat();

    Serial.printf("[WEB] Rota /mover chamada. PAN(%.1f graus, %.1f graus/s) | TILT(%.1f graus, %.1f graus/s)\n", pp, vp, pt, vt);

    eixoPan->setVelocidadeMaxima(vp);
    eixoPan->moverParaGrausRelativo(pp);

    eixoTilt->setVelocidadeMaxima(vt);
    eixoTilt->moverParaGrausRelativo(pt);
  }
  server.send(200, "text/plain", "OK");
}

void WebController::handleJog() {
  if (server.hasArg("eixo") && server.hasArg("vel")) {
    String eixo = server.arg("eixo");
    float vel = server.arg("vel").toFloat();

    Serial.printf("[WEB] Rota /jog chamada. Eixo: %s | Vel: %.1f\n", eixo.c_str(), vel);

    if (eixo == "pan") eixoPan->iniciarMovimentoContinuo(vel);
    if (eixo == "tilt") eixoTilt->iniciarMovimentoContinuo(vel);
  }
  server.send(200, "text/plain", "OK");
}

void WebController::handleStop() {
  Serial.println("[WEB] Rota /stop chamada. Parando todos os motores.");
  eixoPan->parar();
  eixoTilt->parar();
  server.send(200, "text/plain", "OK");
}