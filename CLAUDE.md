# CLAUDE.md — pantilt_firmware

Instruções para o Claude Code neste repositório. Leia este arquivo inteiro antes de qualquer tarefa.

## Contexto

Firmware da ESP32 que aciona o mecanismo pan-tilt do TCC (controle servo visual para inspeção de linhas de transmissão). Recebe comandos do host ROS 2 por serial e devolve telemetria. A arquitetura geral do sistema está no repositório `pantilt_ros`, em `docs/architecture.md`.

## Hardware

- **MCU:** ESP32 (`esp32dev`), USB-serial CP210x (VID:PID `10c4:ea60`).
- **Drivers:** 2 × TMC2209 em UART compartilhada (Serial2, pinos 16/17; endereços 0b00 = PAN, 0b01 = TILT). ENABLE no pino 27 (LOW = habilitado).
- **Motores:** de passo, via AccelStepper (STEP/DIR: PAN 19/18, TILT 25/26).
- **Reduções:** PAN 17:127, TILT 21:64.
- **Encoders:** AS5600. Atualmente em modo **analógico** (pinos 34 = PAN, 35 = TILT; `USAR_I2C false`). O modo I2C via mux PCA9548A (0x70) existe no código, mas não está em uso.
- **Limites físicos:** PAN −30° a +30°, TILT −90° a +90°.

## Código ativo vs. código morto

A build `env:main` (ver `platformio.ini`) compila APENAS:

- `src/main.cpp`: tasks FreeRTOS (motores no core 1; serial e encoder no core 0), fail-safe e watchdog;
- `src/Eixo.cpp` + `include/Eixo.h`: controle de um eixo (rampa de velocidade, posição e encoder);
- `src/Serialprotocol.cpp` + `include/Serialprotocol.h`: protocolo binário com o host.

Código morto (não compilado em `env:main`; candidato a remoção):

- `src/MicroRosAdapter.cpp`, `include/MicroRosAdapter.h`: tentativa com micro-ROS, abandonada;
- `src/WebController.cpp`, `include/WebController.h`, `include/PageIndex.h`, `index.html`: controle antigo por Wi-Fi/HTTP.

Ambientes de teste (válidos, compilam separado):

- `env:teste_motores`: `src/teste_motores.cpp`;
- `env:teste_encoder`: `src/teste_encoder.cpp`;
- `src/teste_encoder_out.cpp`: verificar se ainda é usado.

Não use o código morto como referência de comportamento atual.

## Protocolo serial (CONTRATO)

Definido em `include/Serialprotocol.h`. É espelhado pelo `serial_bridge_node` no repositório `pantilt_ros`.

**Qualquer alteração em tipos, payloads, códigos de erro ou baud rate exige a mesma alteração no `serial_bridge_node` e no `docs/architecture.md` do `pantilt_ros`.**

- Frame: `[0xA5][0x5A][TYPE][LEN][PAYLOAD][CRC8]`; CRC-8 com polinômio 0x07 sobre TYPE+LEN+PAYLOAD; floats little-endian.
- 921600 bps. Telemetria a 20 Hz.
- Unidades na serial: **rad e rad/s**. A conversão para graus acontece só dentro do firmware.
- Fail-safe: motores param após `FAILSAFE_TIMEOUT_MS` (500 ms) sem nenhum frame recebido.

## Comandos

```bash
pio run -e main                      # compilar
pio run -e main -t upload            # gravar
pio device monitor -b 921600         # monitor (a saída é binária; use só para depurar)
pio run -e teste_motores -t upload   # teste isolado dos motores
```

## Regras de trabalho

1. **Planeje antes de editar.** Liste os arquivos que vai tocar e espere aprovação.
2. **Escopo estrito:** não edite arquivos fora da tarefa.
3. **Não faça commits** nem push. O autor revisa e commita.
4. **Não altere o protocolo serial** sem a atualização correspondente no `pantilt_ros` (ver acima).
5. Não bloqueie as tasks FreeRTOS (sem `delay()` longo dentro das tasks). Toda escrita na Serial passa por `sendFrameSafe()`.
6. Comentários em português.
7. Ao terminar, explique o que mudou arquivo por arquivo e como testar em bancada.

## Pendências conhecidas

- [ ] Renomear `include/Serialprotocol.h` e `src/Serialprotocol.cpp` para `SerialProtocol.*`. Os includes usam `SerialProtocol.h`; compila no Windows, mas quebra em Linux/CI.
- [ ] Remover o código morto listado acima.
- [ ] Limites de ângulo por software no firmware (requisito b do TCC); hoje só existem no host.
- [ ] Fim de curso via TMC2209 (StallGuard ou DIAG) (requisito d do TCC).
- [ ] O requisito (c) do TCC (bloquear comandos durante o movimento) deve valer só para comandos de POSIÇÃO. Comandos de velocidade precisam ser aceitos continuamente pelo IBVS. Hoje nenhum bloqueio está implementado, o que está correto para velocidade.
