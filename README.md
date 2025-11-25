# E-wolf Telemetria

Coleção de firmwares Arduino/ESP8266 utilizados nos protótipos de telemetria e controle do projeto E-wolf. Os esboços (`.ino`) implementam leitores de acelerador, sensores ambientais, controle PWM de motor e hubs de telemetria com interface web, MQTT e registro de dados.

## Conteúdo do repositório

| Arquivo | Plataforma | Principais recursos |
|---------|------------|---------------------|
| `motor_controller_uno_v2.ino` | Arduino Uno/Nano | Controlador PWM no pino D9 com rampas configuráveis, leitura de acelerador, RPM, correntes (Ibat/Imot) e DHT22 para telemetria. Suporta EEPROM para persistência de parâmetros. |
| `telemetry_hub_esp8266_v3.ino` | ESP8266 (NodeMCU) | Hub de telemetria completo com UI web, ponte serial, MQTT (pub/sub), OTA, mDNS e registro CSV em LittleFS. Integra sensores apenas para monitoramento, sem afetar o controle. |
| `motor_controller_mega_v2.ino` | Arduino Mega 2560 | Versão para MEGA com PWM no pino D11 (Timer1 OC1A), leitura de acelerador em A0, RPM via PCINT em A8, correntes (Ibat/Imot), DHT22, cálculo de velocidade (wheel/PPR configuráveis) e telemetria em formato chave:valor. Inclui rampas avançadas (soft-start, rampa S, teto de %), modo HOLD e persistência completa de parâmetros na EEPROM.|
| `motor_controller_mega_v3.ino`  | Arduino Mega 2560 | Evolução do controlador do MEGA acrescentando LOG em microSD via SPI (CS em D10), que cria arquivos `LOGxxx.CSV` e grava periodicamente ms, tensão do acelerador, % de pedal, temperatura, umidade, RPM, velocidade em km/h, correntes Ibat/Imot e duty atual/alvo para análise offline. |
| `telemetry_hub_esp32c3_v4.ino` | ESP32-C3 SuperMini | Versão com UI Web completa (dashboard + speedo), MQTT pub/sub, OTA, mDNS, logger CSV em LittleFS e BLE UART para telemetria + comandos. Interpreta o protocolo K:V vindo do MEGA e expõe `/data`, `/log/*`, calibração via HTTP, e velocidade em tempo real.                                    |
| `telemetry_hub_esp32c3_v5.ino` | ESP32-C3 SuperMini | Versão mais enxuta e moderna SEM Web UI, focada em MQTT + BLE + Logging. Inclui controle remoto por BLE/MQTT, override de aceleração, seleção automática da origem do comando (LOCAL/BLE/MQTT), CSV rotativo, OTA, mDNS, e telemetria estendida. Ideal para dashboards externos e apps móveis.  |
| Arquivo                             | Plataforma                                     | Principais recursos
| **`telemetry_dashboard_mqtt.html`** | **Dashboard Web (qualquer navegador moderno)** | Interface avançada de telemetria em **tempo real via MQTT/WebSocket**, sem depender de ESP rodando página própria. Exibe tensão, pedal (%), temperatura, umidade, RPM, velocidade, corrente da bateria/motor e razão Ib/I. Inclui gráficos deslizantes (~120 amostras), controle remoto de aceleração (override), STOP/START, envio de configurações (max_pct, log, etc.) e modal para ajustar host/porta/path TLS/WebSocket do broker. Totalmente responsivo, elegante e independente — ótimo para uso em desktop, tablet ou celular.  |

---


### Arquivos não mais usados

| Arquivo | Plataforma | Principais recursos |
|---------|------------|---------------------|
| `medidor_aceleracao_online.ino` | ESP8266 (Wemos D1 mini) | Lê o acelerador (Hall, 3 fios) no pino A0, disponibilizando a leitura pela porta serial e por uma página web/JSON. Utiliza WiFiManager para configuração de rede. |
| `medidor_aceleracao_temperatura_online.ino` | ESP8266 (Wemos D1 mini) | Expande o medidor adicionando sensor DHT (temperatura/umidade) no pino D4, interface web com gráfico e endpoint `/data` em JSON. |
| `medidor_aceleracao_temperatura_online_pwm.ino` | ESP8266 (Wemos D1 mini) | Adiciona saída PWM no pino D6 para acionamento de motor via MOSFET (ex.: IRLZ44N), mantendo leitura do acelerador e DHT. |
| `medidor_aceleracao_temperatura_online_pwm_rpm.ino` | ESP8266 (Wemos D1 mini) | Inclui medição de RPM com sensor Hall (D5), persistência de configurações em LittleFS, controles START/STOP via HTTP e visualização detalhada de telemetria (Volts, %, Temp, Humi, RPM, Speed). |

## Pré-requisitos

- Arduino IDE (ou PlatformIO) com suporte às placas **ESP8266** e **Arduino AVR** instalado.
- Bibliotecas utilizadas (instale via Gerenciador de Bibliotecas da IDE ou `platformio.ini`):
  - `ESP8266WiFi`
  - `ESP8266WebServer`
  - `WiFiManager`
  - `DHT sensor library`
  - `FS` / `LittleFS`
  - `PubSubClient`
  - `EEPROM` (já inclusa na AVR core)
  - `ArduinoOTA`, `ESP8266mDNS`

Cada sketch possui comentários indicando bibliotecas adicionais específicas (ex.: `time.h` para NTP no hub de telemetria). Consulte a seção superior de cada arquivo para conferir dependências e ajustes finos de hardware.

Segue tudo lapidado em Markdown, prontinho para encaixar no README do E-Wolf v3.

---

## Pinagem resumida – E-Wolf Telemetria & Controle

| Função / Componente                   | Tipo / Módulo             | **Arduino Mega 2560**           | **ESP8266 (NodeMCU / Wemos D1 mini)** | Observações                           |
| ------------------------------------- | ------------------------- | ------------------------------- | ------------------------------------- | ------------------------------------- |
| Acelerador (Hall)                     | Analógico (0–5 V, 3 fios) | A0                              | —                                     | Leitura analógica (Mega controla PWM) |
| Sensor de RPM (Hall)                  | Digital (open collector)  | A8                              | —                                     | Interrupção via PCINT no Mega         |
| Sensor de corrente – bateria (ACS712) | Analógico                 | A2                              | —                                     | Leitura direta do VOUT                |
| Sensor de corrente – motor (ACS712)   | Analógico                 | A3                              | —                                     | Opcional                              |
| Sensor DHT22                          | Digital                   | D4                              | D4 (GPIO2)                            | Pode existir em ambos os módulos      |
| Saída PWM do motor                    | Digital PWM               | D11 (OC1A)                      | —                                     | Saída principal de potência           |
| Módulo microSD (SPI)                  | SPI                       | CS=10, MOSI=51, MISO=50, SCK=52 | —                                     | Logs CSV locais                       |
| Serial de telemetria                  | UART                      | TX1=18, RX1=19                  | RX=D7, TX=D8                          | Mega ↔ ESP8266 @ 115200 baud          |
| Wi-Fi / MQTT / Web UI                 | —                         | —                               | Integrado                             | Hub de telemetria e UI                |
| Alimentação                           | —                         | 5 V / GND                       | VIN / GND                             | Terra comum obrigatório               |
| EEPROM interna                        | Persistência              | Interna                         | —                                     | Armazena rampas e limites             |
| LittleFS                              | Armazenamento             | —                               | Interna                               | Armazena UI e logs do hub             |

---

## Ligação típica entre Mega e ESP8266

| Ligação   | **Mega 2560** | **ESP8266** | Descrição                    |
| --------- | ------------- | ----------- | ---------------------------- |
| TX1 → RX  | D18 (TX1)     | D7 (RX)     | Mega envia telemetria        |
| RX1 ← TX  | D19 (RX1)     | D8 (TX)     | ESP envia comandos           |
| GND ↔ GND | —             | —           | Referência comum obrigatória |
| 5 V → VIN | —             | VIN         | Alimentação do ESP8266       |

---

## 🧩 Fluxo geral do sistema

1. O **Mega 2560** lê acelerador, RPM, correntes e DHT22, controla o PWM e grava logs no microSD.
2. O **ESP8266** recebe telemetria via Serial, exibe UI Web, publica no MQTT e oferece endpoints HTTP.
3. Os dois módulos podem funcionar independentes, mas juntos formam o **E-Wolf v3 completo**: controle + telemetria.

---

Quer que eu adicione uma versão alternativa mais compacta ou um diagrama ASCII para ajudar na visualização?


## Como compilar e carregar

1. Abra a IDE Arduino e selecione o sketch desejado.
2. Escolha a placa correta em **Ferramentas > Placa**:
   - *Wemos D1 mini / NodeMCU* para sketches ESP8266.
   - *Arduino Uno ou Nano* para `motor_controller_uno_v2.ino`.
3. Ajuste as configurações de porta serial e velocidade conforme o hardware.
4. Verifique as constantes de hardware no início do arquivo (pinos, divisores de tensão, tipos de sensor, etc.) e adapte às suas necessidades.
5. Compile e faça o upload normalmente.

## Configuração de rede (ESP8266)

Os sketches baseados em ESP8266 utilizam **WiFiManager**. Caso não haja credenciais salvas, a placa abrirá um ponto de acesso temporário (por padrão `Throttle-Setup` ou definido no código). Conecte-se a esse AP, acesse `192.168.4.1` e forneça as credenciais da rede Wi-Fi desejada.

## Interface web e APIs

- **Página principal** (`/`): dashboards com gráficos em tempo real, botões de controle e indicadores mínimos/máximos (dependendo do sketch).
- **Endpoint `/data`**: retorna JSON com telemetria (voltagem do acelerador, percentual, temperatura, umidade, RPM, velocidade, etc.).
- **Endpoints de controle** (variantes PWM/RPM):
  - `/start` para iniciar o motor com o duty atual.
  - `/stop?duty=` para definir duty cycle (0–100% ou valor bruto 0–1023).
- **MQTT** (hub ESP8266): publica em `pb/telemetry/json`, assina `pb/cmd/motor` e define LWT em `pb/status`.


## Referência de comandos — Arduino + ESP8266

Projeto: Motor Controller (Arduino) + Telemetry Hub (ESP8266)

Formato: comandos via Serial (Arduino), endpoints HTTP (ESP) e tópicos MQTT.

### 1) Gerais (sistema e modos)

**Serial (Arduino)**

```
START
STOP
HOLD <pct>            ; ex.: HOLD 30
DEFAULTS              ; volta aos padrões (RAM)
SAVE                  ; persiste na EEPROM
LOAD_DEFAULTS         ; recarrega padrões
```

**HTTP (ESP)**

```
/start
/stop
/hold_pct?x=<pct>
/defaults
# opcionais, caso implementados
/save
/load_defaults
```

**MQTT**

```
Tópico: pb/cmd/motor
Payload '0' ou 'OFF' -> STOP
Payload '1' ou 'ON'  -> START
```

### 2) Calibração do acelerador (V_MIN_REAL / V_MAX_REAL)

**Serial (Arduino)**

```
SET_MIN_NOW
SET_MAX_NOW
SET_MINV <volts>      ; ex.: SET_MINV 1.200
SET_MAXV <volts>      ; ex.: SET_MAXV 4.250
SAVE                  ; grava na EEPROM depois de ajustar
```

**HTTP (ESP)**

```
/min_now
/max_now
/set_min?v=<volts>    ; ex.: /set_min?v=1.200
/set_max?v=<volts>    ; ex.: /set_max?v=4.250
```

Diagnóstico rápido: abra `/data` e verifique campos `min` e `max`.

### 3) Parâmetros de controle (rampas, PWM, tacômetro)

**Serial (Arduino)**

```
SET_PWMF <hz>         ; 100..8000
SET_STARTMIN <pct>    ; 0..40
SET_RAPIDMS <ms>      ; 50..1500
SET_RAPIDUP <pct/s>   ; 10..400
SET_SLEW <up> <down>  ; up:5..200  down:5..300
SET_ZEROTO <us>       ; timeout tacômetro em microssegundos
SET_STEP_MODE <0/1>   ; 0: taxa (%/s) | 1: passo adapt.
SET_ACCELS_ON <0/1>   ; curva S rápida ON/OFF
SET_RAMPDELAY <ms>    ; delay mínimo por ciclo (0..10)
SET_RPM_MINPULSE <us> ; filtro anti-ruído do sensor RPM
SAVE
```

**HTTP (ESP)**

```
/set_pwmf?hz=<hz>
/set_startmin?x=<pct>
/set_rapid?ms=<ms>&up=<pct/s>
/set_slew?up=<up>&down=<down>
/set_zeroto?ms=<ms>   ; OBS: converte para us no Arduino
```

### 4) Limites de aceleração (teto do piloto)

**Serial (Arduino)**

```
SET_MAXPCT <pct>      ; 1..100  (ex.: 80)
SAVE
```

**HTTP (ESP)**

```
/set_maxpct?x=<pct>
```

Telemetria (verificação): consulte `/data` → campo `max_pct`.

### 5) Configurações de sensores (somente telemetria nesta fase)

**Serial (Arduino)**

```
SET_WHEEL <cm>        ; diâmetro da roda (cm)
SET_PPR <n>           ; pulsos por volta (1..16)
SAVE
```

**HTTP (ESP)**

```
/set_wheel?cm=<cm>
/set_ppr?n=<n>
```

### 6) Modos de saída / telemetria

**Serial (Arduino)**

```
SET_PRINTMODE KVP
SET_PRINTMODE PLOTTER
```

**HTTP (ESP)**

```
/data          ; retorna JSON com todos os campos
/              ; UI principal (dark)
/speedo        ; velocímetro fullscreen
```

**MQTT**

```
Publicação: pb/telemetry/json (a cada 1s)
```

### 7) CSV logger no ESP (LittleFS)

**HTTP (ESP)**

```
/log/start      ; inicia gravação
/log/stop       ; para gravação
/log/csv        ; baixa o arquivo CSV
/log/clear      ; apaga o CSV
/log/status     ; status: enabled, interval, size
/set_log_iv?ms=<ms>  ; intervalo de log (mín. 100 ms)
```

Campos no CSV:

```
ts_iso,ms,volts,pct,temp,humi,rpm,speed_kmh,
current_bat_a,current_mot_a,min,max,wheel_cm,ppr,
override,override_pct,max_pct,rssi
```

### 8) Exibição / UI (ajustes rápidos)

**HTTP (ESP)**

```
/set_pollms?ms=<ms>   ; período de atualização da UI/gráficos
/speedo               ; atalho para velocímetro
```

### 9) Dicas rápidas

- Sempre que ajustar via Serial (`SET_*`), use `SAVE` para persistir.
- Para redefinir padrões compilados, aumente a versão da estrutura de EEPROM no código (`CFG_VER`), reinicie, ajuste fino e use `SAVE`.
- Verifique `/data` para confirmar `min`/`max`, `max_pct`, `ppr`, etc.
- OTA está ativo (hostname: `telemetry`, senha padrão: `espota`).

## Persistência e arquivos

Alguns sketches usam **LittleFS** ou **EEPROM** para salvar configurações como fatores de roda (cm), pulsos por volta ou limites de aceleração. Após ajustar valores via UI ou código, lembre-se de salvar/commitá-los conforme orientações do firmware.

## Contribuindo

1. Crie um fork deste repositório.
2. Trabalhe em uma branch dedicada e documente suas alterações.
3. Envie um Pull Request descrevendo claramente as funcionalidades adicionadas ou correções.

