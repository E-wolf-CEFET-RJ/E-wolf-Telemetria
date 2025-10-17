# E-wolf Telemetria

Coleção de firmwares Arduino/ESP8266 utilizados nos protótipos de telemetria e controle do projeto E-wolf. Os esboços (`.ino`) implementam leitores de acelerador, sensores ambientais, controle PWM de motor e hubs de telemetria com interface web, MQTT e registro de dados.

## Conteúdo do repositório

| Arquivo | Plataforma | Principais recursos |
|---------|------------|---------------------|
| `medidor_aceleracao_online.ino` | ESP8266 (Wemos D1 mini) | Lê o acelerador (Hall, 3 fios) no pino A0, disponibilizando a leitura pela porta serial e por uma página web/JSON. Utiliza WiFiManager para configuração de rede. |
| `medidor_aceleracao_temperatura_online.ino` | ESP8266 (Wemos D1 mini) | Expande o medidor adicionando sensor DHT (temperatura/umidade) no pino D4, interface web com gráfico e endpoint `/data` em JSON. |
| `medidor_aceleracao_temperatura_online_pwm.ino` | ESP8266 (Wemos D1 mini) | Adiciona saída PWM no pino D6 para acionamento de motor via MOSFET (ex.: IRLZ44N), mantendo leitura do acelerador e DHT. |
| `medidor_aceleracao_temperatura_online_pwm_rpm.ino` | ESP8266 (Wemos D1 mini) | Inclui medição de RPM com sensor Hall (D5), persistência de configurações em LittleFS, controles START/STOP via HTTP e visualização detalhada de telemetria (Volts, %, Temp, Humi, RPM, Speed). |
| `motor_controller_uno_v2.ino` | Arduino Uno/Nano | Controlador PWM no pino D9 com rampas configuráveis, leitura de acelerador, RPM, correntes (Ibat/Imot) e DHT22 para telemetria. Suporta EEPROM para persistência de parâmetros. |
| `telemetry_hub_esp8266_v3.ino` | ESP8266 (NodeMCU) | Hub de telemetria completo com UI web, ponte serial, MQTT (pub/sub), OTA, mDNS e registro CSV em LittleFS. Integra sensores apenas para monitoramento, sem afetar o controle. |

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

## Persistência e arquivos

Alguns sketches usam **LittleFS** ou **EEPROM** para salvar configurações como fatores de roda (cm), pulsos por volta ou limites de aceleração. Após ajustar valores via UI ou código, lembre-se de salvar/commitá-los conforme orientações do firmware.

## Contribuindo

1. Crie um fork deste repositório.
2. Trabalhe em uma branch dedicada e documente suas alterações.
3. Envie um Pull Request descrevendo claramente as funcionalidades adicionadas ou correções.

## Licença

Defina aqui a licença apropriada para o projeto (por exemplo, MIT, GPL, etc.). Caso ainda não exista uma licença formal, considere adicioná-la para esclarecer direitos de uso e distribuição.

