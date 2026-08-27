# Nó ESP32 Zenith Edge

Lê um WitMotion WTVB01-BT50 por BLE e publica leituras normalizadas em
MQTT sobre WiFi. Não precisa de PC.

## Por que é simples

O sensor coloca todos os registradores de medição no seu broadcast `0x61`
e envia sem ninguém pedir, então este firmware **nunca escreve no
sensor**. Sem polling de registrador, sem unlock/save, sem codificação de
comando. Ele faz scan, conecta, assina a characteristic de notify e faz o
parse.

As medições **não** estão no advertisement BLE — só os UUIDs de serviço
estão — então a conexão é obrigatória. Escuta passiva não funciona.

## Hardware

| Placa | Funciona | Observação |
|---|---|---|
| ESP32 (clássico) | sim | |
| ESP32-C3 | sim | opção mais barata que serve |
| ESP32-S3 | sim | |
| **ESP32-S2** | **não** | não tem rádio Bluetooth nenhum |

BLE e WiFi dividem a mesma antena no ESP32. Um sensor com intervalo de
notificação de ~200 ms é tranquilo; evite saturar o WiFi com transferências
contínuas em bloco.

O NimBLE permite cerca de três conexões simultâneas por padrão, então um
nó pode atender vários sensores depois de estender o tratamento de
conexão daqui.

## Configuração

```bash
cp src/config.example.h src/config.h
# edite src/config.h com seus dados de WiFi e MQTT
pio run -e esp32-c3 -t upload -t monitor
```

`src/config.h` está no gitignore, então as credenciais ficam só na sua
máquina. Escolha o env da sua placa: `esp32dev`, `esp32-c3` ou `esp32-s3`.

## Dados publicados

Tópico: `zenith/readings/<mac-do-sensor>`

```json
{
  "sensor": "e6:6b:9a:cc:88:25",
  "uptime_ms": 42000,
  "velocity":     { "x": 1.0,   "y": 0.0,   "z": 0.0 },
  "displacement": { "x": 21.0,  "y": 9.0,   "z": 6.0 },
  "angle":        { "x": 0.088, "y": 0.005, "z": 0.033 },
  "frequency":    { "x": 11.0,  "y": 12.0,  "z": 16.0 },
  "device":       { "temperature": 24.9, "power_raw": 418, "rssi": -49 }
}
```

Unidades: velocity mm/s, displacement µm, angle graus, frequency Hz,
temperature °C. O schema é igual ao de `wtvb01.SensorReading` no coletor
Go, então os dois são intercambiáveis a jusante.

`device.temperature` é a temperatura do **módulo sensor**, não da máquina.
Veja [`docs/indicators.md`](../../docs/indicators.md).

`device.power_raw` é um candidato a indicar a carga da bateria do
sensor — o app oficial da WitMotion mostra um campo "Power Percent(%)"
que esse contador pode alimentar, mas a conversão para porcentagem
**não está confirmada** (valores observados em repouso, como 418, ficam
bem fora de 0–100). É publicado cru de propósito; acompanhe a tendência
ao longo de um ciclo de carga/descarga real para calibrar. Veja
[`docs/protocol.md`](../../docs/protocol.md) §5 e §8.

O nó também publica um valor retido `online`/`offline` em
`zenith/status`, com `offline` configurado como last will do MQTT, para
que um nó travado fique visível no broker.

## Testes

`src/wtvb01.{h,cpp}` não tem dependências e compila no host, então o
decoder é testado contra os mesmos bytes reais capturados do sensor que a
implementação em Go usa:

```bash
c++ -std=c++17 -I src -o /tmp/wtvb01_test test/decoder_test.cpp src/wtvb01.cpp
/tmp/wtvb01_test ../../internal/protocol/wtvb01/testdata/capture-wtvb01-bt50.hex
```

A verificação mais forte se apoia no fato de que o broadcast `0x61` e as
leituras de registrador `0x71` codificam os mesmos registradores de forma
independente, então precisam decodificar igual.

## Resolução de problemas

**Sensor não encontrado.** Na maioria das vezes o app do celular ainda
está segurando a conexão — periféricos BLE aceitam um central por vez e
param de anunciar enquanto conectados. Desconecte nas configurações de
Bluetooth do celular, não basta fechar o app. Fora isso, confirme que o
sensor está ligado.

**Compila mas não chega dado.** Confirme que a placa tem BLE de verdade
(não é um ESP32-S2) e que `SENSOR_ADDRESS` no `config.h` está vazio ou
casa em minúsculas com o MAC do sensor.
