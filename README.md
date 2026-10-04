# Zenith Edge Collector

Serviço em Go que descobre, conecta e lê o sensor de vibração BLE
WitMotion WTVB01-BT50, produzindo leituras normalizadas.

Roda como BLE central em Linux via BlueZ/D-Bus, usando
[tinygo.org/x/bluetooth](https://tinygo.org/x/bluetooth).

Dois coletores leem o mesmo sensor e emitem o mesmo schema JSON:

- **`cmd/zenith-edge`** — CLI em Go sobre Linux/BlueZ, para
  desenvolvimento e trabalho de protocolo.
- **`firmware/esp32-zenith-node`** — nó ESP32 autônomo publicando em MQTT
  por WiFi, para campo.

Veja [docs/architecture.md](docs/architecture.md) para o fluxo de dados e
os diagramas.

## Status

Scan, conexão e decodificação funcionam contra hardware físico. Os dois
coletores **não estão no mesmo nível**:

- **Nó ESP32 (`firmware/esp32-zenith-node`)** é o caminho de campo e o mais
  completo: vários sensores simultâneos, buffer em anel para quedas do MQTT,
  detecção automática do formato de pacote, suporte ao modo **Now data**
  (aceleração bruta a 100 amostras/s, janelas contínuas publicadas em
  `zenith/waveform/<mac>`), bateria e alarme decodificados, e diagnóstico do
  nó (`zenith/node/<mac>`). Testado contra o sensor real: 23 janelas
  consecutivas em 60 s sem perda.
- **CLI em Go (`cmd/zenith-edge`)** continua só com `scan` e `test`, mas o
  decoder (`internal/protocol/wtvb01`) agora tem **a mesma lógica do firmware**:
  detecta o pacote de 32 ou 40 bytes pelo fluxo, entrega as amostras Now data
  por `OnInstant`, decodifica bateria e alarme, e é testado contra os mesmos
  bytes reais capturados. O `zenith-edge test` mostra as amostras Now data e
  não lê registradores nesse modo (isso perturba a onda).

O formato dos pacotes, o registrador que troca o modo (`0x96`, que não consta
no manual) e as medições estão em [docs/protocol.md](docs/protocol.md) §9.

## Instalação

```bash
go install github.com/NatanBack77/zenith-edge-collector/cmd/zenith-edge@latest
```

Ou a partir de um clone:

```bash
go install ./cmd/zenith-edge
```

Requer BlueZ funcionando (`systemctl status bluetooth`).

## Uso

Encontrar o sensor:

```console
$ zenith-edge scan
Scanning for 10s (WitMotion devices: "WT" name or service 0000ffe5-...)...
E6:6B:9A:CC:88:25     WTVB01-BT50               RSSI  -49  <- WitMotion service ffe5
```

`--all` lista todos os dispositivos BLE ao alcance, `--verbose` acrescenta
os UUIDs de serviço e IDs de fabricante anunciados — útil quando um sensor
não anuncia nome.

Transmitir leituras decodificadas:

```console
$ zenith-edge test --sensor E6:6B:9A:CC:88:25
Connecting to E6:6B:9A:CC:88:25...
Streaming decoded readings (Ctrl+C to stop)...
[10:38:37.832] angle(0.00,0.01,0.00) vel(1.000,0.000,0.000)mm/s disp(21.0,9.0,6.0)um freq(11.0,12.0,16.0)Hz temp=24.9C
```

`--raw` despeja os payloads de notificação em hex, para capturar fixtures.

## O que ele mede

Cinco indicadores, três deles por eixo:

| Campo | Unidade | O que diz |
|---|---|---|
| `velocity` | mm/s | Saúde geral da máquina — é o alvo dos limites ISO 10816/20816 |
| `displacement` | µm | Falhas de baixa frequência: desbalanceamento, desalinhamento, folga |
| `frequency` | Hz | *Qual* é a falha, via múltiplos da rotação do eixo |
| `angle` | graus | Amplitude angular de vibração — balanço, não inclinação de montagem |
| `device.temperature` | °C | Temperatura do **módulo sensor**, não da máquina |

`device.temperature` tem esse nome de propósito. O manual chama o
registrador `0x40` de "Product temperature": é a temperatura do próprio
módulo, não uma sonda de mancal ou motor. Veja
[docs/indicators.md](docs/indicators.md) para o que cada indicador pega e
como eles se relacionam.

## Protocolo

[docs/protocol.md](docs/protocol.md) documenta o protocolo BLE, derivado
do SDK Python oficial e verificado contra bytes capturados de um sensor
físico.

Uma correção que vale destacar: o SDK genérico BWT901 da WitMotion fixa
um pacote de 20 bytes para os dois tipos. O broadcast `0x61` do
WTVB01-BT50 tem **32 bytes**. Decodificá-lo como 20 lê errado todos os
campos e ainda assim produz números de aparência plausível.

## Estrutura

```
cmd/zenith-edge/            CLI: scan, test
internal/ble/               Scan e conexão BLE (BlueZ)
internal/protocol/wtvb01/   Framing de pacote e decodificação de registrador
  testdata/                 Bytes capturados de um sensor físico
firmware/esp32-zenith-node/ Firmware ESP32: BLE -> MQTT
docs/
  architecture.md           Fluxo de dados e diagramas
  protocol.md               Referência do protocolo BLE
  indicators.md             O que cada medição significa
```

## Testes

```bash
go test ./...
```

Os testes reproduzem bytes reais capturados do sensor. A verificação mais
forte, `TestOutputAndRegisterPacketsAgree`, se apoia no fato de que o
broadcast `0x61` e as leituras de registrador `0x71` codificam os mesmos
registradores de forma independente, então precisam decodificar igual.

## Ainda não verificado

As escalas de velocity, displacement e frequency vêm do manual e produzem
valores fisicamente plausíveis em repouso, mas não foram comparadas lado a
lado com o app oficial da WitMotion sob vibração real. Qualquer correção é
mudança de uma linha em `internal/protocol/wtvb01/registers.go`.

## Nota sobre idioma

A documentação está em português. Comentários e identificadores no código
seguem em inglês, que é a convenção das linguagens e das bibliotecas
usadas aqui.
