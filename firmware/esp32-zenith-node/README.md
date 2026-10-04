# Nó ESP32 Zenith Edge

Lê um WitMotion WTVB01-BT50 por BLE e publica leituras normalizadas em
MQTT sobre WiFi. Não precisa de PC.

## Dois modos de dados do sensor

O sensor tem dois modos (registrador `0x96`, que **não está no manual**: foi
descoberto capturando o app oficial; ver [`docs/protocol.md`](../../docs/protocol.md)):

| | Default | Now data |
|---|---|---|
| Pacote `0x61` | 32 bytes | 40 bytes |
| Conteúdo | amplitudes (sempre positivas), frequência, temperatura, bateria | aceleração XYZ (±16 g) + giroscópio + velocidade/ângulo/deslocamento **com sinal**, com timestamp do chip em ms |
| Taxa medida | ~100 pacotes/s | **100,0 pacotes/s, exatos 10 ms entre amostras** |
| Serve para | monitoramento (o que o app mostra) | FFT: desbalanceamento, folga |

As duas variantes usam o mesmo byte de tipo (`0x61`), então o tamanho não vem
do tipo: o decoder descobre 32 vs 40 pelo próprio fluxo (ver `Decoder` em
`src/wtvb01.h`) e se corrige sozinho se o sensor trocar de modo.

Cada notificação BLE carrega **4 pacotes concatenados** (o ESP32 negocia
ATT MTU 247 com o sensor).

### Escritas no sensor: só opt-in

Por padrão este firmware **não escreve no sensor**: faz scan, conecta, assina
o notify e decodifica. Existem duas escritas opcionais, ambas desligadas em
`config.example.h` e **persistentes** no sensor (terminam com "salvar"):

- `SENSOR_CONFIGURE_DATA_MODE` / `SENSOR_DATA_MODE_INSTANT`: liga/desliga o
  Now data. É a escrita que o app oficial envia (destravar → `0x96` → salvar,
  ~100 ms entre comandos). O nó **só escreve se o sensor ainda não está no
  modo desejado**, para não gastar a flash do sensor a cada reconexão.
- `SENSOR_CONFIGURE_RATE`: escreve o registrador de taxa (`0x03`). Segue o
  manual e os exemplos do SDK, mas **nunca foi visto no ar nesta unidade**.
  A unidade testada já vem em 100 Hz.

## Now data sem quebrar o app

Em Now data o sensor **para de mandar** amplitudes, frequência, temperatura e
bateria. Para o `zenith/readings` continuar com os mesmos campos, o nó os
produz:

- **velocity / displacement / angle**: RMS suavizado (constante de tempo de
  ~1 s) dos valores com sinal (`InstantStats`). **São valores do nó, não os
  registradores de amplitude do sensor** (o manual não define como o sensor os
  calcula); tratar como "RMS do sinal instantâneo".
- **frequency**: frequência dominante de cada janela de 256 amostras, por FFT
  com janela de Hann e interpolação parabólica (≥ 5 Hz; resolução 0,39 Hz).
- **temperature / battery**: lidos do sensor a cada `SENSOR_STATUS_POLL_MS`
  (30 s), porque não vêm no pacote Now data.

Cada mensagem traz `"mode":"instant"` (ou `"default"`).

> **Medido em hardware:** ler registradores do sensor com frequência
> perturba o fluxo de 100 amostras/s. Com leitura a 5 Hz só **1 janela de
> onda fechou em 30 s** (`gaps=12`); sem polling, janelas contínuas. Por isso
> o polling de temperatura/bateria é raro, e as amplitudes não são lidas dos
> registradores (que continuam vivos no Now data, mas custam a continuidade
> da onda).

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

### `zenith/readings/<mac-do-sensor>` (JSON, ~5 por segundo)

```json
{
  "sensor": "e6:6b:9a:cc:88:25",
  "seq": 61,
  "uptime_ms": 48838,
  "published_at_ms": 48838,
  "mode": "instant",
  "velocity":     { "x": 30.8, "y": 68.5, "z": 118.2 },
  "displacement": { "x": 312.4, "y": 367.1, "z": 210.2 },
  "angle":        { "x": 4.03, "y": 8.74, "z": 16.84 },
  "frequency":    { "x": 8.6, "y": 6.3, "z": 6.3 },
  "device": { "temperature": 35.03, "power_raw": 434, "battery_v": 4.34,
              "battery_pct": 100, "alarm": 0, "rssi": -45 }
}
```

Unidades: velocity mm/s, displacement µm, angle graus, frequency Hz,
temperature °C. **Compatível com o schema anterior**: os campos novos
(`mode`, `seq`, `published_at_ms`, `battery_v`, `battery_pct`, `alarm`) são só
acréscimos; quem lia os campos antigos continua lendo igual.

- `device.power_raw` é o registrador de **bateria** (`0x64`), em centivolts.
  Confirmado em hardware: o último valor do pacote Default bate com a leitura
  do `0x64` (437…441 vs 439). `battery_v = power_raw / 100` e `battery_pct`
  usa a tabela do app oficial. (Antes isto era publicado cru por não se saber
  o que era.)
- `device.alarm`: flags de alarme embarcados do sensor (0 = nenhum).
- `device.temperature` é a temperatura do **módulo sensor**, não da máquina.
  Veja [`docs/indicators.md`](../../docs/indicators.md).
- Em `"mode":"instant"` os campos de movimento são os derivados descritos
  acima.

### `zenith/waveform/<mac-do-sensor>` (binário, uma janela a cada 2,56 s)

Só existe em Now data. **Frame binário little-endian**, 1584 bytes para 256
amostras (3,4× menor que JSON e sem custo de formatação no ESP32):

| Offset | Tipo | Campo |
|---|---|---|
| 0 | `char[4]` | `"ZWF1"` (versão do formato) |
| 4 | `u16` | `n`: amostras por eixo (256) |
| 6 | `u8` | `axes` (3) |
| 7 | `u8` | reservado |
| 8 | `u32` | `seq`: contador de frames por sensor |
| 12 | `u32` | `chip_start_ms`: relógio do chip na 1ª amostra |
| 16 | `u32` | `chip_end_ms`: ... na última |
| 20 | `u32` | `gaps`: reinícios de janela por perda de amostras |
| 24 | `u32` | `dropped`: janelas perdidas porque o loop estava ocupado (total) |
| 28 | `u32` | `unsent`: janelas perdidas por MQTT fora/falha (total) |
| 32 | `f32` | `fs_hz`: taxa implícita no relógio do chip |
| 36 | `f32` | `g_per_count`: `g = contagem × g_per_count` (16/32768) |
| 40 | `u32` | `uptime_ms` do ESP32 |
| 44 | `u32` | `published_at_ms` |
| 48 | `i16[n]` | `ax` (contagens brutas) |
| 48+2n | `i16[n]` | `ay` |
| 48+4n | `i16[n]` | `az` |

Janelas são **contínuas**: se o relógio do chip mostra amostras perdidas, a
janela parcial é descartada e recomeça (uma FFT sobre amostras espaçadas
irregularmente estaria errada). Janelas **não ficam em buffer** durante queda
do MQTT (são grandes demais); contam em `unsent`.

Decodificar e ver o espectro: `python3 tools/waveform_listen.py --user ...`
(precisa de `paho-mqtt`; com `numpy` mostra RMS e pico de FFT por eixo).

### `zenith/status`

Valor retido `online`/`offline`, com `offline` configurado como last will do
MQTT, para que um nó travado fique visível no broker.

## Desempenho: o que foi medido

Hardware: ESP32 clássico, sensor a ~1 m, broker na LAN. Estado em 4 out 2026:

| Medida | Resultado |
|---|---|
| Janelas de onda em 60 s | 23 consecutivas (`seq 46→68`), **0 perdidas, 0 gaps, 0 dropped, 0 unsent** |
| Taxa medida das janelas | 100,00 Hz |
| Escala de aceleração | |a| ≈ 1,00 g com o sensor parado (±16 g / 32768 correto) |
| RAM / flash | 36,3 % / 80,0 % |
| `zenith/readings` | ~4,2 leituras/s medidas no broker (alvo 5/s, ver `PUBLISH_INTERVAL_MS`) |

Decisões de desempenho:

- **Frame binário** para a onda, em vez de JSON.
- **Sem re-scan bloqueante** quando já há um sensor conectado: o modo
  automático tem `MAX_AUTO_SENSORS` vagas, e uma vaga vazia fazia o nó
  escanear 10 s em loop, parando leituras e janelas. Agora a vaga vazia é
  tentada a cada 5 min, com scan de 4 s. Fixe o MAC em `SENSOR_ADDRESSES`
  para não escanear nunca.
- **Boot sem esperar redes ausentes**: o nó só tenta as redes WiFi que o scan
  viu (antes, cada rede configurada e ausente custava ~8 s).
- A FFT (3 × 256 pontos) roda no `loop()`, não na task do BLE, para não
  atrasar notificações.
- **Não mexi no modo de economia do WiFi**: a pesquisa que fiz sobre
  coexistência WiFi+BLE do ESP32 foi inconclusiva sobre `WIFI_PS_NONE`. Só
  mudar com medição (os contadores `gaps`/`dropped` mostram a perda).

## Testes

`src/wtvb01.{h,cpp}` não tem dependências e compila no host. Os testes usam
**bytes reais capturados do sensor**: a captura antiga (Go e C++ usam a mesma)
e três capturas do firmware 10057.2.7 em `test/testdata/` (Default 100 Hz,
Now data 100 Hz e a troca de modo, com a notificação mista 32+40 bytes):

```bash
c++ -std=c++17 -Wall -Wextra -I src -o /tmp/wtvb01_test test/decoder_test.cpp src/wtvb01.cpp
/tmp/wtvb01_test ../../internal/protocol/wtvb01/testdata/capture-wtvb01-bt50.hex
```

Cobrem, entre outros: detecção 32/40 bytes, MTU baixo (notificações de 20, 7 e 1
byte), troca de modo no meio do fluxo, rejeição de tempo impossível,
acumulador de janelas (gaps, relógio voltando, 200 Hz), FFT (erro < 0,25 Hz) e
RMS. A verificação mais forte da parte Default continua sendo que o broadcast
`0x61` e as leituras `0x71` codificam os mesmos registradores de forma
independente, então precisam decodificar igual.

Build do firmware: `pio run -e esp32dev` (ou `-t upload`).

## Resolução de problemas

**Sensor não encontrado.** Na maioria das vezes o app do celular ainda
está segurando a conexão — periféricos BLE aceitam um central por vez e
param de anunciar enquanto conectados. Desconecte nas configurações de
Bluetooth do celular, não basta fechar o app. Fora isso, confirme que o
sensor está ligado.

**Compila mas não chega dado.** Confirme que a placa tem BLE de verdade
(não é um ESP32-S2) e que `SENSOR_ADDRESS` no `config.h` está vazio ou
casa em minúsculas com o MAC do sensor.

**MQTT `rc=-4` (timeout) no broker público.** `test.mosquitto.org` demorou
ou recusou conexões do ESP32 nos testes, mesmo alcançável do PC. Use um
broker próprio (há um Mosquitto com autenticação no `docker-compose.yml` do
repositório Zenith, em `infra/mosquitto`).

**Monitoramento parou depois de ligar o Now data.** O app lê o
`zenith/readings`; confira que o nó está publicando `"mode":"instant"` e que
`SENSOR_STATUS_POLL_MS` não é 0. Para voltar ao modo antigo, configure
`SENSOR_DATA_MODE_INSTANT 0` (com `SENSOR_CONFIGURE_DATA_MODE 1`), grave uma
vez, e depois desligue a flag de configuração.
