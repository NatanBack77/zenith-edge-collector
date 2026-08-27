# Protocolo BLE do WTVB01-BT50

Duas fontes sustentam este documento, e toda afirmação abaixo diz de qual
delas vem:

1. **O SDK Python oficial.** `Python/BWT901BLE5.0_python_sdk` do
   [WITMOTION/WitBluetooth_BWT901BLE5_0](https://github.com/WITMOTION/WitBluetooth_BWT901BLE5_0)
   (commit `9efaab0`), arquivos `device_model.py` e `test.py`. Este é o
   único código de SDK que existe naquele repositório. Não existe
   `Wtvb01Processor`, nem `Wtvb01Resolver`, nem projeto
   `Wit.Example_WTVB01BT50`.
2. **Bytes capturados de um WTVB01-BT50 físico** (MAC
   `E6:6B:9A:CC:88:25`), guardados em
   `internal/protocol/wtvb01/testdata/capture-wtvb01-bt50.hex`.

O SDK é genérico para a família WT e **não** decodifica os campos de
vibração do WTVB01, então é a captura que define o layout do pacote.
Onde os dois discordam, a captura vence e isso está sinalizado.

## 1. Descoberta e ciclo de vida da conexão

De `test.py` e `device_model.py`; o pacote `internal/ble` espelha isso:

1. **Scan** — `BleakScanner.discover(timeout=20.0)`, mantendo dispositivos
   cujo nome contenha `"WT"` (`test.py:22`). Nosso scanner também casa
   pelo UUID de serviço anunciado, porque um dispositivo BLE pode não
   anunciar nome nenhum.
2. **Connect** — `BleakClient(BLEDevice, timeout=15)` (`device_model.py:62`).
3. **Resolver characteristics** — itera `client.services` e casa os UUIDs
   fixos listados abaixo (`device_model.py:71-82`).
4. **Subscribe** — `client.start_notify(notify_uuid, onDataReceived)`
   (`device_model.py:93`).
5. **Polling de registrador** — após 3 s de acomodação, uma task em
   background dispara leituras de registrador a cada 100 ms
   (`device_model.py:88, 112-117`). Para o WTVB01 isso é **opcional**: o
   broadcast `0x61` já carrega todos os registradores de medição.
6. **Acumular** — `deviceData` é um dict atualizado a cada pacote, com um
   callback disparado a cada atualização (`device_model.py:39-56, 156`).
   Nosso `Decoder` mantém o mesmo comportamento de acumular e emitir, mas
   com struct tipada.
7. **Fechar** — limpa `isOpen`, depois `stop_notify` (`device_model.py:96-103`).

## 2. UUIDs BLE

De `device_model.py:66-68`, literal:

```python
target_service_uuid              = "0000ffe5-0000-1000-8000-00805f9a34fb"
target_characteristic_uuid_read  = "0000ffe4-0000-1000-8000-00805f9a34fb"  # notify
target_characteristic_uuid_write = "0000ffe9-0000-1000-8000-00805f9a34fb"  # write
```

Confirmado no hardware: o sensor físico anuncia o serviço `ffe5` e as
duas characteristics resolvem.

## 3. Framing

De `device_model.py:121-133`:

```python
for var in tempdata:
    self.TempBytes.append(var)
    if len(self.TempBytes) == 1 and self.TempBytes[0] != 0x55:
        del self.TempBytes[0]; continue
    if len(self.TempBytes) == 2 and (self.TempBytes[1] != 0x61 and self.TempBytes[1] != 0x71):
        del self.TempBytes[0]; continue
    if len(self.TempBytes) == 20:
        self.processData(self.TempBytes); self.TempBytes.clear()
```

Regras: byte 0 é `0x55`; byte 1 é o tipo, `0x61` ou `0x71`; qualquer
outra coisa ressincroniza.

### ⚠️ Tamanho do pacote: o SDK está errado para este sensor

O SDK fixa **20 bytes para os dois tipos**. Medido sobre 3772 bytes
capturados:

| Tipo | SDK assume | Real (WTVB01-BT50) | Pacotes medidos |
|---|---|---|---|
| `0x61` broadcast | 20 | **32** | 110 |
| `0x71` leitura de registrador | 20 | **20** | 11 |

Decodificar um `0x61` como 20 bytes lê errado **todos** os campos — e
produz números de aparência plausível, que foi exatamente por que isso só
foi pego comparando contra as leituras de registrador. Por isso o decoder
escolhe o tamanho pelo tipo (`packetLenFor`).

O layout do `0x71` bate com o SDK e com o manual oficial:
`0x55 0x71 <registrador inicial, 2 bytes LE> <16 bytes = 8 registradores, LE>`.

## 4. Mapa de registradores

Endereços confirmados pelo manual oficial e pela captura; veja §6 para a
verificação cruzada.

| Registrador | Campo | Escala | Unidade |
|---|---|---|---|
| `0x3A` `0x3B` `0x3C` | Velocidade de vibração X/Y/Z | cru | mm/s |
| `0x3D` `0x3E` `0x3F` | Amplitude angular de vibração X/Y/Z | `cru / 32768 * 180` | graus |
| `0x40` | Temperatura | `cru / 100` | °C |
| `0x41` `0x42` `0x43` | Deslocamento de vibração X/Y/Z | cru | µm |
| `0x44` `0x45` `0x46` | Frequência de vibração X/Y/Z | cru | Hz |

Todos os valores são int16 com sinal, little-endian. O `getSignInt16`
(`device_model.py:180-184`) subtrai 2^16 quando o valor cru é ≥ 2^15, o
que é complemento de dois padrão.

Faixas documentadas (manual): velocidade 0–100 mm/s, deslocamento
0–30000 µm, frequência 1–100 Hz, temperatura −40 a +85 °C.

### Temperature é a temperatura do próprio módulo

A tabela de registradores do manual chama `0x40` de **"Product
temperature"** — a temperatura do módulo sensor em si, não da máquina em
que ele está montado e não uma sonda de ambiente calibrada. É modelada
como `device.temperature` e nunca deve ser chamada de
`bearing_temperature` ou `motor_temperature`.

Observado no hardware: o sensor parado na mesa reportou 24,4–25,1 °C numa
sala mais ou menos nessa temperatura, o que é esperado — sem fonte de
calor o módulo entra em equilíbrio com o ar em volta. Montado numa
máquina quente ele lê algo entre a máquina e o ar ambiente, dominado pela
condução através da fixação, e com atraso. Útil como sinal de sanidade e
para detectar deriva; não substitui uma sonda no mancal.

## 5. O pacote broadcast `0x61`

32 bytes: 2 bytes de cabeçalho seguidos de 15 int16 com sinal.

| Índice do valor | Bytes | Significado |
|---|---|---|
| 0-2 | 2-7 | Velocity X, Y, Z (registradores `0x3A`-`0x3C`) |
| 3-5 | 8-13 | Amplitude angular X, Y, Z (`0x3D`-`0x3F`) |
| 6 | 14-15 | Temperature (`0x40`) |
| 7-9 | 16-21 | Displacement X, Y, Z (`0x41`-`0x43`) |
| 10-12 | 22-27 | Frequency X, Y, Z (`0x44`-`0x46`) |
| 13 | 28-29 | Constante `0x0000` em todas as capturas; não decodificado |
| 14 | 30-31 | Contador com deriva lenta; não é registrador de medição documentado. O app oficial mostra um campo "Power Percent(%)", que este valor pode alimentar — não confirmado, então não decodificado |

Isso bate com a ordem declarada no manual: *"vibration velocity XYZ,
vibration angle XYZ, temperature, vibration displacement XYZ, vibration
frequency XYZ, with the low byte first and the high byte last."* O manual
descreve esse pacote como 28 bytes (cabeçalho mais os 13 valores de
medição); o sensor de fato envia 32, com os dois valores extras acima.

## 6. Como o layout foi verificado

O broadcast `0x61` e as leituras de registrador `0x71` são codificações
**independentes dos mesmos registradores**, então podem ser conferidos um
contra o outro sem nenhuma referência externa. Da captura:

```
valores 0x61:            [17, 7, 18, 233, 243, 57, 2455, 300, 19, 239, 9, 13, 10, 0, 418]
registradores 0x71 0x3A: [17, 7, 18, 233, 243, 57, 2436, 300]
registradores 0x71 0x42: [19, 239, 9, 13, 10, -21, 27, -3]
```

Os valores 0-7 do broadcast são idênticos byte a byte aos registradores
`0x3A`-`0x41`, e os valores 8-12 aos registradores `0x42`-`0x46`. A
temperatura difere só porque é recalculada continuamente (2455 contra
2436, ou seja 24,55 contra 24,36 °C).

Por causa disso, o decoder roteia os dois tipos de pacote por um único
dispatch por endereço de registrador (`applyRegister`), em vez de manter
dois layouts paralelos. O teste `TestOutputAndRegisterPacketsAgree`
garante essa equivalência.

A temperatura ainda é confirmada em valor absoluto: `0x40 / 100` dá
24,36 °C, batendo com a sala onde a captura foi feita.

## 7. Formato dos comandos

De `device_model.py:214-246`.

**Ler um registrador** (dispara uma resposta `0x71`):
```
[0xFF, 0xAA, 0x27, regAddr, 0x00]
```
O byte 2 é o registrador fixo de disparo de leitura `0x27`; o byte 3 é o
registrador a ser lido.

**Escrever um registrador:**
```
[0xFF, 0xAA, regAddr, valorLow, valorHigh]
```

**Unlock** antes de escrever configuração: `writeReg(0x69, 0xB588)`.
**Save** depois de escrever: `writeReg(0x00, 0x0000)`.

Todos são enviados para a characteristic de escrita `ffe9`.

## 8. Ainda não confirmado

- **Escalas de velocity, displacement e frequency.** Os endereços de
  registrador e as unidades estão documentados, e os valores são
  fisicamente plausíveis em repouso (≈1 mm/s, 6-21 µm, 9-16 Hz), mas não
  houve comparação lado a lado com o app oficial da WitMotion sob
  vibração real. Se alguma escala estiver errada, é mudança de uma linha
  em `internal/protocol/wtvb01/registers.go`.
- **Valor 14 do pacote `0x61`** (o contador com deriva). Provavelmente a
  porcentagem de bateria mostrada no app, mas não verificado, então não
  decodificado.

Temperatura, tamanhos de pacote, endereços de registrador, framing, UUIDs
e a codificação dos comandos estão todos confirmados.
