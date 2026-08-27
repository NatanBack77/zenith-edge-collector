# Arquitetura e fluxo de dados

Dois coletores leem o mesmo sensor e emitem o mesmo schema JSON. Qual
deles você usa depende de haver ou não uma máquina Linux junto ao
equipamento.

- **Coletor Go** (`cmd/zenith-edge`) — roda em Linux via BlueZ. Usado
  para desenvolvimento, trabalho de protocolo e captura de fixtures.
- **Nó ESP32** (`firmware/esp32-zenith-node`) — hardware autônomo, publica
  em MQTT por WiFi. Usado em campo.

O decoder é a mesma lógica nos dois, portada linha a linha, e ambos são
testados contra os mesmos bytes capturados do sensor.

## O caminho completo, ponta a ponta

```mermaid
flowchart TD
    ACCEL["Acelerometro MEMS<br/>amostra a superficie que vibra"]
    DSP["Processamento no sensor<br/>calcula velocity, displacement,<br/>frequency, angular amplitude"]
    REGS["Registradores 0x3A a 0x46<br/>13 valores int16 com sinal"]
    FRAME["Framing<br/>acha 0x55, le o byte de tipo,<br/>junta o pacote inteiro"]
    DISPATCH["Dispatch por registrador<br/>mapeia cada int16 para seu endereco<br/>e aplica a escala"]
    READING["SensorReading<br/>velocity, displacement, angle,<br/>frequency, device.temperature"]
    OUT["Terminal no Go<br/>ou MQTT no ESP32"]

    ACCEL --> DSP
    DSP --> REGS
    REGS -->|"BLE notify em ffe4<br/>pacote 0x61, 32 bytes, sem pedir"| FRAME
    FRAME --> DISPATCH
    DISPATCH --> READING
    READING -->|JSON| OUT

    style ACCEL fill:#e8f0fe,stroke:#4285f4
    style DSP fill:#e8f0fe,stroke:#4285f4
    style REGS fill:#e8f0fe,stroke:#4285f4
    style FRAME fill:#e6f4ea,stroke:#34a853
    style DISPATCH fill:#e6f4ea,stroke:#34a853
    style READING fill:#e6f4ea,stroke:#34a853
    style OUT fill:#fef7e0,stroke:#fbbc04
```

Azul é o trabalho do próprio sensor, verde o do coletor, amarelo a saída.

**O sensor faz o processamento de sinal.** Quando o dado chega ao
coletor já é velocity, displacement e frequency — o coletor nunca vê
amostras de aceleração bruta e não faz FFT nenhuma.

## Sequência de conexão

As medições **não** estão no advertisement BLE, só os UUIDs de serviço
estão, então conectar é obrigatório. Escuta passiva não funciona.

```mermaid
sequenceDiagram
    participant C as Coletor
    participant S as WTVB01-BT50

    C->>S: scan BLE
    S-->>C: advertisement, nome WTVB01-BT50, servico ffe5
    Note over C: casar pelo UUID de servico e nao pelo nome,<br/>algumas unidades nao anunciam nome

    C->>S: connect
    C->>S: descobre servico ffe5
    S-->>C: characteristic notify ffe4, write ffe9

    C->>S: subscribe em ffe4

    loop continuamente, sem pedir
        S-->>C: pacote 0x61, 32 bytes<br/>todos os 13 registradores de medicao
    end

    Note over C,S: a characteristic de write nunca e necessaria,<br/>o broadcast ja carrega tudo

    opt verificacao opcional
        C->>S: FF AA 27 3A 00, le bloco 0x3A
        S-->>C: pacote 0x71, 20 bytes, 8 registradores
    end
```

O caminho de leitura de registrador existe só para verificar o decoder.
Como `0x61` e `0x71` são codificações independentes dos mesmos
registradores, decodificar os dois e comparar é um teste de corretude que
**não precisa de referência externa** — foi assim que o layout do pacote
foi provado, e isso roda como teste nas duas implementações.

## Dentro do decoder

Os dois tipos de pacote passam por um único dispatch por endereço de
registrador, em vez de dois layouts paralelos, porque a captura mostrou
que os 13 primeiros valores do broadcast são idênticos aos registradores
`0x3A`–`0x46`.

```mermaid
flowchart TD
    IN["bytes BLE crus<br/>podem dividir ou juntar pacotes"]
    B0{"byte 0 e 0x55?"}
    DROP["descarta byte e ressincroniza"]
    B1{"byte 1, qual tipo?"}
    L32["espera 32 bytes"]
    L20["espera 20 bytes"]
    FULL{"pacote inteiro no buffer?"}
    WAIT["continua acumulando"]
    MAP61["valores 0 a 12<br/>mapeia para registradores 0x3A a 0x46"]
    MAP71["le registrador inicial nos bytes 2 e 3,<br/>mapeia 8 registradores"]
    APPLY["applyRegister com endereco e valor cru"]
    SCALE["0x3A a 0x3C velocity, cru mm/s<br/>0x3D a 0x3F angle, cru dividido por 32768 vezes 180 graus<br/>0x40 temperature, cru dividido por 100 C<br/>0x41 a 0x43 displacement, cru um<br/>0x44 a 0x46 frequency, cru Hz"]
    EMIT["atualiza SensorReading e emite"]

    IN --> B0
    B0 -->|nao| DROP
    DROP --> B0
    B0 -->|sim| B1
    B1 -->|0x61| L32
    B1 -->|0x71| L20
    B1 -->|outro| DROP
    L32 --> FULL
    L20 --> FULL
    FULL -->|nao| WAIT
    WAIT --> IN
    FULL -->|sim, 0x61| MAP61
    FULL -->|sim, 0x71| MAP71
    MAP61 --> APPLY
    MAP71 --> APPLY
    APPLY --> SCALE
    SCALE --> EMIT

    style DROP fill:#fce8e6,stroke:#ea4335
    style EMIT fill:#e6f4ea,stroke:#34a853
```

O decoder **acumula**: um pacote `0x61` atualiza todos os campos de uma
vez, enquanto uma leitura `0x71` atualiza só os oito registradores do seu
bloco, deixando o resto no último valor conhecido.

### A cilada dos 20 bytes

O SDK genérico BWT901 da WitMotion fixa 20 bytes para **os dois** tipos
de pacote. Está certo para `0x71` e errado para o `0x61` do WTVB01, que
tem 32 bytes.

Isso é pior que um off-by-one comum porque decodificar um pacote de 32
bytes como 20 **não quebra nem gera lixo óbvio** — lê os campos errados e
produz números *plausíveis*. Na primeira execução reportou um "angle" de
13,35°, que na verdade era o registrador de temperatura lido no offset
errado. Só foi pego comparando contra as leituras de registrador.

## Formas de deploy

```mermaid
flowchart LR
    S1["sensor"]
    GO["zenith-edge<br/>em Linux com BlueZ"]
    TERM["terminal"]
    S2["sensor"]
    ESP["no ESP32"]
    BROKER["broker MQTT"]
    APPS["dashboards,<br/>alarmes, armazenamento"]

    S1 -->|BLE| GO
    GO --> TERM
    S2 -->|BLE| ESP
    ESP -->|"WiFi e MQTT<br/>zenith/readings/mac"| BROKER
    BROKER --> APPS

    style GO fill:#e8f0fe,stroke:#4285f4
    style TERM fill:#e8f0fe,stroke:#4285f4
    style ESP fill:#e6f4ea,stroke:#34a853
    style BROKER fill:#e6f4ea,stroke:#34a853
    style APPS fill:#e6f4ea,stroke:#34a853
```

A linha de cima é o caminho de desenvolvimento, a de baixo o deploy em
campo. Os dois emitem o mesmo JSON, então qualquer consumidor a jusante
funciona com os dois sem saber qual produziu a leitura.

## Por que o gargalo é o sensor, não o link

As notificações BLE chegam a cada 20–50 ms, mas os registradores de
vibração atualizam bem mais devagar — nas capturas os valores de vibração
ficaram parados por dezenas de pacotes enquanto só a temperatura e o
contador final se moviam. O sensor calcula as métricas de vibração sobre
uma janela interna.

Consequências:

- Publicar mais rápido que ~1 Hz envia duplicatas. O nó ESP32 agrupa em
  `PUBLISH_INTERVAL_MS` (1 s por padrão).
- Os timestamps marcam quando o coletor decodificou o pacote, não quando
  o sensor amostrou a superfície.
- Perder um pacote não custa nada; o próximo carrega o estado completo de
  novo. Não há codificação delta para ressincronizar.
