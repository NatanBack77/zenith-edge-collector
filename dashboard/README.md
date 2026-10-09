# Dashboard estático (`dashboard/index.html`)

Página única, sem build: abra o `index.html` no navegador. Ela assina o MQTT por WebSocket e desenha, **por sensor** (um cartão por MAC que publicar), gráficos de linha de **velocidade, deslocamento, ângulo e frequência** nos eixos X, Y e Z, mais temperatura, bateria e RSSI. Mantém só os últimos **40 pontos** de cada gráfico.

## O que ela usa (lido no código)
- **Broker:** `wss://test.mosquitto.org:8081/mqtt`, o **broker público de teste**, fixo na constante `BROKER_URL`. Sem usuário nem senha.
- **Tópicos:** `zenith/readings/#` (leituras em JSON) e `zenith/status` (ignorado na tela, só mantém a conexão).
- **Bibliotecas por CDN:** `mqtt.js` (unpkg) e `Chart.js` 4.4.4 (jsDelivr). Precisa de internet.

## Limites
- **Não usa o broker do projeto** (Mosquitto local ou na AWS, com autenticação). O firmware publica no broker configurado no `config.h`; **esta página só mostra dado se o nó estiver publicando no broker público**, e então qualquer pessoa pode ler e também publicar nesses tópicos: trate os dados como **não confiáveis**.
- Para apontar para o broker do projeto, troque `BROKER_URL` e acrescente usuário e senha na conexão do `mqtt.connect`. Uma senha numa página estática fica **visível para quem abrir a página**: use só o usuário **somente leitura** (`zenith-reader`), nunca o `zenith-edge`.
- Mostra as **leituras derivadas** do `zenith/readings` (velocidade, deslocamento e ângulo que o nó deriva da onda), **não a onda de aceleração** (`zenith/waveform`) nem o diagnóstico do detector. A leitura da onda, a severidade e o espectro estão no app web (repositório `zenith-plataforma`).
- Foi uma tela de apoio para ver o nó publicando; **não foi testada** de novo com o firmware atual (125 Hz).
