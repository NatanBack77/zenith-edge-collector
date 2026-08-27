# O que o sensor mede de verdade

O WTVB01-BT50 reporta cinco coisas. Quatro descrevem vibração e são
fáceis de confundir, porque todas se movem juntas quando a máquina treme.
Elas respondem perguntas diferentes, e cada uma pega um tipo de falha
diferente.

## Os quatro indicadores de vibração

Uma superfície vibrando vai e volta. Descreva esse movimento de três
formas e você tem três dos indicadores:

| Indicador | Responde | Unidade | Eixos |
|---|---|---|---|
| **Displacement** | Quão **longe** ela se move? | µm | X, Y, Z |
| **Velocity** | Quão **rápido** ela se move? | mm/s | X, Y, Z |
| **Frequency** | Com que **frequência** ela se move? | Hz | X, Y, Z |
| **Angular amplitude** | Quanto ela **inclina** ao se mover? | graus | X, Y, Z |

### Displacement — amplitude do percurso

A distância de pico que a superfície percorre num ciclo de vibração. Um
eixo desbalanceado oscilando 200 µm fora de centro reporta 200 µm,
independente da rotação.

Displacement domina em **baixa frequência**. A 5 Hz um movimento físico
grande produz velocidade baixa, então o displacement pega enquanto a
velocity mal registra. É o indicador para desbalanceamento,
desalinhamento, folga, eixo empenado e balanço estrutural — problemas
lentos e de curso grande.

### Velocity — o número geral de saúde

Quão rápido a superfície se move. É o número contra o qual a maioria das
normas de manutenção preditiva é escrita (ISO 10816 / ISO 20816 definem
limites de vibração em mm/s RMS), porque a velocity é razoavelmente plana
na faixa média de frequência onde vive a maior parte das falhas
mecânicas.

**Se você só puder acompanhar um número, acompanhe velocity.** Velocity
subindo numa máquina antes estável é o sinal clássico de "algo está indo
mal".

### Frequency — onde está a energia, o que revela a causa

Quantos ciclos por segundo. Este é o indicador **diagnóstico**: a
amplitude diz *que* algo está errado, a frequência diz *o quê*.

As falhas aparecem em múltiplos característicos da rotação do eixo. Se um
motor gira a 1800 rpm (30 Hz):

- Energia em **30 Hz** (1× a rotação) → desbalanceamento
- Energia em **60 Hz** (2×) → desalinhamento
- Energia em **múltiplos altos e não inteiros** → defeito de rolamento
- Energia em **nº de pás ou dentes × rotação** → problema de pá ou
  engrenagem

Duas máquinas podem mostrar os mesmos 5 mm/s de velocity e precisar de
reparos completamente diferentes. A frequência é o que separa as duas.

### Angular amplitude — balanço, não deslocamento

Quanto o sensor **inclina** ao longo do ciclo de vibração, em graus. O
manual chama isso de "angular vibration amplitude"; no código chamamos de
`angle`.

Isto **não** é a orientação de montagem do sensor. Um sensor deitado e
parado reporta ≈0°, e um sensor parafusado numa parede vertical também.
O que ele mede é oscilação rotacional: a superfície balançando ou
torcendo em vez de se mover em linha reta. Útil para detectar folga, pé
manco ou vibração torcional — casos em que parte da máquina pivota em
torno de um ponto em vez de transladar.

### Como eles se relacionam

Para uma vibração senoidal, os três indicadores lineares **não são
independentes** — estão ligados pela frequência:

```
velocity  ≈  2π × frequency × displacement
```

Então 100 µm a 50 Hz dá cerca de 31 mm/s, enquanto os mesmos 100 µm a
2 Hz dão só 1,3 mm/s. É por isso que displacement e velocity discordam
sobre qual máquina está "pior", e por isso os dois são reportados:

- **Baixa frequência** → displacement é o indicador sensível
- **Média frequência** → velocity é o indicador sensível
- **Alta frequência** → seria aceleração, que este sensor não expõe no
  bloco de vibração

Três eixos importam porque vibração tem direção. Energia radial (X/Y)
aponta desbalanceamento e desgaste de rolamento; energia axial (Z, ao
longo do eixo) aponta desalinhamento e problema em mancal de escora.

## Temperature — leia o rótulo com atenção

A tabela de registradores do manual chama o registrador `0x40` de
**"Product temperature"**: a temperatura do próprio módulo sensor.

**Não** é a temperatura da máquina, nem uma sonda de ambiente calibrada.
Montado numa máquina quente, o módulo lê algo *entre* a superfície da
máquina e o ar em volta — dominado pela condução através da fixação, e
com atraso em relação a mudanças reais.

Por isso o modelo de dados chama isso de `device.temperature` e nunca
`bearing_temperature` ou `motor_temperature`. Nomear pela máquina
convidaria alguém a jusante a criar alarme em cima disso como se fosse
sonda de mancal, o que não é.

Observado no hardware: o sensor parado na mesa reportou 24,4–25,1 °C numa
sala mais ou menos nessa temperatura — esperado, já que sem fonte de
calor o módulo entra em equilíbrio com o ar.

Para o que serve de verdade: detectar que o próprio sensor está
esquentando demais, corrigir deriva térmica nas leituras de vibração e
pegar eventos térmicos grosseiros. Para temperatura real de mancal, use
uma sonda no mancal.

## Na prática

O que importa é uma linha de base saudável seguida de mudança — números
absolutos dizem pouco sem saber o que a máquina normalmente faz.

1. Estabeleça a linha de base de todos os indicadores com a máquina
   comprovadamente boa.
2. Acompanhe **velocity** para saúde geral; é o alvo dos limites ISO.
3. Quando a velocity subir, leia **frequency** para identificar a causa.
4. Cruze com **displacement** para falhas lentas e de curso grande, que a
   velocity subestima.
5. Trate **temperature** como sinal sobre o sensor, não sobre a máquina.

## Fontes

- [Manual do WTVB01-BT50 (espelho RobotShop)](https://cdn.robotshop.com/rbm/f83835f4-5e29-4ee0-9cc2-e49300031503/b/bc40f091-5d65-4712-969d-707ac88c1ca4/8d1ba329_wtvb01-bt50-manual.pdf)
- [Manual WIT WTVB01-BT50 (ManualsLib)](https://www.manualslib.com/manual/3151193/Wit-Wtvb01-Bt50.html)
- [Página de sensores de vibração da WitMotion](https://www.wit-motion.com/Vibration.html)
- [Página do produto WTVB01-BT50](https://witmotion-sensor.com/products/wtvb01-bt50-bluetooth-50m-wireless-multi-connected-vibration-sensor)
