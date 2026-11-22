# MPU-6050 — calibração estática e medição de inclinação (ESP32)

Instrumentação Eletrônica para Engenharia — UnB/FCTE — 02/2026 — Entrega 2
Rafael Silva Frota · Aline de Carvalho Rodrigues

```
firmware/mpu6050_inclinacao/   firmware do ESP32 (Arduino IDE)
scripts/verificar_eixos.py     verificação do sensor: autoteste + 6 posições + gráfico por eixo
scripts/capturar.py            terminal que envia comandos e grava os CSV
scripts/processar.py           calcula ganho/offset, ângulos, gera figuras e tabelas
scripts/simular_dados.py       gera dados FALSOS só para testar o processamento
dados/                         CSV medidos (calibracao.csv, angulos.csv)
relatorio/                     relatório IEEE (entrega2.tex) + figuras/ e tabelas/ geradas
```

## 1. Montagem

| MPU-6050 (GY-521) | ESP32 DevKit |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |
| AD0 | GND (endereço 0x68) |


## 3. Verificar o funcionamento do sensor (gráfico de cada eixo)

```bash

python scripts/verificar_eixos.py --porta COM32   
```

O script conduz tudo:

1. **Autoteste do fabricante.** Com a placa parada, compara a resposta do autoteste com o valor
   gravado de fábrica. Cada eixo é aprovado se a diferença ficar em ±14 %.
2. **Seis posições.** Para cada posição (+Z, −Z, +X, −X, +Y, −Y) o script diz como apoiar o módulo,
   espera o Enter, coleta 200 leituras e mostra a média de cada eixo comparada ao ideal (±1 g no eixo
   alinhado, 0 g nos outros). Se algo parecer errado, digite `r` para repetir.
3. **Análise.** Compara offset, ganho e ruído com o datasheet e gera
   `relatorio/figuras/verificacao_eixos.pdf` (e `.png`), `tabelas/tab_verificacao.tex` e
   `tabelas/tab_autoteste.tex`.

Os mesmos dados (`dados/calibracao.csv`) servem para a calibração, então não precisa coletar as seis
posições de novo no passo seguinte. Para refazer só o gráfico, sem o ESP32:
`python scripts/verificar_eixos.py`.

## 4. Coletar os ângulos

```bash
python scripts/capturar.py --porta COM3          

(Feche o Monitor Serial da IDE antes: só um programa pode usar a porta.)

**Calibração.** Se você já rodou o `verificar_eixos.py`, as seis posições já estão em
`dados/calibracao.csv`. Senão, use `CAL +Z`, `CAL -Z`, `CAL +X`, `CAL -X`, `CAL +Y` e `CAL -Y` no
`capturar.py` (200 leituras cada; repetir uma posição substitui a anterior). Depois rode `python scripts/processar.py` uma vez: ele imprime uma linha `SETCAL ...`. Copie e cole
no terminal do `capturar.py` para gravar a calibração no ESP32 (fica salva mesmo desligado).

**Ângulos.** Monte o sensor no braço. Para cada ângulo lido no transferidor (sugestão: 0 a 180° de
15 em 15°), pare o braço, solte, e digite:

```
comando> ANG 0
comando> ANG 15
...
comando> ANG 180
```
## 5. Processar e gerar o relatório

```bash
python scripts/processar.py
```

Gera em `relatorio/figuras/` e `relatorio/tabelas/`:

- `verificacao_eixos.pdf`: leitura de cada eixo nas 6 posições ±1 g. É o gráfico de verificação
  que faltou na Entrega 1.
- `erro_angulo.pdf`: erro (referência − medido) × ângulo, sem e com calibração.
- `tab_calibracao.tex`, `tab_metricas.tex`, `tab_angulos.tex` (completa, opcional) e `valores.tex`,
  com os números citados no texto, que entram automaticamente no relatório.

No Overleaf: envie a pasta `relatorio/` inteira (com `figuras/` e `tabelas/`), defina
`entrega2.tex` como documento principal e compile. Depois revise e apague todos os trechos
**vermelhos** (`\pendente{...}`), que são as frases que dependem do que vocês observaram, e coloque
o link do repositório na Introdução.

## 6. Demonstração em sala 

Monitor Serial a 115200 baud. Ao ligar, o ESP32 já mostra `theta = xx.xx graus` a cada 100 ms
(média móvel de 10 amostras).

## Comandos do firmware

| Comando | Efeito |
|---|---|
| `SELFTEST` | autoteste do acelerômetro (fabricante), aprovado se a variação for ≤ ±14 % |
| `CAL <+X\|-X\|+Y\|-Y\|+Z\|-Z>` | 200 leituras brutas na posição indicada |
| `ANG <graus>` | 100 leituras no ângulo de referência |
| `STREAM <0\|1>` | desliga/liga a exibição contínua do ângulo |
| `SETCAL gx ox gy oy gz oz` | grava ganhos e offsets (g) na memória |
| `SETMONT <1\|-1> <offset_graus>` | ajuste de montagem: θ = sinal·θ + offset |
| `ZERA` | volta à calibração identidade |
| `INFO` | mostra a configuração |
