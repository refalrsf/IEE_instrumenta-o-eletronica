#!/usr/bin/env python3
"""
verificar_eixos.py — verificação do funcionamento do acelerômetro do MPU-6050.

Roteiro (Entrega 1 / pendência apontada pela professora):
  "Verifique o funcionamento do sensor através da extração e visualização dos
   dados ... Apresente um gráfico da leitura (em unidades de gravidade) para cada
   eixo do sensor ao colocá-lo em posições coincidentes com o vetor gravidade."

Algoritmo:
  1. Conecta ao ESP32 (firmware mpu6050_inclinacao) e desliga o modo STREAM.
  2. Autoteste do fabricante (comando SELFTEST): compara a resposta do
     autoteste com o valor gravado de fábrica; aprovado se a variação for ≤ ±14 %.
  3. Para cada uma das 6 posições (+Z, −Z, +X, −X, +Y, −Y):
       a. mostra como posicionar o módulo e espera o Enter;
       b. envia "CAL <pos>" e recebe 200 leituras (100 Hz) dos três eixos em g;
       c. mostra a média e o desvio de cada eixo e deixa repetir a posição;
       d. grava as leituras em dados/calibracao.csv.
  4. Calcula, para cada eixo, Ā(+1 g), Ā(−1 g), offset = (Ā+ + Ā−)/2,
     ganho = (Ā+ − Ā−)/2, desvio-padrão e a maior leitura transversal, e compara
     com as especificações do datasheet (PS-MPU-6000A-00, Tabela 6.2):
       offset zero-g: ±50 mg (X, Y) e ±80 mg (Z); sensibilidade: ±3 %;
       ruído: 400 µg/√Hz -> ≈3,3 mg RMS com o DLPF de 44 Hz.
  5. Gera o gráfico de cada eixo nas 6 posições e as tabelas LaTeX.

Uso:
  python verificar_eixos.py --porta COM5           # coleta + gráfico
  python verificar_eixos.py --porta COM5 --sem-autoteste
  python verificar_eixos.py                        # pergunta a porta (ou refaz o gráfico se já há dados)
  python verificar_eixos.py --so-grafico           # só refaz gráfico/tabelas a partir dos CSV

Requisitos: Python 3.8+, pip install numpy matplotlib pyserial
Este arquivo funciona sozinho (não depende de capturar.py nem de processar.py).

Saídas:
  dados/calibracao.csv, dados/autoteste.csv   (pastas ao lado do script, se usado sozinho)
  relatorio/figuras/verificacao_eixos.pdf (+ .png)
  relatorio/tabelas/tab_verificacao.tex, relatorio/tabelas/tab_autoteste.tex
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np

# Funciona sozinho (arquivo único). Dentro do repositório, usa as pastas
# dados/ e relatorio/ do projeto; fora dele, cria dados/ e saida/ ao lado do script.
_AQUI = os.path.dirname(os.path.abspath(__file__))
if os.path.isdir(os.path.join(_AQUI, "..", "relatorio")):
    DIR_DADOS = os.path.join(_AQUI, "..", "dados")
    DIR_SAIDA = os.path.join(_AQUI, "..", "relatorio")
else:
    DIR_DADOS = os.path.join(_AQUI, "dados")
    DIR_SAIDA = os.path.join(_AQUI, "saida")

# Evita UnicodeEncodeError no console do Windows (cp1252) com símbolos como ≈ e ±
for _f in (sys.stdout, sys.stderr):
    try:
        _f.reconfigure(errors="replace")
    except Exception:
        pass

ORDEM = ["+Z", "-Z", "+X", "-X", "+Y", "-Y"]
INSTRUCOES = {
    "+Z": "Placa DEITADA sobre a mesa, componentes para CIMA (eixo Z aponta para cima).",
    "-Z": "Placa DE CABEÇA PARA BAIXO, componentes voltados para a mesa.",
    "+X": "Placa EM PÉ, apoiada na borda, com a seta X do módulo apontando para CIMA.",
    "-X": "Placa EM PÉ, com a seta X apontando para BAIXO.",
    "+Y": "Placa EM PÉ, apoiada na borda, com a seta Y apontando para CIMA.",
    "-Y": "Placa EM PÉ, com a seta Y apontando para BAIXO.",
}

# Especificações do acelerômetro (PS-MPU-6000A-00, Tabela 6.2)
TOL_OFFSET_G = np.array([0.050, 0.050, 0.080])
TOL_GANHO = 0.03
RUIDO_ESPERADO_G = 400e-6 * math.sqrt(math.pi / 2 * 44)   # ≈ 3,3 mg RMS
CAB_ST = ["eixo", "codigo", "ft_lsb", "str_lsb", "variacao_pct", "resultado"]


# ---------------------------------------------------------------------------
# Comunicação serial com o firmware (mesmo código de capturar.py)
# ---------------------------------------------------------------------------
CAB_CAL = ["posicao", "amostra", "ax_g", "ay_g", "az_g"]


def substitui_bloco(caminho, cabecalho, chave, linhas_novas):
    """Reescreve o CSV removendo as linhas cuja 1ª coluna == chave e anexa as novas."""
    antigas = []
    if os.path.exists(caminho):
        with open(caminho, newline="", encoding="utf-8") as f:
            for lin in csv.reader(f):
                if not lin or lin[0].startswith("#") or lin == cabecalho:
                    continue
                if _mesma_chave(lin[0], chave):
                    continue
                antigas.append(lin)
    with open(caminho, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cabecalho)
        w.writerows(antigas)
        w.writerows(linhas_novas)
    return len(antigas)


def _mesma_chave(a, b):
    try:
        return abs(float(a) - float(b)) < 1e-6
    except ValueError:
        return a.strip().upper() == b.strip().upper()


def abre_porta(serial, porta, baud):
    """Abre a porta serial com uma mensagem clara se ela estiver ocupada ou não existir."""
    try:
        return serial.Serial(porta, baud, timeout=0.5)
    except serial.SerialException as e:
        ocupada = "PermissionError" in str(e) or "Acesso negado" in str(e) or "Access is denied" in str(e)
        if ocupada:
            sys.exit(f"\nERRO: a porta {porta} está em uso por outro programa.\n"
                     "  Feche o Monitor/Plotter Serial da Arduino IDE e qualquer outra janela que esteja\n"
                     "  rodando um destes scripts (IDLE, terminal). Se persistir, desconecte e reconecte o USB.")
        sys.exit(f"\nERRO: não foi possível abrir {porta}: {e}\n"
                 "  Confira o número da porta (Arduino IDE > Ferramentas > Porta) e o cabo USB.")


def sincroniza(ser, log, silencio_s=0.6, limite_s=8.0):
    """Após abrir a porta (o ESP32 reinicia): desliga o STREAM e descarta tudo
    (boot, configuração, '#FIM' antigos) até a serial ficar em silêncio."""
    time.sleep(1.5)
    ser.write(b"STREAM 0\n")
    t0 = ultimo = time.time()
    while time.time() - t0 < limite_s:
        if ser.readline():
            ultimo = time.time()
            continue
        if time.time() - ultimo >= silencio_s:
            break
    ser.reset_input_buffer()


def verifica_firmware(ser, log, timeout_s=3.0):
    """Confere se o ESP32 está com o firmware mpu6050_inclinacao (responde 'INFO' com '#FIM').
    Se não estiver, encerra com uma explicação em vez de esperar respostas que nunca virão."""
    ser.reset_input_buffer()
    ser.write(b"INFO\n")
    recebidas = []
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        b = ser.readline()
        if not b:
            continue
        linha = b.decode("ascii", errors="replace").strip()
        log.write(linha + "\n")
        if linha == "#FIM":
            return
        if linha and linha.isprintable() and "\ufffd" not in linha:
            recebidas.append(linha)
    exemplo = "\n".join("      " + l for l in recebidas[-3:]) or "      (nada)"
    sys.exit(
        "\nERRO: o ESP32 não respondeu como o firmware mpu6050_inclinacao.ino.\n"
        "  Ele está enviando:\n" + exemplo + "\n"
        "  Isso indica que a placa está com OUTRO programa gravado (ou sem programa).\n"
        "  Grave firmware/mpu6050_inclinacao/mpu6050_inclinacao.ino pela Arduino IDE\n"
        "  (placa 'ESP32 Dev Module'), feche o Monitor Serial e rode este script de novo.\n"
        "  Com o firmware certo, o Monitor Serial mostra '# MPU-6050 pronto' ao ligar.")


def executa(ser, comando, log, timeout_s=30.0):
    """Envia um comando e devolve (linhas_CAL, linhas_ANG, linhas_ST) até receber '#FIM'."""
    ser.reset_input_buffer()
    ser.write((comando.strip() + "\n").encode("ascii"))
    cal, ang, st = [], [], []
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        bruto = ser.readline()
        if not bruto:
            continue
        linha = bruto.decode("ascii", errors="replace").strip()
        log.write(linha + "\n")
        if linha.startswith("CAL,"):
            cal.append(linha.split(",")[1:])
        elif linha.startswith("ANG,"):
            ang.append(linha.split(",")[1:])
        elif linha.startswith("ST,"):
            st.append(linha.split(",")[1:])
            print(linha)
        elif linha == "#FIM":
            return cal, ang, st
        elif "\ufffd" in linha or not linha.isprintable():
            pass  # bytes do boot do ESP32 (115200 ≠ 74880 baud): só vão para o log
        else:
            print(linha)
        t0 = time.time() if (cal or ang) else t0  # renova o prazo enquanto chegam dados
    print("# (tempo esgotado esperando '#FIM')")
    return cal, ang, st


# ---------------------------------------------------------------------------
# Leitura, cálculo e gráfico (mesmo código de processar.py)
# ---------------------------------------------------------------------------
EIXOS = "XYZ"


POSICOES = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]


def le_csv(caminho):
    """Lê CSV com cabeçalho; linhas começando com '#' são comentários.
    Retorna (dict coluna -> lista de strings, simulado?)."""
    simulado = False
    cab, linhas = None, []
    with open(caminho, encoding="utf-8") as f:
        for lin in f:
            lin = lin.strip()
            if not lin:
                continue
            if lin.startswith("#"):
                simulado |= "SIMULADO" in lin.upper()
                continue
            campos = [c.strip() for c in lin.split(",")]
            if cab is None:
                cab = campos
            else:
                linhas.append(campos)
    cols = {c: [l[i] for l in linhas] for i, c in enumerate(cab)}
    return cols, simulado


def calibra(cal):
    pos = np.array([p.upper() for p in cal["posicao"]])
    A = np.column_stack([np.array(cal[c], float) for c in ("ax_g", "ay_g", "az_g")])
    faltando = [p for p in POSICOES if not np.any(pos == p)]
    if faltando:
        sys.exit(f"ERRO: faltam as posições {faltando} em calibracao.csv")
    medias = {p: A[pos == p].mean(axis=0) for p in POSICOES}
    desvios = {p: A[pos == p].std(axis=0, ddof=1) for p in POSICOES}
    n = {p: int(np.sum(pos == p)) for p in POSICOES}
    ganho, offset, a_mais, a_menos, sig = np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
    for i, e in enumerate(EIXOS):
        a_mais[i] = medias["+" + e][i]
        a_menos[i] = medias["-" + e][i]
        ganho[i] = (a_mais[i] - a_menos[i]) / 2.0
        offset[i] = (a_mais[i] + a_menos[i]) / 2.0
        sig[i] = max(desvios["+" + e][i], desvios["-" + e][i])
    return dict(pos=pos, A=A, medias=medias, desvios=desvios, n=n, ganho=ganho, offset=offset,
                a_mais=a_mais, a_menos=a_menos, sigma=sig)


def txt(x, casas):
    """Número com vírgula decimal e sinal de menos tipográfico (rótulos das figuras)."""
    return f"{x:.{casas}f}".replace(".", ",").replace("-", "\u2212")


def num(x, casas, sinal=False):
    fmt = f"{{:+.{casas}f}}" if sinal else f"{{:.{casas}f}}"
    return r"\num{" + fmt.format(x) + "}"


def configura_matplotlib():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import FuncFormatter
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "Nimbus Roman", "STIXGeneral", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 9,
        "axes.titlesize": 9,
        "axes.labelsize": 9,
        "legend.fontsize": 8,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.linewidth": 0.5,
        "lines.linewidth": 0.9,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
    })
    virgula = FuncFormatter(lambda v, _: f"{v:g}".replace(".", ",").replace("-", "\u2212"))
    return plt, virgula


def marca_simulado(fig):
    fig.text(0.5, 0.5, "DADOS SIMULADOS", ha="center", va="center", rotation=25,
             fontsize=38, color="red", alpha=0.18, fontweight="bold")


def fig_verificacao(c, caminho, simulado):
    plt, virgula = configura_matplotlib()
    fig, axs = plt.subplots(3, 1, figsize=(6.6, 4.6), sharex=True)
    cores = ["#1f4e79", "#c0504d", "#4f7f3a"]
    inicio = 0
    limites = []
    for p in POSICOES:
        idx = np.where(c["pos"] == p)[0]
        k = np.arange(inicio, inicio + len(idx))
        for i in range(3):
            axs[i].plot(k, c["A"][idx, i], color=cores[i], lw=0.7)
        limites.append((inicio, inicio + len(idx), p))
        inicio += len(idx)
    for i, ax in enumerate(axs):
        for ref in (-1, 0, 1):
            ax.axhline(ref, color="0.4", lw=0.6, ls="--", zorder=0)
        for a, b, p in limites:
            ax.axvline(a, color="0.75", lw=0.6, zorder=0)
            m = c["medias"][p][i]
            ax.text((a + b) / 2, m + (0.22 if m < 0.5 else -0.32), txt(m, 3),
                    ha="center", fontsize=7, color=cores[i])
        ax.set_ylim(-1.45, 1.45)
        ax.set_yticks([-1, 0, 1])
        ax.yaxis.set_major_formatter(virgula)
        ax.set_ylabel(f"$A_{EIXOS[i].lower()}$ [g]")
    for a, b, p in limites:
        axs[0].text((a + b) / 2, 1.62, p.replace("-", "\u2212"), ha="center", fontsize=8,
                    fontweight="bold")
    axs[-1].set_xlabel("amostra (100 Hz), posições em sequência")
    axs[-1].set_xlim(0, inicio)
    fig.align_ylabels(axs)
    if simulado:
        marca_simulado(fig)
    fig.savefig(caminho)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Coleta
# ---------------------------------------------------------------------------
def resumo_posicao(linhas, pos):
    A = np.array([[float(v) for v in l[2:5]] for l in linhas])
    m, s = A.mean(axis=0), A.std(axis=0, ddof=1)
    i = "XYZ".index(pos[1])
    esperado = [0.0, 0.0, 0.0]
    esperado[i] = 1.0 if pos[0] == "+" else -1.0
    print(f"    {len(A)} leituras  |  esperado (ideal): "
          + "  ".join(f"A{e.lower()}={v:+.0f} g" for e, v in zip("XYZ", esperado)))
    print("    medido (média ± desvio):  "
          + "  ".join(f"A{e.lower()}={m[k]:+.4f}±{1000 * s[k]:.1f}m" for k, e in enumerate("XYZ")))
    if abs(m[i] - esperado[i]) > 0.15 or np.max(np.abs(np.delete(m, i))) > 0.25:
        print("    ATENÇÃO: leitura longe do esperado — confira a orientação da placa e repita (r).")


def coleta(porta, baud, dir_dados, autoteste):
    import serial  # pyserial

    os.makedirs(dir_dados, exist_ok=True)
    arq_cal = os.path.join(dir_dados, "calibracao.csv")
    with abre_porta(serial, porta, baud) as ser, \
            open(os.path.join(dir_dados, "log_serial.txt"), "a", encoding="utf-8", errors="replace") as log:
        sincroniza(ser, log)                 # o ESP32 reinicia ao abrir a porta
        verifica_firmware(ser, log)

        if autoteste:
            input("\n[AUTOTESTE] Deixe a placa parada sobre a mesa e tecle Enter...")
            _, _, st = executa(ser, "SELFTEST", log)
            if st:
                with open(os.path.join(dir_dados, "autoteste.csv"), "w", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow(CAB_ST)
                    w.writerows(st)

        print("\nVerificação nas 6 posições. Use uma superfície plana e nivelada; não toque na placa "
              "durante a coleta (≈3 s por posição).")
        for pos in ORDEM:
            while True:
                r = input(f"\n[{pos}] {INSTRUCOES[pos]}\n    Enter = coletar | p = pular: ").strip().lower()
                if r == "p":
                    break
                cal, _, _ = executa(ser, f"CAL {pos}", log)
                if not cal:
                    print("    Nenhuma leitura recebida; tente de novo.")
                    continue
                resumo_posicao(cal, pos)
                r = input("    Enter = aceitar | r = repetir: ").strip().lower()
                if r != "r":
                    substitui_bloco(arq_cal, CAB_CAL, pos, cal)
                    break
            log.flush()


# ---------------------------------------------------------------------------
# Análise
# ---------------------------------------------------------------------------
def transversais(c):
    """Maior |A_j − offset_j| (g) nos eixos j que deveriam ler 0, para cada eixo j."""
    t = np.zeros(3)
    for p in POSICOES:
        i = "XYZ".index(p[1])
        for j in range(3):
            if j != i:
                t[j] = max(t[j], abs(c["medias"][p][j] - c["offset"][j]))
    return t


def escreve_tab_verificacao(c, transv, caminho):
    L = [r"\begin{tabular}{ccccccc}", r"\toprule",
         r"Eixo & $\bar{A}_{+1g}$ [g] & $\bar{A}_{-1g}$ [g] & Offset [mg] & "
         r"Ganho & $\sigma$ [mg] & Transv. [mg] \\",
         r" & & & (espec.) & (espec.) & & \\", r"\midrule"]
    for i, e in enumerate("XYZ"):
        L.append(f"{e} & {num(c['a_mais'][i], 4, True)} & {num(c['a_menos'][i], 4, True)} & "
                 f"{num(1000 * c['offset'][i], 1, True)} ($\\pm${int(1000 * TOL_OFFSET_G[i])}) & "
                 f"{num(c['ganho'][i], 4)} ($1\\pm0{{,}}03$) & "
                 f"{num(1000 * c['sigma'][i], 1)} & {num(1000 * transv[i], 1)} \\\\")
    L += [r"\bottomrule", r"\end{tabular}"]
    with open(caminho, "w", encoding="utf-8") as f:
        f.write("% Gerado por scripts/verificar_eixos.py — não editar à mão\n" + "\n".join(L) + "\n")


def le_autoteste(caminho):
    if not os.path.exists(caminho):
        return None
    with open(caminho, newline="", encoding="utf-8") as f:
        linhas = [l for l in csv.reader(f) if l and not l[0].startswith("#")]
    return [dict(zip(CAB_ST, l)) for l in linhas[1:]]


def escreve_tab_autoteste(st, caminho):
    L = [r"\begin{tabular}{cccccc}", r"\toprule",
         r"Eixo & Código & FT [LSB] & STR [LSB] & $(\mathrm{STR}-\mathrm{FT})/\mathrm{FT}$ [\%] & Resultado \\",
         r"\midrule"]
    for d in st:
        L.append(f"{d['eixo']} & {d['codigo']} & {num(float(d['ft_lsb']), 0)} & "
                 f"{num(float(d['str_lsb']), 0)} & {num(float(d['variacao_pct']), 1, True)} & "
                 f"{'aprovado' if d['resultado'] == 'OK' else 'reprovado'} \\\\")
    L += [r"\bottomrule", r"\end{tabular}"]
    with open(caminho, "w", encoding="utf-8") as f:
        f.write("% Gerado por scripts/verificar_eixos.py — não editar à mão\n" + "\n".join(L) + "\n")


def analisa(dir_dados, dir_saida):
    arq = os.path.join(dir_dados, "calibracao.csv")
    if not os.path.exists(arq):
        sys.exit(f"ERRO: {arq} não existe. Rode de novo e escolha a porta do ESP32 para coletar.")
    cal, simulado = le_csv(arq)
    c = calibra(cal)
    transv = transversais(c)

    dir_fig = os.path.join(dir_saida, "figuras")
    dir_tab = os.path.join(dir_saida, "tabelas")
    os.makedirs(dir_fig, exist_ok=True)
    os.makedirs(dir_tab, exist_ok=True)
    for ext in ("pdf", "png"):
        fig_verificacao(c, os.path.join(dir_fig, f"verificacao_eixos.{ext}"), simulado)
    escreve_tab_verificacao(c, transv, os.path.join(dir_tab, "tab_verificacao.tex"))

    print("\n=== Verificação dos eixos (comparação com o datasheet) ===")
    print(f"{'eixo':4s} {'A(+1g)':>9s} {'A(-1g)':>9s} {'offset':>10s} {'ganho':>8s} {'ruído':>8s} {'transv.':>8s}")
    for i, e in enumerate("XYZ"):
        ok_off = abs(c["offset"][i]) <= TOL_OFFSET_G[i]
        ok_g = abs(c["ganho"][i] - 1) <= TOL_GANHO
        ok_r = c["sigma"][i] <= 3 * RUIDO_ESPERADO_G
        print(f"{e:4s} {c['a_mais'][i]:+9.4f} {c['a_menos'][i]:+9.4f} "
              f"{1000 * c['offset'][i]:+7.1f} mg {c['ganho'][i]:8.4f} {1000 * c['sigma'][i]:5.1f} mg "
              f"{1000 * transv[i]:5.1f} mg   "
              f"offset {'OK' if ok_off else 'FORA'} (±{1000 * TOL_OFFSET_G[i]:.0f} mg), "
              f"ganho {'OK' if ok_g else 'FORA'} (±3 %), "
              f"ruído {'OK' if ok_r else 'acima do típico'} (típico ≈{1000 * RUIDO_ESPERADO_G:.1f} mg)")
    incl = math.degrees(math.asin(min(1.0, float(np.max(transv)))))
    print(f"Maior leitura transversal: {1000 * np.max(transv):.1f} mg ≈ {incl:.1f}° de desnível/desalinhamento "
          "(inclui a sensibilidade cruzada de ±2 % do datasheet).")

    st = le_autoteste(os.path.join(dir_dados, "autoteste.csv"))
    if st:
        escreve_tab_autoteste(st, os.path.join(dir_tab, "tab_autoteste.tex"))
        print("\n=== Autoteste do fabricante ===")
        for d in st:
            print(f"{d['eixo']}: código {d['codigo']:>2s}  FT={float(d['ft_lsb']):7.1f} LSB  "
                  f"STR={float(d['str_lsb']):7.1f} LSB  variação {float(d['variacao_pct']):+6.2f} %  {d['resultado']}")
    print(f"\nGráfico: {os.path.abspath(os.path.join(dir_fig, 'verificacao_eixos.pdf'))} (e .png)")
    if simulado:
        print("ATENÇÃO: dados SIMULADOS — não usar no relatório.")


def escolhe_porta():
    """Lista as portas seriais e pergunta qual usar (para quem roda pelo botão 'Run' da IDE)."""
    try:
        from serial.tools import list_ports
    except ImportError:
        sys.exit("ERRO: instale o pyserial:  pip install pyserial")
    portas = sorted(list_ports.comports(), key=lambda p: p.device)
    if not portas:
        sys.exit("ERRO: nenhuma porta serial encontrada. Conecte o ESP32 pelo USB (cabo de dados, "
                 "não só de carga) e confira o driver CP210x/CH340.")
    print("Portas seriais encontradas:")
    for k, p in enumerate(portas, 1):
        print(f"  {k}) {p.device}  —  {p.description}")
    padrao = portas[0].device if len(portas) == 1 else ""
    while True:
        r = input(f"Número ou nome da porta do ESP32{f' [Enter = {padrao}]' if padrao else ''}: ").strip()
        if not r and padrao:
            return padrao
        if r.isdigit() and 1 <= int(r) <= len(portas):
            return portas[int(r) - 1].device
        if r:
            return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--porta", help="porta serial do ESP32 (ex.: COM5); sem ela, o script pergunta, "
                                    "a menos que já existam dados coletados")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dados", default=DIR_DADOS)
    ap.add_argument("--saida", default=DIR_SAIDA)
    ap.add_argument("--sem-autoteste", action="store_true")
    ap.add_argument("--so-grafico", action="store_true",
                    help="não coleta; só refaz gráfico e tabelas a partir dos CSV existentes")
    args = ap.parse_args()
    tem_dados = os.path.exists(os.path.join(args.dados, "calibracao.csv"))
    if not args.porta and not args.so_grafico:
        if tem_dados:
            r = input(f"Já existem dados em {os.path.abspath(args.dados)}.\n"
                      "  Enter = só refazer o gráfico | c = coletar de novo: ").strip().lower()
            if r == "c":
                args.porta = escolhe_porta()
        else:
            args.porta = escolhe_porta()
    if args.porta:
        coleta(args.porta, args.baud, args.dados, not args.sem_autoteste)
    analisa(args.dados, args.saida)


if __name__ == "__main__":
    main()
