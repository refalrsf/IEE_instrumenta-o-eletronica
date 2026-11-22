#!/usr/bin/env python3
"""
processar.py — processa os ensaios do MPU-6050 e gera o material do relatório.

Entradas (geradas por capturar.py):
  dados/calibracao.csv  colunas: posicao, amostra, ax_g, ay_g, az_g
  dados/angulos.csv     colunas: ref_graus, amostra, ax_g, ay_g, az_g, theta_fw_graus

Saídas (por padrão em relatorio/):
  figuras/verificacao_eixos.pdf  leitura de cada eixo nas 6 posições ±1 g
  figuras/erro_angulo.pdf        erro (referência − medido) × ângulo de referência
  tabelas/tab_calibracao.tex     médias, ganho e offset por eixo (Tabela do relatório)
  tabelas/tab_metricas.tex       métricas de erro do ângulo (sem e com calibração)
  tabelas/tab_angulos.tex        tabela completa por ângulo (opcional, apêndice)
  tabelas/valores.tex            macros \\newcommand com os números citados no texto
e imprime o comando SETCAL para gravar a calibração no ESP32.

Modelo (Eq. 1 e 2 do roteiro):  A_saida = Ganho·A_ref + A_offset
  Ganho_i  = (Ā+1g,i − Ā−1g,i) / 2
  Offset_i = (Ā+1g,i + Ā−1g,i) / 2
  A_corrigido = (A_saida − Offset_i) / Ganho_i

Ângulo: θ = sinal·atan2(A_num, A_den) + offset_montagem  (mesma convenção do firmware)
"""
import argparse
import os
import sys

import numpy as np

# Evita UnicodeEncodeError no console do Windows (cp1252) com símbolos como ≈ e ±
for _f in (sys.stdout, sys.stderr):
    try:
        _f.reconfigure(errors="replace")
    except Exception:
        pass

EIXOS = "XYZ"
POSICOES = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]


# ---------------------------------------------------------------------------
# Leitura
# ---------------------------------------------------------------------------
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


# ---------------------------------------------------------------------------
# Calibração
# ---------------------------------------------------------------------------
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


def corrige(A, ganho, offset, usar_ganho=True):
    g = ganho if usar_ganho else np.ones(3)
    return (A - offset) / g


def angulo(A, num, den, sinal, off_mont):
    th = sinal * np.degrees(np.arctan2(A[:, num], A[:, den])) + off_mont
    th = np.where(th <= -90.0, th + 360.0, th)
    th = np.where(th > 270.0, th - 360.0, th)
    return th


# ---------------------------------------------------------------------------
# Formatação
# ---------------------------------------------------------------------------
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


# ---------------------------------------------------------------------------
# Figuras
# ---------------------------------------------------------------------------
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


def fig_erro(res, caminho, simulado):
    plt, virgula = configura_matplotlib()
    fig, ax = plt.subplots(figsize=(6.0, 2.9))
    ax.axhline(0, color="0.3", lw=0.7)
    ax.errorbar(res["ref"], res["erro_bruto"], yerr=res["desv_bruto"], fmt="s", ms=3.5, mfc="white",
                color="#c0504d", capsize=2, lw=0.8, label="sem calibração")
    ax.errorbar(res["ref"], res["erro_cal"], yerr=res["desv_cal"], fmt="o", ms=3.5,
                color="#1f4e79", capsize=2, lw=0.8, label="com calibração (ganho e offset)")
    ax.set_xlabel(r"ângulo de referência $\theta_{\mathrm{ref}}$ (transferidor) [°]")
    ax.set_ylabel(r"erro $\theta_{\mathrm{ref}}-\theta_{\mathrm{med}}$ [°]")
    ax.xaxis.set_major_formatter(virgula)
    ax.yaxis.set_major_formatter(virgula)
    lo, hi = min(res["ref"]), max(res["ref"])
    ax.set_xlim(lo - 5, hi + 5)
    if hi - lo >= 90:
        ax.set_xticks(np.arange(np.floor(lo / 30) * 30, hi + 1, 30))
    ax.legend(loc="best", framealpha=0.9)
    if simulado:
        marca_simulado(fig)
    fig.savefig(caminho)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Ângulos
# ---------------------------------------------------------------------------
def processa_angulos(ang, c, num_ax, den_ax, sinal, off_mont):
    ref_amostra = np.array(ang["ref_graus"], float)
    A = np.column_stack([np.array(ang[k], float) for k in ("ax_g", "ay_g", "az_g")])
    th = {
        "bruto": angulo(A, num_ax, den_ax, sinal, off_mont),
        "off": angulo(corrige(A, c["ganho"], c["offset"], usar_ganho=False), num_ax, den_ax, sinal, off_mont),
        "cal": angulo(corrige(A, c["ganho"], c["offset"]), num_ax, den_ax, sinal, off_mont),
    }
    refs = np.unique(ref_amostra)
    res = {"ref": refs, "n": np.array([np.sum(ref_amostra == r) for r in refs])}
    for k, v in th.items():
        med = np.array([v[ref_amostra == r].mean() for r in refs])
        dsv = np.array([v[ref_amostra == r].std(ddof=1) for r in refs])
        res[f"med_{k}"], res[f"desv_{k}"], res[f"erro_{k}"] = med, dsv, refs - med
    return res


def metricas(e):
    return dict(media=float(np.mean(e)), rms=float(np.sqrt(np.mean(e ** 2))),
                maxabs=float(np.max(np.abs(e))))


# ---------------------------------------------------------------------------
# Tabelas e macros LaTeX
# ---------------------------------------------------------------------------
def escreve_tab_calibracao(c, caminho):
    L = [r"\begin{tabular}{cccccc}", r"\toprule",
         r"Eixo & $\bar{A}_{+1g,i}$ [g] & $\bar{A}_{-1g,i}$ [g] & $\sigma_{\max}$ [mg] & "
         r"$\mathit{Ganho}_i$ & $A_{\mathit{offset},i}$ [mg] \\", r"\midrule"]
    for i, e in enumerate(EIXOS):
        L.append(f"{e} & {num(c['a_mais'][i], 4, True)} & {num(c['a_menos'][i], 4, True)} & "
                 f"{num(1000 * c['sigma'][i], 1)} & {num(c['ganho'][i], 4)} & "
                 f"{num(1000 * c['offset'][i], 1, True)} \\\\")
    L += [r"\bottomrule", r"\end{tabular}"]
    with open(caminho, "w", encoding="utf-8") as f:
        f.write("% Gerado por scripts/processar.py — não editar à mão\n" + "\n".join(L) + "\n")


def escreve_tab_metricas(res, caminho):
    linhas = [("Sem calibração", res["erro_bruto"]),
              ("Somente offset", res["erro_off"]),
              ("Ganho e offset", res["erro_cal"])]
    L = [r"\begin{tabular}{lccc}", r"\toprule",
         r"Leitura & Erro médio [°] & Erro RMS [°] & $|e|_{\max}$ [°] \\", r"\midrule"]
    for nome, e in linhas:
        m = metricas(e)
        L.append(f"{nome} & {num(m['media'], 2, True)} & {num(m['rms'], 2)} & {num(m['maxabs'], 2)} \\\\")
    L += [r"\bottomrule", r"\end{tabular}"]
    with open(caminho, "w", encoding="utf-8") as f:
        f.write("% Gerado por scripts/processar.py — não editar à mão\n" + "\n".join(L) + "\n")


def escreve_tab_angulos(res, caminho):
    L = [r"\begin{tabular}{rrrrrr}", r"\toprule",
         r"$\theta_{\mathrm{ref}}$ [°] & $\bar\theta$ bruto [°] & erro bruto [°] & "
         r"$\bar\theta$ calibrado [°] & $\sigma_\theta$ [°] & erro calibrado [°] \\", r"\midrule"]
    for k, r in enumerate(res["ref"]):
        L.append(f"{num(r, 1)} & {num(res['med_bruto'][k], 2)} & {num(res['erro_bruto'][k], 2, True)} & "
                 f"{num(res['med_cal'][k], 2)} & {num(res['desv_cal'][k], 2)} & "
                 f"{num(res['erro_cal'][k], 2, True)} \\\\")
    L += [r"\bottomrule", r"\end{tabular}"]
    with open(caminho, "w", encoding="utf-8") as f:
        f.write("% Gerado por scripts/processar.py — não editar à mão\n" + "\n".join(L) + "\n")


def escreve_valores(c, res, simulado, caminho):
    v = {
        "NAmostrasCal": f"{min(c['n'].values())}",
        "OffsetMaxMg": num(1000 * float(np.max(np.abs(c["offset"]))), 1),
        "SigmaAcelMg": num(1000 * float(np.mean(c["sigma"])), 1),
        "GanhoMin": num(float(np.min(c["ganho"])), 4),
        "GanhoMax": num(float(np.max(c["ganho"])), 4),
    }
    if res is not None:
        mb, mo, mc = metricas(res["erro_bruto"]), metricas(res["erro_off"]), metricas(res["erro_cal"])
        passos = np.diff(res["ref"])
        v.update({
            "NAngulos": f"{len(res['ref'])}",
            "NAmostrasAng": f"{int(np.min(res['n']))}",
            "AngMin": num(res["ref"].min(), 0),
            "AngMax": num(res["ref"].max(), 0),
            "PassoAng": num(np.median(passos), 0) if len(passos) else "--",
            "ErroMedioBruto": num(mb["media"], 2), "ErroRMSBruto": num(mb["rms"], 2), "ErroMaxBruto": num(mb["maxabs"], 2),
            "ErroMedioOff": num(mo["media"], 2), "ErroRMSOff": num(mo["rms"], 2), "ErroMaxOff": num(mo["maxabs"], 2),
            "ErroMedioCal": num(mc["media"], 2), "ErroRMSCal": num(mc["rms"], 2), "ErroMaxCal": num(mc["maxabs"], 2),
            "DesvioAngMedio": num(float(np.mean(res["desv_cal"])), 2),
        })
    with open(caminho, "w", encoding="utf-8") as f:
        f.write("% Gerado por scripts/processar.py — não editar à mão\n")
        for k, val in v.items():
            f.write(f"\\renewcommand{{\\{k}}}{{{val}}}\n")
        f.write("\\dadossimuladostrue\n" if simulado else "\\dadossimuladosfalse\n")


# ---------------------------------------------------------------------------
def main():
    # Dentro do repositório usa dados/ e relatorio/ do projeto; sozinho (ex.: em Downloads),
    # usa dados/ e saida/ ao lado do script — as mesmas pastas do verificar_eixos.py.
    aqui = os.path.dirname(os.path.abspath(__file__))
    if os.path.isdir(os.path.join(aqui, "..", "relatorio")):
        dir_dados, dir_saida = os.path.join(aqui, "..", "dados"), os.path.join(aqui, "..", "relatorio")
    else:
        dir_dados, dir_saida = os.path.join(aqui, "dados"), os.path.join(aqui, "saida")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dados", default=dir_dados)
    ap.add_argument("--saida", default=dir_saida)
    ap.add_argument("--eixos", default="XY",
                    help="eixos do atan2: numerador e denominador (padrão XY = atan2(Ax, Ay), igual ao firmware)")
    ap.add_argument("--sinal", type=float, default=1.0, help="sinal de montagem (igual ao SETMONT)")
    ap.add_argument("--offset-montagem", type=float, default=0.0, help="offset de montagem em graus (igual ao SETMONT)")
    args = ap.parse_args()

    num_ax, den_ax = EIXOS.index(args.eixos[0].upper()), EIXOS.index(args.eixos[1].upper())
    dir_fig = os.path.join(args.saida, "figuras")
    dir_tab = os.path.join(args.saida, "tabelas")
    os.makedirs(dir_fig, exist_ok=True)
    os.makedirs(dir_tab, exist_ok=True)

    arq_cal = os.path.join(args.dados, "calibracao.csv")
    if not os.path.exists(arq_cal):
        sys.exit(f"ERRO: não encontrei {os.path.abspath(arq_cal)}.\n"
                 "  Coloque este script na mesma pasta do verificar_eixos.py (a que tem a pasta dados/)\n"
                 "  ou indique a pasta:  python processar.py --dados CAMINHO\\dados")
    cal, sim1 = le_csv(arq_cal)
    c = calibra(cal)
    simulado = sim1

    print("=== Calibração estática (6 posições) ===")
    for i, e in enumerate(EIXOS):
        print(f"  {e}: A+1g={c['a_mais'][i]:+.5f} g  A-1g={c['a_menos'][i]:+.5f} g  "
              f"Ganho={c['ganho'][i]:.5f}  Offset={1000 * c['offset'][i]:+.2f} mg")
    setcal = "SETCAL " + " ".join(f"{c['ganho'][i]:.6f} {c['offset'][i]:.6f}" for i in range(3))
    print("\nGrave no ESP32 (copie e cole no terminal):\n  " + setcal + "\n")

    fig_verificacao(c, os.path.join(dir_fig, "verificacao_eixos.pdf"), simulado)
    escreve_tab_calibracao(c, os.path.join(dir_tab, "tab_calibracao.tex"))

    arq_ang = os.path.join(args.dados, "angulos.csv")
    if os.path.exists(arq_ang):
        ang, sim2 = le_csv(arq_ang)
        simulado |= sim2
        res = processa_angulos(ang, c, num_ax, den_ax, args.sinal, args.offset_montagem)
        print("=== Ângulo de inclinação ===")
        print("  ref    bruto   erro_b   calib   erro_c   desvio")
        for k, r in enumerate(res["ref"]):
            print(f"  {r:5.1f}  {res['med_bruto'][k]:7.2f}  {res['erro_bruto'][k]:+6.2f}  "
                  f"{res['med_cal'][k]:7.2f}  {res['erro_cal'][k]:+6.2f}  {res['desv_cal'][k]:5.2f}")
        for nome, chave in (("sem calibração", "erro_bruto"), ("só offset", "erro_off"), ("ganho+offset", "erro_cal")):
            m = metricas(res[chave])
            print(f"  {nome:15s}: erro médio {m['media']:+.2f}°, RMS {m['rms']:.2f}°, máx |e| {m['maxabs']:.2f}°")
        fig_erro(res, os.path.join(dir_fig, "erro_angulo.pdf"), simulado)
        escreve_tab_metricas(res, os.path.join(dir_tab, "tab_metricas.tex"))
        escreve_tab_angulos(res, os.path.join(dir_tab, "tab_angulos.tex"))
    else:
        res = None
        print(f"(sem {arq_ang}: só a parte de calibração foi gerada)")
    escreve_valores(c, res, simulado, os.path.join(dir_tab, "valores.tex"))

    print(f"\nArquivos gerados em {os.path.abspath(dir_fig)} e {os.path.abspath(dir_tab)}")
    if simulado:
        print("ATENÇÃO: dados SIMULADOS — figuras marcadas; não usar no relatório final.")


if __name__ == "__main__":
    main()
