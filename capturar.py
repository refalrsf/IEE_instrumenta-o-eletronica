#!/usr/bin/env python3
"""
capturar.py — terminal interativo para o firmware mpu6050_inclinacao (ESP32).

Envia os comandos digitados ao ESP32 e grava automaticamente as linhas de dados:
  CAL,...  ->  dados/calibracao.csv   (6 posições ±1 g)
  ANG,...  ->  dados/angulos.csv      (ângulos de referência do transferidor)
Tudo o que chega pela serial também vai para dados/log_serial.txt.

Se uma posição (ex.: +X) ou um ângulo (ex.: 30) for coletado de novo, as linhas
antigas dessa posição/ângulo são substituídas pelas novas.

Uso:
  python capturar.py                         (pergunta a porta)
  python capturar.py --porta COM5            (Windows)
  python capturar.py --porta /dev/ttyUSB0    (Linux)
Comandos: SELFTEST | CAL +X | ANG 30 | INFO | SETCAL ... | sair
"""
import argparse
import csv
import os
import sys
import time

# Evita UnicodeEncodeError no console do Windows (cp1252) com símbolos como ≈ e ±
for _f in (sys.stdout, sys.stderr):
    try:
        _f.reconfigure(errors="replace")
    except Exception:
        pass

CAB_CAL = ["posicao", "amostra", "ax_g", "ay_g", "az_g"]
CAB_ANG = ["ref_graus", "amostra", "ax_g", "ay_g", "az_g", "theta_fw_graus"]


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
    ap.add_argument("--porta", help="porta serial do ESP32 (ex.: COM5); sem ela, o script pergunta")
    ap.add_argument("--baud", type=int, default=115200)
    aqui = os.path.dirname(os.path.abspath(__file__))
    padrao = (os.path.join(aqui, "..", "dados") if os.path.isdir(os.path.join(aqui, "..", "relatorio"))
              else os.path.join(aqui, "dados"))   # sozinho: dados/ ao lado do script
    ap.add_argument("--dados", default=padrao)
    args = ap.parse_args()

    if not args.porta:
        args.porta = escolhe_porta()
    import serial  # pyserial

    os.makedirs(args.dados, exist_ok=True)
    arq_cal = os.path.join(args.dados, "calibracao.csv")
    arq_ang = os.path.join(args.dados, "angulos.csv")

    with abre_porta(serial, args.porta, args.baud) as ser, \
            open(os.path.join(args.dados, "log_serial.txt"), "a", encoding="utf-8", errors="replace") as log:
        sincroniza(ser, log)  # o ESP32 reinicia ao abrir a porta
        verifica_firmware(ser, log)
        executa(ser, "INFO", log)
        print("Digite um comando (CAL +X, ANG 30, INFO, SETCAL ..., STREAM 1) ou 'sair'.")
        while True:
            try:
                cmd = input("comando> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not cmd:
                continue
            if cmd.lower() in ("sair", "exit", "quit"):
                break
            if cmd.upper().startswith("STREAM 1"):
                print("# STREAM 1 deixa o terminal ocupado; use o Monitor Serial para a demonstração.")
                continue
            cal, ang, _ = executa(ser, cmd, log)
            if cal:
                n_ant = substitui_bloco(arq_cal, CAB_CAL, cal[0][0], cal)
                print(f"# {len(cal)} leituras de {cal[0][0]} gravadas em {arq_cal} "
                      f"({n_ant} linhas de outras posições mantidas)")
            if ang:
                n_ant = substitui_bloco(arq_ang, CAB_ANG, ang[0][0], ang)
                print(f"# {len(ang)} leituras em {ang[0][0]} graus gravadas em {arq_ang} "
                      f"({n_ant} linhas de outros ângulos mantidas)")
            log.flush()
    print("Fim.")


if __name__ == "__main__":
    sys.exit(main())
