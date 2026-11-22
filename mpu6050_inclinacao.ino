
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>


#define PINO_SDA        21
#define PINO_SCL        22
#define I2C_FREQ_HZ     400000UL
#define MPU_ADDR        0x68     

#define REG_SELF_TEST_X   0x0D      // 0x0D..0x10: códigos de fábrica do autoteste
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

// ---------------------------- Configuração do sensor -----------------------
// Escala ±2 g (AFS_SEL = 0): em um ensaio estático de inclinação cada eixo vê
// no máximo 1 g, e esta escala dá a maior sensibilidade (16 384 LSB/g).
#define AFS_SEL           0
#define SENSIBILIDADE     16384.0f  // LSB/g para AFS_SEL = 0
// DLPF_CFG = 3: banda do acelerômetro de 44 Hz (atraso de 4,9 ms).
#define DLPF_CFG          3
// Com o DLPF ativo a taxa interna é 1 kHz; SMPLRT_DIV = 9 -> 100 Hz.
#define SMPLRT_DIV        9
#define PERIODO_US        10000UL   // 100 Hz

// ---------------------------- Ensaios --------------------------------------
#define N_CAL             200       // leituras por posição (mínimo pedido: 100)
#define N_ANG             100       // leituras por ângulo de referência
#define ESPERA_MS         1000      // tempo para o braço/sensor se acomodar

// ---------------------------- Cálculo do ângulo ----------------------------
// θ = atan2(A[EIXO_NUM], A[EIXO_DEN]). Índices: 0 = X, 1 = Y, 2 = Z.
// Montagem de referência: placa do sensor na face do braço, eixo X ao longo
// do braço, eixo Y perpendicular a ele no plano de giro e eixo Z paralelo ao
// pino de articulação. Com o braço na horizontal, Y aponta para cima (A_y = +1 g)
// e θ = atan2(A_x, A_y) = 0°. Ajuste estes índices se a montagem for outra.
#define EIXO_NUM          0
#define EIXO_DEN          1
#define JANELA_STREAM     10        // média móvel (amostras) no modo STREAM
#define PERIODO_STREAM_MS 100

// ---------------------------- Estado ---------------------------------------
Preferences nvs;
float ganho[3]  = {1.0f, 1.0f, 1.0f};
float offset[3] = {0.0f, 0.0f, 0.0f};
float sinalMontagem  = 1.0f;
float offsetMontagem = 0.0f;   // graus
bool  stream = true;

// ===========================================================================
//                         Acesso ao MPU-6050 (I2C)
// ===========================================================================
static bool escreveReg(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

static bool leRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom((uint8_t)MPU_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool iniciaMPU() {
  uint8_t who = 0;
  if (!leRegs(REG_WHO_AM_I, &who, 1) || who != 0x68) {
    Serial.printf("# ERRO: WHO_AM_I = 0x%02X (esperado 0x68)\n", who);
    return false;
  }
  bool ok = true;
  ok &= escreveReg(REG_PWR_MGMT_1, 0x01);        // sai do sleep, clock = PLL gyro X
  delay(100);
  ok &= escreveReg(REG_CONFIG, DLPF_CFG);
  ok &= escreveReg(REG_SMPLRT_DIV, SMPLRT_DIV);
  ok &= escreveReg(REG_ACCEL_CONFIG, AFS_SEL << 3);
  delay(100);
  return ok;
}

// Lê os três eixos em LSB (inteiros de 16 bits em complemento de 2).
static bool leAcelLSB(int16_t r[3]) {
  uint8_t b[6];
  if (!leRegs(REG_ACCEL_XOUT_H, b, 6)) return false;
  for (int i = 0; i < 3; i++) r[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  return true;
}

// Lê os três eixos em unidades de g (sem calibração).
static bool leAcel(float a[3]) {
  int16_t r[3];
  if (!leAcelLSB(r)) return false;
  for (int i = 0; i < 3; i++) a[i] = r[i] / SENSIBILIDADE;
  return true;
}

// ===========================================================================
//                         Calibração e ângulo
// ===========================================================================
// Eq. (1) do roteiro: A_saida = Ganho·A_ref + A_offset  ->  A_ref = (A_saida − A_offset)/Ganho
static void corrige(const float a[3], float ac[3]) {
  for (int i = 0; i < 3; i++) ac[i] = (a[i] - offset[i]) / ganho[i];
}

static float anguloGraus(const float ac[3]) {
  float th = atan2f(ac[EIXO_NUM], ac[EIXO_DEN]) * 180.0f / (float)M_PI;
  th = sinalMontagem * th + offsetMontagem;
  // Faixa de saída (−90°, 270°], para que 0–180° do transferidor seja contínuo.
  while (th <= -90.0f) th += 360.0f;
  while (th > 270.0f)  th -= 360.0f;
  return th;
}

static void carregaNVS() {
  nvs.begin("incl", true);
  const char *kg[3] = {"gx", "gy", "gz"};
  const char *ko[3] = {"ox", "oy", "oz"};
  for (int i = 0; i < 3; i++) {
    ganho[i]  = nvs.getFloat(kg[i], 1.0f);
    offset[i] = nvs.getFloat(ko[i], 0.0f);
  }
  sinalMontagem  = nvs.getFloat("sm", 1.0f);
  offsetMontagem = nvs.getFloat("om", 0.0f);
  nvs.end();
}

static void salvaNVS() {
  nvs.begin("incl", false);
  const char *kg[3] = {"gx", "gy", "gz"};
  const char *ko[3] = {"ox", "oy", "oz"};
  for (int i = 0; i < 3; i++) {
    nvs.putFloat(kg[i], ganho[i]);
    nvs.putFloat(ko[i], offset[i]);
  }
  nvs.putFloat("sm", sinalMontagem);
  nvs.putFloat("om", offsetMontagem);
  nvs.end();
}

static void info() {
  Serial.println("# --- configuracao ---");
  Serial.printf("# escala +-%d g, DLPF_CFG=%d, taxa=%lu Hz\n", 2 << AFS_SEL, DLPF_CFG,
                1000000UL / PERIODO_US);
  Serial.printf("# theta = %+.0f*atan2(A%c, A%c) %+.2f graus\n", sinalMontagem,
                "XYZ"[EIXO_NUM], "XYZ"[EIXO_DEN], offsetMontagem);
  for (int i = 0; i < 3; i++)
    Serial.printf("# eixo %c: ganho=%.6f offset=%+.6f g\n", "XYZ"[i], ganho[i], offset[i]);
  Serial.println("#FIM");
}

// ===========================================================================
//                         Ensaios
// ===========================================================================
// Aguarda o próximo instante de amostragem (100 Hz) e lê o sensor.
static bool amostra(uint32_t &t0, float a[3]) {
  while ((uint32_t)(micros() - t0) < PERIODO_US) { /* espera */ }
  t0 += PERIODO_US;
  return leAcel(a);
}

// ---------------------------------------------------------------------------
// Autoteste do acelerômetro (RM-MPU-6000A-00, seção 4.1; PS-MPU-6000A-00, seção 6.2):
//   1) escala ±8 g (4096 LSB/g); média das saídas com autoteste desligado
//   2) liga XA_ST, YA_ST, ZA_ST (ACCEL_CONFIG bits 7..5); média das saídas
//   3) STR = saída(ligado) − saída(desligado)                      [LSB]
//   4) FT  = 4096·0,34·(0,92/0,34)^((COD−1)/(2^5−2)), COD ≠ 0       [LSB]
//      COD = 5 bits: XA_TEST[4:2] (reg 0x0D bits 7:5) e XA_TEST[1:0] (reg 0x10 bits 5:4)
//   5) variação = (STR − FT)/FT deve ficar em ±14 %
// ---------------------------------------------------------------------------
static bool mediaLSB(int n, double m[3]) {
  int16_t r[3];
  m[0] = m[1] = m[2] = 0;
  uint32_t t0 = micros();
  for (int k = 0; k < n; k++) {
    while ((uint32_t)(micros() - t0) < PERIODO_US) { }
    t0 += PERIODO_US;
    if (!leAcelLSB(r)) return false;
    for (int i = 0; i < 3; i++) m[i] += (double)r[i] / n;
  }
  return true;
}

static void autoteste() {
  const int N = 200;
  uint8_t st[4];
  if (!leRegs(REG_SELF_TEST_X, st, 4)) { Serial.println("# ERRO de leitura I2C\n#FIM"); return; }
  uint8_t cod[3] = {
    (uint8_t)(((st[0] >> 3) & 0x1C) | ((st[3] >> 4) & 0x03)),
    (uint8_t)(((st[1] >> 3) & 0x1C) | ((st[3] >> 2) & 0x03)),
    (uint8_t)(((st[2] >> 3) & 0x1C) | (st[3] & 0x03))};

  Serial.println("# SELFTEST: mantenha o sensor parado");
  double desl[3], lig[3];
  escreveReg(REG_ACCEL_CONFIG, 2 << 3);                 // ±8 g, autoteste desligado
  delay(250);
  bool ok = mediaLSB(N, desl);
  escreveReg(REG_ACCEL_CONFIG, 0xE0 | (2 << 3));        // ±8 g, XA_ST|YA_ST|ZA_ST
  delay(250);
  ok &= mediaLSB(N, lig);
  escreveReg(REG_ACCEL_CONFIG, AFS_SEL << 3);           // volta à escala de trabalho
  delay(250);
  if (!ok) { Serial.println("# ERRO de leitura I2C\n#FIM"); return; }

  bool aprovado = true;
  for (int i = 0; i < 3; i++) {
    double ft = cod[i] ? 4096.0 * 0.34 * pow(0.92 / 0.34, (cod[i] - 1.0) / 30.0) : 0.0;
    double str = lig[i] - desl[i];
    double var = ft > 0 ? 100.0 * (str - ft) / ft : NAN;
    bool passou = ft > 0 && fabs(var) <= 14.0;
    aprovado &= passou;
    Serial.printf("ST,%c,%u,%.1f,%.1f,%.2f,%s\n", "XYZ"[i], cod[i], ft, str, var,
                  passou ? "OK" : "FALHA");
  }
  Serial.printf("# autoteste %s (limite: variacao de +-14%% em relacao ao valor de fabrica)\n",
                aprovado ? "APROVADO" : "REPROVADO");
  Serial.println("#FIM");
}

static void ensaioCal(const String &pos) {
  static const char *validas[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  bool ok = false;
  for (int i = 0; i < 6; i++) ok |= pos.equals(validas[i]);
  if (!ok) { Serial.println("# ERRO: posicao invalida (use +X -X +Y -Y +Z -Z)\n#FIM"); return; }

  Serial.printf("# CAL %s: mantenha o sensor parado\n", pos.c_str());
  delay(ESPERA_MS);
  double soma[3] = {0, 0, 0};
  float a[3];
  int n = 0;
  uint32_t t0 = micros();
  while (n < N_CAL) {
    if (!amostra(t0, a)) { Serial.println("# ERRO de leitura I2C"); continue; }
    Serial.printf("CAL,%s,%d,%.6f,%.6f,%.6f\n", pos.c_str(), n, a[0], a[1], a[2]);
    for (int i = 0; i < 3; i++) soma[i] += a[i];
    n++;
  }
  Serial.printf("# media %s: ax=%+.5f ay=%+.5f az=%+.5f g\n", pos.c_str(),
                soma[0] / n, soma[1] / n, soma[2] / n);
  Serial.println("#FIM");
}

static void ensaioAng(float ref) {
  Serial.printf("# ANG %.1f: mantenha o braco parado\n", ref);
  delay(ESPERA_MS);
  double s = 0, s2 = 0;
  float a[3], ac[3];
  int n = 0;
  uint32_t t0 = micros();
  while (n < N_ANG) {
    if (!amostra(t0, a)) { Serial.println("# ERRO de leitura I2C"); continue; }
    corrige(a, ac);
    float th = anguloGraus(ac);
    Serial.printf("ANG,%.2f,%d,%.6f,%.6f,%.6f,%.4f\n", ref, n, a[0], a[1], a[2], th);
    s += th; s2 += (double)th * th;
    n++;
  }
  double media = s / n;
  double desvio = sqrt(fmax(0.0, (s2 - n * media * media) / (n - 1)));
  Serial.printf("# ref=%.2f media=%.3f desvio=%.3f erro(ref-med)=%+.3f graus\n",
                ref, media, desvio, ref - media);
  Serial.println("#FIM");
}

// Modo de demonstração: média móvel do vetor aceleração e impressão do ângulo.
static void passoStream() {
  static float buf[JANELA_STREAM][3];
  static int idx = 0, cheio = 0;
  static uint32_t tAmostra = 0, tPrint = 0;
  if ((uint32_t)(micros() - tAmostra) < PERIODO_US) return;
  tAmostra = micros();
  float a[3];
  if (!leAcel(a)) return;
  corrige(a, buf[idx]);
  idx = (idx + 1) % JANELA_STREAM;
  if (cheio < JANELA_STREAM) cheio++;
  if (millis() - tPrint < PERIODO_STREAM_MS) return;
  tPrint = millis();
  float m[3] = {0, 0, 0};
  for (int k = 0; k < cheio; k++)
    for (int i = 0; i < 3; i++) m[i] += buf[k][i] / cheio;
  Serial.printf("theta = %7.2f graus   (ax=%+.3f ay=%+.3f az=%+.3f g)\n",
                anguloGraus(m), m[0], m[1], m[2]);
}

// ===========================================================================
//                         Interpretador de comandos
// ===========================================================================
static void trataComando(String linha) {
  linha.trim();
  linha.toUpperCase();
  if (linha.length() == 0) return;
  int sp = linha.indexOf(' ');
  String cmd = sp < 0 ? linha : linha.substring(0, sp);
  String arg = sp < 0 ? "" : linha.substring(sp + 1);
  arg.trim();

  if (cmd == "SELFTEST") {
    autoteste();
  } else if (cmd == "CAL") {
    ensaioCal(arg);
  } else if (cmd == "ANG") {
    if (arg.length() == 0) { Serial.println("# ERRO: use ANG <graus>\n#FIM"); return; }
    ensaioAng(arg.toFloat());
  } else if (cmd == "STREAM") {
    stream = (arg != "0");
    Serial.printf("# stream %s\n#FIM\n", stream ? "ligado" : "desligado");
  } else if (cmd == "SETCAL") {
    float v[6];
    if (sscanf(arg.c_str(), "%f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
      Serial.println("# ERRO: use SETCAL gx ox gy oy gz oz\n#FIM");
      return;
    }
    for (int i = 0; i < 3; i++) { ganho[i] = v[2 * i]; offset[i] = v[2 * i + 1]; }
    salvaNVS();
    info();
  } else if (cmd == "SETMONT") {
    float s, o;
    if (sscanf(arg.c_str(), "%f %f", &s, &o) != 2 || (s != 1.0f && s != -1.0f)) {
      Serial.println("# ERRO: use SETMONT <1|-1> <offset_graus>\n#FIM");
      return;
    }
    sinalMontagem = s; offsetMontagem = o;
    salvaNVS();
    info();
  } else if (cmd == "ZERA") {
    for (int i = 0; i < 3; i++) { ganho[i] = 1.0f; offset[i] = 0.0f; }
    salvaNVS();
    info();
  } else if (cmd == "INFO") {
    info();
  } else {
    Serial.println("# comandos: SELFTEST | CAL <pos> | ANG <graus> | STREAM <0|1> | SETCAL ... | SETMONT s o | ZERA | INFO\n#FIM");
  }
}

// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PINO_SDA, PINO_SCL, I2C_FREQ_HZ);
  while (!iniciaMPU()) {
    Serial.println("# MPU-6050 nao encontrado; verifique as ligacoes");
    delay(1000);
  }
  carregaNVS();
  Serial.println("# MPU-6050 pronto");
  info();
}

void loop() {
  static String linha;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (linha.length()) trataComando(linha);
      linha = "";
    } else {
      linha += c;
    }
  }
  if (stream) passoStream();
}
