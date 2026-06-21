// ============================================================
//  SENSOR DE QUEDA - ESP32 + MPU6050 + Buzzer
//  --------------------------------------------------------
//  - Conexao WiFi via portal de configuracao (WiFiManager)
//  - Alerta de queda enviado pelo Telegram
//  - Servidor web com dashboard completo de desempenho
//  - Operacao 100% baseada em vTasks (FreeRTOS), 3 tarefas:
//      * Nucleo 0 -> deteccao de queda
//      * Nucleo 1 -> rede (servidor web + Telegram)
//      * Monitor  -> coleta de metricas de hardware
//  - Sincronizacao: mutex (barramento I2C e dados) + fila
//  - Calibracao automatica do MPU6050 no boot
//  - Algoritmo de queda livre + impacto (Bourke 2007 / Kangas 2008)
// ============================================================

#define ESP_DRD_USE_SPIFFS true

// ---- Bibliotecas padrao (ja vem com o ESP32) ----
#include <WiFi.h>
#include <WiFiClientSecure.h>   // HTTPS (API do Telegram)
#include <WebServer.h>          // Servidor web do dashboard
#include <HTTPClient.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include "time.h"

// ---- Bibliotecas que precisam ser instaladas ----
#include <WiFiManager.h>             // Portal cativo para configurar o WiFi
#include <ESP_DoubleResetDetector.h> // Detecta reset duplo (entra em modo config)
#include <ArduinoJson.h>             // Leitura/escrita do arquivo de config
#include <MPU6050.h>                 // Acelerometro/giroscopio

// -------------------------------------
// ------------  Definicoes  -----------
// -------------------------------------
#define BUZZER_PIN 18                 // GPIO do buzzer (API LEDC v3.x usa pino, nao canal)
#define BEEP_DURATION 5000            // Tempo (ms) que o buzzer toca apos uma queda
#define FALL_COOLDOWN 8000            // Tempo (ms) ignorando novas quedas apos um alerta

#define JSON_CONFIG_FILE "/sample_config.json"

#define DRD_TIMEOUT 10
#define DRD_ADDRESS 0

const int PIN_LED = 2;

// -------------------------------------
// ------  Objetos / Variaveis  --------
// -------------------------------------
DoubleResetDetector *drd;
MPU6050 mpu;
WebServer server(80);

bool shouldSaveConfig = false;

// Dados configuraveis pelo usuario
char userName[50]       = "Usuario";
char telegramToken[50]  = "TOKEN_AQUI";
char telegramChatID[24] = "CHATID_AQUI";

// Deteccao de queda
const int sampleInterval  = 10;       // Intervalo (ms) entre leituras do acelerometro

// Offsets de calibracao do MPU6050 (calculados no boot)
int16_t accelOffset[3] = {0, 0, 0};
int16_t gyroOffset[3]  = {0, 0, 0};

// Hora (NTP)
const char* ntpServer        = "in.pool.ntp.org";
const long  gmtOffset_sec    = 106200;
const int   daylightOffset_sec = 0;

// -------------------------------------
// -----  PARALELISMO / SINCRONISMO  ---
// -------------------------------------
typedef struct {
  // --- Nucleo 0: tarefa de deteccao ---
  uint32_t core0_loop_us;
  uint32_t core0_iterations;
  int      core0_id;
  float    core0_cpu;
  uint32_t core0_stack_free;

  // --- Nucleo 1: tarefa de rede ---
  uint32_t core1_loop_us;
  uint32_t core1_iterations;
  int      core1_id;
  float    core1_cpu;
  uint32_t core1_stack_free;

  // --- Tarefa de monitoramento ---
  uint32_t mon_stack_free;

  // --- Deteccao de queda / desempenho do alerta ---
  float    jerkMagnitude;        // agora representa magnitude em g
  uint32_t fallsDetected;
  uint32_t alertsSent;
  uint32_t lastDetection_us;
  uint32_t lastAlertSend_ms;
  uint32_t lastAlertLatency_ms;

  // --- Acumuladores internos ---
  uint32_t busyAccum0;
  uint32_t busyAccum1;

  // --- Metricas de hardware ---
  float    temperature;
  int8_t   wifiRssi;
} SystemStats;

SystemStats stats = {0};

SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t statsMutex;
QueueHandle_t fallQueue;

typedef struct {
  float jerk;
  unsigned long timestamp;
} FallEvent;

TaskHandle_t hTaskFall;
TaskHandle_t hTaskNet;
TaskHandle_t hTaskMon;

// -------------------------------------
// -------  CALIBRACAO DO MPU  ---------
// -------------------------------------
//  Mantenha o sensor PARADO e NIVELADO durante o boot.
void calibrateMPU(int samples = 500) {
  Serial.println("\n----- Calibracao do MPU6050 -----");
  Serial.println("Mantenha o sensor PARADO e NIVELADO.");
  Serial.println("Iniciando em 2 segundos...");
  delay(2000);

  long sumA[3] = {0, 0, 0};
  long sumG[3] = {0, 0, 0};

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumA[0] += ax; sumA[1] += ay; sumA[2] += az;
    sumG[0] += gx; sumG[1] += gy; sumG[2] += gz;
    delay(3);
  }

  accelOffset[0] = sumA[0] / samples;
  accelOffset[1] = sumA[1] / samples;
  // Z deve ler ~16384 (1g em escala +-2g) quando parado e nivelado
  accelOffset[2] = (sumA[2] / samples) - 16384;

  gyroOffset[0] = sumG[0] / samples;
  gyroOffset[1] = sumG[1] / samples;
  gyroOffset[2] = sumG[2] / samples;

  Serial.printf("Offsets accel: X=%d  Y=%d  Z=%d\n",
                accelOffset[0], accelOffset[1], accelOffset[2]);
  Serial.printf("Offsets gyro : X=%d  Y=%d  Z=%d\n",
                gyroOffset[0], gyroOffset[1], gyroOffset[2]);
  Serial.println("Calibracao concluida.\n");
}

// -------------------------------------
// ------------  Funcoes  --------------
// -------------------------------------

String urlEncode(const char* str) {
  const char* hex = "0123456789ABCDEF";
  String encodedStr = "";
  while (*str != 0) {
    if (('a' <= *str && *str <= 'z') ||
        ('A' <= *str && *str <= 'Z') ||
        ('0' <= *str && *str <= '9')) {
      encodedStr += *str;
    } else {
      encodedStr += '%';
      encodedStr += hex[*str >> 4];
      encodedStr += hex[*str & 0xf];
    }
    str++;
  }
  return encodedStr;
}

String getDateTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "hora indisponivel";
  char buf[40];
  strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &timeinfo);
  return String(buf);
}

void sendTelegramMessage(String message) {
  unsigned long t0 = millis();

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(telegramToken) +
               "/sendMessage?chat_id=" + String(telegramChatID) +
               "&text=" + urlEncode(message.c_str());

  http.begin(client, url);
  int httpCode = http.GET();
  unsigned long elapsed = millis() - t0;
  http.end();

  if (httpCode > 0)
    Serial.println("[TELEGRAM] Enviado. Codigo HTTP: " + String(httpCode) +
                   " | Tempo: " + String(elapsed) + " ms");
  else
    Serial.println("[TELEGRAM] Falha. Codigo: " + String(httpCode));

  if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
    stats.lastAlertSend_ms = elapsed;
    if (httpCode > 0) stats.alertsSent++;
    xSemaphoreGive(statsMutex);
  }
}

// -------------------------------------
// -------  CONFIGURACAO (SPIFFS)  -----
// -------------------------------------
void saveConfigFile() {
  Serial.println(F("Salvando configuracao"));
  StaticJsonDocument<512> json;
  json["userName"]       = userName;
  json["telegramToken"]  = telegramToken;
  json["telegramChatID"] = telegramChatID;

  File configFile = SPIFFS.open(JSON_CONFIG_FILE, "w");
  if (!configFile) { Serial.println("Falha ao abrir config para escrita"); return; }
  if (serializeJson(json, configFile) == 0) Serial.println(F("Falha ao escrever"));
  configFile.close();
}

bool loadConfigFile() {
  Serial.println("Montando o sistema de arquivos...");
  if (SPIFFS.begin(false) || SPIFFS.begin(true)) {
    Serial.println("Sistema de arquivos montado");
    if (SPIFFS.exists(JSON_CONFIG_FILE)) {
      File configFile = SPIFFS.open(JSON_CONFIG_FILE, "r");
      if (configFile) {
        StaticJsonDocument<512> json;
        DeserializationError error = deserializeJson(json, configFile);
        if (!error) {
          Serial.println("Configuracao lida com sucesso");
          strcpy(userName,       json["userName"]       | "Usuario");
          strcpy(telegramToken,  json["telegramToken"]  | "TOKEN_AQUI");
          strcpy(telegramChatID, json["telegramChatID"] | "CHATID_AQUI");
          configFile.close();
          return true;
        }
        Serial.println("Falha ao ler o JSON de config");
        configFile.close();
      }
    }
  } else {
    Serial.println("Falha ao montar o sistema de arquivos");
  }
  return false;
}

void saveConfigCallback() { shouldSaveConfig = true; }

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entrou no modo de configuracao");
  Serial.print("SSID: ");  Serial.println(myWiFiManager->getConfigPortalSSID());
  Serial.print("IP: ");    Serial.println(WiFi.softAPIP());
}

// -------------------------------------
// ----------  SERVIDOR WEB  -----------
// -------------------------------------
const char dashboardPage[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="pt-br">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Protótipo - Sensor de Queda</title>
<style>
  :root{
    --bg:#0d1117;--panel:#161b22;--line:#30363d;--txt:#e6edf3;
    --muted:#8b949e;--c0:#58a6ff;--c1:#3fb950;--warn:#f0883e;--mon:#bc8cff;
  }
  *{box-sizing:border-box;margin:0;padding:0;}
  body{background:var(--bg);color:var(--txt);
    font-family:'Consolas','Courier New',monospace;padding:22px;line-height:1.5;}
  header{border-bottom:2px solid var(--line);padding-bottom:14px;margin-bottom:20px;}
  header h1{font-size:1.3rem;letter-spacing:1px;}
  header p{color:var(--muted);font-size:.82rem;margin-top:4px;}
  .grid{display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));}
  .card{background:var(--panel);border:1px solid var(--line);
    border-radius:8px;padding:15px;}
  .card h2{font-size:.78rem;color:var(--muted);text-transform:uppercase;
    letter-spacing:1px;margin-bottom:11px;}
  .c0t{border-top:3px solid var(--c0);}
  .c1t{border-top:3px solid var(--c1);}
  .mt{border-top:3px solid var(--mon);}
  .wt{border-top:3px solid var(--warn);}
  .row{display:flex;justify-content:space-between;padding:3px 0;
    border-bottom:1px dashed var(--line);font-size:.86rem;gap:10px;}
  .row:last-child{border-bottom:none;}
  .row span:first-child{color:var(--muted);}
  .row span:last-child{font-weight:bold;text-align:right;}
  .c0{color:var(--c0);}.c1{color:var(--c1);}.w{color:var(--warn);}.mn{color:var(--mon);}
  .bar{height:6px;background:var(--line);border-radius:3px;margin-top:5px;overflow:hidden;}
  .bar>i{display:block;height:100%;border-radius:3px;}
  footer{margin-top:20px;color:var(--muted);font-size:.74rem;text-align:center;}
  .dot{display:inline-block;width:8px;height:8px;border-radius:50%;
    background:var(--c1);margin-right:6px;animation:pulse 1s infinite;}
  @keyframes pulse{0%,100%{opacity:1;}50%{opacity:.3;}}
</style>
</head>
<body>
<header>
  <h1>SENSOR DE QUEDA &mdash; PAINEL DE DESEMPENHO</h1>
  <p><span class="dot"></span>Monitoramento em tempo real &middot; ESP32 dual-core &middot; FreeRTOS</p>
</header>

<div class="grid">
  <div class="card c0t">
    <h2>Nucleo 0 &mdash; Tarefa de Deteccao</h2>
    <div class="row"><span>Roda no core</span><span class="c0" id="core0_id">-</span></div>
    <div class="row"><span>Tempo de iteracao</span><span class="c0"><span id="core0_us">-</span> us</span></div>
    <div class="row"><span>Iteracoes</span><span id="core0_iter">-</span></div>
    <div class="row"><span>Uso de CPU</span><span class="c0"><span id="core0_cpu">-</span> %</span></div>
    <div class="bar"><i id="core0_bar" style="width:0;background:var(--c0)"></i></div>
    <div class="row"><span>Stack livre</span><span><span id="core0_stk">-</span> B</span></div>
  </div>

  <div class="card c1t">
    <h2>Nucleo 1 &mdash; Tarefa de Rede</h2>
    <div class="row"><span>Roda no core</span><span class="c1" id="core1_id">-</span></div>
    <div class="row"><span>Tempo de iteracao</span><span class="c1"><span id="core1_us">-</span> us</span></div>
    <div class="row"><span>Iteracoes</span><span id="core1_iter">-</span></div>
    <div class="row"><span>Uso de CPU</span><span class="c1"><span id="core1_cpu">-</span> %</span></div>
    <div class="bar"><i id="core1_bar" style="width:0;background:var(--c1)"></i></div>
    <div class="row"><span>Stack livre</span><span><span id="core1_stk">-</span> B</span></div>
  </div>

  <div class="card wt">
    <h2>Desempenho do Alerta</h2>
    <div class="row"><span>Tempo de deteccao</span><span class="w"><span id="det">-</span> us</span></div>
    <div class="row"><span>Envio ao Telegram</span><span class="w"><span id="send">-</span> ms</span></div>
    <div class="row"><span>Latencia total</span><span class="w"><span id="lat">-</span> ms</span></div>
    <div class="row"><span>Magnitude atual (g)</span><span id="jerk">-</span></div>
  </div>

  <div class="card">
    <h2>Eventos</h2>
    <div class="row"><span>Quedas detectadas</span><span class="w" id="falls">-</span></div>
    <div class="row"><span>Alertas enviados</span><span class="c1" id="alerts">-</span></div>
  </div>

  <div class="card">
    <h2>Memoria RAM (Heap)</h2>
    <div class="row"><span>Total</span><span><span id="heapT">-</span> KB</span></div>
    <div class="row"><span>Livre</span><span><span id="heapF">-</span> KB</span></div>
    <div class="row"><span>Minimo ja livre</span><span><span id="heapM">-</span> KB</span></div>
    <div class="row"><span>Maior bloco livre</span><span><span id="heapB">-</span> KB</span></div>
    <div class="row"><span>Uso</span><span><span id="heapP">-</span> %</span></div>
    <div class="bar"><i id="heap_bar" style="width:0;background:var(--warn)"></i></div>
  </div>

  <div class="card">
    <h2>PSRAM</h2>
    <div class="row"><span>Total</span><span id="psT">-</span></div>
    <div class="row"><span>Livre</span><span id="psF">-</span></div>
  </div>

  <div class="card">
    <h2>Flash / Programa</h2>
    <div class="row"><span>Tamanho da Flash</span><span><span id="flT">-</span> MB</span></div>
    <div class="row"><span>Velocidade Flash</span><span><span id="flS">-</span> MHz</span></div>
    <div class="row"><span>Tamanho do sketch</span><span><span id="skT">-</span> KB</span></div>
    <div class="row"><span>Espaco livre p/ OTA</span><span><span id="skF">-</span> KB</span></div>
  </div>

  <div class="card">
    <h2>Armazenamento (SPIFFS)</h2>
    <div class="row"><span>Total</span><span><span id="spT">-</span> KB</span></div>
    <div class="row"><span>Usado</span><span><span id="spU">-</span> KB</span></div>
  </div>

  <div class="card mt">
    <h2>Chip / Sistema</h2>
    <div class="row"><span>Modelo</span><span class="mn" id="chip">-</span></div>
    <div class="row"><span>Revisao</span><span id="rev">-</span></div>
    <div class="row"><span>Nucleos</span><span id="cores">-</span></div>
    <div class="row"><span>Frequencia CPU</span><span><span id="cpu">-</span> MHz</span></div>
    <div class="row"><span>Temperatura</span><span><span id="temp">-</span> &deg;C</span></div>
    <div class="row"><span>Versao do SDK</span><span id="sdk">-</span></div>
    <div class="row"><span>Tarefas ativas</span><span id="tasks">-</span></div>
    <div class="row"><span>Monitor: stack livre</span><span><span id="mon_stk">-</span> B</span></div>
    <div class="row"><span>Tempo ligado</span><span id="uptime">-</span></div>
  </div>

  <div class="card">
    <h2>Rede WiFi</h2>
    <div class="row"><span>Endereco IP</span><span id="ip">-</span></div>
    <div class="row"><span>Rede (SSID)</span><span id="ssid">-</span></div>
    <div class="row"><span>Sinal (RSSI)</span><span><span id="rssi">-</span> dBm</span></div>
  </div>
</div>

<footer>Atualizado automaticamente a cada 1s &middot; 3 tarefas FreeRTOS em paralelo</footer>

<script>
function up(s){let h=Math.floor(s/3600),m=Math.floor((s%3600)/60),x=s%60;
  return h+"h "+m+"m "+x+"s";}
function set(id,v){document.getElementById(id).textContent=v;}
async function refresh(){
  try{
    const d = await (await fetch('/stats')).json();
    set('core0_id',d.core0_id); set('core0_us',d.core0_us);
    set('core0_iter',d.core0_iter.toLocaleString()); set('core0_cpu',d.core0_cpu.toFixed(1));
    set('core0_stk',d.core0_stk.toLocaleString());
    document.getElementById('core0_bar').style.width=Math.min(d.core0_cpu,100)+'%';
    set('core1_id',d.core1_id); set('core1_us',d.core1_us);
    set('core1_iter',d.core1_iter.toLocaleString()); set('core1_cpu',d.core1_cpu.toFixed(1));
    set('core1_stk',d.core1_stk.toLocaleString());
    document.getElementById('core1_bar').style.width=Math.min(d.core1_cpu,100)+'%';
    set('det',d.det.toLocaleString()); set('send',d.send); set('lat',d.lat);
    set('jerk',d.jerk.toFixed(2));
    set('falls',d.falls); set('alerts',d.alerts);
    let hp=(100*(d.heapT-d.heapF)/d.heapT);
    set('heapT',(d.heapT/1024).toFixed(1)); set('heapF',(d.heapF/1024).toFixed(1));
    set('heapM',(d.heapM/1024).toFixed(1)); set('heapB',(d.heapB/1024).toFixed(1));
    set('heapP',hp.toFixed(1));
    document.getElementById('heap_bar').style.width=hp+'%';
    set('psT',d.psT>0?(d.psT/1048576).toFixed(2)+' MB':'Sem PSRAM');
    set('psF',d.psT>0?(d.psF/1048576).toFixed(2)+' MB':'-');
    set('flT',(d.flT/1048576).toFixed(1)); set('flS',d.flS);
    set('skT',(d.skT/1024).toFixed(1)); set('skF',(d.skF/1024).toFixed(1));
    set('spT',(d.spT/1024).toFixed(1)); set('spU',(d.spU/1024).toFixed(1));
    set('chip',d.chip); set('rev',d.rev); set('cores',d.cores);
    set('cpu',d.cpu); set('temp',d.temp.toFixed(1)); set('sdk',d.sdk);
    set('tasks',d.tasks); set('mon_stk',d.mon_stk.toLocaleString());
    set('uptime',up(d.uptime));
    set('ip',d.ip); set('ssid',d.ssid); set('rssi',d.rssi);
  }catch(e){ console.log('falha ao buscar /stats',e); }
}
setInterval(refresh,1000); refresh();
</script>
</body>
</html>
)HTML";

void handleRoot() { server.send_P(200, "text/html", dashboardPage); }

void handleStats() {
  SystemStats s;
  if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
    s = stats;
    xSemaphoreGive(statsMutex);
  }

  String j = "{";
  j += "\"core0_id\":"   + String(s.core0_id) + ",";
  j += "\"core0_us\":"   + String(s.core0_loop_us) + ",";
  j += "\"core0_iter\":" + String(s.core0_iterations) + ",";
  j += "\"core0_cpu\":"  + String(s.core0_cpu) + ",";
  j += "\"core0_stk\":"  + String(s.core0_stack_free) + ",";
  j += "\"core1_id\":"   + String(s.core1_id) + ",";
  j += "\"core1_us\":"   + String(s.core1_loop_us) + ",";
  j += "\"core1_iter\":" + String(s.core1_iterations) + ",";
  j += "\"core1_cpu\":"  + String(s.core1_cpu) + ",";
  j += "\"core1_stk\":"  + String(s.core1_stack_free) + ",";
  j += "\"det\":"        + String(s.lastDetection_us) + ",";
  j += "\"send\":"       + String(s.lastAlertSend_ms) + ",";
  j += "\"lat\":"        + String(s.lastAlertLatency_ms) + ",";
  j += "\"jerk\":"       + String(s.jerkMagnitude) + ",";
  j += "\"falls\":"      + String(s.fallsDetected) + ",";
  j += "\"alerts\":"     + String(s.alertsSent) + ",";
  j += "\"heapT\":"      + String(ESP.getHeapSize()) + ",";
  j += "\"heapF\":"      + String(ESP.getFreeHeap()) + ",";
  j += "\"heapM\":"      + String(ESP.getMinFreeHeap()) + ",";
  j += "\"heapB\":"      + String(ESP.getMaxAllocHeap()) + ",";
  j += "\"psT\":"        + String(ESP.getPsramSize()) + ",";
  j += "\"psF\":"        + String(ESP.getFreePsram()) + ",";
  j += "\"flT\":"        + String(ESP.getFlashChipSize()) + ",";
  j += "\"flS\":"        + String(ESP.getFlashChipSpeed() / 1000000) + ",";
  j += "\"skT\":"        + String(ESP.getSketchSize()) + ",";
  j += "\"skF\":"        + String(ESP.getFreeSketchSpace()) + ",";
  j += "\"spT\":"        + String((uint32_t)SPIFFS.totalBytes()) + ",";
  j += "\"spU\":"        + String((uint32_t)SPIFFS.usedBytes()) + ",";
  j += "\"chip\":\""     + String(ESP.getChipModel()) + "\",";
  j += "\"rev\":"        + String(ESP.getChipRevision()) + ",";
  j += "\"cores\":"      + String(ESP.getChipCores()) + ",";
  j += "\"cpu\":"        + String(getCpuFrequencyMhz()) + ",";
  j += "\"temp\":"       + String(s.temperature) + ",";
  j += "\"sdk\":\""      + String(ESP.getSdkVersion()) + "\",";
  j += "\"tasks\":"      + String(uxTaskGetNumberOfTasks()) + ",";
  j += "\"mon_stk\":"    + String(s.mon_stack_free) + ",";
  j += "\"uptime\":"     + String(millis() / 1000) + ",";
  j += "\"ip\":\""       + WiFi.localIP().toString() + "\",";
  j += "\"ssid\":\""     + WiFi.SSID() + "\",";
  j += "\"rssi\":"       + String(s.wifiRssi);
  j += "}";

  server.send(200, "application/json", j);
}

// ============================================================
//  TAREFA 1  (Nucleo 0)  -  Deteccao de queda
//  Algoritmo: queda livre (< 0.5g) -> impacto (> 2.0g) dentro
//  de uma janela de 1.5s. Padrao academico (Bourke 2007).
// ============================================================
void taskFallDetection(void *param) {
  bool buzzerOn = false;
  unsigned long buzzerStart = 0;
  unsigned long lastFallTime = 0;

  // ----- Estado do algoritmo -----
  bool inFreeFall = false;
  unsigned long freeFallStart = 0;
  const float FREE_FALL_G   = 0.5f;    // abaixo disso = queda livre
  const float IMPACT_G      = 2.0f;    // acima disso = impacto
  const uint32_t FF_WINDOW_MS = 1500;  // janela entre queda livre e impacto

  for (;;) {
    uint32_t tIter = micros();
    uint32_t tDetect = micros();

    int16_t ax, ay, az;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      mpu.getAcceleration(&ax, &ay, &az);
      xSemaphoreGive(i2cMutex);
    }

    // Aplica offsets de calibracao
    ax -= accelOffset[0];
    ay -= accelOffset[1];
    az -= accelOffset[2];

    // Magnitude da aceleracao em g (escala +-2g => 16384 LSB/g)
    float aMag = sqrt((float)ax*ax + (float)ay*ay + (float)az*az) / 16384.0;

    bool fall = false;
    unsigned long now = millis();

    // ----- Maquina de estados -----
    if (!inFreeFall && aMag < FREE_FALL_G) {
      inFreeFall = true;
      freeFallStart = now;
      Serial.printf("[CORE0] Queda livre detectada (mag=%.2fg)\n", aMag);
    }

    if (inFreeFall) {
      if (aMag > IMPACT_G && (now - lastFallTime > FALL_COOLDOWN)) {
        Serial.printf("[CORE0] IMPACTO! mag=%.2fg\n", aMag);
        fall = true;
        inFreeFall = false;
      } else if (now - freeFallStart > FF_WINDOW_MS) {
        // Timeout - nao houve impacto, descarta
        inFreeFall = false;
      }
    }

    uint32_t detectDur = micros() - tDetect;

    if (fall) {
      lastFallTime = now;

      ledcWriteTone(BUZZER_PIN, 3000);
      buzzerOn = true;
      buzzerStart = millis();

      FallEvent ev = { aMag, millis() };
      xQueueSend(fallQueue, &ev, 0);

      if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
        stats.fallsDetected++;
        stats.lastDetection_us = detectDur;
        xSemaphoreGive(statsMutex);
      }
    }

    if (buzzerOn && (millis() - buzzerStart > BEEP_DURATION)) {
      ledcWrite(BUZZER_PIN, 0);
      buzzerOn = false;
    }

    uint32_t iterUs = micros() - tIter;
    if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
      stats.core0_loop_us = iterUs;
      stats.core0_iterations++;
      stats.core0_id = xPortGetCoreID();
      stats.core0_stack_free = uxTaskGetStackHighWaterMark(NULL);
      stats.jerkMagnitude = aMag;
      stats.busyAccum0 += iterUs;
      xSemaphoreGive(statsMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(sampleInterval));
  }
}

// ============================================================
//  TAREFA 2  (Nucleo 1)  -  Rede (servidor web + Telegram)
// ============================================================
void taskNetwork(void *param) {
  for (;;) {
    uint32_t tIter = micros();

    drd->loop();
    server.handleClient();
    uint32_t workUs = micros() - tIter;

    FallEvent ev;
    if (xQueueReceive(fallQueue, &ev, 0) == pdTRUE) {
      Serial.println("[CORE1] Evento de queda recebido. Enviando Telegram...");
      String msg = "ALERTA! Queda detectada para o usuario: " + String(userName) +
                   " em " + getDateTime() + ". Impacto: " + String(ev.jerk, 2) + "g";
      sendTelegramMessage(msg);

      uint32_t latency = millis() - ev.timestamp;
      if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
        stats.lastAlertLatency_ms = latency;
        xSemaphoreGive(statsMutex);
      }
      Serial.println("[CORE1] Latencia total do alerta: " + String(latency) + " ms");
    }

    if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
      stats.core1_loop_us = workUs;
      stats.core1_iterations++;
      stats.core1_id = xPortGetCoreID();
      stats.core1_stack_free = uxTaskGetStackHighWaterMark(NULL);
      stats.busyAccum1 += workUs;
      xSemaphoreGive(statsMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ============================================================
//  TAREFA 3  -  Monitor de hardware
// ============================================================
void taskMonitor(void *param) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
      stats.core0_cpu = stats.busyAccum0 / 10000.0;
      stats.core1_cpu = stats.busyAccum1 / 10000.0;
      if (stats.core0_cpu > 100) stats.core0_cpu = 100;
      if (stats.core1_cpu > 100) stats.core1_cpu = 100;
      stats.busyAccum0 = 0;
      stats.busyAccum1 = 0;

      stats.mon_stack_free = uxTaskGetStackHighWaterMark(NULL);
      stats.temperature    = temperatureRead();
      stats.wifiRssi       = WiFi.RSSI();
      xSemaphoreGive(statsMutex);
    }
  }
}

// -------------------------------------
// --------------  SETUP  --------------
// -------------------------------------
void setup() {
  Serial.begin(115200);
  delay(10);

  Wire.begin(21, 17);   // SDA -> GPIO 21, SCL -> GPIO 17

  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 conectado" : "Falha no MPU6050");

  // Calibracao do MPU6050 (sensor deve estar parado e nivelado)
  calibrateMPU();

  // API LEDC v3.x: chamada unica com pino + freq + resolucao
  ledcAttach(BUZZER_PIN, 2000, 13);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  bool forceConfig = false;

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
  if (drd->detectDoubleReset()) {
    Serial.println(F("Modo de config forcado (reset duplo)"));
    forceConfig = true;
  }
  if (!loadConfigFile()) {
    Serial.println(F("Modo de config forcado (sem config salva)"));
    forceConfig = true;
  }

  WiFi.mode(WIFI_STA);

  const char *configInfoText =
    "<div style='margin-bottom:20px;'>Crie um bot no Telegram com o "
    "<b>@BotFather</b> para obter o <b>Token</b>, e use o <b>@userinfobot</b> "
    "para descobrir o seu <b>Chat ID</b>.</div>";

  WiFiManager wm;
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setAPCallback(configModeCallback);

  WiFiManagerParameter config_info_top(configInfoText);
  wm.addParameter(&config_info_top);

  WiFiManagerParameter box_name ("key_name",  "Nome do usuario",    userName,       50);
  WiFiManagerParameter box_token("key_token", "Telegram Bot Token", telegramToken,  49);
  WiFiManagerParameter box_chat ("key_chat",  "Telegram Chat ID",   telegramChatID, 23);
  wm.addParameter(&box_name);
  wm.addParameter(&box_token);
  wm.addParameter(&box_chat);

  if (forceConfig) {
    if (!wm.startConfigPortal("Fall_detector", "clock123")) {
      Serial.println("Falha ao conectar / timeout");
      delay(3000); ESP.restart();
    }
  } else {
    if (!wm.autoConnect("Fall_detector", "clock123")) {
      Serial.println("Falha ao conectar / timeout");
      delay(3000); ESP.restart();
    }
  }

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  digitalWrite(PIN_LED, HIGH);

  Serial.println("\nWiFi conectado");
  Serial.print("Dashboard disponivel em: http://");
  Serial.println(WiFi.localIP());

  strncpy(userName,       box_name.getValue(),  sizeof(userName));
  strncpy(telegramToken,  box_token.getValue(), sizeof(telegramToken));
  strncpy(telegramChatID, box_chat.getValue(),  sizeof(telegramChatID));
  Serial.println("Usuario: " + String(userName));

  if (shouldSaveConfig) saveConfigFile();

  server.on("/", handleRoot);
  server.on("/stats", handleStats);
  server.begin();
  Serial.println("Servidor web iniciado.");

  i2cMutex   = xSemaphoreCreateMutex();
  statsMutex = xSemaphoreCreateMutex();
  fallQueue  = xQueueCreate(5, sizeof(FallEvent));

  xTaskCreatePinnedToCore(taskFallDetection, "Deteccao", 4096, NULL, 3, &hTaskFall, 0);
  xTaskCreatePinnedToCore(taskNetwork,       "Rede",     8192, NULL, 2, &hTaskNet,  1);
  xTaskCreatePinnedToCore(taskMonitor,       "Monitor",  3072, NULL, 1, &hTaskMon,  tskNO_AFFINITY);

  Serial.println("3 vTasks criadas. Sistema em execucao paralela.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}