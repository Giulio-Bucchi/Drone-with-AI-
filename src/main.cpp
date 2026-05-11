#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <Adafruit_PWMServoDriver.h>

// ================================================================
const char* WIFI_SSID     = "TIM-65662396";
const char* WIFI_PASSWORD = "hUzDCcfRxHky7STzzQ6XbY3K";
// ================================================================

// --- I2C PCA9685 via Gravity cable ---
#define PCA_SDA  43
#define PCA_SCL  44
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// Throttle corrente per ogni ESC (0-100%) — usato dalla status API
static volatile uint8_t g_throttle[4] = {0, 0, 0, 0};

// ================================================================
// PCA9685 PWM — controllo ESC sui canali 0..3 del driver
// ================================================================
#define PCA_FREQ     50
#define ESC_MIN_US 1000
#define ESC_MAX_US 2000

static bool g_pca_ready = false;
static volatile bool g_stop_requested = false;

static uint16_t usToPca(uint16_t us) {
    return (uint16_t)(((uint32_t)us * PCA_FREQ * 4096UL) / 1000000UL);
}

static uint16_t pctToUs(uint8_t pct) {
    if (pct > 100) pct = 100;
    return ESC_MIN_US + (uint16_t)((uint32_t)pct * (ESC_MAX_US - ESC_MIN_US) / 100);
}

static void writeEscPulseUs(uint8_t ch, uint16_t pulse_us) {
    if (ch >= 4 || !g_pca_ready) return;
    if (pulse_us < ESC_MIN_US) pulse_us = ESC_MIN_US;
    if (pulse_us > ESC_MAX_US) pulse_us = ESC_MAX_US;
    pca.setPWM(ch, 0, usToPca(pulse_us));
}

static void stopAllEscs() {
    g_stop_requested = true;
    for (uint8_t i = 0; i < 4; i++) {
        g_throttle[i] = 0;
        writeEscPulseUs(i, ESC_MIN_US);
    }
    Serial.println("[ESC] STOP immediato");
}

static bool waitOrStop(uint32_t ms) {
    uint32_t deadline = millis() + ms;
    while ((int32_t)(deadline - millis()) > 0) {
        if (g_stop_requested) return false;
        delay(20);
    }
    return !g_stop_requested;
}

// Imposta velocità ESC (ch: 0-3, pct: 0-100)
static void setESC(uint8_t ch, uint8_t pct) {
    if (ch >= 4) return;
    if (pct > 100) pct = 100;
    g_stop_requested = false;
    g_throttle[ch] = pct;
    if (!g_pca_ready) {
        Serial.printf("[ESC] PCA non pronta, ch=%d pct=%d%% ignorato\n", ch, pct);
        return;
    }
    uint16_t pulse_us = pctToUs(pct);
    uint16_t ticks = usToPca(pulse_us);
    writeEscPulseUs(ch, pulse_us);
    Serial.printf("[ESC] ch=%d  pct=%d%%  pulse=%dus  ticks=%d\n", ch, pct, pulse_us, ticks);
}

// Arming PWM classico: minimo per 2 secondi.
static void armESCs() {
    if (!g_pca_ready) {
        Serial.println("[ESC] PCA non pronta — arm annullato");
        return;
    }
    g_stop_requested = false;
    Serial.printf("[ESC] Arming PWM: %dus per 2s\n", ESC_MIN_US);
    for (uint8_t i = 0; i < 4; i++) {
        g_throttle[i] = 0;
        writeEscPulseUs(i, ESC_MIN_US);
    }
    if (!waitOrStop(2000)) {
        Serial.println("[ESC] Arm interrotto");
        return;
    }
    Serial.println("[ESC] ESC pronti");
}

// Sequenza di calibrazione PWM standard: MAX -> MIN.
static void calibrateESCs() {
    if (!g_pca_ready) {
        Serial.println("[ESC] PCA non pronta — calibrazione annullata");
        return;
    }
    g_stop_requested = false;
    Serial.printf("[ESC] Calibrazione PWM: MAX %dus poi MIN %dus\n", ESC_MAX_US, ESC_MIN_US);
    for (uint8_t i = 0; i < 4; i++) writeEscPulseUs(i, ESC_MAX_US);
    if (!waitOrStop(3000)) {
        Serial.println("[ESC] Calibrazione interrotta");
        return;
    }
    for (uint8_t i = 0; i < 4; i++) writeEscPulseUs(i, ESC_MIN_US);
    if (!waitOrStop(3000)) {
        Serial.println("[ESC] Calibrazione interrotta");
        return;
    }
    Serial.println("[ESC] Calibrazione PWM finita");
}

static void sweepESC(uint8_t ch) {
    if (ch >= 4) return;
    if (!g_pca_ready) {
        Serial.println("[ESC] PCA non pronta — sweep annullato");
        return;
    }
    static const uint16_t pulses[] = {1050, 1075, 1100, 1125, 1150, 1175, 1200, 1250, 1300};
    Serial.printf("[SWEEP] ch=%d start\n", ch);
    armESCs();
    if (g_stop_requested) return;
    for (uint16_t pulse_us : pulses) {
        writeEscPulseUs(ch, pulse_us);
        Serial.printf("[SWEEP] ch=%d pulse=%dus\n", ch, pulse_us);
        if (!waitOrStop(1500)) break;
    }
    writeEscPulseUs(ch, ESC_MIN_US);
    g_throttle[ch] = 0;
    Serial.printf("[SWEEP] ch=%d end\n", ch);
}

static void sweepHighESC(uint8_t ch) {
    if (ch >= 4) return;
    if (!g_pca_ready) {
        Serial.println("[ESC] PCA non pronta — high sweep annullato");
        return;
    }
    static const uint16_t pulses[] = {1350, 1400, 1450, 1500, 1550, 1600, 1650, 1700};
    Serial.printf("[SWEEP-HIGH] ch=%d start\n", ch);
    armESCs();
    if (g_stop_requested) return;
    for (uint16_t pulse_us : pulses) {
        writeEscPulseUs(ch, pulse_us);
        Serial.printf("[SWEEP-HIGH] ch=%d pulse=%dus\n", ch, pulse_us);
        if (!waitOrStop(1200)) break;
    }
    writeEscPulseUs(ch, ESC_MIN_US);
    g_throttle[ch] = 0;
    Serial.printf("[SWEEP-HIGH] ch=%d end\n", ch);
}

// ----------------------------------------------------------------

// --- Pin camera DFRobot DFR1154 (FireBeetle 2 ESP32-S3 AI Cam) ---
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    5
#define SIOD_GPIO_NUM    8
#define SIOC_GPIO_NUM    9
#define Y9_GPIO_NUM      4
#define Y8_GPIO_NUM      6
#define Y7_GPIO_NUM      7
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     17
#define Y4_GPIO_NUM     21
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM     16
#define VSYNC_GPIO_NUM   1
#define HREF_GPIO_NUM    2
#define PCLK_GPIO_NUM   15

// --- MJPEG ---
#define PART_BOUNDARY "frame"
static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t ctrl_httpd   = NULL;
static httpd_handle_t stream_httpd = NULL;

// ================================================================
// RILEVAMENTO PERSONE — Motion detection + blob analysis
//
// Algoritmo:
//  1) Cattura frame in GRAYSCALE 160×120 (bassa risoluzione, veloce)
//  2) Riduzione del rumore: sottrai media della scena precedente
//  3) Conta pixel "attivi" (sopra soglia)
//  4) Se blobs di pixel attivi ≥ soglia dimensione → persona rilevata
//  5) Hysteresis di conferma: 3 frame positivi consecutivi
// ================================================================
#define DET_W       160
#define DET_H       120
#define DET_PIXELS  (DET_W * DET_H)
#define MOT_THRESH  30    // soglia differenza pixel (0-255)
#define MOT_MIN_PX  400   // min pixel attivi per rilevamento
#define MOT_CONFIRM 3     // frame consecutivi per conferma

static uint8_t* g_bg_frame   = nullptr;  // background model (PSRAM)
static uint8_t* g_cur_frame  = nullptr;  // frame corrente (PSRAM)

static volatile bool     g_person_detected  = false;
static volatile uint32_t g_person_count     = 0;    // frame positivi consec.
static volatile uint32_t g_detect_last_ms   = 0;
static volatile int      g_active_pixels    = 0;

static SemaphoreHandle_t g_det_mutex;

// Estrae grayscale 160×120 da un frame JPEG tramite esp_camera
// Usa formato YUV422: byte 0 di ogni coppia = luminanza Y
static bool captureGrayscale(uint8_t* out) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return false;
    // Camera in PIXFORMAT_GRAYSCALE: fb->buf sono pixel reali (1 byte = 1 pixel)
    // Downsampling 2x: 320x240 → 160x120, prendo pixel in posizione pari
    for (int dy = 0; dy < DET_H; dy++) {
        for (int dx = 0; dx < DET_W; dx++) {
            out[dy * DET_W + dx] = fb->buf[(dy * 2) * 320 + (dx * 2)];
        }
    }
    esp_camera_fb_return(fb);
    return true;
}

// Task di rilevamento su Core 0
static void detectionTask(void*) {
    static uint32_t confirm_cnt = 0;

    // Prima acquisizione: inizializza background
    if (!captureGrayscale(g_bg_frame)) {
        vTaskDelete(nullptr);
        return;
    }

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(400));  // ~2.5 analisi/s

        if (!captureGrayscale(g_cur_frame)) continue;

        // Differenza assoluta rispetto al background
        int active = 0;
        for (int i = 0; i < DET_PIXELS; i++) {
            int diff = (int)g_cur_frame[i] - (int)g_bg_frame[i];
            if (diff < 0) diff = -diff;
            if (diff > MOT_THRESH) active++;
        }

        // --- Calcola luminosità media del frame corrente ---
        uint32_t mean_sum = 0;
        for (int i = 0; i < DET_PIXELS; i++) mean_sum += g_cur_frame[i];
        uint8_t mean_cur = (uint8_t)(mean_sum / DET_PIXELS);

        // --- Aggiorna background SOLO quando la scena è ferma ---
        if (active < MOT_MIN_PX / 2) {
            for (int i = 0; i < DET_PIXELS; i++) {
                g_bg_frame[i] = (uint8_t)(((int)g_bg_frame[i] * 15 + (int)g_cur_frame[i]) >> 4);
            }
        }

        // --- Filtri anti-falso-positivo ---
        // 1) Camera oscurata: frame molto scuro rispetto al background
        uint32_t bg_sum = 0;
        for (int i = 0; i < DET_PIXELS; i++) bg_sum += g_bg_frame[i];
        uint8_t mean_bg = (uint8_t)(bg_sum / DET_PIXELS);
        bool covered = (mean_cur < 15) || (mean_bg > 30 && mean_cur < mean_bg / 3);

        // 2) Cambio di luce globale: quasi tutti i pixel attivi → non è una persona
        bool global_change = (active > DET_PIXELS * 85 / 100);

        bool valid_motion = (active >= MOT_MIN_PX) && !covered && !global_change;

        xSemaphoreTake(g_det_mutex, portMAX_DELAY);
        g_active_pixels = active;
        if (valid_motion) {
            confirm_cnt++;
            if (confirm_cnt >= MOT_CONFIRM) {
                if (!g_person_detected) {
                    g_person_detected = true;
                    g_person_count++;
                    // NO Serial.printf qui — cam_task gira su Core 0 e
                    // printf da task concorrente causa stack overflow
                }
            }
        } else {
            confirm_cnt = 0;
            g_person_detected = false;
        }
        g_detect_last_ms = millis();
        xSemaphoreGive(g_det_mutex);
    }
}

// ================================================================
// HTTP HANDLERS
// ================================================================

// --- Stream MJPEG ---
// Camera è in GRAYSCALE → convertiamo ogni frame in JPEG prima di inviarlo
static esp_err_t stream_handler(httpd_req_t* req) {
    camera_fb_t* fb      = NULL;
    uint8_t*     jpg     = NULL;
    size_t       jpg_len = 0;
    char         part_buf[64];

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) { Serial.println("[CAM] Frame fallito"); break; }

        jpg = NULL; jpg_len = 0;
        bool ok = frame2jpg(fb, 60, &jpg, &jpg_len);  // qualità 60 = più veloce
        esp_camera_fb_return(fb); fb = NULL;
        if (!ok || !jpg) { vTaskDelay(1); continue; }

        esp_err_t res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, jpg_len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, (const char*)jpg, jpg_len);
        free(jpg); jpg = NULL;
        if (res != ESP_OK) break;
    }
    if (fb) esp_camera_fb_return(fb);
    if (jpg) free(jpg);
    return ESP_OK;
}

// --- /status — JSON detection + ESC ---
static esp_err_t status_handler(httpd_req_t* req) {
    xSemaphoreTake(g_det_mutex, portMAX_DELAY);
    bool det      = g_person_detected;
    int  active   = g_active_pixels;
    uint32_t cnt  = g_person_count;
    uint32_t age  = millis() - g_detect_last_ms;
    xSemaphoreGive(g_det_mutex);

    char json[256];
    snprintf(json, sizeof(json),
        "{\"person\":%s,\"active_px\":%d,\"count\":%lu,\"age_ms\":%lu,"
        "\"esc\":[%d,%d,%d,%d]}",
        det ? "true" : "false", active,
        (unsigned long)cnt, (unsigned long)age,
        g_throttle[0], g_throttle[1], g_throttle[2], g_throttle[3]);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

// --- /esc?ch=0&pct=50 — Imposta throttle ESC ---
static esp_err_t esc_handler(httpd_req_t* req) {
    char buf[64] = {0};
    int  len = httpd_req_get_url_query_len(req);
    if (len > 0 && len < (int)sizeof(buf)) {
        httpd_req_get_url_query_str(req, buf, sizeof(buf));
        char ch_s[4] = {0}, pct_s[4] = {0};
        if (httpd_query_key_value(buf, "ch",  ch_s,  sizeof(ch_s))  == ESP_OK &&
            httpd_query_key_value(buf, "pct", pct_s, sizeof(pct_s)) == ESP_OK) {
            int ch  = atoi(ch_s);
            int pct = atoi(pct_s);
            if (ch >= 0 && ch <= 3 && pct >= 0 && pct <= 100) {
                setESC((uint8_t)ch, (uint8_t)pct);
            }
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

// --- /arm — Arma ESC ---
static esp_err_t arm_handler(httpd_req_t* req) {
    armESCs();
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t stop_handler(httpd_req_t* req) {
    stopAllEscs();
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t beep_handler(httpd_req_t* req) {
    calibrateESCs();
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t sweep_handler(httpd_req_t* req) {
    uint8_t ch = 0;
    char buf[32] = {0};
    int len = httpd_req_get_url_query_len(req);
    if (len > 0 && len < (int)sizeof(buf)) {
        httpd_req_get_url_query_str(req, buf, sizeof(buf));
        char ch_s[4] = {0};
        if (httpd_query_key_value(buf, "ch", ch_s, sizeof(ch_s)) == ESP_OK) {
            int val = atoi(ch_s);
            if (val >= 0 && val <= 3) ch = (uint8_t)val;
        }
    }
    sweepESC(ch);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t sweep_high_handler(httpd_req_t* req) {
    uint8_t ch = 0;
    char buf[32] = {0};
    int len = httpd_req_get_url_query_len(req);
    if (len > 0 && len < (int)sizeof(buf)) {
        httpd_req_get_url_query_str(req, buf, sizeof(buf));
        char ch_s[4] = {0};
        if (httpd_query_key_value(buf, "ch", ch_s, sizeof(ch_s)) == ESP_OK) {
            int val = atoi(ch_s);
            if (val >= 0 && val <= 3) ch = (uint8_t)val;
        }
    }
    sweepHighESC(ch);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

// --- /test — calibrazione PWM + spin 20% ---
static esp_err_t test_handler(httpd_req_t* req) {
    Serial.println("[TEST] 1/2 calibrazione PWM");
    calibrateESCs();
    if (g_stop_requested) {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "OK", 2);
    }
    Serial.println("[TEST] 2/2 spin 20% per 2s");
    for (uint8_t i = 0; i < 4; i++) setESC(i, 20);
    waitOrStop(2000);
    for (uint8_t i = 0; i < 4; i++) setESC(i, 0);
    Serial.println("[TEST] Fine — canali a 0%");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

// --- /ping — testa ogni canale PCA uno alla volta: 30% per 1.5s ---
static esp_err_t ping_handler(httpd_req_t* req) {
    g_stop_requested = false;
    for (uint8_t ch = 0; ch < 4; ch++) {
        Serial.printf("[PING] PCA ch%d al 30%%\n", ch);
        setESC(ch, 30);
        if (!waitOrStop(1500)) break;
        setESC(ch, 0);
        if (!waitOrStop(300)) break;
    }
    Serial.println("[PING] Fine");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

// ----------------------------------------------------------------
// Pagina HTML
// ----------------------------------------------------------------
static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32-S3 Drone AI</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{background:#0d0d0d;display:flex;flex-direction:column;
         align-items:center;padding:20px;
         font-family:'Segoe UI',sans-serif;color:#e0e0e0;gap:16px}
    h1{font-size:1.3rem;letter-spacing:1px}
    #sc{position:relative;border:2px solid #333;border-radius:10px;
        overflow:hidden;box-shadow:0 0 30px rgba(0,120,255,.3)}
    img{display:block;max-width:95vw}
    #badge{position:absolute;top:10px;left:50%;transform:translateX(-50%);
           padding:5px 18px;border-radius:18px;font-weight:bold;
           background:rgba(0,0,0,.6);color:#888;transition:all .3s;white-space:nowrap}
    #badge.on{background:rgba(210,40,40,.88);color:#fff;box-shadow:0 0 14px rgba(255,50,50,.6)}
    #badge.off{background:rgba(30,160,70,.82);color:#fff}
    .panel{background:#1a1a1a;border:1px solid #333;border-radius:10px;
           padding:16px;width:min(500px,95vw)}
    .panel h2{font-size:.95rem;margin-bottom:12px;color:#aaa}
    .esc-row{display:flex;align-items:center;gap:10px;margin-bottom:8px}
    .esc-row label{width:100px;font-size:.85rem}
    .esc-row input[type=range]{flex:1}
    .esc-row span{width:36px;text-align:right;font-size:.85rem}
    button{padding:8px 20px;border:none;border-radius:6px;cursor:pointer;
           background:#1e3a5f;color:#fff;font-size:.9rem}
    button:hover{background:#2a5490}
    .ch-btn{background:#1a3a1a;border:2px solid #2a5a2a}
    .ch-btn.active{background:#1a6a1a;border-color:#4aaa4a;box-shadow:0 0 10px rgba(50,200,50,.5)}
    #arm-btn{background:#5a2000}
    #arm-btn:hover{background:#8a3500}
    #px-bar{height:8px;border-radius:4px;background:#333;margin-top:4px;overflow:hidden}
    #px-fill{height:100%;background:#3a7fd5;width:0%;transition:width .4s}
    #stats{font-size:.78rem;color:#666;margin-top:4px}
  </style>
</head>
<body>
  <h1>&#128248; ESP32-S3 Drone AI</h1>
  <div id="sc">
    <img id="stream" alt="live stream">
    <div id="badge">avvio...</div>
  </div>

  <div class="panel">
    <h2>Rilevamento movimento / persone</h2>
    <div id="px-bar"><div id="px-fill"></div></div>
    <div id="stats">pixel attivi: 0 / 19200 &nbsp;|&nbsp; rilevamenti: 0</div>
  </div>

  <div class="panel">
        <h2>Controllo ESC motori (PCA9685 PWM)</h2>
    <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px">
      <button class="ch-btn" id="b0" onclick="toggleCh(0)">CH 0 (FL)</button>
      <button class="ch-btn" id="b1" onclick="toggleCh(1)">CH 1 (FR)</button>
      <button class="ch-btn" id="b2" onclick="toggleCh(2)">CH 2 (RL)</button>
      <button class="ch-btn" id="b3" onclick="toggleCh(3)">CH 3 (RR)</button>
    </div>
    <div class="esc-row">
      <label>Throttle</label>
      <input type="range" min="0" max="100" value="50" id="throttle" oninput="onThrottle(this.value)">
      <span id="tval">50%</span>
    </div>
    <div id="ch-vals" style="font-size:.78rem;color:#666;margin-bottom:10px">
      CH0:0% &nbsp; CH1:0% &nbsp; CH2:0% &nbsp; CH3:0%
    </div>
        <div style="display:flex;gap:10px;margin-top:4px;flex-wrap:wrap">
      <button id="arm-btn" onclick="arm()">&#9889; Arma ESC</button>
    <button onclick="allStop()">&#9632; Stop tutti</button>
            <button onclick="beep()">Calibra ESC</button>
            <button onclick="ping()">Ping motori</button>
            <button onclick="sweep()">Rampa attivo</button>
            <button onclick="sweepHigh()">Rampa alta</button>
            <button onclick="test()">Test ESC</button>
    </div>
        <div id="cmd-status" style="font-size:.78rem;color:#888;margin-top:10px">pronto</div>
  </div>

  <script>
        document.getElementById('stream').src = 'http://' + location.hostname + ':81/stream';
    const badge = document.getElementById('badge');
    const fill  = document.getElementById('px-fill');
    const stats = document.getElementById('stats');
    const cmd   = document.getElementById('cmd-status');

    function poll() {
      fetch('/status').then(r=>r.json()).then(d=>{
        // badge
        if(d.age_ms>5000){badge.textContent='...';badge.className='';return;}
        badge.textContent = d.person ? '\uD83D\uDC64 PERSONA RILEVATA' : '\u2714 area libera';
        badge.className   = d.person ? 'on' : 'off';
        // bar
        const pct = Math.min(100, d.active_px / 192);
        fill.style.width  = pct + '%';
        fill.style.background = d.person ? '#d43030' : '#3a7fd5';
        stats.textContent = `pixel attivi: ${d.active_px} / 19200 | rilevamenti: ${d.count}`;
      }).catch(()=>{});
    }
    setInterval(poll, 1000); poll();

    // --- Canali attivi e throttle ---
    const active = [false,false,false,false];
    let   curPct = 50;

    function toggleCh(ch) {
      active[ch] = !active[ch];
      document.getElementById('b'+ch).classList.toggle('active', active[ch]);
      const pct = active[ch] ? curPct : 0;
      fetch('/esc?ch='+ch+'&pct='+pct);
      updateChVals();
    }

    function onThrottle(val) {
      curPct = parseInt(val);
      document.getElementById('tval').textContent = val+'%';
      active.forEach((on,ch)=>{ if(on) fetch('/esc?ch='+ch+'&pct='+val); });
      updateChVals();
    }

    function updateChVals() {
      const s = active.map((on,i)=>'CH'+i+':'+(on?curPct:0)+'%').join(' \u00a0 ');
      document.getElementById('ch-vals').textContent = s;
    }

        function send(path, label) {
            cmd.textContent = label + '...';
            fetch(path).then(r=>{
                cmd.textContent = r.ok ? (label + ' OK') : (label + ' errore');
            }).catch(()=>{
                cmd.textContent = label + ' errore';
            });
        }

        function arm(){ send('/arm', 'arm'); }
        function beep(){ send('/beep', 'calibrazione'); }
                function ping(){ send('/ping', 'ping'); }
                function activeChannel() {
                    for (let i = 0; i < active.length; i++) if (active[i]) return i;
                    return 0;
                }
                function sweep(){ send('/sweep?ch=' + activeChannel(), 'sweep'); }
                function sweepHigh(){ send('/sweep-high?ch=' + activeChannel(), 'sweep-high'); }
        function test(){ send('/test', 'test'); }
    function allStop(){
      for(let i=0;i<4;i++){
                active[i] = false;
                document.getElementById('b'+i).classList.remove('active');
      }
            fetch('/stop').then(()=>{ cmd.textContent = 'stop OK'; }).catch(()=>{ cmd.textContent = 'stop errore'; });
            updateChVals();
    }
  </script>
</body>
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, sizeof(INDEX_HTML) - 1);
}

void startCameraServer() {
    httpd_config_t ctrl_cfg = HTTPD_DEFAULT_CONFIG();
    ctrl_cfg.server_port = 80;
    ctrl_cfg.stack_size = 8192;
    ctrl_cfg.max_uri_handlers = 10;

    httpd_config_t stream_cfg = HTTPD_DEFAULT_CONFIG();
    stream_cfg.server_port = 81;
    stream_cfg.ctrl_port = 32769;
    stream_cfg.stack_size = 8192;
    stream_cfg.max_uri_handlers = 4;

    httpd_uri_t ctrl_uris[] = {
        { "/",       HTTP_GET, index_handler,  NULL },
        { "/status", HTTP_GET, status_handler, NULL },
        { "/esc",    HTTP_GET, esc_handler,    NULL },
        { "/stop",   HTTP_GET, stop_handler,   NULL },
        { "/arm",    HTTP_GET, arm_handler,    NULL },
        { "/beep",   HTTP_GET, beep_handler,   NULL },
        { "/sweep",  HTTP_GET, sweep_handler,  NULL },
        { "/sweep-high", HTTP_GET, sweep_high_handler, NULL },
        { "/test",   HTTP_GET, test_handler,   NULL },
        { "/ping",   HTTP_GET, ping_handler,   NULL },
    };
    httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };

    bool ok_ctrl = false;
    bool ok_stream = false;

    if (httpd_start(&ctrl_httpd, &ctrl_cfg) == ESP_OK) {
        for (auto& u : ctrl_uris) httpd_register_uri_handler(ctrl_httpd, &u);
        ok_ctrl = true;
    }
    if (httpd_start(&stream_httpd, &stream_cfg) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        ok_stream = true;
    }

    if (ok_ctrl && ok_stream) {
        Serial.println("[HTTP] Control server: /, /status, /esc, /stop, /arm, /beep, /sweep, /sweep-high, /test, /ping su porta 80");
        Serial.println("[HTTP] Stream server: /stream su porta 81");
    } else {
        Serial.printf("[HTTP] Errore avvio server ctrl=%d stream=%d\n", ok_ctrl, ok_stream);
    }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0) < 3000) delay(10);
    Serial.println("\n[BOOT] ESP32-S3 Drone AI");

    // --- PCA9685 ---
    Wire.begin(PCA_SDA, PCA_SCL);
    delay(50);
    Serial.printf("[I2C] Scan su SDA=%d SCL=%d...\n", PCA_SDA, PCA_SCL);
    uint8_t pca_addr = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() != 0) continue;

        // Leggi registro 0x00 per identificare il dispositivo
        Wire.beginTransmission(addr);
        Wire.write(0x00);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)addr, (uint8_t)1);
        uint8_t reg0 = Wire.available() ? Wire.read() : 0xFF;

        // ADXL345: DEVID register (0x00) = 0xE5 fisso
        if (reg0 == 0xE5) {
            Serial.printf("[I2C] 0x%02X DEVID=0xE5 <- ADXL345\n", addr);
            continue;
        }
        // MPU-6050/SSD1306 e altri address noti non-PCA
        if (addr == 0x3C || addr == 0x3D || addr == 0x68 || addr == 0x69) {
            Serial.printf("[I2C] 0x%02X <- sensore noto (non PCA9685)\n", addr);
            continue;
        }
        // PCA9685: range completo 0x40-0x7F, MODE1 reset = 0x11
        // Accettiamo anche 0x01 (ALLCALL only) e 0x21 (SLEEP+ALLCALL cleared)
        // ESCLUDIAMO 0x00 che è risposta ADXL345/altro con reg sbagliata
        if (addr >= 0x40 && addr <= 0x7F) {
            bool likely_pca = (reg0 != 0x00) && (reg0 != 0xFF);
            Serial.printf("[I2C] 0x%02X MODE1=0x%02X <- %s\n", addr, reg0,
                          likely_pca ? "PCA9685 TROVATO!" : "dispositivo (MODE1 sospetto)");
            if (likely_pca && pca_addr == 0) pca_addr = addr;
        } else {
            Serial.printf("[I2C] 0x%02X reg0=0x%02X <- altro\n", addr, reg0);
        }
    }
    if (pca_addr == 0) {
        Serial.println("[I2C] *** PCA9685 NON TROVATO (0x40 non risponde) ***");
        Serial.println("[I2C]   Verifica: VCC=3.3V al pin VCC del PCA9685");
        Serial.println("[I2C]   Verifica: OE collegato a GND");
        Serial.println("[I2C]   Verifica: SDA=GPIO43, SCL=GPIO44 non invertiti");
        Serial.println("[I2C]   Il PCA9685 senza power non risponde sull'I2C!");
    } else {
        Serial.printf("[I2C] PCA9685 rilevato su 0x%02X\n", pca_addr);
    }
    if (pca_addr != 0) {
        pca = Adafruit_PWMServoDriver(pca_addr);
        pca.begin();
        pca.setOscillatorFrequency(25000000);
        pca.setPWMFreq(PCA_FREQ);
        delay(10);
        for (uint8_t i = 0; i < 4; i++) pca.setPWM(i, 0, usToPca(ESC_MIN_US));
        g_pca_ready = true;
        Serial.printf("[PCA] Pronta su 0x%02X, PWM=%dHz\n", pca_addr, PCA_FREQ);
    } else {
        g_pca_ready = false;
    }

    // --- Camera (JPEG, QVGA, PSRAM) ---
    camera_config_t cc;
    cc.ledc_channel  = LEDC_CHANNEL_0; cc.ledc_timer    = LEDC_TIMER_0;
    cc.pin_d0 = Y2_GPIO_NUM; cc.pin_d1 = Y3_GPIO_NUM;
    cc.pin_d2 = Y4_GPIO_NUM; cc.pin_d3 = Y5_GPIO_NUM;
    cc.pin_d4 = Y6_GPIO_NUM; cc.pin_d5 = Y7_GPIO_NUM;
    cc.pin_d6 = Y8_GPIO_NUM; cc.pin_d7 = Y9_GPIO_NUM;
    cc.pin_xclk = XCLK_GPIO_NUM; cc.pin_pclk  = PCLK_GPIO_NUM;
    cc.pin_vsync = VSYNC_GPIO_NUM; cc.pin_href = HREF_GPIO_NUM;
    cc.pin_sccb_sda = SIOD_GPIO_NUM; cc.pin_sccb_scl = SIOC_GPIO_NUM;
    cc.pin_pwdn  = PWDN_GPIO_NUM; cc.pin_reset = RESET_GPIO_NUM;
    cc.xclk_freq_hz = 10000000;
    cc.pixel_format = PIXFORMAT_GRAYSCALE;  // pixel reali per detection, frame2jpg per stream
    cc.frame_size   = FRAMESIZE_QVGA;       // 320x240
    cc.jpeg_quality = 12;
    cc.fb_count     = 1;
    cc.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    cc.fb_location  = CAMERA_FB_IN_PSRAM;

    // Silenzia i log interni del driver camera PRIMA dell'init.
    // cam_task ha stack fisso di 2048 byte: la prima chiamata uart_write
    // da quel task fa una malloc per inizializzare il lock newlib,
    // overflow garantito. Disabilitare i log risolve il crash.
    esp_log_level_set("cam_hal",  ESP_LOG_NONE);
    esp_log_level_set("sccb",     ESP_LOG_NONE);
    esp_log_level_set("camera_fb",ESP_LOG_NONE);

    if (esp_camera_init(&cc) != ESP_OK) {
        Serial.println("[CAM] Init fallita"); return;
    }
    esp_log_level_set("cam_hal", ESP_LOG_NONE);
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0); s->set_contrast(s, 0); s->set_saturation(s, 0);
        s->set_whitebal(s, 1); s->set_exposure_ctrl(s, 1); s->set_gain_ctrl(s, 1);
    }
    Serial.println("[CAM] Camera pronta (GRAYSCALE QVGA, XCLK 10MHz)");

    // --- Buffer rilevamento in PSRAM ---
    g_bg_frame  = (uint8_t*)ps_malloc(DET_PIXELS);
    g_cur_frame = (uint8_t*)ps_malloc(DET_PIXELS);
    if (!g_bg_frame || !g_cur_frame) {
        Serial.println("[DET] ps_malloc fallito"); return;
    }
    g_det_mutex = xSemaphoreCreateMutex();
    // Core 1: cam_task del driver camera usa Core 0 → detection va su Core 1
    // Stack 8192: Serial.printf dentro detectionTask richiede stack grande
    xTaskCreatePinnedToCore(detectionTask, "detection", 8192, NULL, 1, NULL, 1);
    Serial.println("[DET] Task rilevamento avviato (Core 1)");

    // --- WiFi ---
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Connessione");
    int att = 0;
    while (WiFi.status() != WL_CONNECTED && att < 30) {
        delay(500); Serial.print("."); att++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[WEB]  http://%s\n", WiFi.localIP().toString().c_str());
        startCameraServer();
    } else {
        Serial.println("\n[WiFi] Connessione fallita.");
    }
}

void loop() {
    delay(10000);
}
