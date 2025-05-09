/*****************************************************************
 *  ESP32-CAM  MJPEG + SSE-таймер  (обратный отсчёт на веб-странице)
 *  –  видеопоток /stream без блокирующего while
 *  –  Server-Sent Events /events: событие “start-countdown”
 *  –  каждый раз при нажатии BTN_PIN таймер запускается заново
 *****************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "DFRobotDFPlayerMini.h"
#include "printer.h"

#include "bilinear.h"
#include "dithering.h"
#include "camera.h"

/* ======================  Пины и Wi-Fi  ======================= */
#define WIFI_SSID "MikroTik-AC3"
#define WIFI_PASS "20000912"

#define BTN_PIN 14
#define RELAY 2
#define LIGHT 15
#define SSERIAL_RX 12
#define SSERIAL_TX 13
#define PLAYER 4

/* ======================  Глобальные  ========================= */
DFRobotDFPlayerMini myDFPlayer;
Printer printer(Serial1);

WiFiServer webServer(80);

/* ––– состояния открытых соединений ––– */
WiFiClient streamClient;
bool streamClientActive = false;  // MJPEG
WiFiClient eventClient;
bool eventClientActive = false;  // /events

/* работа приложения */
bool startCountdown = false;    // нужно отправить “start-countdown”
unsigned long playerStart = 0;  // момент нажатия кнопки
int max_photos = 3;


/* ======================  Прототипы  ========================== */
void handleHttpRequest(WiFiClient &client);
void sendMjpegFrame();
void processEvents();
void checkButton();
void printFrame();
void drawTextOnImage(uint8_t *img, int w, int h,
                     const char *txt, int x, int y, int scale);
const uint8_t font5x7[96][5] = {
        { 0x00, 0x00, 0x00, 0x00, 0x00 },  // ' ' (32)
        { 0x00, 0x00, 0x5F, 0x00, 0x00 },  // '!'
        { 0x00, 0x07, 0x00, 0x07, 0x00 },  // '"'
        { 0x14, 0x7F, 0x14, 0x7F, 0x14 },  // '#'
        { 0x24, 0x2A, 0x7F, 0x2A, 0x12 },  // '$'
        { 0x23, 0x13, 0x08, 0x64, 0x62 },  // '%'
        { 0x36, 0x49, 0x55, 0x22, 0x50 },  // '&'
        { 0x00, 0x05, 0x03, 0x00, 0x00 },  // '''
        { 0x00, 0x1C, 0x22, 0x41, 0x00 },  // '('
        { 0x00, 0x41, 0x22, 0x1C, 0x00 },  // ')'
        { 0x14, 0x08, 0x3E, 0x08, 0x14 },  // '*'
        { 0x08, 0x08, 0x3E, 0x08, 0x08 },  // '+'
        { 0x00, 0x50, 0x30, 0x00, 0x00 },  // ','
        { 0x08, 0x08, 0x08, 0x08, 0x08 },  // '-'
        { 0x00, 0x60, 0x60, 0x00, 0x00 },  // '.'
        { 0x20, 0x10, 0x08, 0x04, 0x02 },  // '/'
        { 0x3E, 0x51, 0x49, 0x45, 0x3E },  // '0'
        { 0x00, 0x42, 0x7F, 0x40, 0x00 },  // '1'
        { 0x42, 0x61, 0x51, 0x49, 0x46 },  // '2'
        { 0x21, 0x41, 0x45, 0x4B, 0x31 },  // '3'
        { 0x18, 0x14, 0x12, 0x7F, 0x10 },  // '4'
        { 0x27, 0x45, 0x45, 0x45, 0x39 },  // '5'
        { 0x3C, 0x4A, 0x49, 0x49, 0x30 },  // '6'
        { 0x01, 0x71, 0x09, 0x05, 0x03 },  // '7'
        { 0x36, 0x49, 0x49, 0x49, 0x36 },  // '8'
        { 0x06, 0x49, 0x49, 0x29, 0x1E },  // '9'
        { 0x00, 0x36, 0x36, 0x00, 0x00 },  // ':'
        { 0x00, 0x56, 0x36, 0x00, 0x00 },  // ';'
        { 0x08, 0x14, 0x22, 0x41, 0x00 },  // '<'
        { 0x14, 0x14, 0x14, 0x14, 0x14 },  // '='
        { 0x00, 0x41, 0x22, 0x14, 0x08 },  // '>'
        { 0x02, 0x01, 0x51, 0x09, 0x06 },  // '?'
        { 0x32, 0x49, 0x79, 0x41, 0x3E },  // '@'
        { 0x7E, 0x11, 0x11, 0x11, 0x7E },  // 'A'
        { 0x7F, 0x49, 0x49, 0x49, 0x36 },  // 'B'
        { 0x3E, 0x41, 0x41, 0x41, 0x22 },  // 'C'
        { 0x7F, 0x41, 0x41, 0x22, 0x1C },  // 'D'
        { 0x7F, 0x49, 0x49, 0x49, 0x41 },  // 'E'
        { 0x7F, 0x09, 0x09, 0x09, 0x01 },  // 'F'
        { 0x3E, 0x41, 0x49, 0x49, 0x7A },  // 'G'
        { 0x7F, 0x08, 0x08, 0x08, 0x7F },  // 'H'
        { 0x00, 0x41, 0x7F, 0x41, 0x00 },  // 'I'
        { 0x20, 0x40, 0x41, 0x3F, 0x01 },  // 'J'
        { 0x7F, 0x08, 0x14, 0x22, 0x41 },  // 'K'
        { 0x7F, 0x40, 0x40, 0x40, 0x40 },  // 'L'
        { 0x7F, 0x02, 0x0C, 0x02, 0x7F },  // 'M'
        { 0x7F, 0x04, 0x08, 0x10, 0x7F },  // 'N'
        { 0x3E, 0x41, 0x41, 0x41, 0x3E },  // 'O'
        { 0x7F, 0x09, 0x09, 0x09, 0x06 },  // 'P'
        { 0x3E, 0x41, 0x51, 0x21, 0x5E },  // 'Q'
        { 0x7F, 0x09, 0x19, 0x29, 0x46 },  // 'R'
        { 0x46, 0x49, 0x49, 0x49, 0x31 },  // 'S'
        { 0x01, 0x01, 0x7F, 0x01, 0x01 },  // 'T'
        { 0x3F, 0x40, 0x40, 0x40, 0x3F },  // 'U'
        { 0x1F, 0x20, 0x40, 0x20, 0x1F },  // 'V'
        { 0x3F, 0x40, 0x38, 0x40, 0x3F },  // 'W'
        { 0x63, 0x14, 0x08, 0x14, 0x63 },  // 'X'
        { 0x07, 0x08, 0x70, 0x08, 0x07 },  // 'Y'
        { 0x61, 0x51, 0x49, 0x45, 0x43 },  // 'Z'
        { 0x00, 0x7F, 0x41, 0x41, 0x00 },  // '['
        { 0x02, 0x04, 0x08, 0x10, 0x20 },  // '\'
        { 0x00, 0x41, 0x41, 0x7F, 0x00 },  // ']'
        { 0x04, 0x02, 0x01, 0x02, 0x04 },  // '^'
        { 0x40, 0x40, 0x40, 0x40, 0x40 },  // '_'
        { 0x00, 0x03, 0x05, 0x00, 0x00 },  // ''
        { 0x20, 0x54, 0x54, 0x54, 0x78 },  // 'a'
        { 0x7F, 0x48, 0x44, 0x44, 0x38 },  // 'b'
        { 0x38, 0x44, 0x44, 0x44, 0x20 },  // 'c'
        { 0x38, 0x44, 0x44, 0x48, 0x7F },  // 'd'
        { 0x38, 0x54, 0x54, 0x54, 0x18 },  // 'e'
        { 0x08, 0x7E, 0x09, 0x01, 0x02 },  // 'f'
        { 0x0C, 0x52, 0x52, 0x52, 0x3E },  // 'g'
        { 0x7F, 0x08, 0x04, 0x04, 0x78 },  // 'h'
        { 0x00, 0x44, 0x7D, 0x40, 0x00 },  // 'i'
        { 0x20, 0x40, 0x44, 0x3D, 0x00 },  // 'j'
        { 0x7F, 0x10, 0x28, 0x44, 0x00 },  // 'k'
        { 0x00, 0x41, 0x7F, 0x40, 0x00 },  // 'l'
        { 0x7C, 0x04, 0x18, 0x04, 0x78 },  // 'm'
        { 0x7C, 0x08, 0x04, 0x04, 0x78 },  // 'n'
        { 0x38, 0x44, 0x44, 0x44, 0x38 },  // 'o'
        { 0x7C, 0x14, 0x14, 0x14, 0x08 },  // 'p'
        { 0x08, 0x14, 0x14, 0x18, 0x7C },  // 'q'
        { 0x7C, 0x08, 0x04, 0x04, 0x08 },  // 'r'
        { 0x48, 0x54, 0x54, 0x54, 0x20 },  // 's'
        { 0x04, 0x3F, 0x44, 0x40, 0x20 },  // 't'
        { 0x3C, 0x40, 0x40, 0x20, 0x7C },  // 'u'
        { 0x1C, 0x20, 0x40, 0x20, 0x1C },  // 'v'
        { 0x3C, 0x40, 0x30, 0x40, 0x3C },  // 'w'
        { 0x44, 0x28, 0x10, 0x28, 0x44 },  // 'x'
        { 0x0C, 0x50, 0x50, 0x50, 0x3C },  // 'y'
        { 0x44, 0x64, 0x54, 0x4C, 0x44 },  // 'z'
        { 0x00, 0x08, 0x36, 0x41, 0x00 },  // '{'
        { 0x00, 0x00, 0x7F, 0x00, 0x00 },  // '|'
        { 0x00, 0x41, 0x36, 0x08, 0x00 },  // '}'
        { 0x08, 0x08, 0x2A, 0x1C, 0x08 }   // '~'
};
/* ========================  setup()  ========================== */
void setup() {
    Serial.begin(115200);

    /* ---- периферия ---- */
    Serial1.begin(9600, SERIAL_8N1, SSERIAL_RX, SSERIAL_TX);
    printer.begin();
    printer.config(10, 140, 4);

    Serial2.begin(9600, SERIAL_8N1, -1, PLAYER);
    myDFPlayer.begin(Serial2);
    myDFPlayer.volume(25);

    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(RELAY, OUTPUT);
    pinMode(LIGHT, OUTPUT);
    digitalWrite(RELAY, 0);
    digitalWrite(LIGHT, 0);

    /* ---- камера ---- */
    if (cam_init(FRAMESIZE_VGA, PIXFORMAT_JPEG) != ESP_OK) {
        Serial.println("Ошибка инициализации камеры");
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_special_effect(s, 2);  // ч/б для трансляции

    /* ---- Wi-Fi ---- */
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Подключение к Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print('.');
    }
    Serial.println();
    Serial.print("IP-адрес: ");
    Serial.println(WiFi.localIP());
    webServer.begin();

    /* ---- OTA ---- */
    ArduinoOTA.setHostname("camera");
    ArduinoOTA.setPassword("1234");
    ArduinoOTA.begin();
    myDFPlayer.play(2);
    printer.print("Locl IP:   ");
    printer.println(WiFi.localIP());
    printer.println();
    printer.println("\n");
}

/* =========================  loop()  ========================== */
void loop() {
    ArduinoOTA.handle();

    /* приём новых HTTP-клиентов */
    WiFiClient newClient = webServer.available();
    if (newClient) handleHttpRequest(newClient);

    /* отправка очередного кадра, если поток активен */
    if (streamClientActive && streamClient.connected()) sendMjpegFrame();
    else streamClientActive = false;

    /* проверка кнопки ─ запускаем обратный отсчёт и печать */
    checkButton();

    /* если нужно – отправить событие “start-countdown” */
    if (eventClientActive && eventClient.connected()) processEvents();
    else eventClientActive = false;
}

/* =================  Обработка HTTP-запроса  ================== */
void handleHttpRequest(WiFiClient &client) {
    String req = client.readStringUntil('\r');
    Serial.println(req);

    /* ----------  MJPEG /stream  ---------- */
    if (req.startsWith("GET /stream")) {
        streamClient = client;
        streamClientActive = true;

        streamClient.println(
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
        return;
    }

    /* ----------  SSE /events  ------------ */
    if (req.startsWith("GET /events")) {
        eventClient = client;
        eventClientActive = true;

        eventClient.println(
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/event-stream\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: keep-alive\r\n\r\n");
        Serial.println("👂 Подключился клиент /events");
        return;
    }

    /* ----------  Главная страница  ---------- */
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html\r\n");
    client.println(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ESP32-CAM</title>
  <style>
    html,body{margin:0;height:100%;background:#000;color:#fff;
              font-family:sans-serif;overflow:hidden}
    .container{display:flex;justify-content:center;align-items:center;height:100%}
    img{height:100%;object-fit:contain;background:#000}
    #countdown{position:absolute;top:10%;font-size:6em;display:none}
  </style>
</head>
<body>
  <div class="container">
    <img src="/stream">
    <div id="countdown">3</div>
  </div>
  <script>
    window.addEventListener("DOMContentLoaded",()=>{
      console.log("Подключение к /events…");
      const countdownEl=document.getElementById("countdown");

      function runCountdown(sec){
        countdownEl.style.display="block";
        countdownEl.innerText=sec;
        let t=setInterval(()=>{
          sec--;
          if(sec<=0){
            clearInterval(t);
            countdownEl.innerText="📸";
            setTimeout(()=>{countdownEl.style.display="none";},800);
          }else countdownEl.innerText=sec;
        },1000);
      }

      const src=new EventSource("/events");
      src.addEventListener("start-countdown",e=>{
        console.log("Получено start-countdown");
        runCountdown(parseInt(e.data));
      });
    });
  </script>
</body>
</html>)rawliteral");
}

/* ==============  Отправка одного кадра MJPEG  =============== */
void sendMjpegFrame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        streamClientActive = false;
        return;
    }

    streamClient.print("--frame\r\n"
                       "Content-Type: image/jpeg\r\n"
                       "Content-Length: ");
    streamClient.print(fb->len);
    streamClient.print("\r\n\r\n");
    streamClient.write(fb->buf, fb->len);
    streamClient.print("\r\n");

    esp_camera_fb_return(fb);
}

/* ==============  Отправка события SSE  ====================== */
void processEvents() {
    static unsigned long lastPing = 0;

    if (startCountdown) {
        Serial.println("📢 Отправка start-countdown");
        eventClient.print("event: start-countdown\n");
        eventClient.print("data: 3\n\n");
        eventClient.flush();
        startCountdown = false;
    }

    /* поддерживающий ping каждые 15 с */
    if (millis() - lastPing > 15000) {
        eventClient.print(": ping\n\n");
        eventClient.flush();
        lastPing = millis();
    }
}

/* ==================  Работа кнопки  ========================= */
void checkButton() {
    if (max_photos) {
        if (playerStart == 0 && !digitalRead(BTN_PIN)) {
            Serial.println("Нажатие кнопки");
            startCountdown = true;
            myDFPlayer.play(1);
            playerStart = millis();
        }
        if (playerStart > 0 && millis() - playerStart > 4000) {
            Serial.println("Печать кадра");
            esp_camera_deinit();
            delay(100);
            cam_init(FRAMESIZE_VGA, PIXFORMAT_GRAYSCALE);
            delay(100);
            sensor_t *s = esp_camera_sensor_get();
            if (s) {
                s->set_special_effect(s, 0);
                s->set_exposure_ctrl(s, 1);
                s->set_gain_ctrl(s, 1);
            }
            printFrame();
            max_photos--;
            delay(100);
            esp_camera_deinit();
            delay(100);
            cam_init(FRAMESIZE_VGA, PIXFORMAT_JPEG);
            sensor_t *sr = esp_camera_sensor_get();
            if (sr) {
                sr->set_special_effect(sr, 2);
            }
            playerStart = 0;
        }
    }
}



/* ==================  Печать кадра  ========================== */
void printFrame() {
    digitalWrite(LIGHT, 1);
    camera_fb_t *fbj = esp_camera_fb_get();
    esp_camera_fb_return(fbj);

    for (int i = 0; i < 10; i++) {
        fbj = esp_camera_fb_get();
        esp_camera_fb_return(fbj);
    }

    fbj = esp_camera_fb_get();
    if (!fbj) {
        printer.println("Camera error");
        return;
    }
    digitalWrite(LIGHT, 0);

    int h = 384;
    int w = h * fbj->width / fbj->height;
    uint8_t *chunks = (uint8_t *)ps_malloc(w * h / 8);
    if (!chunks) {
        esp_camera_fb_return(fbj);
        return;
    }

    uint8_t *resized = (uint8_t *)ps_malloc(w * h);
    if (!resized) {
        free(chunks);
        esp_camera_fb_return(fbj);
        return;
    }

    bilinear_interp(fbj->buf, fbj->width, fbj->height, resized, w, h);
    if (max_photos == 2) {
        drawTextOnImage(resized, w, h, "HELLO!", 100, 100, 10);
    }
    dither(resized, w, h);

    int idx = 0;
    for (int x = 0; x < w; x++) {
        int y = h;
        while (y) {
            uint8_t b = 0;
            int i = 8;
            while (i--) {
                y--;
                b <<= 1;
                b |= !resized[x + y * w];
            }
            chunks[idx++] = b;
        }
    }

    printer.drawBitmap(chunks, h, w);
    printer.println();
    printer.println();

    free(resized);
    free(chunks);
    esp_camera_fb_return(fbj);
    digitalWrite(RELAY, 1);  // Открыть замок
}

// ====== Рисование текста ======
void drawChar(uint8_t *img, int imgWidth, int imgHeight, char c, int posX, int posY, int scale) {
    if (c < 32 || c > 126) return;
    extern const uint8_t font5x7[96][5];
    int index = c - 32;
    for (int col = 0; col < 5; col++) {
        uint8_t colData = font5x7[index][col];
        for (int row = 0; row < 7; row++) {
            if (colData & (1 << row)) {
                for (int dx = 0; dx < scale; dx++) {
                    for (int dy = 0; dy < scale; dy++) {
                        int x = posX + col * scale + dx;
                        int y = posY + row * scale + dy;
                        if (x >= 0 && x < imgWidth && y >= 0 && y < imgHeight) {
                            img[y * imgWidth + x] = 0;
                        }
                    }
                }
            }
        }
    }
}
void drawTextOnImage(uint8_t *img, int imgWidth, int imgHeight, const char *text, int posX, int posY, int scale) {
    int cursorX = posX;
    int cursorY = posY;
    for (const char *p = text; *p; p++) {
        if (*p == '\n') {
            cursorY += 7 * scale + scale;
            cursorX = posX;
        } else {
            drawChar(img, imgWidth, imgHeight, *p, cursorX, cursorY, scale);
            cursorX += 5 * scale + scale;
        }
    }
}