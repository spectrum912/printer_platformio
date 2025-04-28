#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "DFRobotDFPlayerMini.h"
DFRobotDFPlayerMini myDFPlayer;

#define WIFI_SSID "MikroTik-AC3"
#define WIFI_PASS "20000912"

unsigned long playerStart = 0;
int max_photos = 3;

#define BTN_PIN 14
#define RELAY 2
#define LIGHT 15

#define USE_MIFI 1
#define SSERIAL_RX 12
#define SSERIAL_TX 13


#define PLAYER 4

#include "printer.h"
Printer printer(Serial1);

#include "bilinear.h"
#include "blur.h"
#include "camera.h"
#include "camtest.h"
#include "dithering.h"
#include "edges.h"

// Создаём HTTP-сервер на порту 80
WiFiServer server(80);
void checkButton();
void printFrame();
// Минимальный 5x7 шрифт для символов ASCII от 32 до 126
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
        { 0x00, 0x03, 0x05, 0x00, 0x00 },  // '`'
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

// Функция для рисования одного символа с масштабированием
void drawChar(uint8_t *img, int imgWidth, int imgHeight, char c, int posX, int posY, int scale) {
    if (c < 32 || c > 126) return;  // символ вне диапазона
    int index = c - 32;
    // Для каждого столбца символа (исходная ширина — 5 пикселей)
    for (int col = 0; col < 5; col++) {
        uint8_t colData = font5x7[index][col];
        // Для каждого ряда (исходная высота — 7 пикселей)
        for (int row = 0; row < 7; row++) {
            if (colData & (1 << row)) {
                // Заполняем блок размером scale x scale
                for (int dx = 0; dx < scale; dx++) {
                    for (int dy = 0; dy < scale; dy++) {
                        int x = posX + col * scale + dx;
                        int y = posY + row * scale + dy;
                        if (x >= 0 && x < imgWidth && y >= 0 && y < imgHeight) {
                            img[y * imgWidth + x] = 0;  // закрашиваем пиксель (0 — чёрный)
                        }
                    }
                }
            }
        }
    }
}

// Функция для рисования строки с масштабированием
void drawTextOnImage(uint8_t *img, int imgWidth, int imgHeight, const char *text, int posX, int posY, int scale) {
    int cursorX = posX;
    int cursorY = posY;
    for (const char *p = text; *p; p++) {
        if (*p == '\n') {
            cursorY += 7 * scale + scale;  // переход на новую строку
            cursorX = posX;
        } else {
            drawChar(img, imgWidth, imgHeight, *p, cursorX, cursorY, scale);
            cursorX += 5 * scale + scale;  // ширина символа + интервал
        }
    }
}

// Функция для обработки HTTP-запроса клиента
void handleClient(WiFiClient client) {
    String req = client.readStringUntil('\r');
    Serial.println(req);

    if (req.indexOf("GET /stream") >= 0) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
        client.println();

        while (client.connected()) {
            checkButton();
            ArduinoOTA.handle();
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("Ошибка захвата кадра");
                break;
            }
            client.print("--frame\r\n");
            client.print("Content-Type: image/jpeg\r\n");
            client.print("Content-Length: ");
            client.print(fb->len);
            client.print("\r\n\r\n");
            client.write(fb->buf, fb->len);
            client.print("\r\n");

            esp_camera_fb_return(fb);
            delay(30);
        }
    } else {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println();
        client.println("<html><head><meta charset='utf-8'></head><body>");
        client.println("<h1>Камера ESP32 </h1>");
        client.println("<img src=\"/stream\" style=\"width:50%;height:auto;\">");
        client.println("</body></html>");
    }

    delay(1);
    client.stop();
}


void setup() {
    Serial.begin(115200);

    Serial1.begin(9600, SERIAL_8N1, SSERIAL_RX, SSERIAL_TX);
    printer.begin();
    printer.config(10, 140, 4);

    Serial2.begin(9600, SERIAL_8N1, -1, PLAYER);
    Serial.println("df start");
    myDFPlayer.begin(Serial2);
    Serial.println("df end");
    myDFPlayer.setTimeOut(500);  //Set serial communictaion time out 500ms
    Serial.println("df end2");
    myDFPlayer.volume(30);       //Set volume value (0~30).
    Serial.println("df end3");
    myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
    Serial.println("df end4");
    // myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
    Serial.println("df end5");

    // Инициализируем камеру для трансляции (режим JPEG)

    if (cam_init(FRAMESIZE_VGA, PIXFORMAT_JPEG) != ESP_OK) {
        Serial.println("Ошибка инициализации камеры!");
    }

    sensor_t *srt = esp_camera_sensor_get();
    if (srt) {
        srt->set_special_effect(srt, 2);  // режим "black & white" для трансляции
    }

    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(RELAY, OUTPUT);
    pinMode(LIGHT, OUTPUT);
//    pinMode(PLAYER, OUTPUT);
    digitalWrite(RELAY, 0);
    digitalWrite(LIGHT, 0);
//    digitalWrite(PLAYER, 1);


    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Подключение к WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Подключено. IP адрес: ");
    Serial.println(WiFi.localIP());

    server.begin();


    // OTA
    ArduinoOTA.setHostname("camera");  // Hostname defaults to esp8266-[ChipID]
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else  // U_SPIFFS
            type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("OTA Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.setPassword("1234");
    ArduinoOTA.begin();
}

void loop() {
    ArduinoOTA.handle();
    WiFiClient client = server.available();
    if (client) {
        while (client.connected() && !client.available()) {
            delay(1);
        }
        handleClient(client);
    }
    checkButton();
}

void checkButton() {
    if(max_photos) {
        if (playerStart == 0 and !digitalRead(BTN_PIN)) {
            Serial.println("кнопка");

//            digitalWrite(PLAYER, 0);
//            delay(50);
//            digitalWrite(PLAYER, 1);
            myDFPlayer.play(1);
            playerStart = millis();
        }
        if (playerStart > 0 and millis() - playerStart > 4000) {
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
                sr->set_special_effect(sr, 2);  // режим "black & white" для трансляции
            }
            playerStart = 0;
        }
    }
}

// Функция печати кадра с наложением увеличенного текста
void printFrame() {
    digitalWrite(LIGHT, 1);
    camera_fb_t *fbj = esp_camera_fb_get();
    esp_camera_fb_return(fbj);

    for (int p = 0; p < 10; p++) {
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
    int w = h * fbj->width / fbj->height;  // вычисляем ширину
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

    // Масштабирование исходного кадра до нужного разрешения
    bilinear_interp(fbj->buf, fbj->width, fbj->height, resized, w, h);

    // Наложение текста "ESP32" с масштабом 10 (увеличенный шрифт)
    if (max_photos == 2) {
        drawTextOnImage(resized, w, h, "HELLO!", 100, 100, 10);
    }
    // Применяем дизеринг для получения бинарного изображения
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

    free(resized);
    printer.drawBitmap(chunks, h, w);
    printer.println();
    printer.println();

    free(chunks);
    esp_camera_fb_return(fbj);
    digitalWrite(RELAY, 1);  // открыть замок
}
