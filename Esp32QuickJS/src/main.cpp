#include <WiFi.h>
#include "M5Lite.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>

//#define LOCAL_JAVASCRIPT  // ROMに埋め込む場合にはコメントアウトを外す

const char *wifi_ssid = "【WiFiアクセスポイントのSSID】";
const char *wifi_password = "【WiFiアクセスポイントのパスワード】";
const char *jscode_modules_url = "https://raw.githubusercontent.com/poruruba/Esp32QuickJS_sample/main/public_html/modules.json";
const char *jscode_main_url = "https://raw.githubusercontent.com/poruruba/Esp32QuickJS_sample/main/public_html/main_modules.js";

WiFiClient espClient;
WiFiClientSecure espClientSecure;
#include "quickjs_esp32.h"

// see platformio.ini
#ifdef LOCAL_JAVASCRIPT
extern const char jscode_main[] asm("_binary_src_main_js_start");
extern const char jscode_fib[] asm("_binary_src_fib_js_start");
extern const char jscode_math[] asm("_binary_src_math_js_start");
#else
extern const char jscode_default[] asm("_binary_src_default_js_start");

#define JSCODE_BUFFER_SIZE  10000
char jscode[JSCODE_BUFFER_SIZE];
#define JSDOCUMENT_MODULES_SIZE  1000
char modules_buffer[1000];
char load_buffer[20000];

long doHttpGet(String url, uint8_t *p_buffer, unsigned long *p_len);
#endif
void wifi_connect(const char *ssid, const char *password);

ESP32QuickJS qjs;

void setup() {
  M5Lite.begin();
  Serial.begin(9600);

  M5Lite.Axp.ScreenBreath(10);
  M5Lite.Imu.Init();
  
  M5Lite.Lcd.setRotation(3);
  M5Lite.Lcd.fillScreen(BLACK);
  M5Lite.Lcd.setTextColor(WHITE, BLACK);
  M5Lite.Lcd.println("[M5StickC]");

  wifi_connect(wifi_ssid, wifi_password);

#ifdef LOCAL_JAVASCRIPT
  qjs.begin();

  qjs.load_module(jscode_fib, strlen(jscode_fib), "fib.js");
  qjs.load_module(jscode_math, strlen(jscode_math), "math.js");

  qjs.exec(jscode_main);
#else
  qjs.begin();

  bool success = false;
  while(true){
  long ret;
    unsigned long modules_len = sizeof(modules_buffer);
    ret = doHttpGet(jscode_modules_url, (uint8_t*)modules_buffer, &modules_len);
    if( ret != 0 ){
      Serial.println("modules.json get error");
      break;
    }
    
    DynamicJsonDocument doc(JSDOCUMENT_MODULES_SIZE);
    DeserializationError err = deserializeJson(doc, modules_buffer, modules_len);
    if( err ){
      Serial.print("Deserialize error: ");
      Serial.println(err.c_str());
      break;
    }

    unsigned long load_buffer_len = 0;
    JsonArray array = doc.as<JsonArray>();
    for( JsonVariant val : array ){
      const char *url = val["url"];
      const char *name = val["name"];
      
      unsigned long buffer_len = sizeof(load_buffer) - load_buffer_len;
      long ret = doHttpGet(url, (uint8_t*)&load_buffer[load_buffer_len], &buffer_len);
      if( ret != 0 ){
        Serial.println("module get error");
        break;
      }
      load_buffer[load_buffer_len + buffer_len] = '\0';

      Serial.println(name);
      qjs.load_module(&load_buffer[load_buffer_len], buffer_len, name);
      load_buffer_len += buffer_len + 1;
    }

    unsigned long jscode_len = sizeof(jscode);
    ret = doHttpGet(jscode_main_url, (uint8_t*)jscode, &jscode_len);
    if( ret != 0 ){
      Serial.println("main.js get error");
      break;
    }
    jscode[jscode_len] = '\0';

    success = true;
    break;
  }

  if( success ){
    qjs.exec(jscode);
  }else{
    qjs.exec(jscode_default);
  }
#endif
}

void loop() {
  M5Lite.update();
  qjs.loop(); // For timer, async, etc.

  if (M5Lite.BtnB.wasPressed()) {
    Serial.println("BtnB pressed");
    esp_restart();
  }
}

#ifndef LOCAL_JAVASCRIPT
void wifi_connect(const char *ssid, const char *password){
  Serial.println("");
  Serial.print("WiFi Connenting");
  M5Lite.Lcd.print("Connecting");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    M5Lite.Lcd.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.print("Connected : ");
  Serial.println(WiFi.localIP());

  M5Lite.Lcd.fillScreen(BLACK);
  M5Lite.Lcd.setCursor(0, 0);
  M5Lite.Lcd.println(WiFi.localIP());
}

long doHttpGet(String url, uint8_t *p_buffer, unsigned long *p_len){
  Serial.println(url);
  HTTPClient http;

  Serial.print("[HTTP] GET begin...\n");
  // configure traged server and url
  if( url.startsWith("https") )
    http.begin(espClientSecure, url); //HTTPS
  else
    http.begin(espClient, url); //HTTP

  Serial.print("[HTTP] GET...\n");
  // start connection and send HTTP header
  int httpCode = http.GET();
  unsigned long index = 0;

  // httpCode will be negative on error
  if(httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] GET... code: %d\n", httpCode);

      // file found at server
      if(httpCode == HTTP_CODE_OK) {
        // get tcp stream
        WiFiClient * stream = http.getStreamPtr();

        // get lenght of document (is -1 when Server sends no Content-Length header)
        int len = http.getSize();
        Serial.printf("[HTTP] Content-Length=%d\n", len);
        if( len != -1 && len > *p_len ){
          Serial.printf("[HTTP] buffer size over\n");
          http.end();
          return -1;
        }

        // read all data from server
        while(http.connected() && (len > 0 || len == -1)) {
            // get available data size
            size_t size = stream->available();

            if(size > 0) {
                // read up to 128 byte
                if( (index + size ) > *p_len){
                  Serial.printf("[HTTP] buffer size over\n");
                  http.end();
                  return -1;
                }
                int c = stream->readBytes(&p_buffer[index], size);

                index += c;
                if(len > 0) {
                    len -= c;
                }
            }
            delay(1);
        }
      }
  } else {
    http.end();
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    return -1;
  }

  http.end();
  *p_len = index;

  return 0;
}
#endif
