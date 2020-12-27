#pragma once

#if defined(WiFi_h) && !defined(ENABLE_WIFI)
//#define ENABLE_WIFI
#endif

//#include <Arduino.h>
#include "M5Lite.h"

#include <algorithm>
#include <vector>

#ifdef ENABLE_WIFI
#include <HTTPClient.h>
#include <StreamString.h>
#endif

#include "quickjs.h"

static void qjs_dump_exception(JSContext *ctx, JSValue v) {
  if (!JS_IsUndefined(v)) {
    const char *str = JS_ToCString(ctx, v);
    if (str) {
      Serial.println(str);
      JS_FreeCString(ctx, str);
    } else {
      Serial.println("[Exception]");
    }
  }
  JSValue e = JS_GetException(ctx);
  const char *str = JS_ToCString(ctx, e);
  if (str) {
    Serial.println(str);
    JS_FreeCString(ctx, str);
  }
  if (JS_IsError(ctx, e)) {
    JSValue s = JS_GetPropertyStr(ctx, e, "stack");
    if (!JS_IsUndefined(s)) {
      const char *str = JS_ToCString(ctx, s);
      if (str) {
        Serial.println(str);
        JS_FreeCString(ctx, str);
      }
    }
    JS_FreeValue(ctx, s);
  }
  JS_FreeValue(ctx, e);
}

#ifdef ENABLE_WIFI
class JSHttpFetcher {
  struct Entry {
    HTTPClient *client;
    JSValue resolving_funcs[2];
    int status;
    void result(JSContext *ctx, uint32_t func, JSValue body) {
      delete client;  // dispose connection before invoke;
      JSValue r = JS_NewObject(ctx);
      JS_SetPropertyStr(ctx, r, "body", JS_DupValue(ctx, body));
      JS_SetPropertyStr(ctx, r, "status", JS_NewInt32(ctx, status));
      JS_Call(ctx, resolving_funcs[func], JS_UNDEFINED, 1, &r);
      JS_FreeValue(ctx, r);
      JS_FreeValue(ctx, resolving_funcs[0]);
      JS_FreeValue(ctx, resolving_funcs[1]);
    }
  };
  std::vector<Entry *> queue;

 public:
  JSValue fetch(JSContext *ctx, JSValueConst jsUrl, JSValueConst options) {
    if (WiFi.status() != WL_CONNECTED) {
      return JS_EXCEPTION;
    }
    const char *url = JS_ToCString(ctx, jsUrl);
    if (!url) {
      return JS_EXCEPTION;
    }
    const char *method = nullptr, *body = nullptr;
    if (JS_IsObject(options)) {
      JSValue m = JS_GetPropertyStr(ctx, options, "method");
      if (JS_IsString(m)) {
        method = JS_ToCString(ctx, m);
      }
      JSValue b = JS_GetPropertyStr(ctx, options, "body");
      if (JS_IsString(m)) {
        body = JS_ToCString(ctx, b);
      }
    }

    Entry *ent = new Entry();
    ent->client = new HTTPClient();
    ent->client->begin(url);
    JS_FreeCString(ctx, url);

    // TODO: remove blocking calls.
    if (method) {
      ent->status = ent->client->sendRequest(method, (uint8_t *)body,
                                             body ? strlen(body) : 0);
    } else {
      ent->status = ent->client->GET();
    }
    queue.push_back(ent);

    JS_FreeCString(ctx, method);
    JS_FreeCString(ctx, body);
    return JS_NewPromiseCapability(ctx, ent->resolving_funcs);
  }

  void loop(JSContext *ctx) {
    int doneCount = 0;
    for (auto &pent : queue) {
      WiFiClient *stream = pent->client->getStreamPtr();
      if (stream == nullptr || pent->status <= 0) {
        // reject.
        pent->result(ctx, 1, JS_UNDEFINED);
        delete pent;
        pent = nullptr;
        doneCount++;
        continue;
      }
      if (stream->available()) {
        String body = pent->client->getString();
        JSValue bodyStr = JS_NewString(ctx, body.c_str());
        body.clear();
        pent->result(ctx, 0, bodyStr);
        JS_FreeValue(ctx, bodyStr);
        delete pent;
        pent = nullptr;
        doneCount++;
      }
    }

    if (doneCount > 0) {
      queue.erase(std::remove_if(queue.begin(), queue.end(),
                                 [](Entry *pent) { return pent == nullptr; }),
                  queue.end());
    }
  }
};
#endif  // ENABLE_WIFI

class JSTimer {
  // 20 bytes / entry.
  struct TimerEntry {
    uint32_t id;
    int32_t timeout;
    int32_t interval;
    JSValue func;
  };
  std::vector<TimerEntry> timers;
  uint32_t id_counter = 0;

 public:
  uint32_t RegisterTimer(JSValue f, int32_t time, int32_t interval = -1) {
    uint32_t id = ++id_counter;
    timers.push_back(TimerEntry{id, time, interval, f});
    return id;
  }
  void RemoveTimer(uint32_t id) {
    timers.erase(std::remove_if(timers.begin(), timers.end(),
                                [id](TimerEntry &t) { return t.id == id; }),
                 timers.end());
  }
  int32_t GetNextTimeout(int32_t now) {
    if (timers.empty()) {
      return -1;
    }
    std::sort(timers.begin(), timers.end(),
              [now](TimerEntry &a, TimerEntry &b) -> bool {
                return (a.timeout - now) >
                       (b.timeout - now);  // 2^32 wraparound
              });
    int next = timers.back().timeout - now;
    return max(next, 0);
  }
  bool ConsumeTimer(JSContext *ctx, int32_t now) {
    std::vector<TimerEntry> t;
    int32_t eps = 2;
    while (!timers.empty() && timers.back().timeout - now <= eps) {
      t.push_back(timers.back());
      timers.pop_back();
    }
    for (auto &ent : t) {
      // NOTE: may update timers in this JS_Call().
      JSValue r = JS_Call(ctx, ent.func, ent.func, 0, nullptr);
      if (JS_IsException(r)) {
        qjs_dump_exception(ctx, r);
      }
      JS_FreeValue(ctx, r);

      if (ent.interval >= 0) {
        ent.timeout = now + ent.interval;
        timers.push_back(ent);
      } else {
        JS_FreeValue(ctx, ent.func);
      }
    }
    return !t.empty();
  }
};

class ESP32QuickJS {
 public:
  JSRuntime *rt;
  JSContext *ctx;
  JSTimer timer;
  JSValue loop_func = JS_UNDEFINED;
#ifdef ENABLE_WIFI
  JSHttpFetcher httpFetcher;
#endif

  void begin() {
    JSRuntime *rt = JS_NewRuntime();
    begin(rt, JS_NewContext(rt));
  }

  void begin(JSRuntime *rt, JSContext *ctx, int memoryLimit = 0) {
    this->rt = rt;
    this->ctx = ctx;
    if (memoryLimit == 0) {
      memoryLimit = ESP.getFreeHeap() >> 1;
    }
    JS_SetMemoryLimit(rt, memoryLimit);
    JS_SetGCThreshold(rt, memoryLimit >> 3);
    JSValue global = JS_GetGlobalObject(ctx);
    setup(ctx, global);
    JS_FreeValue(ctx, global);
  }

  void end() {
    JS_FreeContext(ctx);
    JS_FreeRuntime(rt);
  }

  void loop(bool callLoopFn = true) {
    // async
    JSContext *c;
    int ret = JS_ExecutePendingJob(JS_GetRuntime(ctx), &c);
    if (ret < 0) {
      qjs_dump_exception(ctx, JS_UNDEFINED);
    }

    // timer
    uint32_t now = millis();
    if (timer.GetNextTimeout(now) >= 0) {
      timer.ConsumeTimer(ctx, now);
    }

#ifdef ENABLE_WIFI
    httpFetcher.loop(ctx);
#endif

    // loop()
    if (callLoopFn && JS_IsFunction(ctx, loop_func)) {
      JSValue ret = JS_Call(ctx, loop_func, loop_func, 0, nullptr);
      if (JS_IsException(ret)) {
        qjs_dump_exception(ctx, ret);
      }
      JS_FreeValue(ctx, ret);
    }
  }

  void runGC() { JS_RunGC(rt); }

  bool exec(const char *code) {
    JSValue result = eval(code);
    bool ret = JS_IsException(result);
    JS_FreeValue(ctx, result);
    return ret;
  }

  JSValue eval(const char *code) {
    JSValue ret =
        JS_Eval(ctx, code, strlen(code), "<eval>", JS_EVAL_TYPE_MODULE);
    if (JS_IsException(ret)) {
      qjs_dump_exception(ctx, ret);
    }
    return ret;
  }

  void setLoopFunc(const char *fname) {
    JSValue global = JS_GetGlobalObject(ctx);
    setLoopFunc(JS_GetPropertyStr(ctx, global, fname));
    JS_FreeValue(ctx, global);
  }

 protected:
  void setLoopFunc(JSValue f) {
    JS_FreeValue(ctx, loop_func);
    loop_func = f;
  }

  virtual void setup(JSContext *ctx, JSValue global) {
    this->ctx = ctx;
    JS_SetContextOpaque(ctx, this);

    // setup console.log()
    JSValue console = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "console", console);
    JS_SetPropertyStr(ctx, console, "log",
                      JS_NewCFunction(ctx, console_log, "log", 1));

    // timer
    JS_SetPropertyStr(ctx, global, "setTimeout",
                      JS_NewCFunction(ctx, set_timeout, "setTimeout", 2));
    JS_SetPropertyStr(ctx, global, "clearTimeout",
                      JS_NewCFunction(ctx, clear_timeout, "clearTimeout", 1));
    JS_SetPropertyStr(ctx, global, "setInterval",
                      JS_NewCFunction(ctx, set_interval, "setInterval", 2));
    JS_SetPropertyStr(ctx, global, "clearInterval",
                      JS_NewCFunction(ctx, clear_timeout, "clearInterval", 1));

    static const JSCFunctionListEntry gpio_funcs[] = {
        JSCFunctionListEntry{"pinMode", 0, JS_DEF_CFUNC, 0, {
                               func : {2, JS_CFUNC_generic, esp32_gpio_mode}
                             }},
        JSCFunctionListEntry{
            "analogRead", 0, JS_DEF_CFUNC, 0, {
              func : {1, JS_CFUNC_generic, esp32_gpio_analog_read}
            }},
        JSCFunctionListEntry{
            "digitalRead", 0, JS_DEF_CFUNC, 0, {
              func : {1, JS_CFUNC_generic, esp32_gpio_digital_read}
            }},
        JSCFunctionListEntry{
            "digitalWrite", 0, JS_DEF_CFUNC, 0, {
              func : {2, JS_CFUNC_generic, esp32_gpio_digital_write}
            }},
    };

    static const JSCFunctionListEntry wire_funcs[] = {
        JSCFunctionListEntry{"begin", 0, JS_DEF_CFUNC, 0, {
                               func : {0, JS_CFUNC_generic, esp32_wire_begin}
                             }},
        JSCFunctionListEntry{
            "requestFrom", 0, JS_DEF_CFUNC, 0, {
              func : {2, JS_CFUNC_generic, esp32_wire_requestFrom}
            }},
        JSCFunctionListEntry{
            "beginTransmission", 0, JS_DEF_CFUNC, 0, {
              func : {1, JS_CFUNC_generic, esp32_wire_beginTransmission}
            }},
        JSCFunctionListEntry{
            "endTransmission", 0, JS_DEF_CFUNC, 0, {
              func : {0, JS_CFUNC_generic, esp32_wire_endTransmission}
            }},
        JSCFunctionListEntry{
            "write", 0, JS_DEF_CFUNC, 0, {
              func : {1, JS_CFUNC_generic, esp32_wire_write}
            }},
        JSCFunctionListEntry{
            "write_buf", 0, JS_DEF_CFUNC, 0, {
              func : {1, JS_CFUNC_generic, esp32_wire_write_buf}
            }},
        JSCFunctionListEntry{
            "available", 0, JS_DEF_CFUNC, 0, {
              func : {0, JS_CFUNC_generic, esp32_wire_available}
            }},
        JSCFunctionListEntry{
            "read", 0, JS_DEF_CFUNC, 0, {
              func : {1, JS_CFUNC_generic, esp32_wire_read}
            }},
    };

    static const JSCFunctionListEntry lcd_funcs[] = {
        JSCFunctionListEntry{"setRotation", 0, JS_DEF_CFUNC, 0, {
                               func : {2, JS_CFUNC_generic, esp32_lcd_setRotation}
                             }},
        JSCFunctionListEntry{"setBrightness", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_lcd_setBrigthness}
                             }},
        JSCFunctionListEntry{"setTextColor", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_lcd_setTextColor}
                             }},
        JSCFunctionListEntry{"setTextSize", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_lcd_setTextSize}
                             }},                             
        JSCFunctionListEntry{"drawPixel", 0, JS_DEF_CFUNC, 0, {
                               func : {3, JS_CFUNC_generic, esp32_lcd_drawPixel}
                             }},
        JSCFunctionListEntry{"drawLine", 0, JS_DEF_CFUNC, 0, {
                               func : {5, JS_CFUNC_generic, esp32_lcd_drawLine}
                             }},
        JSCFunctionListEntry{"setCursor", 0, JS_DEF_CFUNC, 0, {
                               func : {2, JS_CFUNC_generic, esp32_lcd_setCursor}
                             }},
        JSCFunctionListEntry{"print", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_lcd_print}
                             }},
        JSCFunctionListEntry{"println", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_lcd_println}
                             }},
        JSCFunctionListEntry{"getWidth", 0, JS_DEF_CFUNC, 0, {
                               func : {0, JS_CFUNC_generic, esp32_lcd_getWidth}
                             }},
        JSCFunctionListEntry{"getHeight", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_lcd_getHeight}
                             }},
        JSCFunctionListEntry{"getDepth", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_lcd_getDepth}
                             }},
        JSCFunctionListEntry{"fillScreen", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_lcd_fillScreen}
                             }},
    };

    static const JSCFunctionListEntry esp32_funcs[] = {
        JSCFunctionListEntry{"millis", 0, JS_DEF_CFUNC, 0, {
                               func : {0, JS_CFUNC_generic, esp32_millis}
                             }},
        JSCFunctionListEntry{"pinMode", 0, JS_DEF_CFUNC, 0, {
                               func : {2, JS_CFUNC_generic, esp32_gpio_mode}
                             }},
        JSCFunctionListEntry{
            "digitalRead", 0, JS_DEF_CFUNC, 0, {
              func : {1, JS_CFUNC_generic, esp32_gpio_digital_read}
            }},
        JSCFunctionListEntry{
            "digitalWrite", 0, JS_DEF_CFUNC, 0, {
              func : {2, JS_CFUNC_generic, esp32_gpio_digital_write}
            }},
        JSCFunctionListEntry{"deepSleep", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_deep_sleep}
                             }},
        JSCFunctionListEntry{"restart", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_restart}
                             }},
        JSCFunctionListEntry{"setLoop", 0, JS_DEF_CFUNC, 0, {
                               func : {1, JS_CFUNC_generic, esp32_set_loop}
                             }},
#ifdef ENABLE_WIFI
        JSCFunctionListEntry{"isWifiConnected", 0, JS_DEF_CFUNC, 0, {
                               func : {0, JS_CFUNC_generic, wifi_is_connected}
                             }},
        JSCFunctionListEntry{"fetch", 0, JS_DEF_CFUNC, 0, {
                               func : {2, JS_CFUNC_generic, http_fetch}
                             }},
#endif
    };

#ifndef GLOBAL_ESP32
    JSModuleDef *m =
        JS_NewCModule(ctx, "esp32", [](JSContext *ctx, JSModuleDef *m) {
          return JS_SetModuleExportList(
              ctx, m, esp32_funcs,
              sizeof(esp32_funcs) / sizeof(JSCFunctionListEntry));
        });
    if (m) {
      JS_AddModuleExportList(
          ctx, m, esp32_funcs,
          sizeof(esp32_funcs) / sizeof(JSCFunctionListEntry));
    }

    JSModuleDef *m2 =
        JS_NewCModule(ctx, "gpio", [](JSContext *ctx, JSModuleDef *m) {
          JS_SetModuleExport( ctx, m, "LOW", JS_NewUint32(ctx, 0x0) );
          JS_SetModuleExport( ctx, m, "HIGH", JS_NewUint32(ctx, 0x1) );
          JS_SetModuleExport( ctx, m, "INPUT", JS_NewUint32(ctx, 0x01) );
          JS_SetModuleExport( ctx, m, "OUTPUT", JS_NewUint32(ctx, 0x02) );
          JS_SetModuleExport( ctx, m, "PULLUP", JS_NewUint32(ctx, 0x04) );
          JS_SetModuleExport( ctx, m, "INPUT_PULLUP", JS_NewUint32(ctx, 0x05) );
          JS_SetModuleExport( ctx, m, "PULLDOWN", JS_NewUint32(ctx, 0x08) );
          JS_SetModuleExport( ctx, m, "INPUT_PULLDOWN", JS_NewUint32(ctx, 0x09) );
          JS_SetModuleExport( ctx, m, "OPEN_DRAIN", JS_NewUint32(ctx, 0x10) );
          JS_SetModuleExport( ctx, m, "OUTPUT_OPEN_DRAIN", JS_NewUint32(ctx, 0x12) );
          return JS_SetModuleExportList(
              ctx, m, gpio_funcs,
              sizeof(gpio_funcs) / sizeof(JSCFunctionListEntry));
        });
    if (m2) {
      JS_AddModuleExportList(
          ctx, m2, gpio_funcs,
          sizeof(gpio_funcs) / sizeof(JSCFunctionListEntry));
      JS_AddModuleExport( ctx, m2, "LOW");
      JS_AddModuleExport( ctx, m2, "HIGH");
      JS_AddModuleExport( ctx, m2, "INPUT");
      JS_AddModuleExport( ctx, m2, "OUTPUT");
      JS_AddModuleExport( ctx, m2, "PULLUP");
      JS_AddModuleExport( ctx, m2, "INPUT_PULLUP");
      JS_AddModuleExport( ctx, m2, "PULLDOWN");
      JS_AddModuleExport( ctx, m2, "INPUT_PULLDOWN");
      JS_AddModuleExport( ctx, m2, "OPEN_DRAIN");
      JS_AddModuleExport( ctx, m2, "OUTPUT_OPEN_DRAIN");
    }

    JSModuleDef *m3 =
        JS_NewCModule(ctx, "wire", [](JSContext *ctx, JSModuleDef *m) {
          return JS_SetModuleExportList(
              ctx, m, wire_funcs,
              sizeof(wire_funcs) / sizeof(JSCFunctionListEntry));
        });
    if (m3) {
      JS_AddModuleExportList(
          ctx, m3, wire_funcs,
          sizeof(wire_funcs) / sizeof(JSCFunctionListEntry));
    }

    JSModuleDef *m4 =
        JS_NewCModule(ctx, "lcd", [](JSContext *ctx, JSModuleDef *m) {
          return JS_SetModuleExportList(
              ctx, m, lcd_funcs,
              sizeof(lcd_funcs) / sizeof(JSCFunctionListEntry));
        });
    if (m4) {
      JS_AddModuleExportList(
          ctx, m4, lcd_funcs,
          sizeof(lcd_funcs) / sizeof(JSCFunctionListEntry));
    } 
#else
    // import * as esp32 from "esp32";
    JSValue esp32 = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, global, "esp32", esp32);
    JS_SetPropertyFunctionList(
        ctx, esp32, esp32_funcs,
        sizeof(esp32_funcs) / sizeof(JSCFunctionListEntry));
#endif
  }

  static JSValue console_log(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    for (int i = 0; i < argc; i++) {
      const char *str = JS_ToCString(ctx, argv[i]);
      if (str) {
        Serial.println(str);
        JS_FreeCString(ctx, str);
      }
    }
    return JS_UNDEFINED;
  }

  static JSValue set_timeout(JSContext *ctx, JSValueConst jsThis, int argc,
                             JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    uint32_t t;
    JS_ToUint32(ctx, &t, argv[1]);
    uint32_t id =
        qjs->timer.RegisterTimer(JS_DupValue(ctx, argv[0]), millis() + t);
    return JS_NewUint32(ctx, id);
  }

  static JSValue clear_timeout(JSContext *ctx, JSValueConst jsThis, int argc,
                               JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    uint32_t tid;
    JS_ToUint32(ctx, &tid, argv[0]);
    qjs->timer.RemoveTimer(tid);
    return JS_UNDEFINED;
  }

  static JSValue set_interval(JSContext *ctx, JSValueConst jsThis, int argc,
                              JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    uint32_t t;
    JS_ToUint32(ctx, &t, argv[1]);
    uint32_t id =
        qjs->timer.RegisterTimer(JS_DupValue(ctx, argv[0]), millis() + t, t);
    return JS_NewUint32(ctx, id);
  }

  static JSValue esp32_millis(JSContext *ctx, JSValueConst jsThis, int argc,
                              JSValueConst *argv) {
    return JS_NewUint32(ctx, millis());
  }

  static JSValue esp32_gpio_mode(JSContext *ctx, JSValueConst jsThis, int argc,
                                 JSValueConst *argv) {
    uint32_t pin, mode;
    JS_ToUint32(ctx, &pin, argv[0]);
    JS_ToUint32(ctx, &mode, argv[1]);
    pinMode(pin, mode);
    return JS_UNDEFINED;
  }

  static JSValue esp32_gpio_digital_read(JSContext *ctx, JSValueConst jsThis,
                                         int argc, JSValueConst *argv) {
    uint32_t pin;
    JS_ToUint32(ctx, &pin, argv[0]);
    return JS_NewUint32(ctx, digitalRead(pin));
  }

  static JSValue esp32_gpio_analog_read(JSContext *ctx, JSValueConst jsThis,
                                         int argc, JSValueConst *argv) {
    uint32_t pin;
    JS_ToUint32(ctx, &pin, argv[0]);
    return JS_NewUint32(ctx, analogRead(pin));
  }

  static JSValue esp32_gpio_digital_write(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t pin, value;
    JS_ToUint32(ctx, &pin, argv[0]);
    JS_ToUint32(ctx, &value, argv[1]);
    digitalWrite(pin, value);
    return JS_UNDEFINED;
  }

  static JSValue esp32_wire_begin(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    Wire.begin(32, 33);
    return JS_UNDEFINED;
  }

  static JSValue esp32_wire_requestFrom(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t address;
    uint32_t count;
    JS_ToUint32(ctx, &address, argv[0]);
    JS_ToUint32(ctx, &count, argv[1]);
    return JS_NewUint32(ctx, Wire.requestFrom((uint8_t)address, (uint8_t)count));
  }

  static JSValue esp32_wire_beginTransmission(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t address;
    JS_ToUint32(ctx, &address, argv[0]);
    Wire.beginTransmission((uint8_t)address);
    return JS_UNDEFINED;
  }

  static JSValue esp32_wire_endTransmission(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    return JS_NewUint32(ctx, Wire.endTransmission());
  }

  static JSValue esp32_wire_write(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);
    return JS_NewUint32(ctx, Wire.write((uint8_t)value));
  }

  static JSValue esp32_wire_write_buf(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    size_t size = 0;
    uint8_t *buf = JS_GetArrayBuffer(ctx, &size, argv[0]);
    for( size_t i = 0 ; i < size ; i++ ){
      if( Wire.write(buf[i]) != 1 )
        return JS_EXCEPTION;
    }
    return JS_NewUint32(ctx, size); 
  }

  static JSValue esp32_wire_available(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    return JS_NewUint32(ctx, Wire.available());
  }

  static JSValue esp32_wire_read(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    if( argc > 0 ){
        uint32_t value;
        JS_ToUint32(ctx, &value, argv[0]);
        JSValue array = JS_NewArray(ctx);
      for( uint32_t i = 0 ; i < value ; i++ ){
        int c = Wire.read();
        JS_SetPropertyUint32(ctx, array, i, c);
      }
      return array;
    }else{
      return JS_NewUint32(ctx, Wire.read());
    }
  }

  static JSValue esp32_lcd_setRotation(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);
    M5Lite.Lcd.setRotation(value);
    return JS_UNDEFINED;
  }

  static JSValue esp32_lcd_setBrigthness(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);
    M5Lite.Lcd.setBrightness(value);
    return JS_UNDEFINED;
  }

  static JSValue esp32_lcd_setTextColor(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);
    M5Lite.Lcd.setTextColor(value);
    return JS_UNDEFINED;
  }

  static JSValue esp32_lcd_setTextSize(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t value;
    JS_ToUint32(ctx, &value, argv[0]);
    M5Lite.Lcd.setTextSize(value);
    return JS_UNDEFINED;
  }

  static JSValue esp32_lcd_print(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    const char *text = JS_ToCString(ctx, argv[0]);
    M5Lite.Lcd.print(text);
    return JS_UNDEFINED;
  }

  static JSValue esp32_lcd_println(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    const char *text = JS_ToCString(ctx, argv[0]);
    M5Lite.Lcd.println(text);
    return JS_UNDEFINED;
  }

  static JSValue esp32_lcd_setCursor(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t value0;
    uint32_t value1;
    JS_ToUint32(ctx, &value0, argv[0]);
    JS_ToUint32(ctx, &value1, argv[1]);
    M5Lite.Lcd.setCursor(value0, value1);
    return JS_UNDEFINED;
  }

  static JSValue esp32_lcd_drawPixel(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t value0;
    uint32_t value1;
    uint32_t value2;
    JS_ToUint32(ctx, &value0, argv[0]);
    JS_ToUint32(ctx, &value1, argv[1]);
    JS_ToUint32(ctx, &value2, argv[2]);
    M5Lite.Lcd.drawPixel(value0, value1, value2);
    return JS_UNDEFINED;
  }

  static JSValue esp32_lcd_drawLine(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    uint32_t value0;
    uint32_t value1;
    uint32_t value2;
    uint32_t value3;
    uint32_t value4;
    JS_ToUint32(ctx, &value0, argv[0]);
    JS_ToUint32(ctx, &value1, argv[1]);
    JS_ToUint32(ctx, &value2, argv[2]);
    JS_ToUint32(ctx, &value3, argv[3]);
    JS_ToUint32(ctx, &value4, argv[4]);
    M5Lite.Lcd.drawLine(value0, value1, value2, value3, value4);
    return JS_UNDEFINED;
  }

  static JSValue esp32_lcd_getWidth(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    return JS_NewUint32(ctx, M5Lite.Lcd.width());
  }

  static JSValue esp32_lcd_getHeight(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    return JS_NewUint32(ctx, M5Lite.Lcd.height());
  }

  static JSValue esp32_lcd_getDepth(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    return JS_NewUint32(ctx, M5Lite.Lcd.getColorDepth());
  }

  static JSValue esp32_lcd_fillScreen(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    M5Lite.Lcd.fillScreen();
    return JS_UNDEFINED;
  }

  static JSValue esp32_deep_sleep(JSContext *ctx, JSValueConst jsThis, int argc,
                                  JSValueConst *argv) {
    uint32_t t;
    JS_ToUint32(ctx, &t, argv[0]);
    ESP.deepSleep(t);  // never return.
    return JS_UNDEFINED;
  }

  static JSValue esp32_restart(JSContext *ctx, JSValueConst jsThis,
                                          int argc, JSValueConst *argv) {
    ESP.restart();
    return JS_UNDEFINED;
  }

  static JSValue esp32_set_loop(JSContext *ctx, JSValueConst jsThis, int argc,
                                JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    qjs->setLoopFunc(JS_DupValue(ctx, argv[0]));
    return JS_UNDEFINED;
  }

#ifdef ENABLE_WIFI
  static JSValue wifi_is_connected(JSContext *ctx, JSValueConst jsThis,
                                   int argc, JSValueConst *argv) {
    return JS_NewBool(ctx, WiFi.status() == WL_CONNECTED);
  }

  static JSValue http_fetch(JSContext *ctx, JSValueConst jsThis, int argc,
                            JSValueConst *argv) {
    ESP32QuickJS *qjs = (ESP32QuickJS *)JS_GetContextOpaque(ctx);
    return qjs->httpFetcher.fetch(ctx, argv[0], argv[1]);
  }
#endif
};
