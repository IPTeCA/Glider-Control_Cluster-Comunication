/*
   parent_uart_bridge.ino
   Airframe-side ESP32 wireless bridge:
   - Receives HDR/DAT/LOG text lines from XIAO over UART and packs into binary ESP-NOW frames.
   - Forwards ground commands received via ESP-NOW to XIAO over UART.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_mac.h>
#include <esp_err.h>
#include <Preferences.h>

#ifndef LED_PIN
#  ifdef LED_BUILTIN
#    define LED_PIN LED_BUILTIN
#  else
#    define LED_PIN 2
#  endif
#endif

#define UART_BAUD   460800 // Maximum baud rate for UART1
#define UART_PORT   Serial1
#ifndef D7
  #define D7 20
#endif
#ifndef D6
  #define D6 21
#endif
#define UART_RX_PIN D7
#define UART_TX_PIN D6
#define WIFI_CHANNEL 1

// Child ESP32 (ground / PC side) STA MAC address
const uint8_t peerMac[6] = { 0x58,0x8C,0x81,0xA4,0xC1,0x54 }; 

const int CHANNEL = 1;
#define LINE_MAX     384
#define PAYLOAD_MAX  220

// Protocol definitions (includes FT_LOG)
enum : uint8_t { FT_HDR=1, FT_DATA=2, FT_HB=3, FT_LOG=4 };

struct __attribute__((packed)) NowFrameHdr {
  uint8_t ver, type; uint16_t node_id; uint32_t tx_seq; uint16_t payload_len, crc16;
};
struct __attribute__((packed)) PayloadHDR { uint16_t schema_id, fields_len; };
struct __attribute__((packed)) PayloadDATAHead { uint16_t schema_id; uint32_t src_seq, t_ms; uint16_t value_count; };
struct __attribute__((packed)) PayloadHB { uint32_t tx_ms; uint16_t q_depth; uint16_t inflight; };

struct __attribute__((packed)) QueueItem { uint8_t type; uint16_t len; uint8_t payload[PAYLOAD_MAX]; };
#define Q_CAP 256
static QueueItem qbuf[Q_CAP];
static uint16_t  q_head=0, q_tail=0;
static uint32_t  q_dropped=0;

static uint32_t g_tx_seq = 0;
static uint16_t g_schema_id = 0;
static String   g_fields_csv = "";
static bool     g_have_hdr = false;
volatile bool g_send_ok=false, g_send_fail=false;
volatile int  g_inflight=0;
const    int  MAX_INFLIGHT = 6;

static uint32_t last_tx_ok_ms = 0;
static uint32_t last_tx_attempt_ms = 0;
static uint32_t last_hb_ms = 0;
static uint32_t last_hdr_ms = 0;
static uint32_t next_reinit_allowed_ms = 0;
static uint32_t reinit_backoff_ms = 500;
static const uint32_t reinit_backoff_max = 8000;
Preferences prefs;

// Utilities
static inline uint16_t q_count(){ return (uint16_t)((q_head - q_tail) & (Q_CAP-1)); }
static bool queue_push(uint8_t type, const uint8_t* payload, uint16_t len){
  uint16_t next = (q_head + 1) & (Q_CAP-1);
  if (next == q_tail){ q_tail = (q_tail + 1) & (Q_CAP-1); q_dropped++; }
  QueueItem& it = qbuf[q_head]; it.type = type; it.len = len; memcpy(it.payload, payload, len);
  q_head = next; return true;
}
static bool queue_pop(QueueItem& out){
  if (q_head == q_tail) return false;
  out = qbuf[q_tail]; q_tail = (q_tail + 1) & (Q_CAP-1); return true;
}
static uint16_t crc16_ccitt(const uint8_t* data, size_t len){
  uint16_t crc = 0xFFFF;
  for(size_t i=0;i<len;i++){ crc ^= (uint16_t)data[i] << 8; for(int b=0;b<8;b++) crc = (crc & 0x8000) ? (crc<<1)^0x1021 : (crc<<1); }
  return crc;
}
static uint16_t hash16_fnv1a(const String& s){
  uint32_t h = 2166136261u; for(size_t i=0;i<s.length();++i){ h ^= (uint8_t)s[i]; h *= 16777619u; } return (uint16_t)((h>>16) ^ (h & 0xFFFF));
}

// Callbacks
void onSend(const uint8_t*, esp_now_send_status_t s){
  if (s==ESP_NOW_SEND_SUCCESS){ g_send_ok=true; last_tx_ok_ms = millis(); }
  else { g_send_fail=true; }
  if (g_inflight>0) g_inflight--;
}

void onRecvFromPC(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == 2 && incomingData[0] == 0xFE) {
    char cmd = (char)incomingData[1];
    UART_PORT.write(cmd);
  }
}

// Transmission handling
static bool tx_now(uint8_t type, const uint8_t* payload, uint16_t plen){
  if (plen > PAYLOAD_MAX) return false;
  if (g_inflight >= MAX_INFLIGHT)  return false;

  NowFrameHdr h{}; h.ver=1; h.type=type; h.node_id=1; h.tx_seq=++g_tx_seq; h.payload_len=plen; h.crc16=0;
  uint8_t buf[sizeof(NowFrameHdr) + PAYLOAD_MAX];
  memcpy(buf, &h, sizeof(h));
  memcpy(buf+sizeof(h), payload, plen);
  ((NowFrameHdr*)buf)->crc16 = crc16_ccitt(buf, sizeof(h)+plen);

  last_tx_attempt_ms = millis();
  esp_err_t e = esp_now_send(peerMac, buf, sizeof(h)+plen);
  if (e != ESP_OK) return false;
  g_inflight++;
  return true;
}

// UART parser
static void enqueueHDRNow();
static void saveSchemaToNVS(){
  prefs.begin("glink", false); prefs.putString("fields", g_fields_csv); prefs.putUShort("schema", g_schema_id); prefs.end();
}
static bool enqueueHDR(const char* line){
  const char* p = strstr(line, "fields="); if(!p) return false;
  String rest = String(p+7); rest.trim();
  int pos = rest.indexOf("rate="); if (pos>0){ int c = rest.lastIndexOf(',', pos); if (c>=0) rest = rest.substring(0, c); }
  g_fields_csv = rest; g_schema_id  = hash16_fnv1a(g_fields_csv); g_have_hdr = true;
  saveSchemaToNVS(); enqueueHDRNow(); return true;
}
static bool enqueueDAT(const char* line){
  if(!g_have_hdr) return false;
  char buf[LINE_MAX]; strncpy(buf, line, LINE_MAX-1); buf[LINE_MAX-1]=0;
  char* save=nullptr; char* tok=strtok_r(buf, ",", &save);
  if(!tok || strcmp(tok,"DAT")!=0) return false;
  tok = strtok_r(nullptr, ",", &save); if(!tok) return false;
  uint32_t src_seq = strtoul(tok, nullptr, 10);
  tok = strtok_r(nullptr, ",", &save); if(!tok) return false;
  uint32_t t_ms = (uint32_t)(atof(tok)+0.5f);
  float values[64]; int n=0;
  while((tok=strtok_r(nullptr, ",", &save)) && n<64){ values[n++] = atof(tok); }
  uint8_t payload[PAYLOAD_MAX]; size_t off=0;
  PayloadDATAHead dh{}; dh.schema_id=g_schema_id; dh.src_seq=src_seq; dh.t_ms=t_ms; dh.value_count=(uint16_t)n;
  memcpy(payload+off, &dh, sizeof(dh)); off+=sizeof(dh);
  memcpy(payload+off, values, n*sizeof(float)); off+= n*sizeof(float);
  return queue_push(FT_DATA, payload, off);
}
// LOG packet enqueue
static bool enqueueLOG(const char* line){
    size_t len = strlen(line);
    if (len > PAYLOAD_MAX) len = PAYLOAD_MAX;
    return queue_push(FT_LOG, (const uint8_t*)line, len);
}

// Peer management and NVS storage
static void ensurePeer(){
  if (!esp_now_is_peer_exist(peerMac)){
    esp_now_peer_info_t p{}; memcpy(p.peer_addr, peerMac, 6);
    p.ifidx = WIFI_IF_STA; p.channel = CHANNEL; p.encrypt = false;
    esp_now_add_peer(&p);
  }
}
static void enqueueHB(){ PayloadHB hb{ (uint32_t)millis(), (uint16_t)q_count(), (uint16_t)g_inflight }; queue_push(FT_HB, (uint8_t*)&hb, sizeof(hb)); }
static void enqueueHDRNow(){
  if (!g_have_hdr) return;
  const uint16_t len = g_fields_csv.length();
  uint8_t payload[4 + 256]; PayloadHDR ph{}; ph.schema_id = g_schema_id; ph.fields_len = len;
  memcpy(payload, &ph, sizeof(ph)); memcpy(payload+sizeof(ph), g_fields_csv.c_str(), len);
  queue_push(FT_HDR, payload, sizeof(ph)+len); last_hdr_ms = millis();
}
static bool loadSchemaFromNVS(){
  prefs.begin("glink", true); String f = prefs.getString("fields", ""); uint16_t s = prefs.getUShort("schema", 0); prefs.end();
  if (f.length()==0 || s==0) return false;
  uint16_t s_calc = hash16_fnv1a(f); if (s != s_calc) s = s_calc;
  g_fields_csv = f; g_schema_id = s; g_have_hdr = true;
  enqueueHDRNow(); return true;
}

void setup(){
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  delay(200);

  UART_PORT.setRxBufferSize(4096);
  UART_PORT.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  WiFi.mode(WIFI_STA);
  ESP_ERROR_CHECK( esp_wifi_start() );
  delay(50);
  ESP_ERROR_CHECK( esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE) );
  WiFi.setSleep(false);

  ESP_ERROR_CHECK( esp_now_init() );
  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onRecvFromPC);
  
  ensurePeer();
  loadSchemaFromNVS();
  last_tx_ok_ms = 0; last_tx_attempt_ms = 0; next_reinit_allowed_ms = millis();
}

void loop(){
  static char line[LINE_MAX]; static size_t idx=0;
  while (UART_PORT.available()){
    int c = UART_PORT.read(); if (c<0) break;
    if (c=='\r' || c=='\n'){
      if (idx>0){
        line[idx]=0;
        char* s=line; while(*s==' ' || *s=='\t') ++s;
        if      (strncmp(s,"HDR,",4)==0){ enqueueHDR(s); }
        else if (strncmp(s,"DAT,",4)==0){ enqueueDAT(s); }
        else if (strncmp(s,"LOG,",4)==0){ enqueueLOG(s); } // Enqueue LOG lines
        idx=0;
      }
      continue;
    }
    if (idx < LINE_MAX-1) line[idx++]=(char)c;
  }

  // Deadlock detection and recovery
  if (g_inflight > 0 && (millis() - last_tx_attempt_ms > 1000)) { g_inflight = 0; }

  // Periodic tasks (heartbeat & periodic HDR resend)
  if (millis() - last_hb_ms >= 1000){ enqueueHB(); last_hb_ms = millis(); }
  if (g_have_hdr && millis() - last_hdr_ms > 5000){ enqueueHDRNow(); }

  // Transmit queued frames
  if (g_inflight < MAX_INFLIGHT){
    QueueItem it;
    while (g_inflight < MAX_INFLIGHT && queue_pop(it)){
      if (!tx_now(it.type, it.payload, it.len)) break;
    }
  }

  // Process transmission results
  if (g_send_ok)   { g_send_ok=false; }
  if (g_send_fail) { g_send_fail=false; }

  // Link reconnection and backoff
  const uint32_t now = millis();
  bool link_stalled = (now - last_tx_ok_ms > 2000) && (now - last_tx_attempt_ms < 1000);
  if (link_stalled && now >= next_reinit_allowed_ms){
    esp_now_deinit(); esp_wifi_stop(); delay(50); esp_wifi_start(); delay(50);
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE); esp_now_init();
    esp_now_register_send_cb(onSend);
    esp_now_register_recv_cb(onRecvFromPC); 
    ensurePeer(); if(g_have_hdr) enqueueHDRNow();
    next_reinit_allowed_ms = millis() + reinit_backoff_ms;
    reinit_backoff_ms = min<uint32_t>(reinit_backoff_ms*2, reinit_backoff_max);
  }
  
  // LED status indicator
  static uint32_t lastBlink=0; static uint8_t phase=0;
  bool connected = (now - last_tx_ok_ms < 2000);
  if (g_inflight >= MAX_INFLIGHT) {
     if (now - lastBlink > 50) { lastBlink=now; phase^=1; digitalWrite(LED_PIN, phase); }
  } else if (connected) { 
     digitalWrite(LED_PIN, HIGH); 
  } else { 
     if(now-lastBlink>500){ lastBlink=now; phase^=1; digitalWrite(LED_PIN, phase); } 
  }
  delay(1);
}