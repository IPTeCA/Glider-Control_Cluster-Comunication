/*
  child_uart_bridge.ino
  Ground-side ESP32 wireless bridge: Receives ESP-NOW telemetry packets and outputs CSV lines to PC.
  Passes all commands from PC (serial) directly to the parent ESP32 via ESP-NOW.
*/
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#ifndef LED_PIN
#  ifdef LED_BUILTIN
#    define LED_PIN LED_BUILTIN
#  else
#    define LED_PIN 2
#  endif
#endif

#define SERIAL_BAUD 115200
const int CHANNEL = 1;

// Protocol definitions
enum : uint8_t { FT_HDR=1, FT_DATA=2, FT_HB=3, FT_LOG=4 };
struct __attribute__((packed)) NowFrameHdr{
  uint8_t ver, type; uint16_t node_id; uint32_t tx_seq; uint16_t payload_len, crc16;
};

// ==========================================
// Settings: Parent ESP32 (airframe side) STA MAC address
// If unknown, {0xFF,0xFF,...} (broadcast) can be used,
// but specifying the exact MAC address is recommended.
const uint8_t parentMac[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; 
// ==========================================

// Check if destination is broadcast
bool isBroadcast = true;

void onRecv(const uint8_t* mac_addr, const uint8_t* data, int len);

void setup(){
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  Serial.begin(SERIAL_BAUD);
  delay(200);

  // Determine broadcast mode if MAC address is all zeros
  isBroadcast = true;
  for(int i=0; i<6; i++) { if(parentMac[i]!=0) isBroadcast=false; }
  // Also treat as broadcast if explicitly set to {0xFF, ...}
  if(parentMac[0]==0xFF) isBroadcast=true;

  WiFi.mode(WIFI_STA);
  ESP_ERROR_CHECK( esp_wifi_start() );
  esp_wifi_set_max_tx_power(84); 

  ESP_ERROR_CHECK( esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE) );
  WiFi.setSleep(false);

  ESP_ERROR_CHECK( esp_now_init() );
  esp_now_register_recv_cb(onRecv);
  
  // Peer registration
  esp_now_peer_info_t p{}; 
  if (isBroadcast) {
     const uint8_t bc[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
     memcpy(p.peer_addr, bc, 6);
  } else {
     memcpy(p.peer_addr, parentMac, 6);
  }
  p.channel = CHANNEL; 
  p.ifidx = WIFI_IF_STA; 
  p.encrypt = false;
  
  if (esp_now_add_peer(&p) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer Added");
  }

  Serial.println("Child Sink Ready (Pass-All)");
}

void sendCmdToParent(char c){
  uint8_t buf[2] = { 0xFE, (uint8_t)c };
  if (isBroadcast) {
    const uint8_t bc[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_send(bc, buf, 2);
  } else {
    esp_now_send(parentMac, buf, 2);
  }
}

void loop(){
  // Input from USB serial
  if (Serial.available()){
    char c = Serial.read();
    
    // Forward all non-newline characters to parent
    if (c != '\r' && c != '\n') {
       sendCmdToParent(c); 
       
       // Feedback LED (brief blink)
       digitalWrite(LED_PIN, HIGH);
       delay(10);
       digitalWrite(LED_PIN, LOW);
    }
  }
  delay(1);
}

// ESP-NOW packet receive callback
void onRecv(const uint8_t* mac, const uint8_t* data, int len){
  if(len < (int)sizeof(NowFrameHdr)) return;
  const NowFrameHdr* h = (const NowFrameHdr*)data;
  const uint8_t* p = data + sizeof(NowFrameHdr);
  
  // Turn on LED upon receiving data
  digitalWrite(LED_PIN, HIGH);
  
  if(h->type==FT_HDR){
    Serial.print("HDR,1,GLDR,fields=");
    for(int i=4; i<h->payload_len; i++) Serial.write(p[i]);
    Serial.println();
  }else if(h->type==FT_DATA){
    uint32_t src_seq; memcpy(&src_seq, p+2, 4);
    uint32_t t_ms;    memcpy(&t_ms,    p+6, 4);
    uint16_t count;   memcpy(&count,   p+10,2);
    const float* vals = (const float*)(p + 12);

    Serial.print("DAT,"); Serial.print(src_seq);
    Serial.print(',');    Serial.print(t_ms);
    for(int i=0;i<count;i++){ Serial.print(','); Serial.print(vals[i], 3); }
    Serial.println();
  }else if(h->type==FT_LOG){
    Serial.write(p, h->payload_len);
    Serial.println();
  }
  digitalWrite(LED_PIN, LOW);
}