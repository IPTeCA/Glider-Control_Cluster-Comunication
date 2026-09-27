/*
   imu_control.ino
   IMU-based Flight Control Firmware
   Board: Seeed XIAO nRF52840 Sense
*/
#include <LSM6DS3.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <Servo.h>
#include <Arduino.h>
#include <nrf_timer.h>

//2026
//develop

// --- LED Control ---
#define LED_OFF  digitalWrite(LED_RED, HIGH);digitalWrite(LED_BLUE, HIGH);digitalWrite(LED_GREEN, HIGH);
#define RED_ON   digitalWrite(LED_RED, LOW);digitalWrite(LED_BLUE, HIGH);digitalWrite(LED_GREEN, HIGH);
#define BLUE_ON  digitalWrite(LED_RED, HIGH);digitalWrite(LED_BLUE, LOW);digitalWrite(LED_GREEN, HIGH);
#define GREEN_ON digitalWrite(LED_RED, HIGH);digitalWrite(LED_BLUE, HIGH);digitalWrite(LED_GREEN, LOW);
#define CYAN_ON  digitalWrite(LED_RED, HIGH);digitalWrite(LED_BLUE, LOW);digitalWrite(LED_GREEN, LOW);
#define YELLOW_ON digitalWrite(LED_RED, LOW);digitalWrite(LED_BLUE, HIGH);digitalWrite(LED_GREEN, LOW);

Servo servo[3];
#define MEASURING_FREQ (30)
LSM6DS3 IMU(I2C_MODE, 0x6A);
Madgwick m_;

typedef struct { float x, y, z; } pos3d_t;
pos3d_t gyr_ = {0}, acc_ = {0}, ang_ = {0};

unsigned long t=0, t_=0;

// --- Configurable Parameters ---
float kp_pit = 2.0; float kd_pit = 3.0; float ki_pit = 0.0;
float kp_rol = 2.0; float kd_rol = 3.0; float ki_rol = 0.0;
int ang_goal = 0;

// Trim (L, R, E)
int ang_trim[] = {85, 105, 70}; 

// --- System Variables ---
float cur_servo_ang[3] = {85, 105, 70}; 
#define SERVO_STEP_LIMIT 3.0 

float preangx=0, preangy=0, dx, dy, ix=0, iy=0, T;
int val[3] = {0,0,0};
int mode = 2; // Start in Manual mode
bool param_changed = false;

// Auto Boost & Launch Detect
unsigned long launch_time = 0;
bool launch_detected = false;
bool gain_boosted = false;
const float LAUNCH_THRESHOLD = -3.0;       
const unsigned long BOOST_DELAY_MS = 1000; 

// UART
HardwareSerial& OUT = Serial1; 
const unsigned OUT_BAUD = 460800;  // Maximum baud rate for UART1
const unsigned DBG_BAUD = 115200;
uint32_t seq = 0;
unsigned long prev_ms = 0;

// Header Definition
const char* HDR_STR = "HDR,1,GLDR,fields=dt_ms,ax,ay,az,gx,gy,gz,roll,pitch,yaw,s0,s1,s2,rate=30";

// --- Interrupt Control Variables ---
volatile bool control_ready = false;
volatile unsigned long control_t = 0;
volatile unsigned long control_t_prev = 0;
volatile uint8_t control_pending = 0; // Count of pending control executions
NRF_TIMER_Type* control_timer = NRF_TIMER1;
const uint32_t CONTROL_INTERVAL_US = 1000000 / MEASURING_FREQ; // 33.33ms for 30Hz

// Timer Interrupt Handler for Control Loop
extern "C" {
  void TIMER1_IRQHandler(void) {
    if (control_timer->EVENTS_COMPARE[0]) {
      control_timer->EVENTS_COMPARE[0] = 0;
      control_timer->TASKS_CLEAR = 1;
      control_t = millis();
      control_ready = true;
      if (control_pending < 255) control_pending++; // Track missed interrupts
    }
  }
}

void setupControlTimer() {
  // Configure Timer1 for periodic interrupts
  control_timer->MODE = TIMER_MODE_MODE_Timer;
  control_timer->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  control_timer->PRESCALER = 4; // 16MHz / 2^4 = 1MHz (1us resolution)
  control_timer->CC[0] = CONTROL_INTERVAL_US;
  control_timer->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  control_timer->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
  
  NVIC_SetPriority(TIMER1_IRQn, 1); // High priority for control
  NVIC_EnableIRQ(TIMER1_IRQn);
  control_timer->TASKS_START = 1;
}

// Non-blocking send function: sends only if buffer has enough space
static void sendNonBlocking(const char* str) {
  int n = strlen(str);
  if (n > 0 && OUT.availableForWrite() >= n + 2) {  // +2 for \r\n
    OUT.write((uint8_t*)str, n);
    OUT.write("\r\n", 2);
  }
  // If not enough space, discard (servo priority)
}

void setup() {
  Serial.begin(DBG_BAUD);
  OUT.begin(OUT_BAUD);

  IMU.settings.gyroRange = 2000;
  IMU.settings.accelRange = 4;
  while (IMU.begin() != 0){ LED_OFF; delay(200); RED_ON; delay(200); }

  m_.begin((int)MEASURING_FREQ);
  servo[0].attach(0); servo[1].attach(1); servo[2].attach(2);

  // Initialize servo positions
  cur_servo_ang[0] = ang_trim[0];
  cur_servo_ang[1] = ang_trim[1];
  cur_servo_ang[2] = ang_trim[2];

  delay(100);
  
  // Setup timer interrupt for control loop
  setupControlTimer();
  control_t_prev = millis();
  
  // Send Header once at startup
  Serial.println(HDR_STR); sendNonBlocking(HDR_STR);
  Serial.println("LOG,mode:Manual"); sendNonBlocking("LOG,mode:Manual");
  CYAN_ON;
}

void sendParamLog() {
  char buf[256];
  int n = snprintf(buf, sizeof(buf), 
    "LOG,Param:Pp=%.2f,Dp=%.2f,Ip=%.2f,Pr=%.2f,Dr=%.2f,Ir=%.2f,G=%d,T0=%d,T1=%d,T2=%d", 
    kp_pit, kd_pit, ki_pit, kp_rol, kd_rol, ki_rol, ang_goal,
    ang_trim[0], ang_trim[1], ang_trim[2]);
  if (n > 0) {
    Serial.println(buf);
    sendNonBlocking(buf);
  }
  param_changed = false;
}

void printSaveCode() {
  char buf[256];
  Serial.println("\n\n// !!! COPY AND PASTE THESE LINES INTO YOUR CODE !!!");
  snprintf(buf, sizeof(buf), "float kp_pit = %.2f; float kd_pit = %.2f; float ki_pit = %.2f;", kp_pit, kd_pit, ki_pit);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "float kp_rol = %.2f; float kd_rol = %.2f; float ki_rol = %.2f;", kp_rol, kd_rol, ki_rol);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "int ang_goal = %d;", ang_goal);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "int ang_trim[] = {%d, %d, %d};", ang_trim[0], ang_trim[1], ang_trim[2]);
  Serial.println(buf);
  Serial.println("// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
  sendNonBlocking("LOG,Code printed to Serial Monitor.");
}

void processCommand(char c) {
  // --- Mode Change ---
  if (c=='A' || c=='a') { 
    mode=1; GREEN_ON; 
    ix=0; iy=0; 
    launch_detected = false; 
    gain_boosted = false;
    Serial.println("LOG,mode:Auto (Waiting for Launch...)"); sendNonBlocking("LOG,mode:Auto (Waiting for Launch...)"); 
  }
  else if (c=='M' || c=='m') { 
    mode=2; CYAN_ON;  
    Serial.println("LOG,mode:Manual"); sendNonBlocking("LOG,mode:Manual"); 
  }
  
  // --- Gains (PID) ---
  else if (c=='p') { kp_pit -= 0.1; param_changed=true; } else if (c=='P') { kp_pit += 0.1; param_changed=true; }
  else if (c=='d') { kd_pit -= 0.1; param_changed=true; } else if (c=='D') { kd_pit += 0.1; param_changed=true; }
  else if (c=='i') { ki_pit -= 0.01; param_changed=true; } else if (c=='I') { ki_pit += 0.01; param_changed=true; }
  else if (c=='x') { kp_rol -= 0.1; param_changed=true; } else if (c=='X') { kp_rol += 0.1; param_changed=true; }
  else if (c=='y') { kd_rol -= 0.1; param_changed=true; } else if (c=='Y') { kd_rol += 0.1; param_changed=true; }
  else if (c=='z') { ki_rol -= 0.01; param_changed=true; } else if (c=='Z') { ki_rol += 0.01; param_changed=true; }
  else if (c=='g') { ang_goal -= 1; param_changed=true; } else if (c=='G') { ang_goal += 1; param_changed=true; }
  
  // --- Trim (L, R, E) +/- 1 deg ---
  else if (c=='l') { ang_trim[0] = constrain(ang_trim[0] - 1, 15, 165); param_changed=true; } 
  else if (c=='L') { ang_trim[0] = constrain(ang_trim[0] + 1, 15, 165); param_changed=true; }
  else if (c=='r') { ang_trim[1] = constrain(ang_trim[1] - 1, 15, 165); param_changed=true; } 
  else if (c=='R') { ang_trim[1] = constrain(ang_trim[1] + 1, 15, 165); param_changed=true; }
  else if (c=='e') { ang_trim[2] = constrain(ang_trim[2] - 1, 15, 165); param_changed=true; } 
  else if (c=='E') { ang_trim[2] = constrain(ang_trim[2] + 1, 15, 165); param_changed=true; }

  // --- Center Command ---
  else if (c=='c' || c=='C') {
    ang_trim[0] = 90; ang_trim[1] = 90; ang_trim[2] = 90; 
    param_changed=true; 
  }
  
  // --- Save Code ---
  else if (c=='s' || c=='S') {
    printSaveCode();
  }

  // --- Header Resend (Handshake & Reset) ---
  else if (c=='h' || c=='H') {
    seq = 0; // Reset sequence counter
    Serial.println(HDR_STR); 
    sendNonBlocking(HDR_STR);
    sendParamLog(); 
  }
}

void updateServoSmooth(int id, float target_angle) {
  if (target_angle < 15.0) target_angle = 15.0;
  if (target_angle > 165.0) target_angle = 165.0;

  float diff = target_angle - cur_servo_ang[id];
  if (diff > SERVO_STEP_LIMIT) diff = SERVO_STEP_LIMIT;
  if (diff < -SERVO_STEP_LIMIT) diff = -SERVO_STEP_LIMIT;

  cur_servo_ang[id] += diff;
  servo[id].write((int)cur_servo_ang[id]);
}

// Control loop executed in interrupt context (called from ISR or main loop when ready)
void executeControlLoop() {
  unsigned long t_now = control_t;
  unsigned long t_prev = control_t_prev;
  control_t_prev = t_now;
  
  T = (t_now - t_prev); if(T<=0) T=1;
  
  // Read IMU
  acc_.x = IMU.readFloatAccelX(); acc_.y = IMU.readFloatAccelY(); acc_.z = IMU.readFloatAccelZ();
  gyr_.x = IMU.readFloatGyroX();  gyr_.y = IMU.readFloatGyroY();  gyr_.z = IMU.readFloatGyroZ();
  m_.updateIMU(gyr_.x, gyr_.y, gyr_.z, acc_.x, acc_.y, acc_.z);
  ang_.x = m_.getRoll(); ang_.y = m_.getPitch(); ang_.z = m_.getYaw();

  if (mode == 1){ 
    // --- Auto Mode ---
    if (!launch_detected) {
      if (acc_.x < LAUNCH_THRESHOLD) {
        launch_detected = true;
        launch_time = t_now; 
        Serial.println("LOG,Launch Detected!"); sendNonBlocking("LOG,Launch Detected!");
      }
    } 
    else if (!gain_boosted && (t_now - launch_time > BOOST_DELAY_MS)) {
      kp_rol *= 2.0; kd_rol *= 2.0; ki_rol *= 2.0;
      gain_boosted = true;
      YELLOW_ON; 
      Serial.println("LOG,Boost: Roll Gains Doubled!"); sendNonBlocking("LOG,Boost: Roll Gains Doubled!");
      sendParamLog();
    }

    dx = (ang_.x - preangx)/T; dy = (ang_.y - preangy)/T;
    ix += (ang_.x + preangx)*T/2000.0; iy += (ang_.y + preangy)*T/2000.0;

    // Control (Aileron Reversed)
    float roll_control = (ang_.x) * kp_rol - dx * kd_rol + ix * ki_rol;
    
    float target0 = ang_trim[0] - roll_control; 
    float target1 = ang_trim[1] - roll_control; 
    float target2 = ang_trim[2] + (ang_.y - ang_goal) * kp_pit + dy * kd_pit + iy * ki_pit;
    
    updateServoSmooth(0, target0);
    updateServoSmooth(1, target1);
    updateServoSmooth(2, target2);

    preangx = ang_.x; preangy = ang_.y;
  }
  else if (mode == 2){
    // Manual Mode
    updateServoSmooth(0, (float)ang_trim[0]);
    updateServoSmooth(1, (float)ang_trim[1]);
    updateServoSmooth(2, (float)ang_trim[2]);
    ix = 0; iy = 0; preangx = ang_.x; preangy = ang_.y;
  }

  // Update time for data logging
  t = t_now;
  t_ = t_prev;
}

void loop() {
  // Priority 1: Execute control loop FIRST (highest priority)
  // This ensures control is never blocked by UART communication
  if (control_ready) {
    control_ready = false; // Clear flag
    if (control_pending > 0) control_pending--; // Decrement pending counter
    
    executeControlLoop();
    
    // Send data log (but don't block if UART is busy)
    // Use non-blocking writes to prevent control delay
    float dt_ms = (float)(t - prev_ms); prev_ms = t;
    char line[128];
    
    int n = snprintf(line, sizeof(line), "DAT,%lu,%lu,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
      ++seq, t, dt_ms, 
      acc_.x, acc_.y, acc_.z, 
      gyr_.x, gyr_.y, gyr_.z, 
      ang_.x, ang_.y, ang_.z,
      cur_servo_ang[0], cur_servo_ang[1], cur_servo_ang[2]);
    
    // Non-blocking UART writes (only send if buffer has space)
    if (Serial.availableForWrite() > 50) Serial.println(line);
    if (OUT.availableForWrite() > 50) OUT.println(line);
    
    // If control was missed, execute immediately again
    if (control_pending > 0) {
      control_pending = 0; // Reset to prevent accumulation
      // Time has passed, so use current time
      control_t_prev = control_t;
      control_t = millis();
      control_ready = true; // Re-trigger
    }
    return; // Exit immediately after control to maintain priority
  }

  // Priority 2: Process commands (non-blocking, limited processing)
  static uint8_t cmd_count = 0;
  cmd_count++;
  if (cmd_count >= 10) { // Process commands every 10 loop iterations to reduce overhead
    cmd_count = 0;
    while (Serial.available() && control_pending == 0) {
      processCommand(Serial.read());
      if (Serial.available() > 20) break; // Limit command processing
    }
    while (OUT.available() && control_pending == 0) {
      processCommand(OUT.read());
      if (OUT.available() > 20) break; // Limit command processing
    }
  }

  // Priority 3: Send parameter log if changed (non-critical)
  if (param_changed && control_pending == 0) {
    sendParamLog();
  }

  // Priority 4: Periodic parameter log (non-critical)
  static unsigned long last_info_ms = 0;
  unsigned long t_now = millis();
  if ((t_now - last_info_ms > 2000) && control_pending == 0) {
    last_info_ms = t_now;
    sendParamLog();
  }
  
  // Small delay to prevent busy-waiting (but keep it minimal)
  if (control_pending == 0) delay(1);
}