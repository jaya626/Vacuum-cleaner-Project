/* =====================================================================
   ESP32 SMART AUTOMATIC VACUUM CLEANER  (ESP32-WROOM-32, Arduino IDE 2.x)
   ---------------------------------------------------------------------
   * Single file. The ESP32 hosts the dashboard (no internet / CDN needed).
   * Two modes: MANUAL and AUTO CLEAN.
   * Non-blocking millis() state machines, settings saved in NVS (Preferences).
   * Works with arduino-esp32 core 2.x and 3.x (LEDC API handled below).
   * Libraries: WiFi, WebServer, Preferences (bundled) + ESP32Servo.
   ===================================================================== */
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESP32Servo.h>

/* =====================================================================
   USER CONFIGURATION — CHANGE VALUES HERE
   Format:  value | unit | description | useful/safe range
   Values marked [NVS] are the FACTORY DEFAULTS of settings that can also be
   edited live in the web Settings page and saved permanently.
   ===================================================================== */

// ---- Wi-Fi (credentials can also be changed in Settings -> Wi-Fi) ----
#define WIFI_SSID_DEFAULT  "ROG"      // text | 2.4 GHz network name       | 1-32 chars
#define WIFI_PASS_DEFAULT  "ASUSROG9025"  // text | network password           | 0-63 chars
const int32_t D_WIFI_RECONNECT_MS = 10000;  // [NVS] ms | retry interval while disconnected | 3000-60000

// ---- GPIO map (EXACT hardware map - do not change unless you rewire) ----
const uint8_t PIN_ENA      = 25;  // L298N ENA (left PWM)   - remove ENA jumper
const uint8_t PIN_IN1      = 26;  // L298N IN1
const uint8_t PIN_IN2      = 27;  // L298N IN2
const uint8_t PIN_ENB      = 14;  // L298N ENB (right PWM)  - remove ENB jumper
const uint8_t PIN_IN3      = 16;  // L298N IN3
const uint8_t PIN_IN4      = 17;  // L298N IN4
const uint8_t PIN_SERVO    = 18;  // SG90 signal (carries FRONT ultrasonic)
const uint8_t PIN_RELAY    = 19;  // Vacuum relay IN2
const uint8_t PIN_LED_R    = 4;   // RGB red
const uint8_t PIN_LED_G    = 22;  // RGB green
const uint8_t PIN_LED_B    = 23;  // RGB blue
const uint8_t PIN_BUZZER   = 32;  // Buzzer
const uint8_t PIN_IR_REAR  = 39;  // Rear IR (VN/SVN, input only)
const uint8_t PIN_TRIG_F   = 21;  // Front TRIG
const uint8_t PIN_ECHO_F   = 34;  // Front ECHO (input only, needs voltage divider to 3.3 V)
const uint8_t PIN_TRIG_L   = 33;  // Left TRIG
const uint8_t PIN_ECHO_L   = 35;  // Left ECHO  (input only, needs voltage divider)
const uint8_t PIN_TRIG_R   = 5;   // Right TRIG
const uint8_t PIN_ECHO_R   = 36;  // Right ECHO (VP/SVP, input only, needs voltage divider)

// ---- Compile-time hardware constants ----
const uint32_t MOTOR_PWM_HZ   = 2000;  // Hz | L298N PWM frequency            | 500-20000
const uint8_t  MOTOR_PWM_BITS = 8;     // bit| PWM resolution (0-255)         | keep 8
const int      SERVO_MIN_US   = 500;   // us | SG90 pulse for 0 deg           | 400-600
const int      SERVO_MAX_US   = 2400;  // us | SG90 pulse for 180 deg         | 2300-2600

// ---- General ----
const int32_t D_AUTO_VACUUM      = 1;    // [NVS] bool | AUTO CLEAN turns the vacuum on (with 2 s beep)  | 0-1

// ---- Motor ----
const int32_t D_SPD_NORMAL       = 150;  // [NVS] PWM | auto forward speed        | 50-255
const int32_t D_SPD_TURN         = 130;  // [NVS] PWM | auto turning speed         | 50-255
const int32_t D_SPD_REV          = 120;  // [NVS] PWM | auto reverse speed         | 50-255
const int32_t D_SPD_MANUAL       = 150;  // [NVS] PWM | manual top speed (slider)  | 50-255
const int32_t D_SPD_LIMIT        = 100;  // [NVS] %   | overall speed cap applied to EVERY motor output (lower it if the robot is too fast) | 20-100
const int32_t D_MIN_PWM          = 60;  // [NVS] PWM | lowest PWM that moves the robot (manual) | 0-150
const int32_t D_INV_LEFT         = 0;    // [NVS] bool| invert left motor direction (wiring fix)   | 0-1
const int32_t D_INV_RIGHT        = 0;    // [NVS] bool| invert right motor direction (wiring fix)  | 0-1

// ---- Vacuum ----
const int32_t D_RELAY_LOW_ON     = 1;    // [NVS] bool| 1 = relay ON when pin LOW (RELAY_ON=LOW, OFF=HIGH), 0 = ON when HIGH | 0-1

// ---- Buzzer ----
const int32_t D_BZ_ENABLE        = 1;    // [NVS] bool| buzzer master enable                | 0-1
const int32_t D_BZ_FREQ          = 2700; // [NVS] Hz  | normal tone frequency               | 200-8000
const int32_t D_BZ_SHORT_MS      = 150;  // [NVS] ms  | short beep length                   | 50-1000
const int32_t D_BZ_LONG_MS       = 600;  // [NVS] ms  | long beep (emergency STOP, tests)   | 200-3000
const int32_t D_BZ_MANUAL_MS     = 1500; // [NVS] ms  | manual command beep                 | 200-5000
const int32_t D_BZ_VAC_MS        = 2000; // [NVS] ms  | vacuum warning beep before relay ON | 500-5000
const int32_t D_BZ_GAP_MS        = 120;  // [NVS] ms  | silence between repeated beeps      | 50-1000
const int32_t D_BZ_START_CNT     = 3;    // [NVS] cnt | power-on beeps                      | 1-10
const int32_t D_BZ_WIFI_CNT      = 2;    // [NVS] cnt | Wi-Fi connected beeps               | 1-10
const int32_t D_BZ_RETRY_MS      = 5000; // [NVS] ms  | Wi-Fi disconnected beep interval    | 1000-60000
const int32_t D_BZ_EV_START      = 1;    // [NVS] bool| beep at power on                    | 0-1
const int32_t D_BZ_EV_WIFI       = 1;    // [NVS] bool| beep for Wi-Fi events               | 0-1
const int32_t D_BZ_EV_OBST       = 1;    // [NVS] bool| beep when an obstacle is detected   | 0-1
const int32_t D_BZ_EV_MANUAL     = 1;    // [NVS] bool| beep on a new manual command        | 0-1
const int32_t D_BZ_EV_VAC        = 1;    // [NVS] bool| beep before vacuum starts          | 0-1
const int32_t D_BZ_EV_FAULT      = 1;    // [NVS] bool| fault / emergency warning tone     | 0-1
const int32_t D_BZ_FAULT_FREQ    = 1800; // [NVS] Hz  | fault warning tone frequency        | 200-8000
const int32_t D_BZ_FAULT_CNT     = 3;    // [NVS] cnt | fault warning beeps                 | 1-10
const int32_t D_BZ_FAULT_MS      = 250;  // [NVS] ms  | each fault beep length              | 50-1000

// ---- RGB LED ----
const int32_t D_RGB_COMMON_CATH  = 1;    // [NVS] bool| 1 = common cathode (HIGH = on), 0 = common anode | 0-1
const int32_t D_RGB_BLINK_MS     = 500;  // [NVS] ms  | Wi-Fi disconnected blue blink half-period | 100-2000
const int32_t D_RGB_START_MS     = 1000; // [NVS] ms  | each startup colour (R, G, B)       | 200-3000
const int32_t D_RGB_OBST_MS      = 250;  // [NVS] ms  | green obstacle blink half-period    | 100-1000

// ---- Servo (SG90 is about 0-180 deg, NOT 270) ----
const int32_t D_SV_LEFT          = 10;   // [NVS] deg | safe LEFT limit (swap with right if mounted mirrored) | 0-180
const int32_t D_SV_CENTER        = 90;   // [NVS] deg | CENTER (boot position)           | 0-180
const int32_t D_SV_RIGHT         = 170;  // [NVS] deg | safe RIGHT limit                 | 0-180
const int32_t D_SV_HOME          = 90;   // [NVS] deg | HOME position used by test button | 0-180
const int32_t D_SV_SETTLE        = 300;  // [NVS] ms  | wait after every servo move       | 100-1500

// ---- Ultrasonic ----
const int32_t D_US_TIMEOUT_US    = 30000;// [NVS] us  | echo timeout (30000 us = about 5 m) | 5000-60000
const int32_t D_US_INTERVAL      = 100;  // [NVS] ms  | time for one full cycle of all 3 sensors | 60-500
const int32_t D_SCAN_SAMPLES     = 3;    // [NVS] cnt | readings taken at each scan angle | 1-7
const int32_t D_SCAN_WAIT_MS     = 1200; // [NVS] ms  | max wait for scan readings        | 400-4000

// ---- Sensor filtering ----
const int32_t D_US_SAMPLES       = 3;    // [NVS] cnt | median window                     | 1-7
const int32_t D_US_MIN_CM        = 2;    // [NVS] cm  | readings below are invalid        | 1-10
const int32_t D_US_MAX_CM        = 400;  // [NVS] cm  | readings above are invalid        | 100-600
const int32_t D_IR_ACTIVE        = 0;    // [NVS] lvl | rear IR level that means OBSTACLE (0=LOW,1=HIGH) | 0-1

// ---- Obstacle avoidance (AUTO) ----
const int32_t D_FRONT_CM         = 20;   // [NVS] cm  | front obstacle threshold          | 10-100
const int32_t D_EMERG_CM         = 10;   // [NVS] cm  | emergency distance (no reverse)   | 3-50
const int32_t D_SIDE_CM          = 15;   // [NVS] cm  | side obstacle threshold           | 5-80
const int32_t D_REV_MS           = 500;  // [NVS] ms  | normal reverse time               | 100-3000
const int32_t D_TURN_MS          = 500;  // [NVS] ms  | normal turn time (about 90 deg, time-based) | 100-3000
const int32_t D_EMERG_TURN_MS    = 700;  // [NVS] ms  | turn time after emergency scan    | 100-3000
const int32_t D_RESUME_MS        = 800;  // [NVS] ms  | forward time before side corrections resume | 100-3000
const int32_t D_OBS_CONFIRM      = 2;    // [NVS] cnt | consecutive front readings to confirm obstacle | 1-10
const int32_t D_REV_LONG_MS      = 1000; // [NVS] ms  | longer reverse when both sides blocked | 200-5000
const int32_t D_TURN_RECOV_MS    = 1400; // [NVS] ms  | recovery turn (about 180 deg, time-based) | 500-4000
const int32_t D_BLOCKED_CM       = 20;   // [NVS] cm  | scan distance below this = side blocked | 5-100
const int32_t D_CORR_TURN_MS     = 250;  // [NVS] ms  | small side-obstacle correction    | 80-1000
const int32_t D_CORR_COOL_MS     = 700;  // [NVS] ms  | minimum gap between corrections   | 100-5000
const int32_t D_STRATEGY         = 0;    // [NVS] 0 clearer side, 1 prefer left, 2 prefer right | 0-2
const int32_t D_TIE_MARGIN       = 5;    // [NVS] cm  | sides this close count as equal   | 0-50
const int32_t D_TIE_DEFAULT      = 0;    // [NVS] 0 left, 1 right | tie-break for CLEARER_SIDE | 0-1
const int32_t D_STOP_SETTLE_MS   = 200;  // [NVS] ms  | pause after stopping              | 50-1000

// ---- Manual safety / joystick ----
const int32_t D_MAN_ASSIST       = 1;    // [NVS] bool| MANUAL_OBSTACLE_ASSIST: block FORWARD below emergency distance | 0-1
const int32_t D_MAN_REAR_ASSIST  = 1;    // [NVS] bool| block REVERSE when rear IR sees obstacle | 0-1
const int32_t D_JOY_DEAD         = 15;   // [NVS] %   | joystick dead zone                | 0-60
const int32_t D_JOY_SENS         = 100;  // [NVS] %   | joystick sensitivity              | 30-150
const int32_t D_MAN_WATCHDOG_MS  = 600;  // [NVS] ms  | stop if no manual command arrives | 200-5000


// ---- Hardware tests ----
const int32_t D_MOTOR_TEST_MS    = 1500; // [NVS] ms  | motor test duration               | 500-5000
const int32_t D_MOTOR_TEST_SPD   = 130;  // [NVS] PWM | motor test speed                  | 60-200
const int32_t D_VAC_TEST_MS      = 3000; // [NVS] ms  | vacuum relay test duration        | 1000-10000
const int32_t D_RGB_TEST_MS      = 2000; // [NVS] ms  | RGB colour test duration          | 500-10000

/* ===================== END OF USER CONFIGURATION ===================== */

// LEDC API selection (arduino-esp32 core 3.x changed it)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define LEDC_NEW_API 1
#else
  #define LEDC_NEW_API 0
  const uint8_t LEDC_CH_ENA = 10, LEDC_CH_ENB = 11, LEDC_CH_BUZZ = 12;
#endif

#define NVS_NS "vacuum"
#define US_BUF_MAX 7

// ===================== PERSISTENT SETTINGS =====================
struct Cfg {
  int32_t autoVacuum, wifiReconMs;
  int32_t spdNormal, spdTurn, spdRev, spdManual, spdLimit, minPwm, invLeft, invRight;
  int32_t relayLowOn;
  int32_t bzEnable, bzFreq, bzShortMs, bzLongMs, bzManualMs, bzVacMs, bzGapMs, bzStartCnt, bzWifiCnt, bzRetryMs;
  int32_t bzEvStart, bzEvWifi, bzEvObst, bzEvManual, bzEvVac, bzEvFault, bzFaultFreq, bzFaultCnt, bzFaultMs;
  int32_t rgbCommonCath, rgbBlinkMs, rgbStartMs, rgbObstMs;
  int32_t svLeft, svCenter, svRight, svHome, svSettle;
  int32_t usTimeoutUs, usInterval, scanSamples, scanWaitMs;
  int32_t usSamples, usMinCm, usMaxCm, irActive;
  int32_t frontCm, emergCm, sideCm, revMs, turnMs, emergTurnMs, resumeMs, obsConfirm, revLongMs, turnRecovMs;
  int32_t blockedCm, corrTurnMs, corrCoolMs, strategy, tieMargin, tieDefault, stopSettleMs;
  int32_t manAssist, manRearAssist, joyDead, joySens, manWatchdogMs;
  int32_t motorTestMs, motorTestSpd, vacTestMs, rgbTestMs;
};
Cfg cfg;

struct SetDef {
  const char* key; const char* sec; const char* label; const char* unit;
  int32_t mn, mx, df; int32_t* p; uint8_t type; const char* opts;
};
#define SD(key,sec,label,unit,mn,mx,df,field,type,opts) {key,sec,label,unit,mn,mx,df,&cfg.field,type,opts}

// type: 0 number, 1 on/off, 2 choice (opts separated by |)
const SetDef SETTINGS[] = {
  SD("autoVacuum","General","Vacuum ON in AUTO CLEAN","",0,1,D_AUTO_VACUUM,autoVacuum,1,""),
  SD("wifiReconMs","Wi-Fi","Reconnect interval","ms",3000,60000,D_WIFI_RECONNECT_MS,wifiReconMs,0,""),
  SD("spdNormal","Motor","Normal speed (auto)","PWM",50,255,D_SPD_NORMAL,spdNormal,0,""),
  SD("spdTurn","Motor","Turn speed (auto)","PWM",50,255,D_SPD_TURN,spdTurn,0,""),
  SD("spdRev","Motor","Reverse speed (auto)","PWM",50,255,D_SPD_REV,spdRev,0,""),
  SD("spdManual","Motor","Manual top speed","PWM",50,255,D_SPD_MANUAL,spdManual,0,""),
  SD("spdLimit","Motor","Overall speed cap","%",20,100,D_SPD_LIMIT,spdLimit,0,""),
  SD("minPwm","Motor","Minimum moving PWM (manual)","PWM",0,150,D_MIN_PWM,minPwm,0,""),
  SD("invLeft","Motor","Invert left motors","",0,1,D_INV_LEFT,invLeft,1,""),
  SD("invRight","Motor","Invert right motors","",0,1,D_INV_RIGHT,invRight,1,""),
  SD("relayLowOn","Vacuum","Relay ON when pin LOW","",0,1,D_RELAY_LOW_ON,relayLowOn,1,""),
  SD("bzEnable","Buzzer","Buzzer master enable","",0,1,D_BZ_ENABLE,bzEnable,1,""),
  SD("bzFreq","Buzzer","Tone frequency","Hz",200,8000,D_BZ_FREQ,bzFreq,0,""),
  SD("bzShortMs","Buzzer","Short beep","ms",50,1000,D_BZ_SHORT_MS,bzShortMs,0,""),
  SD("bzLongMs","Buzzer","Long beep","ms",200,3000,D_BZ_LONG_MS,bzLongMs,0,""),
  SD("bzManualMs","Buzzer","Manual command beep","ms",200,5000,D_BZ_MANUAL_MS,bzManualMs,0,""),
  SD("bzVacMs","Buzzer","Vacuum warning beep","ms",500,5000,D_BZ_VAC_MS,bzVacMs,0,""),
  SD("bzGapMs","Buzzer","Gap between beeps","ms",50,1000,D_BZ_GAP_MS,bzGapMs,0,""),
  SD("bzStartCnt","Buzzer","Power-on beeps","count",1,10,D_BZ_START_CNT,bzStartCnt,0,""),
  SD("bzWifiCnt","Buzzer","Wi-Fi connected beeps","count",1,10,D_BZ_WIFI_CNT,bzWifiCnt,0,""),
  SD("bzRetryMs","Buzzer","Wi-Fi disconnected beep interval","ms",1000,60000,D_BZ_RETRY_MS,bzRetryMs,0,""),
  SD("bzEvStart","Buzzer","Event: power on","",0,1,D_BZ_EV_START,bzEvStart,1,""),
  SD("bzEvWifi","Buzzer","Event: Wi-Fi","",0,1,D_BZ_EV_WIFI,bzEvWifi,1,""),
  SD("bzEvObst","Buzzer","Event: obstacle","",0,1,D_BZ_EV_OBST,bzEvObst,1,""),
  SD("bzEvManual","Buzzer","Event: manual command","",0,1,D_BZ_EV_MANUAL,bzEvManual,1,""),
  SD("bzEvVac","Buzzer","Event: vacuum ON","",0,1,D_BZ_EV_VAC,bzEvVac,1,""),
  SD("bzEvFault","Buzzer","Event: fault / emergency","",0,1,D_BZ_EV_FAULT,bzEvFault,1,""),
  SD("bzFaultFreq","Buzzer","Fault tone frequency","Hz",200,8000,D_BZ_FAULT_FREQ,bzFaultFreq,0,""),
  SD("bzFaultCnt","Buzzer","Fault tone beeps","count",1,10,D_BZ_FAULT_CNT,bzFaultCnt,0,""),
  SD("bzFaultMs","Buzzer","Fault beep length","ms",50,1000,D_BZ_FAULT_MS,bzFaultMs,0,""),
  SD("rgbCommonCath","RGB","Common cathode LED","",0,1,D_RGB_COMMON_CATH,rgbCommonCath,1,""),
  SD("rgbBlinkMs","RGB","Wi-Fi blue blink period","ms",100,2000,D_RGB_BLINK_MS,rgbBlinkMs,0,""),
  SD("rgbStartMs","RGB","Startup colour time","ms",200,3000,D_RGB_START_MS,rgbStartMs,0,""),
  SD("rgbObstMs","RGB","Obstacle green blink period","ms",100,1000,D_RGB_OBST_MS,rgbObstMs,0,""),
  SD("svLeft","Servo","Left limit","deg",0,180,D_SV_LEFT,svLeft,0,""),
  SD("svCenter","Servo","Center","deg",0,180,D_SV_CENTER,svCenter,0,""),
  SD("svRight","Servo","Right limit","deg",0,180,D_SV_RIGHT,svRight,0,""),
  SD("svHome","Servo","Home","deg",0,180,D_SV_HOME,svHome,0,""),
  SD("svSettle","Servo","Settle time","ms",100,1500,D_SV_SETTLE,svSettle,0,""),
  SD("usTimeoutUs","Ultrasonic","Echo timeout","us",5000,60000,D_US_TIMEOUT_US,usTimeoutUs,0,""),
  SD("usInterval","Ultrasonic","Update interval (all sensors)","ms",60,500,D_US_INTERVAL,usInterval,0,""),
  SD("scanSamples","Ultrasonic","Scan readings per angle","count",1,7,D_SCAN_SAMPLES,scanSamples,0,""),
  SD("scanWaitMs","Ultrasonic","Scan max wait","ms",400,4000,D_SCAN_WAIT_MS,scanWaitMs,0,""),
  SD("usSamples","Sensor Filtering","Median window","count",1,7,D_US_SAMPLES,usSamples,0,""),
  SD("usMinCm","Sensor Filtering","Minimum valid distance","cm",1,10,D_US_MIN_CM,usMinCm,0,""),
  SD("usMaxCm","Sensor Filtering","Maximum valid distance","cm",100,600,D_US_MAX_CM,usMaxCm,0,""),
  SD("irActive","Sensor Filtering","Rear IR obstacle level","",0,1,D_IR_ACTIVE,irActive,2,"LOW = obstacle|HIGH = obstacle"),
  SD("frontCm","Obstacle Avoidance","Front obstacle distance","cm",10,100,D_FRONT_CM,frontCm,0,""),
  SD("emergCm","Obstacle Avoidance","Emergency distance","cm",3,50,D_EMERG_CM,emergCm,0,""),
  SD("sideCm","Obstacle Avoidance","Side obstacle distance","cm",5,80,D_SIDE_CM,sideCm,0,""),
  SD("revMs","Obstacle Avoidance","Reverse time","ms",100,3000,D_REV_MS,revMs,0,""),
  SD("turnMs","Obstacle Avoidance","Turn time","ms",100,3000,D_TURN_MS,turnMs,0,""),
  SD("emergTurnMs","Obstacle Avoidance","Emergency turn time","ms",100,3000,D_EMERG_TURN_MS,emergTurnMs,0,""),
  SD("resumeMs","Obstacle Avoidance","Resume time","ms",100,3000,D_RESUME_MS,resumeMs,0,""),
  SD("obsConfirm","Obstacle Avoidance","Obstacle confirmations","count",1,10,D_OBS_CONFIRM,obsConfirm,0,""),
  SD("revLongMs","Obstacle Avoidance","Long reverse (both blocked)","ms",200,5000,D_REV_LONG_MS,revLongMs,0,""),
  SD("turnRecovMs","Obstacle Avoidance","Recovery turn (about 180 deg)","ms",500,4000,D_TURN_RECOV_MS,turnRecovMs,0,""),
  SD("blockedCm","Obstacle Avoidance","Scan: side blocked below","cm",5,100,D_BLOCKED_CM,blockedCm,0,""),
  SD("corrTurnMs","Obstacle Avoidance","Side correction time","ms",80,1000,D_CORR_TURN_MS,corrTurnMs,0,""),
  SD("corrCoolMs","Obstacle Avoidance","Side correction gap","ms",100,5000,D_CORR_COOL_MS,corrCoolMs,0,""),
  SD("strategy","Obstacle Avoidance","Avoidance strategy","",0,2,D_STRATEGY,strategy,2,"Clearer side|Prefer left|Prefer right"),
  SD("tieMargin","Obstacle Avoidance","Nearly-equal margin","cm",0,50,D_TIE_MARGIN,tieMargin,0,""),
  SD("tieDefault","Obstacle Avoidance","Clearer-side tie break","",0,1,D_TIE_DEFAULT,tieDefault,2,"Left|Right"),
  SD("stopSettleMs","Obstacle Avoidance","Stop settle time","ms",50,1000,D_STOP_SETTLE_MS,stopSettleMs,0,""),
  SD("manAssist","Manual Safety","Front obstacle assist","",0,1,D_MAN_ASSIST,manAssist,1,""),
  SD("manRearAssist","Manual Safety","Rear IR reverse assist","",0,1,D_MAN_REAR_ASSIST,manRearAssist,1,""),
  SD("joyDead","Manual Safety","Joystick dead zone","%",0,60,D_JOY_DEAD,joyDead,0,""),
  SD("joySens","Manual Safety","Joystick sensitivity","%",30,150,D_JOY_SENS,joySens,0,""),
  SD("manWatchdogMs","Manual Safety","Command timeout (auto stop)","ms",200,5000,D_MAN_WATCHDOG_MS,manWatchdogMs,0,""),
  SD("motorTestMs","Hardware Test","Motor test duration","ms",500,5000,D_MOTOR_TEST_MS,motorTestMs,0,""),
  SD("motorTestSpd","Hardware Test","Motor test speed","PWM",60,200,D_MOTOR_TEST_SPD,motorTestSpd,0,""),
  SD("vacTestMs","Hardware Test","Vacuum relay test","ms",1000,10000,D_VAC_TEST_MS,vacTestMs,0,""),
  SD("rgbTestMs","Hardware Test","RGB colour test","ms",500,10000,D_RGB_TEST_MS,rgbTestMs,0,"")
};
const size_t N_SET = sizeof(SETTINGS) / sizeof(SETTINGS[0]);

// ===================== ENUMS / STRUCTS =====================
enum CurMode : uint8_t { MODE_MANUAL = 0, MODE_AUTO = 1 };
enum SensorId : uint8_t { S_FRONT = 0, S_LEFT = 1, S_RIGHT = 2, S_REAR = 3 };
enum VacState : uint8_t { VAC_OFF = 0, VAC_BEEP = 1, VAC_ON = 2 };
enum BootStep : uint8_t { B_R, B_G, B_B, B_BEEP, B_DONE };
enum AutoState : uint8_t {
  A_IDLE, A_VAC_START, A_FORWARD, A_STOP, A_EMERGENCY_SCAN, A_REVERSE, A_SCAN,
  A_TURN_LEFT, A_TURN_RIGHT, A_CORRECT_LEFT, A_CORRECT_RIGHT, A_RESUME
};
enum ScanStep : uint8_t { SC_IDLE, SC_LEFT_MOVE, SC_LEFT_READ, SC_CENTER1, SC_RIGHT_MOVE, SC_RIGHT_READ, SC_CENTER2, SC_DONE };
enum ScanOwner : uint8_t { OWN_NONE, OWN_AUTO, OWN_TEST };
enum ManClass : uint8_t { MC_STOP, MC_FWD, MC_BACK, MC_LEFT, MC_RIGHT };
enum BzPrio : uint8_t { PRIO_LOW = 1, PRIO_START = 2, PRIO_MANUAL = 3, PRIO_VAC = 4, PRIO_FAULT = 5 };

struct Ultra {
  uint8_t trig, echo;
  float buf[US_BUF_MAX];
  uint8_t head, count;
  float dist; bool distValid;
  uint32_t seq;
};
struct Scanner { ScanStep step; uint32_t t; float left, right; bool leftOk, rightOk; };
struct BuzzState { uint8_t remain; uint16_t onMs, offMs, freq; bool on; uint32_t ts; uint8_t prio; };

// ===================== GLOBALS =====================
Preferences prefs;
WebServer server(80);
Servo frontServo;
String wifiSsid, wifiPass;
bool serverStarted = false, wifiUp = false, wifiApplyNew = false, bootDone = false;
uint32_t wifiBeepT = 0, wifiTryT = 0;

CurMode curMode = MODE_MANUAL;
bool emergency = false;
int curL = 0, curR = 0;

VacState vacState = VAC_OFF;
uint32_t vacBeepEnd = 0, vacTestEnd = 0;

BuzzState bz = {0, 0, 0, 0, false, 0, 0};

BootStep bootStep = B_R;
uint32_t bootT = 0;
uint32_t tRgbEnd = 0; uint8_t tRgbMask = 0;

int svAngle = 90; uint32_t svLastMove = 0;
Scanner scn = {SC_IDLE, 0, 0, 0, false, false};
ScanOwner scanOwner = OWN_NONE;
float lastScanL = -1, lastScanR = -1;

Ultra us[3];
uint32_t usLastTick = 0; uint8_t usNext = 0;
bool irObstacle = false, irRaw = false; uint8_t irCnt = 0;
uint32_t irLast = 0;


AutoState autoState = A_IDLE;
uint32_t autoT = 0, autoLastCorr = 0, autoSeq = 0;
uint32_t autoRevMs = 500, autoTurnMs = 500;
uint8_t obsCnt = 0, autoBlockedTries = 0;
bool autoEmerg = false, autoPaused = false;

float manX = 0, manY = 0; bool manActive = false; uint8_t manClass = MC_STOP;
uint32_t manLast = 0, manTickT = 0, manBlockUntil = 0; bool manBlockLatch = false;

bool tMotorActive = false; uint32_t tMotorEnd = 0;

// ===================== PROTOTYPES =====================
void stopMotors();
void vacuumOff();
bool vacuumRequestOn();
void buzzStart(uint8_t count, uint16_t onMs, uint16_t offMs, uint16_t freq, uint8_t prio);
void buzzStop();
void servoGo(int a);
void scanCancel();
void usReset(uint8_t i);
void autoGoto(AutoState s);
void emergencyStop();
void setMode(CurMode m);
void testCancel();
void routesSetup();
void wifiBegin();

// ===================== HELPERS =====================
static inline int32_t iclamp(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float fclampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline bool due(uint32_t t, uint32_t ms) { return (uint32_t)(millis() - t) >= ms; }

static float medianOf(float* a, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) { float k = a[i]; int8_t j = i - 1; while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; j--; } a[j + 1] = k; }
  return (n & 1) ? a[n / 2] : (a[n / 2 - 1] + a[n / 2]) * 0.5f;
}
String jsonEsc(const String& in) {
  String o; o.reserve(in.length() + 4);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

// ===================== MOTORS =====================
void motorPwmWrite(bool left, uint8_t duty) {
#if LEDC_NEW_API
  ledcWrite(left ? PIN_ENA : PIN_ENB, duty);
#else
  ledcWrite(left ? LEDC_CH_ENA : LEDC_CH_ENB, duty);
#endif
}
void setMotor(bool left, int spd) {
  spd = (int)iclamp(spd, -255, 255);
  if (left) curL = spd; else curR = spd;
  int phys = (left ? cfg.invLeft : cfg.invRight) ? -spd : spd;
  uint8_t a = left ? PIN_IN1 : PIN_IN3, b = left ? PIN_IN2 : PIN_IN4;
  if (phys > 0)      { digitalWrite(a, HIGH); digitalWrite(b, LOW); }
  else if (phys < 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); }
  else               { digitalWrite(a, LOW);  digitalWrite(b, LOW); }
  int duty = (phys < 0 ? -phys : phys) * (int)cfg.spdLimit / 100;   // overall speed cap
  motorPwmWrite(left, (uint8_t)iclamp(duty, 0, 255));
}
void stopMotors()  { setMotor(true, 0); setMotor(false, 0); }
void forward()     { setMotor(true, cfg.spdNormal); setMotor(false, cfg.spdNormal); }
void reverse()     { setMotor(true, -cfg.spdRev);   setMotor(false, -cfg.spdRev); }
void turnLeft()    { setMotor(true, -cfg.spdTurn);  setMotor(false, cfg.spdTurn); }
void turnRight()   { setMotor(true, cfg.spdTurn);   setMotor(false, -cfg.spdTurn); }
void softLeft()    { setMotor(true, cfg.spdNormal / 3); setMotor(false, cfg.spdNormal); }
void softRight()   { setMotor(true, cfg.spdNormal);     setMotor(false, cfg.spdNormal / 3); }

// ===================== VACUUM =====================
void relayWrite(bool on) {
  uint8_t onLvl = cfg.relayLowOn ? LOW : HIGH;
  digitalWrite(PIN_RELAY, on ? onLvl : (uint8_t)!onLvl);
}
bool buzzerAllows(int32_t evFlag) { return cfg.bzEnable && evFlag; }
bool vacuumRequestOn() {
  if (emergency || vacState != VAC_OFF) return false;
  if (buzzerAllows(cfg.bzEvVac)) {
    buzzStart(1, (uint16_t)cfg.bzVacMs, 0, (uint16_t)cfg.bzFreq, PRIO_VAC);
    vacBeepEnd = millis() + (uint32_t)cfg.bzVacMs;
    vacState = VAC_BEEP;
  } else { relayWrite(true); vacState = VAC_ON; }
  return true;
}
void vacuumOff() {
  if (vacState == VAC_BEEP) buzzStop();
  relayWrite(false); vacState = VAC_OFF; vacTestEnd = 0;
}
void vacuumTick() {
  if (vacState == VAC_BEEP && (int32_t)(millis() - vacBeepEnd) >= 0) { relayWrite(true); vacState = VAC_ON; }
  if (vacTestEnd && (int32_t)(millis() - vacTestEnd) >= 0) vacuumOff();
}

// ===================== BUZZER =====================
void buzzTone(uint32_t f) {
#if LEDC_NEW_API
  ledcWriteTone(PIN_BUZZER, f);
#else
  ledcWriteTone(LEDC_CH_BUZZ, f);
#endif
}
bool buzzBusy() { return bz.remain > 0; }
void buzzStop() { bz.remain = 0; bz.on = false; bz.prio = 0; buzzTone(0); }
void buzzStart(uint8_t count, uint16_t onMs, uint16_t offMs, uint16_t freq, uint8_t prio) {
  if (!cfg.bzEnable || count == 0) return;
  if (bz.remain > 0 && prio < bz.prio) return;
  bz.remain = count; bz.onMs = onMs; bz.offMs = offMs; bz.freq = freq; bz.prio = prio;
  bz.on = true; bz.ts = millis(); buzzTone(freq);
}
void buzzTick() {
  if (bz.remain == 0) return;
  uint32_t now = millis();
  if (bz.on) {
    if ((uint32_t)(now - bz.ts) >= bz.onMs) {
      buzzTone(0); bz.on = false; bz.ts = now;
      if (--bz.remain == 0) { bz.prio = 0; return; }
    }
  } else if ((uint32_t)(now - bz.ts) >= bz.offMs) { bz.on = true; bz.ts = now; buzzTone(bz.freq); }
}
void beepShort(uint8_t prio) { if (cfg.bzEnable) buzzStart(1, (uint16_t)cfg.bzShortMs, 0, (uint16_t)cfg.bzFreq, prio); }

// ===================== RGB =====================
void rgbWrite(bool r, bool g, bool b) {
  uint8_t on = cfg.rgbCommonCath ? HIGH : LOW, off = cfg.rgbCommonCath ? LOW : HIGH;
  digitalWrite(PIN_LED_R, r ? on : off);
  digitalWrite(PIN_LED_G, g ? on : off);
  digitalWrite(PIN_LED_B, b ? on : off);
}
bool obstacleActive() {
  if (curMode == MODE_AUTO) {
    switch (autoState) {
      case A_STOP: case A_EMERGENCY_SCAN: case A_REVERSE: case A_SCAN: case A_TURN_LEFT:
      case A_TURN_RIGHT: case A_CORRECT_LEFT: case A_CORRECT_RIGHT: return true;
      default: return false;
    }
  }
  return (int32_t)(manBlockUntil - millis()) > 0;
}
void rgbTick() {
  if (!bootDone) return;
  uint32_t now = millis();
  if (tRgbMask && (int32_t)(tRgbEnd - now) > 0) { rgbWrite(tRgbMask & 1, tRgbMask & 2, tRgbMask & 4); return; }
  tRgbMask = 0;
  if (emergency) { rgbWrite(true, false, false); return; }
  if (obstacleActive()) { rgbWrite(false, ((now / (uint32_t)cfg.rgbObstMs) & 1) == 0, false); return; }
  if (WiFi.status() == WL_CONNECTED) rgbWrite(false, true, false);
  else rgbWrite(false, false, ((now / (uint32_t)cfg.rgbBlinkMs) & 1) == 0);
}

// ===================== SERVO / SCANNER =====================
int servoClamp(int a) {
  a = (int)iclamp(a, 0, 180);
  int lo = cfg.svLeft < cfg.svRight ? cfg.svLeft : cfg.svRight;
  int hi = cfg.svLeft < cfg.svRight ? cfg.svRight : cfg.svLeft;
  return (int)iclamp(a, lo, hi);
}
void servoGo(int a) { a = servoClamp(a); svAngle = a; frontServo.write(a); svLastMove = millis(); }
bool scanActive() { return scn.step != SC_IDLE && scn.step != SC_DONE; }
void scanStart() {
  stopMotors();                       // never scan while moving
  scn.step = SC_LEFT_MOVE; scn.t = millis(); scn.leftOk = scn.rightOk = false;
  servoGo(cfg.svLeft);
}
void scanCancel() {
  if (scn.step != SC_IDLE) { scn.step = SC_IDLE; servoGo(cfg.svCenter); usReset(0); }
  scanOwner = OWN_NONE;
}
void scanReset() { scn.step = SC_IDLE; scanOwner = OWN_NONE; }
bool scanReadReady() {
  uint8_t need = (uint8_t)cfg.scanSamples;
  if (need > cfg.usSamples) need = (uint8_t)cfg.usSamples;
  return us[0].count >= need || due(scn.t, (uint32_t)cfg.scanWaitMs);
}
void scanTick() {
  if (scn.step == SC_IDLE || scn.step == SC_DONE) return;
  uint32_t settle = (uint32_t)cfg.svSettle;
  switch (scn.step) {
    case SC_LEFT_MOVE:  if (due(scn.t, settle)) { usReset(0); scn.step = SC_LEFT_READ; scn.t = millis(); } break;
    case SC_LEFT_READ:  if (scanReadReady()) { scn.leftOk = us[0].distValid; scn.left = us[0].dist; servoGo(cfg.svCenter); scn.step = SC_CENTER1; scn.t = millis(); } break;
    case SC_CENTER1:    if (due(scn.t, settle)) { servoGo(cfg.svRight); scn.step = SC_RIGHT_MOVE; scn.t = millis(); } break;
    case SC_RIGHT_MOVE: if (due(scn.t, settle)) { usReset(0); scn.step = SC_RIGHT_READ; scn.t = millis(); } break;
    case SC_RIGHT_READ: if (scanReadReady()) { scn.rightOk = us[0].distValid; scn.right = us[0].dist; servoGo(cfg.svCenter); scn.step = SC_CENTER2; scn.t = millis(); } break;
    case SC_CENTER2:    if (due(scn.t, settle)) { usReset(0); scn.step = SC_DONE; } break;
    default: break;
  }
}
void scanStoreResults() { lastScanL = scn.leftOk ? scn.left : -1; lastScanR = scn.rightOk ? scn.right : -1; }

// ===================== ULTRASONIC =====================
void usReset(uint8_t i) { us[i].head = 0; us[i].count = 0; us[i].dist = -1; us[i].distValid = false; }
bool usMeasure(uint8_t i, float& cm) {
  digitalWrite(us[i].trig, LOW); delayMicroseconds(3);
  digitalWrite(us[i].trig, HIGH); delayMicroseconds(10);
  digitalWrite(us[i].trig, LOW);
  unsigned long d = pulseIn(us[i].echo, HIGH, (unsigned long)cfg.usTimeoutUs);
  if (d == 0) return false;                    // timeout = INVALID (never 0 cm)
  cm = (float)d * 0.01715f;
  return cm >= cfg.usMinCm && cm <= cfg.usMaxCm;
}
void usPush(uint8_t i, bool ok, float cm) {
  uint8_t n = (uint8_t)cfg.usSamples; if (n > US_BUF_MAX) n = US_BUF_MAX; if (n < 1) n = 1;
  Ultra& u = us[i];
  if (u.head >= n) u.head = 0;
  if (u.count > n) u.count = n;
  u.buf[u.head] = ok ? cm : -1.0f;
  u.head = (u.head + 1) % n;
  if (u.count < n) u.count++;
  float tmp[US_BUF_MAX]; uint8_t m = 0;
  for (uint8_t k = 0; k < u.count; k++) if (u.buf[k] >= 0) tmp[m++] = u.buf[k];
  if (m > 0) { u.dist = medianOf(tmp, m); u.distValid = true; } else { u.dist = -1; u.distValid = false; }
  u.seq++;
}
void usTick() {
  if (!bootDone) return;
  bool scanning = scanActive();
  uint32_t tickMs = scanning ? 40 : (uint32_t)(cfg.usInterval / 3);
  if (tickMs < 20) tickMs = 20;
  if (!due(usLastTick, tickMs)) return;
  usLastTick = millis();
  uint8_t i = scanning ? 0 : usNext;
  if (!scanning) usNext = (usNext + 1) % 3;
  float cm = 0; bool ok = usMeasure(i, cm);
  usPush(i, ok, cm);
}
// Effective distance used by logic: -1 = unknown/invalid
float effDist(uint8_t i) { return us[i].distValid ? us[i].dist : -1.0f; }

// ===================== REAR IR =====================
bool rearBlocked() { return irObstacle; }
void irTick() {
  if (!bootDone || !due(irLast, 50)) return;
  irLast = millis();
  bool raw = (digitalRead(PIN_IR_REAR) == (cfg.irActive ? HIGH : LOW));
  if (raw == irRaw) { if (irCnt < 250) irCnt++; } else { irRaw = raw; irCnt = 1; }
  if (irCnt >= 2) irObstacle = irRaw;
}

void safetyEnforce() {
  if (emergency) {
    if (curL != 0 || curR != 0) stopMotors();
    if (vacState != VAC_OFF) vacuumOff();
  }
}

// ===================== TESTS =====================
void testCancel() { tMotorActive = false; tRgbMask = 0; }
void testTick() {
  if (tMotorActive && (int32_t)(millis() - tMotorEnd) >= 0) { stopMotors(); tMotorActive = false; }
  if (scn.step == SC_DONE && scanOwner == OWN_TEST) { scanStoreResults(); scanReset(); }
}

// ===================== EMERGENCY / MODE =====================
void emergencyStop() {
  emergency = true;
  stopMotors(); vacuumOff();
  autoState = A_IDLE; autoPaused = false; obsCnt = 0; autoBlockedTries = 0;
  scanCancel(); testCancel();
  manActive = false; manClass = MC_STOP; manBlockLatch = false;
  servoGo(cfg.svCenter);
  if (buzzerAllows(cfg.bzEvFault)) buzzStart(1, (uint16_t)cfg.bzLongMs, 0, (uint16_t)cfg.bzFaultFreq, PRIO_FAULT);
}
void autoStopInternal() {
  autoState = A_IDLE; autoPaused = false; stopMotors(); vacuumOff(); scanCancel();
}
void setMode(CurMode m) {
  if (curMode == m) return;
  stopMotors(); vacuumOff(); autoStopInternal(); testCancel(); manActive = false; manClass = MC_STOP;
  servoGo(cfg.svCenter); usReset(0);
  curMode = m;
}

// ===================== MANUAL =====================
int pctToPwm(float v) {
  float a = fabsf(v);
  if (a < 1.0f) return 0;
  int p = (int)(cfg.minPwm + (cfg.spdManual - cfg.minPwm) * a / 100.0f);
  p = (int)iclamp(p, 0, 255);
  return v < 0 ? -p : p;
}
bool manualDrive(float x, float y, bool fresh) {
  if (emergency || curMode != MODE_MANUAL) { stopMotors(); manActive = false; return false; }
  x = fclampf(x, -100, 100); y = fclampf(y, -100, 100);
  float mag = sqrtf(x * x + y * y);
  if (mag > 100.0f) { x = x / mag * 100.0f; y = y / mag * 100.0f; mag = 100.0f; }
  float dead = (float)cfg.joyDead;
  if (mag <= dead || mag < 1.0f) { x = 0; y = 0; }
  else {
    float f = ((mag - dead) / (100.0f - dead)) * ((float)cfg.joySens / 100.0f);
    x = x / mag * f * 100.0f; y = y / mag * f * 100.0f;
    x = fclampf(x, -100, 100); y = fclampf(y, -100, 100);
  }
  if (x == 0 && y == 0) {
    stopMotors(); manActive = false; manClass = MC_STOP; manBlockLatch = false;
    return true;
  }
  if (fresh) { manX = x; manY = y; manLast = millis(); }
  if (tMotorActive) testCancel();
  if (scanActive() || scn.step == SC_DONE) { scanCancel(); }
  manActive = true;
  float rx = fresh ? x : manX, ry = fresh ? y : manY;   // stored command used by safety re-check
  x = rx; y = ry;
  // classification of the command (for the one-beep-per-command rule)
  uint8_t cls = (fabsf(y) >= fabsf(x)) ? (y > 0 ? MC_FWD : MC_BACK) : (x > 0 ? MC_RIGHT : MC_LEFT);
  // safety assist
  bool blocked = false;
  if (cfg.manAssist && y > 0) {
    bool fb = (effDist(S_FRONT) >= 0 && effDist(S_FRONT) < cfg.emergCm);
    if (fb) { y = 0; blocked = true; }
  }
  if (cfg.manRearAssist && y < 0 && rearBlocked()) { y = 0; blocked = true; }
  if (blocked) {
    manBlockUntil = millis() + 400;
    if (!manBlockLatch) { manBlockLatch = true; if (buzzerAllows(cfg.bzEvObst)) beepShort(PRIO_LOW); }
  } else manBlockLatch = false;
  float l = y - x, r = y + x;   // steering sign swapped: left arrow now turns LEFT, right arrow turns RIGHT
  float m2 = fabsf(l) > fabsf(r) ? fabsf(l) : fabsf(r);
  if (m2 > 100.0f) { l = l / m2 * 100.0f; r = r / m2 * 100.0f; }
  setMotor(true, pctToPwm(l)); setMotor(false, pctToPwm(r));
  if (fresh && cls != manClass) {
    manClass = cls;
    if (!blocked && buzzerAllows(cfg.bzEvManual) && !buzzBusy())
      buzzStart(1, (uint16_t)cfg.bzManualMs, 0, (uint16_t)cfg.bzFreq, PRIO_MANUAL);
  }
  return true;
}
void manualTick() {
  if (curMode != MODE_MANUAL || !manActive) return;
  uint32_t now = millis();
  if (due(manLast, (uint32_t)cfg.manWatchdogMs)) { stopMotors(); manActive = false; manClass = MC_STOP; return; }
  if (due(manTickT, 100)) { manTickT = now; manualDrive(manX, manY, false); }
}

// ===================== AUTO CLEAN =====================
const char* autoName() {
  switch (autoState) {
    case A_IDLE: return "IDLE"; case A_VAC_START: return "VACUUM START"; case A_FORWARD: return "FORWARD";
    case A_STOP: return "STOP"; case A_EMERGENCY_SCAN: return "EMERGENCY SCAN"; case A_REVERSE: return "REVERSE";
    case A_SCAN: return "SCAN"; case A_TURN_LEFT: return "TURN LEFT"; case A_TURN_RIGHT: return "TURN RIGHT";
    case A_CORRECT_LEFT: return "CORRECT LEFT"; case A_CORRECT_RIGHT: return "CORRECT RIGHT";
    case A_RESUME: return "RESUME";
  }
  return "?";
}
void autoGoto(AutoState s) {
  autoState = s; autoT = millis();
  switch (s) {
    case A_IDLE: case A_STOP: stopMotors(); break;
    case A_VAC_START: stopMotors(); if (cfg.autoVacuum) vacuumRequestOn(); break;
    case A_FORWARD: autoPaused = false; forward(); break;
    case A_REVERSE: if (rearBlocked()) autoGoto(A_SCAN); else reverse(); break;
    case A_SCAN: case A_EMERGENCY_SCAN: stopMotors(); scanOwner = OWN_AUTO; scanStart(); break;
    case A_TURN_LEFT: turnLeft(); break;
    case A_TURN_RIGHT: turnRight(); break;
    case A_CORRECT_LEFT: softLeft(); break;
    case A_CORRECT_RIGHT: softRight(); break;
    case A_RESUME: usReset(0); obsCnt = 0; autoSeq = us[0].seq; forward(); break;
  }
}
const char* autoStart() {
  if (emergency) return "Emergency STOP active - reset it first";
  setMode(MODE_AUTO);
  stopMotors(); scanCancel(); servoGo(cfg.svCenter); usReset(0);
  autoBlockedTries = 0; obsCnt = 0; autoSeq = us[0].seq; autoLastCorr = millis();
  autoGoto(A_VAC_START);
  return NULL;
}
void autoUpdateFrontCounts() {
  if (us[0].seq == autoSeq) return;
  autoSeq = us[0].seq;
  float f = effDist(S_FRONT);
  if (f < 0) return;
  if (f <= cfg.frontCm) { if (obsCnt < 250) obsCnt++; } else obsCnt = 0;
}
bool autoCheckFront() {
  if (obsCnt >= cfg.obsConfirm) {
    float f = effDist(S_FRONT);
    autoEmerg = (f >= 0 && f < cfg.emergCm);
    if (buzzerAllows(cfg.bzEvObst)) beepShort(PRIO_LOW);
    autoGoto(A_STOP);
    return true;
  }
  return false;
}
bool pickRight(float l, float r) {
  bool tieRight = (cfg.strategy == 2) || (cfg.strategy == 0 && cfg.tieDefault == 1);
  bool lb = (l >= 0 && l < cfg.blockedCm), rb = (r >= 0 && r < cfg.blockedCm);
  if (l < 0 && r < 0) return tieRight;
  if (l < 0) return !rb;          // left unknown: take right only if it is known clear
  if (r < 0) return lb;           // right unknown: take right only if left is blocked
  if (fabsf(r - l) <= (float)cfg.tieMargin) return tieRight;
  return r > l;
}
void autoDecide() {
  scanStoreResults();
  float l = lastScanL, r = lastScanR;
  bool lb = (l >= 0 && l < cfg.blockedCm), rb = (r >= 0 && r < cfg.blockedCm);
  if (lb && rb) {
    if (autoBlockedTries < 1) { autoBlockedTries++; autoRevMs = (uint32_t)cfg.revLongMs; autoEmerg = false; autoGoto(A_REVERSE); return; }
    autoBlockedTries = 0; autoTurnMs = (uint32_t)cfg.turnRecovMs;
    autoGoto(pickRight(l, r) ? A_TURN_RIGHT : A_TURN_LEFT);
    return;
  }
  autoBlockedTries = 0;
  autoTurnMs = autoEmerg ? (uint32_t)cfg.emergTurnMs : (uint32_t)cfg.turnMs;
  autoGoto(pickRight(l, r) ? A_TURN_RIGHT : A_TURN_LEFT);
}
void autoTick() {
  if (curMode != MODE_AUTO || autoState == A_IDLE || emergency) return;
  uint32_t now = millis();
  autoUpdateFrontCounts();
  switch (autoState) {
    case A_VAC_START:
      if (!cfg.autoVacuum || vacState == VAC_ON) autoGoto(A_FORWARD);
      break;
    case A_FORWARD: {
      if (autoCheckFront()) break;
      float f = effDist(S_FRONT);
      if (f < 0) { if (!autoPaused) { stopMotors(); autoPaused = true; } break; }   // unknown front: pause
      if (autoPaused) { forward(); autoPaused = false; }
      if (due(autoLastCorr, (uint32_t)cfg.corrCoolMs)) {
        float l = effDist(S_LEFT), r = effDist(S_RIGHT);
        bool lb = (l >= 0 && l < cfg.sideCm), rb = (r >= 0 && r < cfg.sideCm);
        if (lb && !rb) autoGoto(A_CORRECT_RIGHT);
        else if (rb && !lb) autoGoto(A_CORRECT_LEFT);
      }
      break;
    }
    case A_STOP:
      if (due(autoT, (uint32_t)cfg.stopSettleMs)) {
        float f = effDist(S_FRONT);
        if (f >= 0 && f < cfg.emergCm) autoEmerg = true;
        if (autoEmerg) autoGoto(A_EMERGENCY_SCAN);
        else { autoRevMs = (uint32_t)cfg.revMs; autoGoto(A_REVERSE); }
      }
      break;
    case A_REVERSE:
      if (rearBlocked() || due(autoT, autoRevMs)) { stopMotors(); autoGoto(A_SCAN); }
      break;
    case A_SCAN: case A_EMERGENCY_SCAN:
      if (scn.step == SC_DONE && scanOwner == OWN_AUTO) { scanReset(); autoDecide(); }
      break;
    case A_TURN_LEFT: case A_TURN_RIGHT:
      if (due(autoT, autoTurnMs)) { stopMotors(); autoGoto(A_RESUME); }
      break;
    case A_CORRECT_LEFT: case A_CORRECT_RIGHT:
      if (autoCheckFront()) break;
      if (due(autoT, (uint32_t)cfg.corrTurnMs)) { autoLastCorr = now; autoGoto(A_FORWARD); }
      break;
    case A_RESUME:
      if (autoCheckFront()) break;
      if (due(autoT, (uint32_t)cfg.resumeMs)) autoGoto(A_FORWARD);
      break;
    default: break;
  }
}

// ===================== PREFERENCES =====================
void settingsDefaults() { for (size_t i = 0; i < N_SET; i++) *SETTINGS[i].p = SETTINGS[i].df; }
void settingsSanitize() {
  for (size_t i = 0; i < N_SET; i++) *SETTINGS[i].p = iclamp(*SETTINGS[i].p, SETTINGS[i].mn, SETTINGS[i].mx);
  if (cfg.emergCm > cfg.frontCm) cfg.emergCm = cfg.frontCm;
  if (cfg.scanSamples > cfg.usSamples) cfg.scanSamples = cfg.usSamples;
  if (cfg.minPwm > cfg.spdManual) cfg.spdManual = cfg.minPwm;
  if (cfg.usMinCm >= cfg.usMaxCm) cfg.usMinCm = 1;
}
void settingsApply() {
  for (uint8_t i = 0; i < 3; i++) usReset(i);
  relayWrite(vacState == VAC_ON);          // re-drive relay with the (possibly new) polarity
  if (!bz.on) buzzTone(0);
  if (!cfg.bzEnable) buzzStop();
  servoGo(svAngle);
}
void wifiDefaults() { wifiSsid = WIFI_SSID_DEFAULT; wifiPass = WIFI_PASS_DEFAULT; }
void settingsLoad() {
  prefs.begin(NVS_NS, false);
  for (size_t i = 0; i < N_SET; i++) *SETTINGS[i].p = prefs.getInt(SETTINGS[i].key, SETTINGS[i].df);
  wifiSsid = prefs.getString("wssid", WIFI_SSID_DEFAULT);
  wifiPass = prefs.getString("wpass", WIFI_PASS_DEFAULT);
  prefs.end();
  settingsSanitize();
}
bool hasSavedSettings() { prefs.begin(NVS_NS, false); bool b = prefs.isKey("saved"); prefs.end(); return b; }
void settingsSave() {
  prefs.begin(NVS_NS, false);
  for (size_t i = 0; i < N_SET; i++) prefs.putInt(SETTINGS[i].key, *SETTINGS[i].p);
  prefs.putString("wssid", wifiSsid);
  prefs.putString("wpass", wifiPass);
  prefs.putUChar("saved", 1);
  prefs.end();
}
void settingsFactory() {
  String s = wifiSsid, p = wifiPass;          // Wi-Fi credentials are kept so the robot stays reachable
  prefs.begin(NVS_NS, false); prefs.clear(); prefs.end();
  settingsDefaults(); settingsSanitize();
  wifiSsid = s; wifiPass = p;
  prefs.begin(NVS_NS, false); prefs.putString("wssid", wifiSsid); prefs.putString("wpass", wifiPass); prefs.end();
}

// ===================== WI-FI =====================
void wifiBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  wifiTryT = millis(); wifiBeepT = millis();
}
void wifiTick() {
  if (!bootDone) return;
  uint32_t now = millis();
  bool c = (WiFi.status() == WL_CONNECTED);
  if (wifiApplyNew) { wifiApplyNew = false; WiFi.disconnect(); WiFi.begin(wifiSsid.c_str(), wifiPass.c_str()); wifiTryT = now; return; }
  if (c) {
    if (!wifiUp) {
      wifiUp = true;
      if (buzzerAllows(cfg.bzEvWifi)) buzzStart((uint8_t)cfg.bzWifiCnt, (uint16_t)cfg.bzShortMs, (uint16_t)cfg.bzGapMs, (uint16_t)cfg.bzFreq, PRIO_START);
      if (!serverStarted) { routesSetup(); server.begin(); serverStarted = true; Serial.print("READY - open http://"); Serial.println(WiFi.localIP()); }
    }
  } else {
    if (wifiUp) { wifiUp = false; wifiBeepT = now; }
    if (due(wifiBeepT, (uint32_t)cfg.bzRetryMs)) { wifiBeepT = now; if (buzzerAllows(cfg.bzEvWifi)) beepShort(PRIO_LOW); }
    if (due(wifiTryT, (uint32_t)cfg.wifiReconMs)) { wifiTryT = now; WiFi.disconnect(); WiFi.begin(wifiSsid.c_str(), wifiPass.c_str()); }
  }
}

// ===================== BOOT SEQUENCE =====================
void bootTick() {
  if (bootDone) return;
  uint32_t now = millis();
  switch (bootStep) {
    case B_R: rgbWrite(true, false, false); if (due(bootT, (uint32_t)cfg.rgbStartMs)) { bootStep = B_G; bootT = now; } break;
    case B_G: rgbWrite(false, true, false); if (due(bootT, (uint32_t)cfg.rgbStartMs)) { bootStep = B_B; bootT = now; } break;
    case B_B:
      rgbWrite(false, false, true);
      if (due(bootT, (uint32_t)cfg.rgbStartMs)) {
        bootStep = B_BEEP; bootT = now; rgbWrite(false, false, false);
        if (buzzerAllows(cfg.bzEvStart)) buzzStart((uint8_t)cfg.bzStartCnt, (uint16_t)cfg.bzShortMs, (uint16_t)cfg.bzGapMs, (uint16_t)cfg.bzFreq, PRIO_START);
      }
      break;
    case B_BEEP: if (!buzzBusy()) { bootStep = B_DONE; bootDone = true; wifiBegin(); } break;
    default: break;
  }
}

// ===================== WEB API =====================
void jI(String& s, const char* k, long v) { s += ",\""; s += k; s += "\":"; s += String(v); }
void jF(String& s, const char* k, float v) { s += ",\""; s += k; s += "\":"; s += String(v, 1); }
void jS(String& s, const char* k, const String& v) { s += ",\""; s += k; s += "\":\""; s += jsonEsc(v); s += "\""; }
void sendJson(int code, const String& body) { server.sendHeader("Cache-Control", "no-store"); server.send(code, "application/json", body); }
void replyOk(const String& msg) { sendJson(200, "{\"ok\":true,\"msg\":\"" + jsonEsc(msg) + "\"}"); }
void replyNo(const String& msg) { sendJson(200, "{\"ok\":false,\"msg\":\"" + jsonEsc(msg) + "\"}"); }
void replyBad(const String& msg) { sendJson(400, "{\"ok\":false,\"msg\":\"" + jsonEsc(msg) + "\"}"); }

String statusJson() {
  String s; s.reserve(900);
  s += "{\"up\":"; s += String((uint32_t)(millis() / 1000));
  bool wc = (WiFi.status() == WL_CONNECTED);
  jI(s, "w", wc); jI(s, "rssi", wc ? WiFi.RSSI() : 0);
  jI(s, "mode", curMode); jI(s, "est", emergency);
  jI(s, "ml", curL); jI(s, "mr", curR); jI(s, "vac", vacState);
  jF(s, "f", us[0].distValid ? us[0].dist : -1.0f);
  jF(s, "l", us[1].distValid ? us[1].dist : -1.0f);
  jF(s, "r", us[2].distValid ? us[2].dist : -1.0f);
  jI(s, "ir", irObstacle); jI(s, "sv", svAngle);
  jS(s, "as", String(autoName()));
  jI(s, "scn", scanActive() ? 1 : 0);
  jF(s, "sl", lastScanL); jF(s, "sr", lastScanR);
  jI(s, "spd", cfg.spdManual);
  jI(s, "asst", cfg.manAssist); jI(s, "rdy", bootDone ? 1 : 0);
  s += "}";
  return s;
}
void handleStatus() { sendJson(200, statusJson()); }

void handleManual() {
  if (server.hasArg("speed")) cfg.spdManual = iclamp(server.arg("speed").toInt(), cfg.minPwm > 50 ? cfg.minPwm : 50, 255);
  if (!server.hasArg("x") && !server.hasArg("y")) { replyOk("Speed set"); return; }
  if (emergency) { stopMotors(); replyNo("Emergency STOP active"); return; }
  if (curMode != MODE_MANUAL) { replyNo("Switch to MANUAL mode"); return; }
  manualDrive(server.arg("x").toFloat(), server.arg("y").toFloat(), true);
  replyOk("OK");
}
void handleStop() {
  if (curMode == MODE_AUTO) autoStopInternal();
  else { stopMotors(); manActive = false; manClass = MC_STOP; testCancel(); scanCancel(); }
  replyOk("Stopped");
}
void handleEstop() { emergencyStop(); replyOk("EMERGENCY STOP"); }
void handleEstopReset() {
  emergency = false; autoState = A_IDLE; manActive = false; stopMotors(); vacuumOff();
  replyOk("E-STOP reset - movement needs an explicit restart");
}
void handleMode() {
  if (emergency) { replyNo("Emergency STOP active"); return; }
  int m = server.arg("m").toInt();
  if (m != 0 && m != 1) { replyBad("Bad mode"); return; }
  setMode(m ? MODE_AUTO : MODE_MANUAL);
  replyOk(m ? "AUTO CLEAN mode" : "MANUAL mode");
}
void handleVacuum() {
  if (emergency) { replyNo("Emergency STOP active"); return; }
  if (curMode != MODE_MANUAL) { replyNo("AUTO CLEAN controls the vacuum"); return; }
  if (!server.hasArg("on")) { replyBad("Missing 'on'"); return; }
  if (server.arg("on").toInt()) { vacuumRequestOn(); replyOk("Vacuum starting"); }
  else { vacuumOff(); replyOk("Vacuum OFF"); }
}
void handleServo() {
  if (emergency) { replyNo("Emergency STOP active"); return; }
  if (curMode != MODE_MANUAL) { replyNo("Servo is controlled by AUTO"); return; }
  String pos = server.arg("pos");
  if (pos.length()) {
    if (pos == "scan") { stopMotors(); manActive = false; testCancel(); scanCancel(); scanOwner = OWN_TEST; scanStart(); replyOk("Full scan started"); return; }
    scanCancel();
    if (pos == "home") servoGo(cfg.svHome);
    else if (pos == "left") servoGo(cfg.svLeft);
    else if (pos == "center") servoGo(cfg.svCenter);
    else if (pos == "right") servoGo(cfg.svRight);
    else { replyBad("Bad position"); return; }
    replyOk("Servo " + pos);
    return;
  }
  if (!server.hasArg("angle")) { replyBad("Missing angle"); return; }
  scanCancel();
  servoGo((int)iclamp(server.arg("angle").toInt(), 0, 180));
  replyOk("Servo moved");
}
void handleAutoStart() {
  const char* e = autoStart();
  if (e) replyNo(e); else replyOk("AUTO CLEAN started");
}
void handleAutoStop() { autoStopInternal(); replyOk("AUTO CLEAN stopped"); }

void handleSettingsGet() {
  String s; s.reserve(16000);
  s += "{\"ssid\":\""; s += jsonEsc(wifiSsid); s += "\",\"wc\":"; s += String(WiFi.status() == WL_CONNECTED ? 1 : 0);
  s += ",\"st\":{\"saved\":"; s += String(hasSavedSettings() ? 1 : 0);
  s += ",\"heap\":"; s += String((uint32_t)ESP.getFreeHeap());
  s += ",\"sketch\":"; s += String((uint32_t)ESP.getSketchSize());
  s += ",\"flash\":"; s += String((uint32_t)ESP.getFlashChipSize());
  s += ",\"n\":"; s += String((uint32_t)N_SET);
  s += "},\"s\":[";
  for (size_t i = 0; i < N_SET; i++) {
    const SetDef& d = SETTINGS[i];
    if (i) s += ',';
    s += "{\"k\":\""; s += d.key; s += "\",\"sec\":\""; s += d.sec; s += "\",\"l\":\""; s += d.label;
    s += "\",\"u\":\""; s += d.unit; s += "\",\"mn\":"; s += String((long)d.mn); s += ",\"mx\":"; s += String((long)d.mx);
    s += ",\"df\":"; s += String((long)d.df); s += ",\"v\":"; s += String((long)*d.p);
    s += ",\"t\":"; s += String((int)d.type); s += ",\"o\":\""; s += d.opts; s += "\"}";
  }
  s += "]}";
  sendJson(200, s);
}
void handleSettingsSave() {
  int clamped = 0;
  for (size_t i = 0; i < N_SET; i++) {
    const SetDef& d = SETTINGS[i];
    if (!server.hasArg(d.key)) continue;
    String a = server.arg(d.key);
    if (a.length() == 0) continue;
    long v = a.toInt();
    long c = iclamp(v, d.mn, d.mx);
    if (c != v) clamped++;
    *d.p = (int32_t)c;
  }
  settingsSanitize();
  bool wchg = false;
  if (server.hasArg("wssid")) {
    String ns = server.arg("wssid"); ns.trim();
    if (ns.length() >= 1 && ns.length() <= 32 && ns != wifiSsid) { wifiSsid = ns; wchg = true; }
  }
  if (server.hasArg("wpass") && server.arg("wpass").length() > 0 && server.arg("wpass").length() <= 63) { wifiPass = server.arg("wpass"); wchg = true; }
  settingsApply();
  settingsSave();
  if (wchg) wifiApplyNew = true;
  replyOk(String("Saved permanently") + (clamped ? " (" + String(clamped) + " value(s) adjusted to valid range)" : "") + (wchg ? " - reconnecting Wi-Fi" : ""));
}
void handleSettingsRestore() {
  String s = wifiSsid, p = wifiPass;
  settingsLoad(); settingsApply();
  if (s != wifiSsid || p != wifiPass) wifiApplyNew = true;
  replyOk("Saved settings restored");
}
void handleSettingsFactory() { settingsFactory(); settingsApply(); replyOk("Factory defaults restored (Wi-Fi credentials kept)"); }

// ---- Hardware tests ----
void handleTestBuzzer() {
  String t = server.arg("type");
  if (!cfg.bzEnable) { replyNo("Buzzer is disabled in Settings"); return; }
  uint16_t f = (uint16_t)cfg.bzFreq;
  if (t == "short") buzzStart(1, (uint16_t)cfg.bzShortMs, 0, f, PRIO_FAULT);
  else if (t == "long") buzzStart(1, (uint16_t)cfg.bzLongMs, 0, f, PRIO_FAULT);
  else if (t == "manual") buzzStart(1, (uint16_t)cfg.bzManualMs, 0, f, PRIO_FAULT);
  else if (t == "vacuum") buzzStart(1, (uint16_t)cfg.bzVacMs, 0, f, PRIO_FAULT);
  else if (t == "fault") buzzStart((uint8_t)cfg.bzFaultCnt, (uint16_t)cfg.bzFaultMs, (uint16_t)cfg.bzGapMs, (uint16_t)cfg.bzFaultFreq, PRIO_FAULT);
  else { replyBad("Bad type"); return; }
  replyOk("Buzzer test");
}
void handleTestRgb() {
  if (emergency) { replyNo("Emergency STOP active"); return; }
  String c = server.arg("c");
  if (c == "r") tRgbMask = 1; else if (c == "g") tRgbMask = 2; else if (c == "b") tRgbMask = 4;
  else if (c == "off") { tRgbMask = 0; replyOk("RGB test off"); return; }
  else { replyBad("Bad colour"); return; }
  tRgbEnd = millis() + (uint32_t)cfg.rgbTestMs;
  replyOk("RGB test");
}
void handleTestUltra() {
  int i = server.arg("id").toInt();
  if (!server.hasArg("id") || i < 0 || i > 2) { replyBad("Bad sensor"); return; }
  float cm = 0; bool ok = usMeasure(i, cm);
  String s = "{\"ok\":true,\"valid\":"; s += ok ? "true" : "false"; s += ",\"cm\":"; s += String(ok ? cm : -1.0f, 1); s += "}";
  sendJson(200, s);
}
void handleTestIr() {
  String s = "{\"ok\":true,\"obstacle\":"; s += irObstacle ? "true" : "false"; s += ",\"raw\":"; s += String(digitalRead(PIN_IR_REAR)); s += "}";
  sendJson(200, s);
}
void handleTestMotor() {
  if (emergency) { replyNo("Emergency STOP active"); return; }
  if (curMode != MODE_MANUAL) { replyNo("Switch to MANUAL mode for hardware tests"); return; }
  String side = server.arg("side"), dir = server.arg("dir");
  if ((side != "l" && side != "r") || (dir != "f" && dir != "b")) { replyBad("Bad arguments"); return; }
  manActive = false; stopMotors();
  setMotor(side == "l", dir == "f" ? cfg.motorTestSpd : -cfg.motorTestSpd);
  tMotorActive = true; tMotorEnd = millis() + (uint32_t)cfg.motorTestMs;
  replyOk(String(side == "l" ? "Left" : "Right") + " motor test");
}
void handleTestVacuum() {
  if (emergency) { replyNo("Emergency STOP active"); return; }
  if (curMode != MODE_MANUAL) { replyNo("Switch to MANUAL mode for hardware tests"); return; }
  if (vacState != VAC_OFF) { replyNo("Vacuum already running"); return; }
  relayWrite(true); vacState = VAC_ON; vacTestEnd = millis() + (uint32_t)cfg.vacTestMs;
  replyOk("Relay test - turns OFF automatically");
}

void handleRoot();   // defined after the HTML

void routesSetup() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/manual", HTTP_POST, handleManual);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.on("/api/estop", HTTP_POST, handleEstop);
  server.on("/api/estop/reset", HTTP_POST, handleEstopReset);
  server.on("/api/mode", HTTP_POST, handleMode);
  server.on("/api/vacuum", HTTP_POST, handleVacuum);
  server.on("/api/servo", HTTP_POST, handleServo);
  server.on("/api/auto/start", HTTP_POST, handleAutoStart);
  server.on("/api/auto/stop", HTTP_POST, handleAutoStop);
  server.on("/api/settings", HTTP_GET, handleSettingsGet);
  server.on("/api/settings/save", HTTP_POST, handleSettingsSave);
  server.on("/api/settings/restore", HTTP_POST, handleSettingsRestore);
  server.on("/api/settings/factory", HTTP_POST, handleSettingsFactory);
  server.on("/api/test/buzzer", HTTP_POST, handleTestBuzzer);
  server.on("/api/test/rgb", HTTP_POST, handleTestRgb);
  server.on("/api/test/ultra", HTTP_POST, handleTestUltra);
  server.on("/api/test/ir", HTTP_POST, handleTestIr);
  server.on("/api/test/motor", HTTP_POST, handleTestMotor);
  server.on("/api/test/vacuum", HTTP_POST, handleTestVacuum);
  server.onNotFound([]() { replyBad("Not found"); });
}

// ===================== EMBEDDED HTML / CSS / JS =====================
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Automatic Vacuum Cleaner</title>
<style>
:root{--bg:#0b0f14;--card:#151c26;--fg:#e8eef5;--mut:#8aa0b5;--acc:#19c37d;--bad:#ff3b3b;--warn:#ffb020;--blu:#3b82f6;--bd:#243040}
[data-theme=light]{--bg:#eef2f6;--card:#fff;--fg:#10202e;--mut:#5a6b7c;--bd:#cfd8e3}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html{font-size:16px}
body{margin:0;background:var(--bg);color:var(--fg);font-family:system-ui,Segoe UI,Roboto,sans-serif;padding-bottom:5.5rem;user-select:none}
header{display:flex;align-items:center;gap:.5rem;padding:.6rem .8rem;background:var(--card);border-bottom:1px solid var(--bd);position:sticky;top:0;z-index:5;flex-wrap:wrap}
header h1{font-size:1rem;margin:0;flex:1;min-width:8rem;letter-spacing:.04em}
.dot{display:inline-block;width:.7rem;height:.7rem;border-radius:50%;background:var(--bad);margin-right:.3rem}
.dot.ok{background:var(--acc)}
.ib{background:transparent;color:var(--fg);border:1px solid var(--bd);border-radius:.5rem;padding:.35rem .6rem;font-size:.85rem;cursor:pointer}
main{padding:.8rem;max-width:70rem;margin:0 auto}
.tabs{display:flex;gap:.5rem;margin-bottom:.8rem}
.tab{flex:1;padding:.8rem;border-radius:.7rem;border:2px solid var(--bd);background:var(--card);color:var(--fg);font-weight:700;font-size:1rem;cursor:pointer}
.tab.on{border-color:var(--acc);color:var(--acc)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(18rem,1fr));gap:.8rem}
.card{background:var(--card);border:1px solid var(--bd);border-radius:.9rem;padding:.9rem}
.card h2{margin:0 0 .6rem;font-size:.8rem;letter-spacing:.12em;color:var(--mut);text-transform:uppercase}
.b{background:var(--blu);color:#fff;border:0;border-radius:.6rem;padding:.7rem 1rem;font-weight:700;font-size:.9rem;cursor:pointer;touch-action:manipulation}
.b.g{background:var(--acc)}.b.r{background:var(--bad)}.b.w{background:var(--warn);color:#222}.b.o{background:transparent;border:1px solid var(--bd);color:var(--fg)}
.b:disabled{opacity:.4}
.row{display:flex;gap:.5rem;align-items:center;flex-wrap:wrap;margin:.4rem 0}
#joyWrap{display:flex;justify-content:center;margin:.5rem 0}
#joy{position:relative;width:14rem;height:14rem;border-radius:50%;border:3px solid var(--acc);background:radial-gradient(circle,rgba(25,195,125,.12),transparent 70%);touch-action:none}
#knob{position:absolute;left:50%;top:50%;width:5rem;height:5rem;margin:-2.5rem 0 0 -2.5rem;border-radius:50%;background:var(--acc);box-shadow:0 4px 14px rgba(0,0,0,.5);pointer-events:none}
.dpad{display:grid;grid-template-columns:repeat(3,3.6rem);grid-template-rows:repeat(3,3.2rem);gap:.35rem;justify-content:center;margin:.6rem 0}
.dpad button{border-radius:.7rem;border:1px solid var(--bd);background:var(--card);color:var(--fg);font-size:1.3rem;touch-action:none}
.dpad button:active{background:var(--acc);color:#000}
.kv{display:flex;justify-content:space-between;padding:.25rem 0;border-bottom:1px dashed var(--bd);font-size:.9rem}
.kv b{font-variant-numeric:tabular-nums}
.bar{height:.5rem;border-radius:.3rem;background:var(--bd);overflow:hidden;margin:.15rem 0 .5rem}.bar i{display:block;height:100%;background:var(--acc);width:0}
.chip{display:inline-block;padding:.2rem .55rem;border-radius:1rem;font-size:.75rem;border:1px solid var(--bd);margin:.15rem .15rem 0 0}
.chip.ok{color:var(--acc);border-color:var(--acc)}.chip.bad{color:#fff;background:var(--bad);border-color:var(--bad)}.chip.ovr{color:#222;background:var(--warn);border-color:var(--warn)}
input[type=range]{width:100%}
#estop{position:fixed;left:0;right:0;bottom:0;height:4.6rem;background:var(--bad);color:#fff;font-weight:900;font-size:1.5rem;letter-spacing:.1em;border:0;z-index:20;box-shadow:0 -4px 18px rgba(0,0,0,.5);touch-action:manipulation}
.banner{display:none;margin-bottom:.8rem;padding:.8rem;border-radius:.7rem;font-weight:700;text-align:center}
.banner.show{display:block}.banner.red{background:var(--bad);color:#fff}.banner.amb{background:var(--warn);color:#222}
#intro{position:fixed;inset:0;background:var(--bg);z-index:100;display:flex;flex-direction:column;align-items:center;justify-content:center;text-align:center;transition:opacity .6s;padding:1rem}
#intro h1{font-size:1.8rem;letter-spacing:.12em;color:var(--acc);margin:.2rem}#intro p{color:var(--mut);letter-spacing:.2em;font-size:.85rem}
#intro .sp{width:3rem;height:3rem;border:4px solid var(--bd);border-top-color:var(--acc);border-radius:50%;animation:spin 1s linear infinite;margin-bottom:1rem}
@keyframes spin{to{transform:rotate(360deg)}}
#set{position:fixed;inset:0;background:var(--bg);z-index:60;overflow:auto;display:none;padding:.8rem .8rem 7rem}
#set.open{display:block}
details.sec{background:var(--card);border:1px solid var(--bd);border-radius:.8rem;margin-bottom:.6rem}
details.sec summary{padding:.8rem;font-weight:700;cursor:pointer}
.sb{padding:0 .8rem .8rem}
.fr{display:flex;justify-content:space-between;align-items:center;gap:.6rem;padding:.4rem 0;border-bottom:1px dashed var(--bd);font-size:.88rem}
.fr small{display:block;color:var(--mut);font-size:.72rem}
.fr input[type=number],.fr input[type=text],.fr input[type=password],.fr select,.sb input{background:var(--bg);color:var(--fg);border:1px solid var(--bd);border-radius:.4rem;padding:.35rem;width:7.5rem}
.fr input[type=checkbox]{width:1.3rem;height:1.3rem}
#setBar{position:fixed;left:0;right:0;bottom:0;background:var(--card);border-top:1px solid var(--bd);padding:.6rem;display:flex;gap:.5rem;flex-wrap:wrap;z-index:61;justify-content:center}
.hint{color:var(--mut);font-size:.78rem}
.res{font-weight:700;margin-left:.3rem;font-size:.85rem}
#toast{position:fixed;left:50%;bottom:6rem;transform:translateX(-50%);background:#222;color:#fff;padding:.6rem 1rem;border-radius:2rem;opacity:0;transition:opacity .25s;z-index:95;pointer-events:none;max-width:90vw;text-align:center}
#toast.show{opacity:1}#toast.bad{background:var(--bad)}
</style></head><body data-theme="dark">
<div id="intro"><div class="sp"></div><h1>AUTOMATIC VACUUM CLEANER</h1><p>SMART ROBOTIC CLEANING SYSTEM</p></div>
<header>
 <h1>AUTOMATIC VACUUM CLEANER</h1>
 <span><span id="wdot" class="dot"></span><span id="wtxt">Wi-Fi</span></span>
 <button class="ib" id="dmBtn" onclick="cycleDM()">Normal</button>
 <button class="ib" id="thBtn" onclick="toggleTheme()">Theme</button>
 <button class="ib" onclick="openSet()">&#9881; Settings</button>
</header>
<main>
 <div id="estBanner" class="banner red">EMERGENCY STOP ACTIVE &mdash; movement locked <button class="b o" style="margin-left:.6rem" onclick="act('/api/estop/reset')">RESET E-STOP</button></div>
 <div class="tabs"><button id="tabM" class="tab" onclick="setMode(0)">MANUAL</button><button id="tabA" class="tab" onclick="setMode(1)">AUTO CLEAN</button></div>
 <div class="grid">
  <section class="card" id="pM">
   <h2>Manual control</h2>
   <div id="joyWrap"><div id="joy"><div id="knob"></div></div></div>
   <div class="dpad">
    <span></span><button data-x="0" data-y="100">&#9650;</button><span></span>
    <button data-x="-100" data-y="0">&#9664;</button><button id="dpStop" style="background:var(--bad);color:#fff;font-size:.8rem;font-weight:800">STOP</button><button data-x="100" data-y="0">&#9654;</button>
    <span></span><button data-x="0" data-y="-100">&#9660;</button><span></span>
   </div>
   <div class="kv"><span>Speed <b id="spdTxt">150</b></span></div>
   <input type="range" id="spd" min="50" max="255" value="150">
   <div class="row"><button class="b g" onclick="act('/api/vacuum',{on:1})">VACUUM ON</button><button class="b o" onclick="act('/api/vacuum',{on:0})">VACUUM OFF</button></div>
   <h2 style="margin-top:.8rem">Front sensor servo</h2>
   <div class="row"><button class="b o" onclick="act('/api/servo',{pos:'left'})">LEFT</button><button class="b o" onclick="act('/api/servo',{pos:'center'})">CENTER</button><button class="b o" onclick="act('/api/servo',{pos:'right'})">RIGHT</button></div>
   <input type="range" id="svr" min="0" max="180" value="90">
  </section>
  <section class="card" id="pA">
   <h2>Auto clean</h2>
   <div class="kv"><span>State</span><b id="aState">IDLE</b></div>
   <div class="row"><button class="b g" onclick="act('/api/auto/start')">START AUTO CLEAN</button><button class="b r" onclick="act('/api/auto/stop')">STOP</button></div>
   <p class="hint">Vacuum starts after a 2 s warning beep. Obstacles: stop, scan, turn toward the clearer side. Turning is time-based (no encoders).</p>
  </section>
  <section class="card">
   <h2>Live status</h2>
   <div class="kv"><span>Front</span><b id="vF">--</b></div><div class="bar"><i id="bF"></i></div>
   <div class="kv"><span>Left</span><b id="vL">--</b></div><div class="bar"><i id="bL"></i></div>
   <div class="kv"><span>Right</span><b id="vR">--</b></div><div class="bar"><i id="bR"></i></div>
   <div class="kv"><span>Rear IR</span><b id="vIR">--</b></div>
   <div class="kv"><span>Servo</span><b id="vSV">--</b></div>
   <div class="kv"><span>Motors L / R</span><b id="vM">--</b></div>
   <div class="kv"><span>Vacuum</span><b id="vV">--</b></div>
   <div class="kv"><span>Mode</span><b id="vMode">--</b></div>
   <div class="kv"><span>Wi-Fi</span><b id="vW">--</b></div>
   <div class="kv"><span>Uptime</span><b id="vUp">--</b></div>
  </section>
 </div>
</main>
<button id="estop" onclick="doEstop()">EMERGENCY STOP</button>
<div id="toast"></div>


<div id="set">
 <div class="row"><h2 style="margin:0;flex:1">SETTINGS</h2><button class="b o" onclick="closeSet()">CLOSE</button></div>
 <p class="hint">Changes apply when you press SAVE PERMANENTLY. Values are stored in the ESP32 NVS flash and survive reboot and power cycles.</p>
 <div id="setBody"></div>
 <div id="setBar"><button class="b g" onclick="saveSet()">SAVE PERMANENTLY</button><button class="b o" onclick="restoreSet()">RESTORE SAVED</button><button class="b r" onclick="factorySet()">FACTORY DEFAULTS</button></div>
</div>

<script>
var $=function(i){return document.getElementById(i)};
var S={},link=false,spVal=150,toastT=0,audioCtx=null;
function esc(s){return String(s).replace(/&/g,'&amp;').replace(/"/g,'&quot;').replace(/</g,'&lt;')}
function toast(m,bad){var t=$('toast');t.textContent=m;t.className='show'+(bad?' bad':'');clearTimeout(toastT);toastT=setTimeout(function(){t.className=''},2400)}
function post(p,d){return fetch(p,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(d||{}).toString()}).then(function(r){return r.json()}).catch(function(){return{ok:false,msg:'No connection to robot'}})}
function act(p,d,quiet){return post(p,d).then(function(j){if(!quiet&&j.msg)toast(j.msg,!j.ok);return j})}
function vib(n){try{if(navigator.vibrate)navigator.vibrate(n||25)}catch(e){}}
function click(){try{if(!audioCtx)audioCtx=new (window.AudioContext||window.webkitAudioContext)();var o=audioCtx.createOscillator(),g=audioCtx.createGain();o.frequency.value=700;g.gain.value=.04;o.connect(g);g.connect(audioCtx.destination);o.start();o.stop(audioCtx.currentTime+.05)}catch(e){}}
document.addEventListener('pointerdown',function(e){if(e.target.closest&&e.target.closest('button'))click()},true);

/* theme + display mode */
var DM=['Compact','Normal','Large','Fullscreen'],dm=parseInt(localStorage.getItem('dm')||'1');
function applyDM(user){document.documentElement.style.fontSize=[13,16,19,19][dm]+'px';$('dmBtn').textContent=DM[dm];
 if(user){if(dm==3){if(!document.fullscreenElement&&document.documentElement.requestFullscreen)document.documentElement.requestFullscreen().catch(function(){})}else if(document.fullscreenElement&&document.exitFullscreen)document.exitFullscreen()}
 localStorage.setItem('dm',dm)}
function cycleDM(){dm=(dm+1)%4;applyDM(true)}
function toggleTheme(){var t=document.body.getAttribute('data-theme')=='dark'?'light':'dark';setTheme(t)}
function setTheme(t){document.body.setAttribute('data-theme',t);document.documentElement.setAttribute('data-theme',t);localStorage.setItem('theme',t)}
setTheme(localStorage.getItem('theme')||'dark');applyDM(false);
setTimeout(function(){var i=$('intro');i.style.opacity=0;setTimeout(function(){i.style.display='none'},650)},3000);

/* status polling */
function fmtUp(s){var h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h '+m+'m '+(s%60)+'s'}
function dist(v){return v<0?'--':v.toFixed(0)+' cm'}
function bar(id,v){$(id).style.width=(v<0?0:Math.min(100,v/2))+'%'}
function setMode(m){act('/api/mode',{m:m})}
function render(){
 $('wdot').className='dot'+(S.w?' ok':'');$('wtxt').textContent=S.w?('Wi-Fi '+S.rssi+' dBm'):'Wi-Fi lost';
 $('tabM').className='tab'+(S.mode==0?' on':'');$('tabA').className='tab'+(S.mode==1?' on':'');
 $('pM').style.display=S.mode==0?'':'none';$('pA').style.display=S.mode==1?'':'none';
 $('estBanner').className='banner red'+(S.est?' show':'');
 $('vF').textContent=dist(S.f);$('vL').textContent=dist(S.l);$('vR').textContent=dist(S.r);bar('bF',S.f);bar('bL',S.l);bar('bR',S.r);
 $('vIR').textContent=S.ir?'OBSTACLE':'CLEAR';$('vSV').textContent=S.sv+'\u00b0'+(S.scn?' (scanning)':'');
 $('vM').textContent=S.ml+' / '+S.mr;$('vV').textContent=['OFF','STARTING (beep)','ON'][S.vac];
 $('vMode').textContent=S.est?'EMERGENCY':(S.mode==0?'MANUAL':'AUTO CLEAN');$('vW').textContent=S.w?'Connected '+S.rssi+' dBm':'Disconnected';$('vUp').textContent=fmtUp(S.up);
 $('aState').textContent=S.as;
 if(document.activeElement!==$('spd')){$('spd').value=S.spd;$('spdTxt').textContent=S.spd;spVal=S.spd}
}
function renderLink(){$('wdot').className='dot';$('wtxt').textContent='Link lost'}
function poll(){fetch('/api/status',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){S=j;link=true;render()}).catch(function(){link=false;renderLink()}).then(function(){setTimeout(poll,250)})}
poll();

/* emergency stop */
function doEstop(){vib([200]);post('/api/estop').then(function(j){toast(j.msg,true)})}

/* manual driving */
var drv={x:0,y:0,on:false,busy:false,timer:0};
function sendDrive(force){if(drv.busy&&!force)return;drv.busy=true;post('/api/manual',{x:Math.round(drv.x),y:Math.round(drv.y),speed:spVal}).then(function(j){drv.busy=false;if(j&&j.ok===false&&drv.on&&!drv.warn){drv.warn=1;toast(j.msg,true)}})}
function driveStart(x,y){drv.x=x;drv.y=y;if(!drv.on){drv.on=true;drv.warn=0;vib();drv.timer=setInterval(function(){sendDrive(false)},120)}sendDrive(false)}
function driveStop(){if(drv.on){clearInterval(drv.timer);drv.on=false}drv.x=0;drv.y=0;drv.busy=false;sendDrive(true)}
var J=$('joy'),K=$('knob'),jid=null;
function jpos(e){var r=J.getBoundingClientRect(),R=r.width/2,dx=e.clientX-(r.left+R),dy=e.clientY-(r.top+R),m=Math.sqrt(dx*dx+dy*dy);if(m>R){dx=dx/m*R;dy=dy/m*R}K.style.transform='translate('+dx+'px,'+dy+'px)';driveStart(dx/R*100,-dy/R*100)}
J.addEventListener('pointerdown',function(e){jid=e.pointerId;try{J.setPointerCapture(jid)}catch(x){}jpos(e);e.preventDefault()});
J.addEventListener('pointermove',function(e){if(e.pointerId===jid)jpos(e)});
function jend(e){if(e.pointerId===jid){jid=null;K.style.transform='translate(0,0)';driveStop()}}
J.addEventListener('pointerup',jend);J.addEventListener('pointercancel',jend);
document.querySelectorAll('.dpad button[data-x]').forEach(function(b){
 b.addEventListener('pointerdown',function(e){e.preventDefault();try{b.setPointerCapture(e.pointerId)}catch(x){}driveStart(+b.dataset.x,+b.dataset.y)});
 var up=function(e){driveStop()};b.addEventListener('pointerup',up);b.addEventListener('pointercancel',up);b.addEventListener('pointerleave',function(){if(drv.on&&jid===null)driveStop()})});
$('dpStop').addEventListener('click',function(){driveStop();act('/api/stop')});
$('spd').addEventListener('input',function(){spVal=+this.value;$('spdTxt').textContent=spVal});
$('spd').addEventListener('change',function(){post('/api/manual',{speed:spVal})});
$('svr').addEventListener('change',function(){act('/api/servo',{angle:this.value},true)});
document.addEventListener('visibilitychange',function(){if(document.hidden&&drv.on)driveStop()});

/* settings */
var SECS=['General','Wi-Fi','Motor','Vacuum','Buzzer','RGB','Servo','Ultrasonic','Obstacle Avoidance','Manual Safety','Sensor Filtering','Hardware Test','Storage'];
function openSet(){$('set').classList.add('open');loadSet()}
function closeSet(){$('set').classList.remove('open')}
function fr(l,sm,inp){return '<div class="fr"><span>'+l+(sm?'<small>'+sm+'</small>':'')+'</span>'+inp+'</div>'}
function fld(x){var sm=(x.u?x.u+' &middot; ':'')+'default '+(x.t==1?(x.df?'on':'off'):(x.t==2?x.o.split('|')[x.df]:x.df))+(x.t==0?' &middot; '+x.mn+'..'+x.mx:'');var i;
 if(x.t==1)i='<input type="checkbox" data-k="'+x.k+'"'+(x.v?' checked':'')+'>';
 else if(x.t==2){i='<select data-k="'+x.k+'">';x.o.split('|').forEach(function(o,n){i+='<option value="'+n+'"'+(n==x.v?' selected':'')+'>'+esc(o)+'</option>'});i+='</select>'}
 else i='<input type="number" data-k="'+x.k+'" min="'+x.mn+'" max="'+x.mx+'" value="'+x.v+'">';
 return fr(esc(x.l),sm,i)}
function tb(label,p,d,out){return '<button class="b o" style="margin:.2rem" onclick="T(\''+p+'\','+JSON.stringify(d).replace(/"/g,'&quot;')+',\''+out+'\')">'+label+'</button>'}
function testHtml(){var h='<p class="hint">Hardware tests work in MANUAL mode. Lift the robot so the wheels are free. Emergency STOP always overrides tests.</p>';
 h+='<div><b>Buzzer</b><br>'+tb('Short','/api/test/buzzer',{type:'short'},'')+tb('Long','/api/test/buzzer',{type:'long'},'')+tb('Manual','/api/test/buzzer',{type:'manual'},'')+tb('Vacuum','/api/test/buzzer',{type:'vacuum'},'')+tb('Fault','/api/test/buzzer',{type:'fault'},'')+'</div>';
 h+='<div><b>RGB</b><br>'+tb('Red','/api/test/rgb',{c:'r'},'')+tb('Green','/api/test/rgb',{c:'g'},'')+tb('Blue','/api/test/rgb',{c:'b'},'')+tb('Off','/api/test/rgb',{c:'off'},'')+'</div>';
 h+='<div><b>Servo</b><br>'+tb('HOME','/api/servo',{pos:'home'},'')+tb('LEFT LIMIT','/api/servo',{pos:'left'},'')+tb('CENTER','/api/servo',{pos:'center'},'')+tb('RIGHT LIMIT','/api/servo',{pos:'right'},'')+tb('FULL SCAN','/api/servo',{pos:'scan'},'')+'<span class="res" id="scRes"></span></div>';
 h+='<div><b>Ultrasonic</b><br>'+tb('Front','/api/test/ultra',{id:0},'rU0')+'<span class="res" id="rU0"></span><br>'+tb('Left','/api/test/ultra',{id:1},'rU1')+'<span class="res" id="rU1"></span><br>'+tb('Right','/api/test/ultra',{id:2},'rU2')+'<span class="res" id="rU2"></span></div>';
 h+='<div><b>Rear IR</b><br>'+tb('Read IR','/api/test/ir',{},'rIR')+'<span class="res" id="rIR"></span></div>';
 h+='<div><b>Motors</b><br>'+tb('Left FWD','/api/test/motor',{side:'l',dir:'f'},'')+tb('Left REV','/api/test/motor',{side:'l',dir:'b'},'')+tb('Right FWD','/api/test/motor',{side:'r',dir:'f'},'')+tb('Right REV','/api/test/motor',{side:'r',dir:'b'},'')+'<button class="b r" style="margin:.2rem" onclick="act(\'/api/stop\')">STOP</button></div>';
 h+='<div><b>Vacuum relay</b><br>'+tb('Relay test (auto OFF)','/api/test/vacuum',{},'')+'</div>';return h}
function T(p,d,o){post(p,d).then(function(j){var e=o?$(o):null;
 if(e){if(j.cm!==undefined)e.textContent=j.valid?(j.cm.toFixed(1)+' cm - OK'):'ERROR (no valid reading)';else if(j.obstacle!==undefined)e.textContent=(j.obstacle?'OBSTACLE':'CLEAR')+' - OK (raw '+j.raw+')';else e.textContent=j.msg||''}
 else toast(j.msg||'OK',!j.ok)})}
setInterval(function(){var e=$('scRes');if(e&&S.sl!==undefined)e.textContent=S.scn?'scanning...':'last scan L '+(S.sl<0?'--':S.sl.toFixed(0))+' / R '+(S.sr<0?'--':S.sr.toFixed(0))+' cm'},500);
function storHtml(s){return fr('Saved settings in NVS','',s.saved?'<b>YES</b>':'<b>NO (defaults)</b>')+fr('Settings count','',s.n)+fr('Free heap','',Math.round(s.heap/1024)+' KB')+fr('Sketch size','',Math.round(s.sketch/1024)+' KB')+fr('Flash chip','',Math.round(s.flash/1048576)+' MB')+'<p class="hint">Settings are stored in NVS via Preferences. The firmware itself is not rewritten.</p>'}
function loadSet(){fetch('/api/settings',{cache:'no-store'}).then(function(r){return r.json()}).then(function(d){var h='';
 SECS.forEach(function(sec){h+='<details class="sec"><summary>'+sec+'</summary><div class="sb">';
  if(sec=='Wi-Fi')h+=fr('SSID','','<input type="text" id="wssid" maxlength="32" value="'+esc(d.ssid)+'">')+fr('Password','','<input type="password" id="wpass" maxlength="63" placeholder="(unchanged)">')+'<p class="hint">'+(d.wc?'Connected.':'Not connected.')+' New credentials reconnect immediately; if wrong, fix them via the sketch.</p>';
  d.s.forEach(function(x){if(x.sec==sec)h+=fld(x)});
  if(sec=='Hardware Test')h+=testHtml();
  if(sec=='Storage')h+=storHtml(d.st);
  h+='</div></details>'});
 $('setBody').innerHTML=h})}
function saveSet(){var d={};document.querySelectorAll('#setBody [data-k]').forEach(function(e){d[e.dataset.k]=e.type=='checkbox'?(e.checked?1:0):e.value});
 if($('wssid'))d.wssid=$('wssid').value;if($('wpass')&&$('wpass').value)d.wpass=$('wpass').value;
 act('/api/settings/save',d).then(function(j){if(j.ok)loadSet()})}
function restoreSet(){act('/api/settings/restore').then(function(j){if(j.ok)loadSet()})}
function factorySet(){if(confirm('Restore ALL settings to factory defaults? (Wi-Fi credentials are kept)'))act('/api/settings/factory').then(function(j){if(j.ok)loadSet()})}
</script></body></html>)HTMLPAGE";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  settingsDefaults(); wifiDefaults();
  settingsLoad();                                   // NVS values (or defaults)

  // 1. Safe GPIO initialisation: motors OFF, vacuum OFF
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
  pinMode(PIN_RELAY, OUTPUT); relayWrite(false);
  pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_B, OUTPUT); rgbWrite(false, false, false);
  pinMode(PIN_IR_REAR, INPUT);
  pinMode(PIN_ENA, OUTPUT); digitalWrite(PIN_ENA, LOW);   // enable pins LOW until PWM is attached (no full-speed glitch)
  pinMode(PIN_ENB, OUTPUT); digitalWrite(PIN_ENB, LOW);

  // Servo first: it claims its own LEDC timer so the motor PWM is never reconfigured by it
  ESP32PWM::allocateTimer(3);
  frontServo.setPeriodHertz(50);
  frontServo.attach(PIN_SERVO, SERVO_MIN_US, SERVO_MAX_US);
  servoGo(cfg.svCenter);
#if LEDC_NEW_API
  ledcAttach(PIN_ENA, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttach(PIN_ENB, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttach(PIN_BUZZER, 2000, 8);
#else
  ledcSetup(LEDC_CH_ENA, MOTOR_PWM_HZ, MOTOR_PWM_BITS); ledcAttachPin(PIN_ENA, LEDC_CH_ENA);
  ledcSetup(LEDC_CH_ENB, MOTOR_PWM_HZ, MOTOR_PWM_BITS); ledcAttachPin(PIN_ENB, LEDC_CH_ENB);
  ledcSetup(LEDC_CH_BUZZ, 2000, 8); ledcAttachPin(PIN_BUZZER, LEDC_CH_BUZZ);
#endif
  stopMotors();
  buzzTone(0);

  // ultrasonic sensors
  us[0].trig = PIN_TRIG_F; us[0].echo = PIN_ECHO_F;
  us[1].trig = PIN_TRIG_L; us[1].echo = PIN_ECHO_L;
  us[2].trig = PIN_TRIG_R; us[2].echo = PIN_ECHO_R;
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(us[i].trig, OUTPUT); digitalWrite(us[i].trig, LOW);
    pinMode(us[i].echo, INPUT);
    us[i].seq = 0; usReset(i);
  }

  // 5. Boot indication starts: RGB R->G->B, then beeps, then Wi-Fi (see bootTick)
  bootStep = B_R; bootT = millis();
  Serial.println("Booting...");
}

// ===================== LOOP =====================
void loop() {
  bootTick();
  usTick();
  irTick();
  scanTick();
  safetyEnforce();
  autoTick();
  manualTick();
  testTick();
  vacuumTick();
  buzzTick();
  rgbTick();
  wifiTick();
  if (serverStarted) server.handleClient();
}
