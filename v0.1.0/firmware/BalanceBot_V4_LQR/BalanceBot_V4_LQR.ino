// ============================================================================
//  BalanceBot V4 - LQR state feedback, SINGLE FILE
// ============================================================================
//  This is the OLD pre-split BalanceBot_V2_LQR sketch, back in one .ino.
//
//  The literal V2 .ino no longer exists on disk - it was split into modules on
//  10 Aug 2026 at 02:13, the folder was renamed BalanceBot_V3_LQR, and the one
//  backup copy went to a /tmp scratchpad that has since been cleared. What DOES
//  survive is the Arduino build cache, which froze every module as it stood at
//  02:53 that morning, before any of the later V3 work. Those files are merged
//  back together here; the split was a pure code move, so this is the V2 sketch.
//
//  It is therefore V2 as it ran that night - including the fixes made in the
//  ~40 min after the split (EN latch order, gyro dt on a stalled cycle, the
//  separate /a-poll watchdog for the speed test, addLog formatting outside the
//  lock, maxAccel default) - and NONE of the later V3 additions: no rail/stall
//  watchdog, no esp_reset_reason boot reason, no no-cache headers on /, no
//  uptime in telemetry, and the older Page.cpp UI.
//
//  Merge edits are mechanical only: #include "..." lines and #pragma once are
//  gone, <system> includes hoisted to the top, and the prototypes that lived in
//  the headers gathered into one forward-declaration block below.
//
//  Layout, top to bottom:
//    1. includes + pin/timing constants        (was Config.h)
//    2. every global                           (was State.h)
//    3. I2CLock + forward declarations         (was the module headers)
//    4. the web page                           (was Page.cpp)
//    5. step ISR and motor plumbing            (was Hardware.cpp)
//    6. MPU + Kalman                           (was Imu.cpp)
//    7. config persistence                     (was Storage.cpp)
//    8. log ring + serial plotter              (was Telemetry.cpp)
//    9. the LQR control step                   (was Control.cpp)
//   10. the HTTP handlers                      (was Web.cpp)

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// ==========================================================================
// 1. CONFIG - pins, timing, limits
// ==========================================================================

// WiFi credentials live in secrets.h, which is gitignored.
// Copy secrets.h.example to secrets.h and fill in your own network.
#include "secrets.h"

const int STEP_PIN = 25, DIR_PIN = 26, EN_PIN = 27;      // RIGHT
const int STEP2_PIN = 18, DIR2_PIN = 19, EN2_PIN = 23;   // LEFT
// ================= HARDWARE STEP GENERATOR =================
// The old code emitted at most ONE step per loop() pass and clamped the step period
// to 8000us, which pinned the wheels to a flat 125 steps/s for every angle under ~6deg.
// A timer ISR running a DDS phase accumulator gives an exact, jitter-free step rate
// that is truly proportional to the PID output and completely immune to WiFi stalls.
const uint32_t ISR_HZ      = 20000;             // 50us tick
const uint32_t CTRL_MS     = 5;                 // control task period, 200Hz
const float    MAX_STEP_HZ = ISR_HZ / 2 - 1000; // DDS cannot exceed half the tick rate
const float    MIN_STEP_HZ = 2.0;               // below this, just stop

const uint32_t MASK_R = 1UL << STEP_PIN;
const uint32_t MASK_L = 1UL << STEP2_PIN;
const uint32_t MASK_DIR_R = 1UL << DIR_PIN;
const uint32_t MASK_DIR_L = 1UL << DIR2_PIN;

const uint32_t I2C_HZ = 400000;   // MPU6050 is rated for 400k; 100k made every sample 4x slower
const float MAX_OUT = 12000;

const unsigned long CMD_TIMEOUT_MS  = 1000;   // /a polls every 150ms
const unsigned long TEST_TIMEOUT_MS = 3000;


// ==========================================================================
// 2. STATE - every global, once
// ==========================================================================

// Every global lives here ONCE, as a C++17 inline variable. The usual Arduino/MultiWii
// pattern is a definition in one .cpp plus an extern in a header - each global written
// twice, and a type that drifts between the two links cleanly and then corrupts memory.
// inline variables have no second declaration to get wrong.

inline hw_timer_t *stepTimer = nullptr;
inline volatile uint32_t phaseR = 0, phaseL = 0;
inline volatile uint32_t incR = 0, incL = 0;   // step rate = inc * ISR_HZ / 2^32
inline volatile bool pulseR = false, pulseL = false;

// Free odometry: we emit every step ourselves, so counting them IS wheel position.
// Counted in "output space" (sign of the commanded output), so invLeft does
// not matter and averaging the two wheels cancels any turn.
inline volatile int32_t posR = 0, posL = 0;
inline volatile int8_t  sgnR = 1, sgnL = 1;

// posR/posL are read-modify-written by the ISR, so anything outside the ISR that
// touches them needs the same lock - leanReset() zeroing them from a web handler could
// land between the ISR's read and its write and silently lose the reset.
inline portMUX_TYPE stepMux = portMUX_INITIALIZER_UNLOCKED;

// DIR is applied BY THE ISR, on a tick that emitted no pulse. Writing it from the
// control loop raced the step pulses: a reversal could land nanoseconds either side of
// a STEP rising edge, violating the driver's 200ns DIR setup/hold and stepping one
// count the wrong way - which the odometry then counted the NEW way.
inline volatile bool   dirPendR = false, dirPendL = false;
inline volatile bool   dirLvlR  = false, dirLvlL  = false;
inline volatile int8_t sgnPendR = 1,     sgnPendL = 1;
// ---- DIRECTION FIXES ----
// These two are the settings most likely to be wrong on any given build, so they are
// RUNTIME toggles with a UI button, not compile-time constants. Getting invLeft wrong
// makes the two wheels fight each other (the robot spins instead of driving, and can
// never balance); getting invOut wrong makes it drive the way it is already falling.
inline bool invLeft = false;   // the motors are mounted mirrored, so one usually needs flipping
inline bool invOut  = true;    // sign of the balance correction
// The third sign trap, and the one that used to need a reflash: the Kalman is only
// valid if gyro X really is the derivative of atan2(accY,accZ). If plot group 1 shows
// gyroI walking away from acc in the OPPOSITE direction, flip this.
inline bool invGyro = false;

// ---- SERIAL PLOTTER: set false to silence ----
inline bool plotEnabled = true;

inline Adafruit_MPU6050 mpu;
inline WebServer server(80);

// The control loop now runs in its own task, so the web handlers that touch the MPU
// (/raw, /scan, /kreset) are genuinely concurrent with it. Recursive, because
// seedFilter() reads the sensor AND calls calibrateGyroBias(), which reads it too.
inline SemaphoreHandle_t i2cMux = nullptr;
// ================= LQR STATE FEEDBACK =================
// One flat control law over all four states of the inverted pendulum, after
// remrc/Two-Wheel-Balancing-Robot. Replaces V1's cascade (angle PID + separate
// position loop) - the position states are inside the SAME equation, not a slower
// outer loop, so there are no two loops that can fight each other.
//
//   u = K1*angle + K2*angleRate + K3*wheelSpeed + K4*position     [acceleration]
//
// All four gains are normally POSITIVE. u is an ACCELERATION: steppers are velocity
// devices, so it is integrated into velCmd before it reaches the wheels.
inline float K1 = 1200.0;   // angle          deg
inline float K2 = 60.0;     // angle rate     deg/s
inline float K3 = 1000.0;   // wheel speed    ksteps/s
inline float K4 = 400.0;    // position       ksteps
inline float targetAngle = 0.0, turnBias = 0.0;
inline float driveSpeed = 0.0;      // D-pad: biases the position setpoint, steps/s

// ---- mahowik's POSITION HOLD, ported from BalancingWii.cpp:1241 ----
// K4 measures position from the STEP COUNTER, which lies whenever the steppers skip -
// exactly what happens during the hard push you are trying to recover from. mahowik
// instead integrates the FILTERED SPEED, which skipping cannot fake as cleanly, and
// only while the operator is not driving. Same 0.92/0.08 filter he uses.
inline float Kph = 0.0;             // 0 = off. correction per 1000 steps of accumulated drift
// mahowik has this as a MODE box (BOXPOSHOLD), not just a gain - so you can switch
// holding on and off while it is balancing and watch the difference immediately.
// Turning it ON re-homes: "hold HERE", not "go back to where you were engaged".
inline bool  posHold = true;
inline float posDrift = 0;          // integral of filtered speed = distance from "home"
inline float speedFilt = 0;         // 0.92/0.08 low pass on wheel speed, as in BalancingWii
inline float t5 = 0;                // the position-hold term, for the plot

// ---- SECOND DERIVATIVE of angle (angular acceleration) ----
// Asked for explicitly. Be aware of what it is: theta-doubledot is not an independent
// state of an inverted pendulum - the dynamics fix it as f(theta, theta_dot, u) - so in
// the ideal case this is redundant with K1/K2. And it is obtained by differentiating
// the gyro, which multiplies noise by frequency, so it needs the low pass below, which
// gives back some of the lead it was supposed to buy. It defaults to 0 for those two
// reasons. Where it CAN earn its keep is real hardware: the true plant has flex,
// backlash and stepper lag that the ideal model does not, and a little theta-doubledot
// can anticipate those. Raise it slowly and watch for buzz at rest.
inline int   mpuBandHz = 44;        // MPU6050 DLPF corner, Hz - see configureMPU()
inline float Kaa = 0.0;             // gain on angular acceleration, deg/s^2 (0 = off)
inline float aaTau = 0.02;          // low pass on the differentiated gyro, seconds
inline float gRatePrev = 0, gAccFilt = 0;
inline float t6 = 0;                // the angular-acceleration term, for the plot
inline float angle = 0, output = 0;
inline float uAccel = 0;            // last acceleration command, for the plot
inline float t1=0,t2=0,t3=0,t4=0;   // individual LQR term contributions, for the plot
inline float accAngleGlobal = 0;
inline bool mpuOK = false;
inline unsigned long lastLoop = 0, lastRetry = 0, lastPlot = 0;

inline bool testMode = false;
inline volatile int testInterval = 2000;

// Runtime-tunable: d-pad feel is a per-robot thing (wheel size, gearing, how twitchy
// you want it), so it does not belong behind a recompile.
inline float DRIVE_STEPS = 1200.0;   // setpoint slew while a direction is held, steps/s
inline float TURN_AMOUNT = 120.0;    // differential steps/s while turning


// ---- A/B experiment switches, toggled from the web UI while watching the plot ----
inline bool adaptiveR = false;   // inflate Kalman R when the accel vector is not pure gravity
inline float velCmd = 0;         // integrated wheel-speed command, output units

// ---- effort / output scaling ----
// stepGain is steps/s per unit of PID output. The old hard-coded 1/1.2 is the default
// so top speed is unchanged; raise it for more wheel authority at a given Kp.
inline float stepGain = 0.83;

inline double posTarget = 0;      // double, not long: driveSpeed*dt is fractional per cycle
inline float posErrSteps = 0, wheelVel = 0;

// Copied from the reference project (robot_position is clamped to +/-30 there).
// Without a clamp the position state grows without bound while the robot is held off
// the ground or blocked, and K4 then swamps every other term the moment it engages.
inline float MAX_POS_ERR = 6000.0;   // steps, ~38cm at 1/16 microstep / 65mm wheels

// Auto engage/disengage, after the reference project's "vertical" flag. BALANCE ON
// only ARMS the robot; it engages itself when stood upright and drops out when tipped,
// so you can pick it up and set it down without touching the phone.
inline bool  armed = false, engaged = false;
inline float ENGAGE_DEG    = 2.0;    // stand it inside this and it catches itself
inline float DISENGAGE_DEG = 40.0;   // tipped past this, motors off

// Step-rate slew limit. A stepper commanded 0 -> 2500 steps/s in one 5ms cycle loses
// sync, produces no torque and skips - and the ISR still counts those steps, so the
// odometry silently lies. 0 = unlimited (old behaviour); tune while watching group 6.
inline float maxAccel = 0;               // steps/s^2, 0 = off
inline float sRateR = 0, sRateL = 0;     // signed applied rate, steps/s
inline float rateWanted = 0;             // pre-clamp demand, drives the SAT indicator
inline float ctrlHz = 0;                 // measured control-task rate, Hz

// Drive-command watchdog. /move has no repeat, so a dropped WiFi link or a closed tab
// while holding a direction used to leave the lean latched and the robot driving away.
inline unsigned long lastCmdMs = 0;
// The speed test needs the OPPOSITE watchdog to the d-pad. Its buttons are one-shot -
// nothing repeats /test - so timing it off lastCmdMs killed every test after exactly
// 3s and logged "link lost" on a perfectly healthy link. What the test actually wants
// to detect is "the operator's page went away", and /a polling every 150ms is exactly
// that signal. Kept separate so /a can never become a d-pad keepalive again.
inline unsigned long lastPollMs = 0;

// ---- signals kept around only so the plotter can show them ----
inline float gyroRate = 0, gyroIntAngle = 0, compAngle = 0;
inline float accMag = 0, accErr = 0;
inline float stepHzR = 0, stepHzL = 0;
inline float dtLast = 0;
inline float accNoise = 0;   // running RMS of accel-vs-fused disagreement, in degrees
inline float accErrRms = 0;  // running RMS of ||a||-g, in m/s^2
inline int   plotGroup = 1;

// ================= KALMAN FILTER =================
inline float Q_angle = 0.001;
inline float Q_bias  = 0.003;
inline float R_meas  = 0.03;
inline float R_used  = 0.03;   // R actually applied this cycle (differs from R_meas when adaptiveR)

inline float K_angle = 0;
inline float K_bias  = 0;
inline float K_innov = 0;      // accel angle minus predicted angle - the filter's health signal
inline float P[2][2] = {{0,0},{0,0}};
// ---- tuning persistence ----
// Without this every reset silently reverted Kp to 25, which at the new proportional
// step rate is only ~21 steps/s per degree - indistinguishable from "not responding".
inline Preferences prefs;
inline bool cfgDirty = false;
inline unsigned long cfgTouched = 0;

inline char i2cInfo[96] = "not scanned";
inline int i2cCount = 0;
inline bool foundMPU = false;
// Fixed buffers, not String: these are retained for the life of the program, and a
// retained String reassigned on every state change is how an ESP32 heap fragments.
inline char logLines[6][72];
inline int logIdx = 0;
inline portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;


// ==========================================================================
// 3. FORWARD DECLARATIONS - what the headers used to publish
// ==========================================================================

// The control loop now runs in its own task, so the web handlers that touch the MPU
// (/raw, /scan, /kreset) are genuinely concurrent with it. Recursive, because
// seedFilter() reads the sensor AND calls calibrateGyroBias(), which reads it too.
struct I2CLock {
  I2CLock(){ if (i2cMux) xSemaphoreTakeRecursive(i2cMux, portMAX_DELAY); }
  ~I2CLock(){ if (i2cMux) xSemaphoreGiveRecursive(i2cMux); }
};

void IRAM_ATTR onStepTimer();
int32_t readPosAvg();
void resetPos();
void publishDir(bool lvlR, bool lvlL, int8_t sR, int8_t sL);
uint32_t rateToInc(float hz);
void leanReset();
void motorsOff();
void motorsOn();
float velLimit();
void setStepRates();
float kalmanUpdate(float newAngle, float newRate, float dt, float R);
void kalmanReset(float startAngle, float startBias);
void configureMPU();
float calibrateGyroBias();
void seedFilter();
void scanI2C();
bool jNum(const String& j, const char* key, float& out);
bool jBool(const String& j, const char* key, bool& out);
void saveCfg();
void loadCfg();
void markCfgDirty();
void addLog(const String& s);
String logsAsJson();
void plotSignals();
void controlStep();
void setupWeb();
void controlTask(void*);


// ==========================================================================
// 4. THE WEB PAGE
// ==========================================================================

// PROGMEM, so the 23KB of markup stays in flash and is streamed by send_P() rather
// than copied into the ~300KB of RAM the WiFi stack is already competing for.
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
 *{box-sizing:border-box}
 /* touch-action:manipulation kills the 300ms tap delay and stops a press being
    stolen by scroll gestures, which is why buttons felt dead or laggy on phones */
 button{touch-action:manipulation;-webkit-tap-highlight-color:transparent;cursor:pointer}
 input[type=range]{touch-action:pan-x}
 /* user-select:none used to sit on BODY, which made the whole page uncopyable -
    you could not select a log line or a value to paste somewhere. It only ever needed
    to cover the controls, so a press-and-hold on the d-pad does not start a selection. */
 body{font-family:system-ui;background:#0f1220;color:#eee;text-align:center;padding:18px;margin:0}
 button,.btn,input[type=range]{user-select:none;-webkit-user-select:none}
 #log,#noi,#i2c,#v,#st,#hz{user-select:text;-webkit-user-select:text;cursor:text}
 #log{text-align:left;white-space:pre-wrap;word-break:break-word}
 h2{margin:4px 0 10px}
 #v{font-size:25px;color:#4ade80;font-weight:600;margin:6px 0 2px}
 #st{font-size:13px;margin-bottom:4px;font-weight:600}
 #i2c{font-size:12px;color:#94a3b8;margin-bottom:10px;font-family:monospace}
 .main button{font-size:16px;padding:13px 26px;margin:5px;border:0;border-radius:14px;color:#fff;font-weight:600}
 .go{background:#22c55e}.stop{background:#ef4444}.scan{background:#6366f1;font-size:13px;padding:9px 18px}
 .sp{background:#0ea5e9;color:#fff;border:0;border-radius:10px;font-size:13px;padding:11px 14px;margin:3px;font-weight:600}
 .pad{display:grid;grid-template-columns:repeat(3,80px);grid-gap:9px;justify-content:center;margin:14px auto}
 .btn{height:80px;border:0;border-radius:18px;display:flex;align-items:center;justify-content:center;
      background:linear-gradient(145deg,#3b82f6,#2563eb);box-shadow:0 4px 12px rgba(0,0,0,.4);
      transition:transform .08s,filter .08s}
 .btn:active{transform:scale(.92);filter:brightness(1.3)}
 .empty{visibility:hidden}
 .arrow{width:0;height:0}
 .up .arrow{border-left:13px solid transparent;border-right:13px solid transparent;border-bottom:19px solid #fff}
 .down .arrow{border-left:13px solid transparent;border-right:13px solid transparent;border-top:19px solid #fff}
 .lf .arrow{border-top:13px solid transparent;border-bottom:13px solid transparent;border-right:19px solid #fff}
 .rt .arrow{border-top:13px solid transparent;border-bottom:13px solid transparent;border-left:19px solid #fff}
 .ctr{background:#334155}
 .dot{width:15px;height:15px;background:#94a3b8;border-radius:50%}
 .row{margin:12px 0;font-size:13px;color:#9aa4bf}
 .hdr{margin-top:16px;font-size:12px;color:#64748b;letter-spacing:1px;text-transform:uppercase}
 input[type=range]{width:76%;margin-top:6px}
 input[type=number]{width:88px;margin-left:8px;background:#1e2438;color:#eee;
   border:1px solid #334155;border-radius:6px;padding:5px;font-size:14px;text-align:center}
 #log{margin-top:14px;background:#0a0d18;border-radius:10px;padding:8px;font-family:monospace;
      font-size:11px;color:#64748b;text-align:left;min-height:70px;line-height:1.6}
</style></head><body>
<h2>Balance Control &mdash; V2 LQR</h2>
<div id="v">angle 0.0</div>
<div id="st">connecting to board...</div>
<div id="ack" style="width:100%;height:3px;border-radius:2px;background:#334155;opacity:0.15;transition:opacity .2s;margin:2px 0 6px"></div>
<div id="hz" style="font-size:13px;color:#94a3b8;font-family:monospace;margin-bottom:4px">0 st/s</div>
<div id="i2c">i2c: --</div>
<div id="noi" style="font-size:12px;color:#94a3b8;margin-bottom:8px;font-family:monospace">noise: --</div>
<div class="main">
<button class="go" onclick="send('/on')">ARM</button>
<button class="stop" onclick="send('/off')">DISARM</button><br>
<button class="scan" onclick="send('/scan')">SCAN I2C BUS</button>
<button class="scan" onclick="send('/kreset')">RESET KALMAN</button>
<button class="scan" onclick="send('/plot')">PLOT ON/OFF</button>
<button class="scan" style="background:#0f766e" onclick="window.location='/cfgfile?dl=1'">SAVE CONFIG</button>
<button class="scan" style="background:#0f766e" onclick="copyCfg()">COPY CONFIG</button>
<button class="scan" style="background:#7c2d12" onclick="if(confirm('Reset all TUNING to defaults?\n\nMotor direction (swap L / invert output / invert gyro) is KEPT - those are wiring facts, not tuning. Use RESET DIRECTION for those.')){send('/defaults');setTimeout(loadCfg,300);}">RESET TUNING</button>
</div>

<div class="pad">
  <div class="empty"></div>
  <button class="btn up" onpointerdown="hold(event,'fwd')" onpointerup="relD('fwd')" onpointercancel="relD('fwd')"><div class="arrow"></div></button>
  <div class="empty"></div>
  <button class="btn lf" onpointerdown="hold(event,'left')" onpointerup="relD('left')" onpointercancel="relD('left')"><div class="arrow"></div></button>
  <button class="btn ctr" onclick="rel()"><div class="dot"></div></button>
  <button class="btn rt" onpointerdown="hold(event,'right')" onpointerup="relD('right')" onpointercancel="relD('right')"><div class="arrow"></div></button>
  <div class="empty"></div>
  <button class="btn down" onpointerdown="hold(event,'back')" onpointerup="relD('back')" onpointercancel="relD('back')"><div class="arrow"></div></button>
  <div class="empty"></div>
</div>

<div class="hdr">Wheel speed test (no PID)</div>
<div>
<button class="sp" onclick="send('/test?v=4000')">4000us</button>
<button class="sp" onclick="send('/test?v=1500')">1500us</button>
<button class="sp" onclick="send('/test?v=800')">800us</button>
<button class="sp" onclick="send('/test?v=400')">400us</button>
<button class="sp" onclick="send('/test?v=200')">200us</button>
<button class="sp" onclick="send('/test?v=100')">100us</button>
<button class="sp" style="background:#ef4444" onclick="send('/test?v=50')">50us</button>
<br>
<button class="stop" style="font-size:14px;padding:10px 24px;margin-top:6px" onclick="send('/teststop')">STOP TEST</button>
</div>

<div class="hdr">Serial plot group</div>
<div>
<button class="sp" onclick="send('/plotg?g=1')">1 fusion</button>
<button class="sp" onclick="send('/plotg?g=2')">2 kalman</button>
<button class="sp" onclick="send('/plotg?g=3')">3 pid</button>
<button class="sp" onclick="send('/plotg?g=4')">4 timing</button>
<button class="sp" onclick="send('/plotg?g=5')">5 overview</button>
<button class="sp" onclick="send('/plotg?g=6')">6 motor</button>
<button class="sp" onclick="send('/plotg?g=7')">7 position</button>
<button class="sp" style="background:#64748b" onclick="send('/plotg?g=0')">off</button>
</div>

<div class="hdr">Experiments (A/B while plotting)</div>
<div>
<button class="sp" id="bil" onclick="tog('invl')">swap L motor</button>
<button class="sp" id="bio" onclick="tog('invo')">invert output</button>
<button class="sp" id="big" onclick="tog('igyro')">invert gyro</button>
<div style="font-size:11px;color:#94a3b8;margin-top:4px">
 These three are wiring facts, so RESET TUNING deliberately leaves them alone.
 They are stored in flash and survive a reflash - the compile-time value is only
 used the first time, so editing the source does nothing once a value is saved.
</div>
<button class="scan" style="background:#7c2d12;font-size:12px"
  onclick="if(confirm('Reset motor direction to compile-time defaults?\n(swap L = off, invert output = ON, invert gyro = off)')){send('/dirreset');setTimeout(loadCfg,300);}">RESET DIRECTION</button>
<button class="sp" id="bar" onclick="tog('adaptr')">adaptive R</button>

</div>

<div class="hdr">LQR gains - the whole controller</div>
<div class="row" style="font-size:12px;color:#64748b">u = K1&middot;angle + K2&middot;rate + K3&middot;speed + K4&middot;position &nbsp; (all normally positive)<br>
Tune in order: K1 and K2 with K3=K4=0. Then K3. Then K4.</div>

<div class="row">K1 &mdash; angle <span id="k1v">1200</span>
<input type="number" id="k1n" value="1200" step="50" onchange="setv('k1',this.value,'k1v','k1s')"><br>
<input type="range" id="k1s" min="0" max="6000" step="10" value="1200" oninput="setv('k1',this.value,'k1v','k1n')"></div>

<div class="row">K2 &mdash; angle rate <span id="k2v">60</span>
<input type="number" id="k2n" value="60" step="5" onchange="setv('k2',this.value,'k2v','k2s')"><br>
<input type="range" id="k2s" min="0" max="600" step="1" value="60" oninput="setv('k2',this.value,'k2v','k2n')"></div>

<div class="row">K3 &mdash; wheel speed <span id="k3v">1000</span>
<input type="number" id="k3n" value="1000" step="50" onchange="setv('k3',this.value,'k3v','k3s')"><br>
<input type="range" id="k3s" min="0" max="6000" step="10" value="1000" oninput="setv('k3',this.value,'k3v','k3n')"></div>

<div class="row">K4 &mdash; position <span id="k4v">400</span>
<input type="number" id="k4n" value="400" step="25" onchange="setv('k4',this.value,'k4v','k4s')"><br>
<input type="range" id="k4s" min="0" max="3000" step="5" value="400" oninput="setv('k4',this.value,'k4v','k4n')"></div>

<div class="row">Target angle <span id="tav">0.0</span>
<input type="number" id="tan" value="0" step="0.1" onchange="setv('ta',this.value,'tav','tas')"><br>
<input type="range" id="tas" min="-20" max="20" step="0.1" value="0" oninput="setv('ta',this.value,'tav','tan')"></div>

<div class="hdr">Motor</div>

<div class="row">Step gain (steps/s per output) <span id="sgv">0.83</span>
<input type="number" id="sgn" value="0.83" step="0.05" onchange="setv('sg',this.value,'sgv','sgs')"><br>
<input type="range" id="sgs" min="0.1" max="10" step="0.01" value="0.83" oninput="setv('sg',this.value,'sgv','sgn')"></div>
<div class="row" style="font-size:12px;color:#64748b">wheel rate caps at 9000 st/s - "SAT" above means more gain buys nothing</div>

<div class="row">Max accel (0 = off) <span id="mav">0</span>
<input type="number" id="man" value="0" step="1000" onchange="setv('ma',this.value,'mav','mas')"><br>
<input type="range" id="mas" min="0" max="200000" step="1000" value="0" oninput="setv('ma',this.value,'mav','man')"></div>
<div class="row"><button class="sp" id="bpk" onclick="tog('poshold')">POSITION HOLD</button>
<span style="font-size:12px;color:#64748b">&nbsp;purple = holding. Pressing it re-homes to where the robot is right now.</span></div>

<div class="row">Kph - position hold (0 = off) <span id="phv">0</span>
<input type="number" id="phn" value="0" step="10" onchange="setv('ph',this.value,'phv','phs')"><br>
<input type="range" id="phs" min="0" max="3000" step="10" value="0" oninput="setv('ph',this.value,'phv','phn')"></div>
<div class="row" style="font-size:12px;color:#64748b">mahowik's hold: integrates filtered SPEED, not the step counter, so skipped steps cannot fake it. Only accumulates while the d-pad is idle.</div>

<div class="row">Kaa - angle 2nd derivative (0 = off) <span id="aav">0</span>
<input type="number" id="aan" value="0" step="10" onchange="setv('aa',this.value,'aav','aas')"><br>
<input type="range" id="aas" min="0" max="2000" step="10" value="0" oninput="setv('aa',this.value,'aav','aan')"></div>

<div class="row">Kaa low-pass tau (s) <span id="atv">0.020</span>
<input type="number" id="atn" value="0.02" step="0.005" onchange="setv('at',this.value,'atv','ats')"><br>
<input type="range" id="ats" min="0.002" max="0.2" step="0.002" value="0.02" oninput="setv('at',this.value,'atv','atn')"></div>
<div class="row" style="font-size:12px;color:#64748b">smaller tau = more lead but more noise. This term is redundant in the ideal model - it earns its keep only against real flex/backlash/stepper lag.</div>

<h2>Sensor lag - the biggest delay in the loop</h2>
<div class="row" style="font-size:12px;color:#64748b">MPU6050 filter group delay: 44Hz = 4.9ms, 94Hz = 3.0ms, 184Hz = 2.0ms, 260Hz = 0.98ms. Wider reacts sooner but feeds more noise to the gyro and the accel angle.</div>
<div class="row">
<button class="sp" onclick="send('/set?bw=44');setTimeout(loadCfg,250)">44 Hz</button>
<button class="sp" onclick="send('/set?bw=94');setTimeout(loadCfg,250)">94 Hz</button>
<button class="sp" onclick="send('/set?bw=184');setTimeout(loadCfg,250)">184 Hz</button>
<button class="sp" onclick="send('/set?bw=260');setTimeout(loadCfg,250)">260 Hz</button>
<span id="bwv" style="margin-left:8px">44</span> Hz
</div>

<h2>Drive (d-pad or WASD / arrow keys) and limits</h2>
<div class="row" style="font-size:12px;color:#64748b">Forward/back moves the position SETPOINT, so the LQR chases it and the position term keeps working while you drive. Hold two directions together to arc. Hold left or right alone to spin in place - keep holding for a full 360.</div>

<div class="row">Drive speed (steps/s) <span id="dsv">1200</span>
<input type="number" id="dsn" value="1200" step="100" onchange="setv('ds',this.value,'dsv','dss')"><br>
<input type="range" id="dss" min="0" max="6000" step="50" value="1200" oninput="setv('ds',this.value,'dsv','dsn')"></div>

<div class="row">Turn rate (steps/s) <span id="tnv">120</span>
<input type="number" id="tnn" value="120" step="10" onchange="setv('tn',this.value,'tnv','tns')"><br>
<input type="range" id="tns" min="0" max="2000" step="10" value="120" oninput="setv('tn',this.value,'tnv','tnn')"></div>

<div class="row">Max position error (steps) <span id="mpv">6000</span>
<input type="number" id="mpn" value="6000" step="500" onchange="setv('mp',this.value,'mpv','mps')"><br>
<input type="range" id="mps" min="500" max="40000" step="500" value="6000" oninput="setv('mp',this.value,'mpv','mpn')"></div>
<div class="row" style="font-size:12px;color:#64748b">how far it may drift before K4 stops growing - 6000 steps is about 38cm</div>

<div class="row">Engage window (deg) <span id="edv">2.0</span>
<input type="number" id="edn" value="2" step="0.5" onchange="setv('ed',this.value,'edv','eds')"><br>
<input type="range" id="eds" min="0.2" max="20" step="0.1" value="2" oninput="setv('ed',this.value,'edv','edn')"></div>

<div class="row">Fall cutoff (deg) <span id="ddv">40.0</span>
<input type="number" id="ddn" value="40" step="1" onchange="setv('dd',this.value,'ddv','dds')"><br>
<input type="range" id="dds" min="5" max="90" step="1" value="40" oninput="setv('dd',this.value,'ddv','ddn')"></div>
<div class="row" style="font-size:12px;color:#64748b">limits how fast the step rate may change - stops stalling and DIR dithering</div>

<div class="hdr">Kalman filter</div>

<div class="row">Q angle <span id="qav">0.001</span>
<input type="number" id="qan" value="0.001" step="0.0005" onchange="setv('qa',this.value,'qav','qas')"><br>
<input type="range" id="qas" min="0.0001" max="0.05" step="0.0001" value="0.001" oninput="setv('qa',this.value,'qav','qan')"></div>

<div class="row">Q bias <span id="qbv">0.003</span>
<input type="number" id="qbn" value="0.003" step="0.001" onchange="setv('qb',this.value,'qbv','qbs')"><br>
<input type="range" id="qbs" min="0.0001" max="0.05" step="0.0001" value="0.003" oninput="setv('qb',this.value,'qbv','qbn')"></div>

<div class="row">R measure <span id="rmv">0.03</span>
<input type="number" id="rmn" value="0.03" step="0.005" onchange="setv('rm',this.value,'rmv','rms')"><br>
<input type="range" id="rms" min="0.001" max="1" step="0.001" value="0.03" oninput="setv('rm',this.value,'rmv','rmn')"></div>

<h2>Load a saved config</h2>
<div class="row" style="font-size:12px;color:#64748b">Paste a file saved above. Stands the robot down, applies every key it recognises, and writes it to flash. Unknown keys are ignored, so a partial paste works.</div>
<textarea id="cfgin" rows="6" style="width:92%;background:#0b1020;color:#dbeafe;border:1px solid #334155;border-radius:6px;padding:8px;font-family:monospace;font-size:12px" placeholder='{ "k1": 1200, "k3": 400, ... }'></textarea><br>
<button class="scan" style="background:#7c2d12" onclick="loadCfgFile()">LOAD CONFIG</button>

<div id="drv" style="font-size:12px;color:#7dd3fc">drv 0  turn 0  posErr 0  drift 0</div>
<div id="log">log...</div>
<button class="sp" style="margin-top:6px" onclick="copyAll()">COPY LOG + STATE</button>
<script>
// The ESP32 WebServer serves ONE connection at a time. The old page fired a /set on
// every pixel of slider travel (dozens per second) on top of a /a poll every 80ms,
// so button taps queued behind a backlog the board could never drain - that was the lag.
var busy=0;
function req(u,ms){
  var c=new AbortController(), t=setTimeout(function(){c.abort();},ms||1500);
  return fetch(u,{signal:c.signal,cache:'no-store'})
    .finally(function(){clearTimeout(t);});
}
// Commands are QUEUED, never dropped. The page allows one request at a time (the ESP32
// serves one connection), and a poll runs every 150ms - so silently discarding a tap
// that landed during a poll meant buttons randomly "did nothing". Only /move collapses,
// because for a direction command just the newest one matters.
var q=[];
function send(u){
  // Only the NEWEST direction matters, so a repeat must REPLACE the queued one. This
  // matched only '/move'; after the d-pad moved to '/drive' the 300ms repeat pushed a
  // new entry every time, filled the queue to its cap of 8, and then dropped button
  // presses behind stale drive commands.
  var mv = (u.indexOf('/move')===0) || (u.indexOf('/drive')===0);
  if(mv) q=q.filter(function(x){
    return x.indexOf('/move')!==0 && x.indexOf('/drive')!==0;
  });
  if(q.length<8) q.push(u);
  pump();
}
function pump(){
  if(busy || !q.length) return;
  var u=q.shift();
  busy++;
  req(u).then(function(){flash('#4ade80');})
        .catch(function(){flash('#ef4444');})
        .finally(function(){busy--; setTimeout(pump,0);});
}
// Instant visual acknowledgement of every tap, so a slow round-trip never reads as
// a dead button. Delegated, so it covers every button without touching the markup.
document.addEventListener('pointerdown',function(e){
  var b=e.target && e.target.closest && e.target.closest('button');
  if(b){ b.style.filter='brightness(1.7)'; setTimeout(function(){b.style.filter='';},130); }
},true);
function copyCfg(){
  req('/cfgfile',4000).then(function(r){return r.text();}).then(function(t){
    var ta=document.createElement('textarea');
    ta.value=t; ta.style.position='fixed'; ta.style.opacity='0';
    document.body.appendChild(ta); ta.select();
    try{ document.execCommand('copy'); flash('#4ade80'); }catch(e){ flash('#ef4444'); }
    document.body.removeChild(ta);
  }).catch(function(){ flash('#ef4444'); });
}
function loadCfgFile(){
  var t=document.getElementById('cfgin').value;
  if(!t || t.length<2){ flash('#ef4444'); return; }
  // Not through send(): the queue only does GETs, and this needs a body.
  fetch('/loadcfg',{method:'POST',body:t,cache:'no-store'})
    .then(function(r){return r.text();})
    .then(function(m){ flash('#4ade80'); addToast(m); setTimeout(loadCfg,300); })
    .catch(function(){ flash('#ef4444'); });
}
function addToast(m){
  var d=document.getElementById('drv'); if(d) d.textContent=m;
}
function copyAll(){
  var parts=[];
  ['v','st','hz','i2c','noi'].forEach(function(id){
    var e=document.getElementById(id); if(e) parts.push(e.textContent.trim());
  });
  var lg=document.getElementById('log');
  if(lg) parts.push('--- log ---\n'+lg.textContent.trim());
  var txt=parts.join('\n');
  // navigator.clipboard is a SECURE-CONTEXT api and this page is plain http, so it is
  // undefined on Chrome here. The textarea+execCommand path still works everywhere.
  var ta=document.createElement('textarea');
  ta.value=txt; ta.style.position='fixed'; ta.style.opacity='0';
  document.body.appendChild(ta); ta.select();
  try{ document.execCommand('copy'); flash('#4ade80'); }
  catch(e){ flash('#ef4444'); }
  document.body.removeChild(ta);
}
function flash(c){
  var d=document.getElementById('ack'); if(!d) return;
  d.style.background=c; d.style.opacity='1';
  setTimeout(function(){d.style.opacity='0.15';},250);
}

// Sliders coalesce into one request 120ms after the last move. Unlike button taps a
// setting must NEVER be dropped, so this retries instead of discarding - and re-reads
// /cfg afterwards so the page can only ever show what the board actually holds.
var pend={}, tmr=null;
function flush(){
  if(busy||q.length){ tmr=setTimeout(flush,80); return; }   // retry, do not discard
  tmr=null;
  var parts=[], snap={};   // NOT `q` - that name is the global command queue
  for(var k in pend){ parts.push(k+'='+pend[k]); snap[k]=pend[k]; }
  if(!parts.length) return;
  pend={};
  busy++;
  req('/set?'+parts.join('&'))
    .then(function(){ setTimeout(verify,400); })
    .catch(function(){
      for(var k in snap) if(!(k in pend)) pend[k]=snap[k];   // requeue, newer wins
      if(!tmr) tmr=setTimeout(flush,300);
    })
    .finally(function(){ busy--; pump(); });
}
// Only re-sync when the user is not mid-adjustment, so it never fights a drag.
function verify(){ if(!tmr && !Object.keys(pend).length) loadCfg(); }
function setv(k,v,lbl,other){
  document.getElementById(lbl).textContent=v;
  var o=document.getElementById(other);
  // The number boxes are unbounded on purpose. If a typed value falls outside its
  // slider's range, grow the slider to fit instead of letting it silently clamp -
  // otherwise the next touch of the slider would yank the value back down.
  if(o.type=='range'){
    var n=parseFloat(v);
    if(!isNaN(n)){
      if(n>parseFloat(o.max)) o.max=n;
      if(n<parseFloat(o.min)) o.min=n;
    }
  }
  o.value=v;
  pend[k]=v;
  if(!tmr) tmr=setTimeout(flush,120);
}

// d-pad: capture the pointer so a finger sliding off the button still releases cleanly
// The direction REPEATS while held. The server expires a drive command after 1s and no
// longer treats the /a poll as a keepalive, so a release that gets dropped now costs
// one second of drift instead of latching the lean until you notice.
// Directions are a SET, not one-at-a-time, so forward+right arcs instead of one
// cancelling the other. Both axes go in a single /drive request.
var held={fwd:0,back:0,left:0,right:0}, driveTmr=null;
function anyHeld(){ return held.fwd||held.back||held.left||held.right; }
function driveNow(){
  var v=(held.fwd?1:0)-(held.back?1:0);
  var t=(held.right?1:0)-(held.left?1:0);
  send('/drive?v='+v+'&t='+t);
}
function setHeld(d,on){
  held[d]=on?1:0;
  driveNow();
  if(anyHeld()){ if(!driveTmr) driveTmr=setInterval(driveNow,300); }
  else if(driveTmr){ clearInterval(driveTmr); driveTmr=null; }
}
function hold(e,d){
  try{ e.currentTarget.setPointerCapture(e.pointerId); }catch(x){}
  setHeld(d,true);
}
function relD(d){ setHeld(d,false); }
function rel(){ held.fwd=held.back=held.left=held.right=0; setHeld('fwd',false); }

// Keyboard: WASD or arrows. This is a completely independent path to the same
// endpoint - if the d-pad does nothing but the keys work, the fault is in the touch
// handling, not the link or the robot.
var KEYMAP={ArrowUp:'fwd',ArrowDown:'back',ArrowLeft:'left',ArrowRight:'right',
            w:'fwd',s:'back',a:'left',d:'right',
            W:'fwd',S:'back',A:'left',D:'right'};
function typingNow(e){
  var t=e.target&&e.target.tagName;
  return t=='INPUT'||t=='TEXTAREA';     // never drive while editing a gain
}
document.addEventListener('keydown',function(e){
  if(typingNow(e)) return;
  var d=KEYMAP[e.key]; if(!d) return;
  e.preventDefault();
  if(!held[d]) setHeld(d,true);         // ignore auto-repeat
});
document.addEventListener('keyup',function(e){
  var d=KEYMAP[e.key]; if(!d) return;
  setHeld(d,false);
});
window.addEventListener('blur',function(){ rel(); });   // tab away = stop

// green below a, amber below b, red above - each figure judged on its own scale
function col(v,a,b){ return v<a?'#4ade80':(v<b?'#f59e0b':'#ef4444'); }

// polling: self-chaining, so a slow response delays the next poll instead of stacking up
function pollA(){
 if(busy||q.length){ setTimeout(pollA,60); return; }   // one connection at a time, always
 busy++;
 req('/a').then(r=>r.json()).then(o=>{
  document.getElementById('v').textContent='angle '+o.a.toFixed(1)+(o.b?'  [ON]':'  [off]');
  var st=document.getElementById('st');
  st.textContent = o.s ? 'sensor OK' : 'SENSOR LOST - check SDA/SCL wires';
  st.style.color = o.s ? '#4ade80' : '#ef4444';
  // "SAT" = the requested rate exceeded the 9000 st/s ceiling and was clipped,
  // so any further step-gain or Kp increase buys nothing at this angle.
  var hz=document.getElementById('hz');
  hz.textContent = o.h+' st/s'+(o.x?'   SAT':'')+'   |   ctrl '+o.c+' Hz';
  // If pressing the d-pad does not change drv here, the request never reached the
  // board and the problem is the link, not the control law.
  var dv=document.getElementById('drv');
  if(dv) dv.textContent='drv '+o.dv+'  turn '+o.tb+'  posErr '+o.pe+'  drift '+o.pd;
  hz.style.color = o.x ? '#ef4444' : '#94a3b8';
 }).catch(function(e){
  var st=document.getElementById('st');
  st.textContent='link busy - retrying'; st.style.color='#f59e0b';
 }).finally(function(){busy--; pump(); setTimeout(pollA,150);});
}
function pollData(){
 if(busy||q.length){ setTimeout(pollData,120); return; }
 busy++;
 req('/data',3000).then(r=>r.json()).then(o=>{
  var ic=document.getElementById('i2c');
  ic.textContent = 'i2c: '+o.i;
  ic.style.color = o.f ? '#4ade80' : '#f59e0b';
  // noise <0.3deg = sensor floor. |a|-g is only non-zero when the chassis really moves,
  // so it is what separates mechanical vibration from electrical/wiring noise.
  document.getElementById('noi').innerHTML =
    'acc noise <b style="color:'+col(o.n,0.3,1.0)+'">'+o.n.toFixed(2)+'</b> deg RMS'+
    ' &nbsp; |a|-g <b style="color:'+col(o.e,0.5,2.0)+'">'+o.e.toFixed(2)+'</b> m/s2';
  document.getElementById('log').innerHTML = o.l.join('<br>');
 }).catch(function(e){}).finally(function(){busy--; pump(); setTimeout(pollData,2000);});
}
// Pull every widget's value from the board. Hardcoded HTML values were a trap: after a
// reload the page showed 25 while the board held 300, and the next touch sent 25 back.
function put(v,lbl,num,sld){
  // Defensive: one absent widget used to throw, which aborted loadCfg entirely and left
  // it retrying every second forever - no values loaded, and the server saturated.
  var l=document.getElementById(lbl), n=document.getElementById(num),
      s=document.getElementById(sld);
  if(!l||!n||!s) return;
  l.textContent=v;
  n.value=v;
  var f=parseFloat(v);
  if(!isNaN(f)){
    if(f>parseFloat(s.max)) s.max=f;
    if(f<parseFloat(s.min)) s.min=f;
  }
  s.value=v;
}
function mark(id,on){
  var b=document.getElementById(id);
  b.style.background = on ? '#a855f7' : '#334155';
  b.style.opacity = on ? '1' : '0.65';
}
function loadCfg(){
 if(busy||q.length){ setTimeout(loadCfg,120); return; }
 busy++;
 req('/cfg').then(r=>r.json()).then(o=>{
  put(o.k1,'k1v','k1n','k1s'); put(o.k2,'k2v','k2n','k2s');
  put(o.k3,'k3v','k3n','k3s'); put(o.k4,'k4v','k4n','k4s');
  put(o.ta,'tav','tan','tas');
  put(o.qa,'qav','qan','qas'); put(o.qb,'qbv','qbn','qbs');
  put(o.rm,'rmv','rmn','rms'); put(o.sg,'sgv','sgn','sgs');
  put(o.ma,'mav','man','mas'); put(o.ph,'phv','phn','phs');
  put(o.aa,'aav','aan','aas'); put(o.at,'atv','atn','ats');
  var bw=document.getElementById('bwv'); if(bw) bw.textContent=o.bw;
  put(o.ds,'dsv','dsn','dss'); put(o.tn,'tnv','tnn','tns');
  put(o.mp,'mpv','mpn','mps'); put(o.ed,'edv','edn','eds');
  put(o.dd,'ddv','ddn','dds');
  mark('bil',o.il); mark('bio',o.io); mark('big',o.ig); mark('bpk',o.pk); mark('bar',o.ar);
 }).catch(function(){setTimeout(loadCfg,1000);}).finally(function(){busy--; pump();});
}
function tog(x){ send('/tog?x='+x); setTimeout(loadCfg,150); }
pollA(); setTimeout(loadCfg,250); setTimeout(pollData,700);
</script>
</body></html>
)HTML";


// ==========================================================================
// 5. HARDWARE - step ISR, odometry, motor enable
// ==========================================================================

void IRAM_ATTR onStepTimer(){
  portENTER_CRITICAL_ISR(&stepMux);
  uint32_t setMask = 0, clrMask = 0;
  if (pulseR){ clrMask |= MASK_R; pulseR = false; }   // end the previous pulse (50us high)
  if (pulseL){ clrMask |= MASK_L; pulseL = false; }

  uint32_t i = incR;
  if (i){ uint32_t p = phaseR + i; if (p < phaseR){ setMask |= MASK_R; pulseR = true; posR += sgnR; } phaseR = p; }
  i = incL;
  if (i){ uint32_t p = phaseL + i; if (p < phaseL){ setMask |= MASK_L; pulseL = true; posL += sgnL; } phaseL = p; }

  if (clrMask) REG_WRITE(GPIO_OUT_W1TC_REG, clrMask);
  if (setMask) REG_WRITE(GPIO_OUT_W1TS_REG, setMask);

  // Only on a tick with no pulse: >=50us after the last STEP edge (hold) and >=50us
  // before the next possible one (setup). The DDS is clamped below ISR_HZ/2, so a
  // quiet tick is always at hand.
  if (dirPendR && !pulseR){
    if (dirLvlR) REG_WRITE(GPIO_OUT_W1TS_REG, MASK_DIR_R);
    else         REG_WRITE(GPIO_OUT_W1TC_REG, MASK_DIR_R);
    sgnR = sgnPendR; dirPendR = false;
  }
  if (dirPendL && !pulseL){
    if (dirLvlL) REG_WRITE(GPIO_OUT_W1TS_REG, MASK_DIR_L);
    else         REG_WRITE(GPIO_OUT_W1TC_REG, MASK_DIR_L);
    sgnL = sgnPendL; dirPendL = false;
  }
  portEXIT_CRITICAL_ISR(&stepMux);
}
// The only safe ways to touch the odometry from outside the ISR.
int32_t readPosAvg(){
  int32_t r, l;
  portENTER_CRITICAL(&stepMux);
  r = posR; l = posL;
  portEXIT_CRITICAL(&stepMux);
  return (int32_t)(((int64_t)r + (int64_t)l) / 2);
}
void resetPos(){
  portENTER_CRITICAL(&stepMux);
  posR = 0; posL = 0;
  portEXIT_CRITICAL(&stepMux);
}
void publishDir(bool lvlR, bool lvlL, int8_t sR, int8_t sL){
  portENTER_CRITICAL(&stepMux);
  if (lvlR != dirLvlR || sR != sgnPendR){ dirLvlR = lvlR; sgnPendR = sR; dirPendR = true; }
  if (lvlL != dirLvlL || sL != sgnPendL){ dirLvlL = lvlL; sgnPendL = sL; dirPendL = true; }
  portEXIT_CRITICAL(&stepMux);
}

// steps/s -> DDS increment
uint32_t rateToInc(float hz){
  // NaN fails BOTH comparisons below, so without this guard a NaN rate reached
  // (uint32_t)(NaN * 214748.36) - undefined, and on Xtensa typically 0xFFFFFFFF,
  // i.e. a step every single tick with the motors live.
  if (!isfinite(hz)) return 0;
  if (hz < MIN_STEP_HZ) return 0;
  if (hz > MAX_STEP_HZ) hz = MAX_STEP_HZ;
  return (uint32_t)(hz * (4294967296.0 / (double)ISR_HZ));
}
// "Here and now" becomes the state to hold.
// resetPos() takes the ISR's lock. Zeroing posR/posL directly from a web handler could
// land between the ISR's read and its write and lose the reset - and in V2's /on the
// wheels can still be stepping when this runs.
void leanReset(){ resetPos(); posTarget = 0.0; driveSpeed = 0; turnBias = 0; }

void motorsOff(){ incR = 0; incL = 0; digitalWrite(EN_PIN,HIGH); digitalWrite(EN2_PIN,HIGH); }
void motorsOn(){  digitalWrite(EN_PIN,LOW);  digitalWrite(EN2_PIN,LOW);  }

// The wheels cap at MAX_STEP_HZ, so any command beyond MAX_STEP_HZ/stepGain asks for a
// rate that physically cannot be delivered. Everything above that line is pure windup:
// it has to unwind before a REVERSAL reaches the wheels, which is felt as a sluggish,
// late recovery after a push. At stepGain 0.83 it wasted 10% of the range; at 8.78 it
// wastes 92%, so the integrator spends most of its travel doing nothing at all.
float velLimit(){ return fminf(MAX_OUT, MAX_STEP_HZ / fmaxf(stepGain, 0.01f)); }
// Turn the PID output into two step rates. Purely proportional now - no period clamp.
void setStepRates(){
  if (!engaged){
    incR = 0; incL = 0; stepHzR = stepHzL = 0; sRateR = sRateL = 0;
    rateWanted = 0; return;          // else the SAT readout freezes at the last value
  }
  // Reserve headroom for steering BEFORE the speed term (from mahowik/BalancingWii).
  // Without this a turn at speed clips one wheel while the other still has room, which
  // steals balance authority asymmetrically and veers the robot as it saturates.
  float spd   = output * stepGain;
  float steer = constrain(turnBias * stepGain, -MAX_STEP_HZ*0.5f, MAX_STEP_HZ*0.5f);
  float room  = MAX_STEP_HZ - fabs(steer);
  // What was ASKED for, before the clamp. stepHzR/L are recorded AFTER clamping, so the
  // old SAT test compared a clamped value against its own ceiling and was never true.
  rateWanted = fabs(spd) + fabs(steer);
  spd = constrain(spd, -room, room);
  float tgtR = spd + steer;                     // signed steps/s
  float tgtL = spd - steer;
  if (maxAccel > 0 && dtLast > 0){
    float md = maxAccel * dtLast;                // slew on the SIGNED rate, so a
    sRateR += constrain(tgtR - sRateR, -md, md); // direction reversal is limited too
    sRateL += constrain(tgtL - sRateL, -md, md);
  } else {
    sRateR = tgtR; sRateL = tgtL;
  }
  bool dL = (sRateL > 0);
  if (invLeft) dL = !dL;
  // Odometry counts in output space, not pin space. >=0 counts forward so a rate of
  // exactly zero does not flip the sign.
  publishDir(sRateR > 0, dL, (sRateR < 0) ? -1 : 1, (sRateL < 0) ? -1 : 1);
  stepHzR = fabs(sRateR);
  stepHzL = fabs(sRateL);
  incR = rateToInc(stepHzR);
  incL = rateToInc(stepHzL);
}


// ==========================================================================
// 6. IMU - Kalman filter, MPU6050 setup, I2C scan
// ==========================================================================

float kalmanUpdate(float newAngle, float newRate, float dt, float R) {
  float rate = newRate - K_bias;
  K_angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - K_angle;
  K_innov = y;
  K_angle += K[0] * y;
  K_bias  += K[1] * y;

  float P00 = P[0][0], P01 = P[0][1];
  P[0][0] -= K[0] * P00;
  P[0][1] -= K[0] * P01;
  P[1][0] -= K[1] * P00;
  P[1][1] -= K[1] * P01;

  return K_angle;
}

void kalmanReset(float startAngle, float startBias) {
  K_angle = startAngle;
  K_bias  = startBias;
  K_innov = 0;
  // P must NOT start at zero: with P=0 the gain is 0, so the filter ignores the
  // accelerometer entirely until Q has grown P back, which takes seconds.
  P[0][0] = 1.0; P[0][1] = 0; P[1][0] = 0; P[1][1] = 0.1;
}
void scanI2C(){
  I2CLock lk;
  i2cCount = 0; foundMPU = false;
  String found = "";
  for (byte addr = 1; addr < 127; addr++){
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0){
      i2cCount++;
      found += "0x" + String(addr, HEX) + " ";
      if (addr == 0x68 || addr == 0x69) foundMPU = true;
    }
  }
  if (i2cCount == 0) snprintf(i2cInfo,sizeof(i2cInfo),"NO devices on bus - check SDA(D21) / SCL(D22) / VCC / GND");
  else if (foundMPU)  snprintf(i2cInfo,sizeof(i2cInfo),"MPU found at %s", found.c_str());
  else                snprintf(i2cInfo,sizeof(i2cInfo),"other device(s): %s but no MPU (expect 0x68)", found.c_str());
  addLog(String("I2C scan: ") + i2cInfo);
}
// The single biggest delay between a push and the wheels reacting is this filter, not
// anything in the control law. Group delay: 44Hz = 4.9ms, 94Hz = 3.0ms, 184Hz = 2.0ms,
// 260Hz = 0.98ms. Wider = faster response but more noise into the gyro and the accel
// angle. This is now runtime-selectable so it can be A/B'd against the plot.
void configureMPU(){
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu6050_bandwidth_t b = MPU6050_BAND_44_HZ;
  if      (mpuBandHz >= 260) b = MPU6050_BAND_260_HZ;
  else if (mpuBandHz >= 184) b = MPU6050_BAND_184_HZ;
  else if (mpuBandHz >= 94)  b = MPU6050_BAND_94_HZ;
  else if (mpuBandHz >= 44)  b = MPU6050_BAND_44_HZ;
  else if (mpuBandHz >= 21)  b = MPU6050_BAND_21_HZ;
  else                       b = MPU6050_BAND_10_HZ;
  mpu.setFilterBandwidth(b);
}

// Average the gyro while the robot is held still. The Kalman estimates bias on its
// own, but starting from a measured value instead of 0 saves it several seconds of
// convergence during which the angle is visibly wrong.
float calibrateGyroBias(){
  I2CLock lk;
  const int N = 200;
  float sum = 0, mn = 1e9, mx = -1e9;
  for (int i = 0; i < N; i++){
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    float r = g.gyro.x * 180.0 / PI;
    if (invGyro) r = -r;
    sum += r;
    if (r < mn) mn = r;
    if (r > mx) mx = r;
    delay(3);
  }
  if (mx - mn > 2.0){        // it was moving, so the mean is not a bias
    addLog("gyro cal skipped, moved (spread " + String(mx-mn,2) + " dps)");
    return 0;
  }
  float mean = sum / N;
  addLog("gyro bias " + String(mean,3) + " dps");
  return mean;
}

// Start the filter from what the sensor actually reads, not from 0.
void seedFilter(){
  I2CLock lk;
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);
  float a0 = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  kalmanReset(a0, calibrateGyroBias());
  angle = a0; gyroIntAngle = a0; compAngle = a0;
  addLog("filter seeded at " + String(a0,2) + " deg");
}


// ==========================================================================
// 7. STORAGE - tiny JSON reader, NVS save/load
// ==========================================================================

// The config file is flat - "key":number or "key":true - so a full JSON parser would
// be 20KB of flash to solve a problem indexOf() solves in six lines.
bool jNum(const String& j, const char* key, float& out){
  String pat = String("\"") + key + "\":";
  int i = j.indexOf(pat);
  if (i < 0) return false;
  out = j.substring(i + pat.length()).toFloat();
  return true;
}
bool jBool(const String& j, const char* key, bool& out){
  String pat = String("\"") + key + "\":";
  int i = j.indexOf(pat);
  if (i < 0) return false;
  // /cfgfile writes "io": true - WITH a space. Without this skip, startsWith() tested
  // from the space, so EVERY boolean imported as false while still being counted in
  // the "loaded N values" total. invOut defaults true, so a save/load round-trip
  // silently inverted the balance correction and saveCfg() wrote that to flash.
  // jNum never had the bug only because toFloat()/atof skips leading whitespace.
  i += pat.length();
  while (i < (int)j.length() && (j[i] == ' ' || j[i] == '\t')) i++;
  out = j.startsWith("true", i);
  return true;
}

void saveCfg(){
  prefs.putFloat("k1",K1);          prefs.putFloat("k2",K2);
  prefs.putFloat("k3",K3);          prefs.putFloat("k4",K4);
  prefs.putFloat("ta",targetAngle);
  prefs.putFloat("qa",Q_angle);     prefs.putFloat("qb",Q_bias);
  prefs.putFloat("rm",R_meas);      prefs.putFloat("sg",stepGain);
  prefs.putFloat("ma",maxAccel);
  prefs.putFloat("ph",Kph);   prefs.putBool ("pk",posHold);   prefs.putFloat("aa",Kaa);
  prefs.putFloat("at",aaTau); prefs.putInt  ("bw",mpuBandHz);
  prefs.putFloat("ds",DRIVE_STEPS);  prefs.putFloat("tn",TURN_AMOUNT);
  prefs.putFloat("mp",MAX_POS_ERR);  prefs.putFloat("ed",ENGAGE_DEG);
  prefs.putFloat("dd",DISENGAGE_DEG);
  prefs.putBool ("ar",adaptiveR);
  prefs.putBool ("il",invLeft);    prefs.putBool ("io",invOut);   prefs.putBool ("ig",invGyro);
}
void loadCfg(){
  K1=prefs.getFloat("k1",K1);            K2=prefs.getFloat("k2",K2);
  K3=prefs.getFloat("k3",K3);            K4=prefs.getFloat("k4",K4);
  targetAngle=prefs.getFloat("ta",targetAngle);
  Q_angle=prefs.getFloat("qa",Q_angle);  Q_bias=prefs.getFloat("qb",Q_bias);
  R_meas=prefs.getFloat("rm",R_meas);    stepGain=prefs.getFloat("sg",stepGain);
  maxAccel=prefs.getFloat("ma",maxAccel);
  Kph=prefs.getFloat("ph",Kph);   posHold=prefs.getBool("pk",posHold);   Kaa=prefs.getFloat("aa",Kaa);
  aaTau=prefs.getFloat("at",aaTau); mpuBandHz=prefs.getInt("bw",mpuBandHz);
  DRIVE_STEPS=prefs.getFloat("ds",DRIVE_STEPS);  TURN_AMOUNT=prefs.getFloat("tn",TURN_AMOUNT);
  MAX_POS_ERR=prefs.getFloat("mp",MAX_POS_ERR);  ENGAGE_DEG=prefs.getFloat("ed",ENGAGE_DEG);
  DISENGAGE_DEG=prefs.getFloat("dd",DISENGAGE_DEG);
  adaptiveR=prefs.getBool("ar",adaptiveR);
  invLeft=prefs.getBool("il",invLeft);   invOut=prefs.getBool("io",invOut); invGyro=prefs.getBool("ig",invGyro);
}
// Deferred so a slider drag does not hammer the flash.
void markCfgDirty(){ cfgDirty = true; cfgTouched = millis(); }


// ==========================================================================
// 8. TELEMETRY - log ring, serial plotter
// ==========================================================================

void addLog(const String& s){
  // Format OUTSIDE the lock. portENTER_CRITICAL masks interrupts on this core, and
  // both tasks and the 20kHz step ISR live on core 1 - a newlib snprintf with a 60-char
  // %s can outrun the 50us tick and cost a step pulse. A memcpy cannot.
  char line[sizeof(logLines[0])];
  snprintf(line, sizeof(line), "%lus  %s", (unsigned long)(millis()/1000), s.c_str());
  portENTER_CRITICAL(&logMux);                 // called from both tasks now
  memcpy(logLines[logIdx], line, sizeof(line));
  logIdx = (logIdx + 1) % 6;
  portEXIT_CRITICAL(&logMux);
  if (!plotEnabled) Serial.println(s);
}
// The log ring is written by two tasks, so /data must not read it bare - and its
// contents are not all ours: addLog("move: " + d) stores server.arg("d") verbatim, so
// one quote in a URL used to produce malformed JSON. JSON.parse then threw, the page's
// .catch swallowed it, and the whole status panel froze until the entry rotated out.
String logsAsJson(){
  char snap[6][sizeof(logLines[0])];
  int idx;
  portENTER_CRITICAL(&logMux);
  memcpy(snap, logLines, sizeof(snap));
  idx = logIdx;
  portEXIT_CRITICAL(&logMux);

  String l = "[";
  for (int i = 0; i < 6; i++){
    const char* p = snap[(idx - 1 - i + 12) % 6];
    if (!p[0]) continue;
    if (l.length() > 1) l += ",";
    l += '"';
    for (; *p; p++){
      // The page renders these with innerHTML, so angle brackets have to go too.
      if      (*p == '"' || *p == '\\')      { l += '\\'; l += *p; }
      else if (*p == '<')                     l += "&lt;";
      else if (*p == '>')                     l += "&gt;";
      else if ((unsigned char)*p < 0x20)      l += ' ';
      else                                    l += *p;
    }
    l += '"';
  }
  return l + "]";
}
// ================= SERIAL PLOTTER =================
// One group at a time - the Arduino plotter is unreadable past ~5 traces, and every
// extra float costs UART time inside the balance loop. Labels carry their scale
// factor so traces of different magnitude can share one Y axis.
void plotSignals(){
  if (!plotEnabled || plotGroup == 0) return;
  if (millis() - lastPlot < 20) return;                 // 50 Hz
  if (Serial.availableForWrite() < 128) return;         // never block the loop on the UART
  lastPlot = millis();

  switch (plotGroup) {
    case 1:  // FUSION - do the three estimators agree? gyroI drifting the *opposite*
             // way from acc means the gyro sign is wrong for this accel axis pair.
      Serial.printf("acc:%.2f,gyroI:%.2f,comp:%.2f,kal:%.2f\n",
                    accAngleGlobal, gyroIntAngle, compAngle, angle);
      break;
    case 2:  // KALMAN HEALTH - innov should be zero-mean noise. A sustained offset
             // means the filter is being dragged; accErr says whether the accel is lying.
      Serial.printf("innov:%.2f,bias:%.3f,P00x10:%.2f,accErr:%.2f,R:%.3f\n",
                    K_innov, K_bias, P[0][0]*10.0, accErr, R_used);
      break;
    case 3:  // LQR terms - which state is actually driving the wheels. Kph and Kaa are
             // here too: a gain you cannot see on the plot is a gain you cannot tune.
      Serial.printf("K1ang:%.0f,K2rate:%.0f,K3vel:%.0f,K4pos:%.0f,Kph:%.0f,Kaa:%.0f,u:%.0f\n",
                    t1, t2, t3, t4, t5, t6, uAccel);
      break;
    case 4:  // TIMING - dt spikes are WiFi/webserver stalls and they corrupt the filter
      Serial.printf("dt_ms:%.2f,hz:%.0f,accErr:%.2f,rate:%.1f\n",
                    dtLast*1000.0, dtLast > 0 ? 1.0/dtLast : 0, accErr, gyroRate);
      break;
    case 7:  // POSITION - both position signals side by side. posErr comes from the
             // step COUNTER (lies when the steppers skip), drift is the integral of
             // filtered SPEED (what mahowik and remrc both use). When they disagree
             // after a hard push, the counter is the one that is wrong.
      Serial.printf("kal:%.2f,posErr100:%.2f,drift100:%.2f,vel100:%.2f,out100:%.2f\n",
                    angle, posErrSteps/100.0, posDrift/100.0, wheelVel/100.0, output/100.0);
      break;
    case 6:  // MOTOR - wheel step rate must now track the angle, not sit on a floor
      Serial.printf("kal:%.2f,out100:%.2f,hzR100:%.2f,hzL100:%.2f\n",
                    angle, output/100.0, stepHzR/100.0, stepHzL/100.0);
      break;
    default: // 5 = OVERVIEW
      Serial.printf("acc:%.2f,kal:%.2f,rate10:%.2f,out100:%.2f,bias:%.3f\n",
                    accAngleGlobal, angle, gyroRate/10.0, output/100.0, K_bias);
      break;
  }
}


// ==========================================================================
// 9. CONTROL - the LQR step
// ==========================================================================

// ================= CONTROL LOOP =================
// This used to share loop() with server.handleClient(). Serving the 17KB page blocks
// for hundreds of milliseconds, and for that whole window the robot got NO control
// updates - clamping dt kept the filter sane but the wheels still held their last
// commanded step rate. Running it as a higher-priority task fixes the real problem.
void controlStep() {
  // Sensor recovery runs even during a speed test. The old order returned early on
  // testMode, so an MPU that dropped out mid-test was never re-probed.
  if (!mpuOK) {
    if (!testMode){ armed=false; engaged=false; motorsOff(); }
    if (millis()-lastRetry > 400) {
      lastRetry = millis();
      bool ok = false;
      { I2CLock lk;
        Wire.end(); delay(5);
        Wire.begin(21,22); Wire.setClock(I2C_HZ); Wire.setTimeOut(10);
        ok = mpu.begin();
        if (ok){
          mpuOK=true; configureMPU();
          driveSpeed=0; turnBias=0;
          seedFilter();
          lastLoop=micros();
        }
      }
      if (ok){ scanI2C(); addLog("MPU RECONNECTED - Kalman reseeded"); }
    }
  }

  if (testMode) {
    publishDir(true, invLeft ? false : true, 1, 1);
    uint32_t inc = rateToInc(1000000.0 / (float)testInterval);
    incR = inc; incL = inc;
    return;
  }
  if (!mpuOK) return;

  unsigned long now = micros();
  float dt = (now-lastLoop)/1000000.0;
  if (dt <= 0) return;         // vTaskDelayUntil paces this task; no rate gate needed
  bool stalled = (dt > 0.02);
  if (stalled) dt = 0.02;
  lastLoop = now;
  dtLast = dt;
  ctrlHz = ctrlHz*0.99f + (1.0f/dt)*0.01f;   // shown on the page: 200 = healthy

  sensors_event_t a,g,temp;
  { I2CLock lk; mpu.getEvent(&a,&g,&temp); }
  if (a.acceleration.x==0 && a.acceleration.y==0 && a.acceleration.z==0){
    mpuOK=false; armed=false; engaged=false; motorsOff();
    snprintf(i2cInfo,sizeof(i2cInfo),"lost mid-run"); foundMPU = false;
    addLog("MPU LOST - all zeros - motors off");
    return;
  }

  float accAngle = atan2(a.acceleration.y, a.acceleration.z)*180.0/PI;
  gyroRate = g.gyro.x*180.0/PI;
  if (invGyro) gyroRate = -gyroRate;
  accAngleGlobal = accAngle;

  // |a| only equals g when the robot is not accelerating. Any deviation means the
  // accel angle is contaminated by the robot's own motion, which is the single
  // biggest error source in a balancing bot and the thing adaptiveR reacts to.
  accMag = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
  accErr = accMag - 9.81;

  // Reference estimators, computed only so the plot can show them next to the Kalman.
  gyroIntAngle += (gyroRate - K_bias) * dt;
  compAngle = 0.98*(compAngle + (gyroRate - K_bias)*dt) + 0.02*accAngle;

  R_used = adaptiveR ? R_meas * (1.0 + 10.0*accErr*accErr) : R_meas;
  angle = kalmanUpdate(accAngle, gyroRate, dt, R_used);

  // Exponentially-weighted RMS. accNoise tells you how far the accelerometer angle is
  // scattering around the fused estimate; accErrRms tells you whether that scatter is
  // real chassis acceleration (accErrRms high) or pure sensor/wiring noise (near 0).
  float ew = constrain(dt/1.0f, 0.0f, 1.0f);   // 1s time constant, independent of rate
  accNoise  = sqrt((1.0f-ew)*accNoise*accNoise   + ew*K_innov*K_innov);
  accErrRms = sqrt((1.0f-ew)*accErrRms*accErrRms + ew*accErr*accErr);

  // ---- arm / engage: the robot decides when it can balance ----
  if (armed) {
    // NaN fails every comparison, so this alone would silently DISARM the cutoff on a
    // NaN angle while engaged stayed true - and NaN then flows into the step rate.
    if (engaged && (!isfinite(angle) || fabs(angle - targetAngle) > DISENGAGE_DEG)) {
      engaged = false; motorsOff(); velCmd = 0; driveSpeed = 0; turnBias = 0;
      addLog("tipped over - disengaged (still armed)");
    } else if (!engaged && isfinite(angle) && fabs(angle - targetAngle) < ENGAGE_DEG) {
      engaged = true; velCmd = 0; leanReset(); motorsOn();
      addLog("upright - engaged");
    }
  }

  if (engaged) {
    // ---- the whole controller: four states, one equation ----
    long pos = readPosAvg();                                  // averaging cancels turn
    wheelVel = (sRateR + sRateL) * 0.5f;                      // commanded rate = true rate

    posTarget += driveSpeed * dt;                             // driving moves the SETPOINT
    // Clamping the ERROR bounds the control authority but not the DEBT: held off the
    // ground or against a wall, the setpoint ran away at driveSpeed forever, and on
    // release the robot drove metres to catch up to it. Clamp the setpoint too, so it
    // can never get further ahead of the wheels than the error clamp can act on.
    posTarget = constrain(posTarget, (double)pos - MAX_POS_ERR, (double)pos + MAX_POS_ERR);
    posErrSteps = constrain((float)(pos - posTarget), -MAX_POS_ERR, MAX_POS_ERR);

    t1 = K1 * (angle - targetAngle);        // deg
    t2 = K2 * (gyroRate - K_bias);          // deg/s
    // Was (wheelVel/1000): K3 always drove the wheels toward ZERO, so the d-pad could
    // only reach them through K4 and the position integrator - the response built
    // quadratically from nothing and a short press did nothing visible. Giving the
    // speed term a target is what mahowik does (error = targetSpeed - actualSpeed);
    // K4 then handles position tracking behind it. Exact no-op when driveSpeed is 0.
    t3 = K3 * ((wheelVel - driveSpeed) / 1000.0f);   // ksteps/s
    t4 = K4 * (posErrSteps / 1000.0f);      // ksteps

    // ---- position hold, after BalancingWii ----
    speedFilt = speedFilt*0.92f + wheelVel*0.08f;
    if (posHold && fabs(driveSpeed) < 50.0f && fabs(turnBias) < 50.0f) {
      // Only integrate while nobody is driving, else holding a direction fights the
      // operator and it crawls back the instant you release.
      posDrift = constrain(posDrift + speedFilt*dt, -MAX_POS_ERR, MAX_POS_ERR);
    } else {
      posDrift = 0;                         // driving redefines "home" as here
    }
    t5 = posHold ? Kph * (posDrift / 1000.0f) : 0.0f;

    // ---- second derivative: d(gyro)/dt, low-passed ----
    float gRate = gyroRate - K_bias;
    // On a stalled cycle dt was CLAMPED to 0.02 but the gyro really moved for the full
    // (much longer) interval, so dividing by the clamp over-reads the acceleration by
    // the whole stall ratio - 10x after a 200ms webserver stall. Carry gRatePrev
    // forward and skip the sample instead of feeding Kaa a spike.
    if (!stalled){
      float gAcc = (gRate - gRatePrev) / dt;                    // deg/s^2, very noisy
      gAccFilt  += (gAcc - gAccFilt) * constrain(dt / fmaxf(aaTau, 0.001f), 0.0f, 1.0f);
    }
    gRatePrev = gRate;
    t6 = Kaa * (gAccFilt / 1000.0f);
    // t1/t2/t6 are PHYSICAL (which way the chassis leans, independent of wiring) so they
    // follow invOut. t3/t4/t5 are counted in OUTPUT space - sgnR tracks sign(output) -
    // so the output-to-physical mapping is already baked into them and invOut must not
    // touch them again.
    //
    // But they are ADDED, not subtracted. Linearising the cart-pendulum
    //     theta_ddot = (g*theta - a)/L ,  a = A*theta + B*theta_dot + C*x_dot + D*x
    // gives the closed-loop polynomial
    //     L*s^4 + (B - C*L)*s^3 + (A - g - D*L)*s^2 + (C*g)*s + (D*g)
    // and Routh-Hurwitz needs every coefficient positive, so C and D must carry the
    // SAME sign as A. Subtracting them makes the s and constant terms negative: one
    // slow right-half-plane root while the fast angle modes stay stable. The robot
    // balances beautifully and drives away in a straight line, never returning.
    //
    // It is genuinely counter-intuitive, and it is the non-minimum-phase part of the
    // problem: to get back to where it started the robot must FIRST accelerate further
    // the way it is already going, so the body tips back, then ride that lean home.
    float uAng = t1 + t2 + t6;
    if (invOut) uAng = -uAng;
    uAccel = uAng + (t3 + t4 + t5);

    // Steppers take a velocity, but righting a pendulum needs sustained ACCELERATION,
    // so the control law is integrated rather than applied directly.
    velCmd += uAccel * dt;
    velCmd = constrain(velCmd, -velLimit(), velLimit());
    output = velCmd;
  } else {
    output = 0; velCmd = 0; uAccel = 0; t1=t2=t3=t4=t5=t6=0;
    posDrift = 0; speedFilt = 0; gRatePrev = 0; gAccFilt = 0;
    posTarget = (double)readPosAvg();
    posErrSteps = 0; wheelVel = 0;
  }

  setStepRates();
  plotSignals();
}


// ==========================================================================
// 10. WEB - HTTP handlers
// ==========================================================================

// ENGAGE_DEG and DISENGAGE_DEG are clamped independently - 0.2..20 and 5..90 - so the
// UI can reach ENGAGE_DEG >= DISENGAGE_DEG, and every angle between them then satisfies
// BOTH tests: controlStep() engages on one cycle and disengages on the next, forever.
// That is motorsOn/motorsOff at 100Hz and two addLog() calls per control cycle, each
// taking logMux with interrupts masked. Keep a margin so the two bands cannot touch.
// DISENGAGE_DEG wins: it is the safety cutoff, so the engage window is what gives way.
static void clampEngageWindow(){
  if (ENGAGE_DEG >= DISENGAGE_DEG) ENGAGE_DEG = fmaxf(0.2f, DISENGAGE_DEG - 0.5f);
}

void setupWeb(){
  server.on("/",[]{ server.send_P(200,"text/html",PAGE); });
  server.on("/scan",[]{
    // A 126-address sweep is tens of ms of blocked I2C. /kreset already stands down for
    // exactly this reason; this used to run with the robot live.
    if (armed||engaged){ armed=false; engaged=false; motorsOff(); addLog("stood down for I2C scan"); }
    scanI2C(); server.send(200,"text/plain","scanned"); });
  server.on("/kreset",[]{
    // seedFilter() blocks ~600ms calibrating the gyro. Doing that while balancing
    // freezes the control loop and the robot falls, so stand down first.
    if (armed){ armed=false; engaged=false; motorsOff(); addLog("disarmed for reset"); }
    if (mpuOK) seedFilter(); else kalmanReset(0,0);
    addLog("Kalman reset"); server.send(200,"text/plain","reset"); });
  server.on("/plot",[]{ plotEnabled=!plotEnabled;
    server.send(200,"text/plain", plotEnabled?"plot on":"plot off"); });
  server.on("/plotg",[]{ plotGroup = server.arg("g").toInt();
    addLog("plot group " + String(plotGroup)); server.send(200,"text/plain","ok"); });
  server.on("/tog",[]{
    String x = server.arg("x");
    if (x=="adaptr"){ adaptiveR = !adaptiveR; addLog("adaptive R " + String(adaptiveR?"ON":"off")); }
    if (x=="igyro") { armed=false; engaged=false; motorsOff();
                      invGyro = !invGyro;
                      if (mpuOK) seedFilter();
                      addLog("invert gyro " + String(invGyro?"ON":"off") + " - stood down"); }
    if (x=="poshold"){
      posHold = !posHold;
      // Re-home on every enable, so the button means "hold HERE".
      posDrift = 0; resetPos(); posTarget = 0;
      addLog("POSITION HOLD " + String(posHold?"ON - holding here":"off - free to roll"));
    }
    if (x=="invl" || x=="invo"){
      armed=false; engaged=false; motorsOff(); velCmd=0;   // never flip a sign mid-flight
      if (x=="invl"){ invLeft = !invLeft; addLog("swap L motor " + String(invLeft?"ON":"off")); }
      else          { invOut  = !invOut;  addLog("invert output " + String(invOut?"ON":"off")); }
      addLog("disarmed - direction changed");
    }
    markCfgDirty();
    server.send(200,"text/plain","ok"); });

  server.on("/a",[]{
    // NOT a keepalive any more. Refreshing lastCmdMs here meant the drive watchdog
    // could only fire when the page stopped polling entirely - so a d-pad RELEASE that
    // got dropped left the lean latched forever. The page now repeats /move while a
    // direction is held, so the 1s timeout is the real safety net it was meant to be.
    lastPollMs = millis();                          // page is alive - see lastPollMs
    float raw = rateWanted;                         // rate asked for, before clamping
    server.send(200,"application/json",
      "{\"a\":"+String(angle,1)+",\"b\":"+((engaged||testMode)?"true":"false")+
      ",\"s\":"+(mpuOK?"true":"false")+
      ",\"h\":"+String(fminf(raw,MAX_STEP_HZ),0)+
      ",\"x\":"+((raw>MAX_STEP_HZ)?"true":"false")+
      ",\"c\":"+String(ctrlHz,0)+
      ",\"dv\":"+String(driveSpeed,0)+",\"tb\":"+String(turnBias,0)+
      ",\"pe\":"+String(posErrSteps,0)+",\"pd\":"+String(posDrift,0)+"}");
  });

  server.on("/test",[]{
    testInterval = server.arg("v").toInt();
    if (testInterval < 40) testInterval = 40;
    testMode = true; armed = false; engaged = false;
    lastCmdMs = millis(); lastPollMs = millis();
    motorsOn();
    addLog("SPEED TEST " + String(testInterval) + "us");
    server.send(200,"text/plain","test");
  });
  server.on("/teststop",[]{
    testMode = false; motorsOff();
    addLog("test stopped");
    server.send(200,"text/plain","stopped");
  });

  server.on("/raw",[]{
    sensors_event_t a,g,temp;
    { I2CLock lk; mpu.getEvent(&a,&g,&temp); }
    String s = "ACC x=" + String(a.acceleration.x,2) +
               " y=" + String(a.acceleration.y,2) +
               " z=" + String(a.acceleration.z,2) +
               "   GYRO x=" + String(g.gyro.x,2) +
               " y=" + String(g.gyro.y,2) +
               " z=" + String(g.gyro.z,2) +
               "   bias=" + String(K_bias,3);
    server.send(200,"text/plain",s);
  });

  server.on("/on",[]{
    if(!mpuOK){ addLog("BALANCE refused - no sensor"); server.send(200,"text/plain","no sensor"); return; }
    testMode=false;
    // lastError is deliberately NOT cleared: the idle branch keeps it tracking the
    // live angle, so the first D sample after switching on is 0 instead of error/dt.
    armed=true; engaged=false; velCmd=0; leanReset();
    addLog("ARMED - stand it upright to engage");
    server.send(200,"text/plain","on");
  });
  server.on("/off",[]{ armed=false; engaged=false; testMode=false;
    driveSpeed=0; turnBias=0; velCmd=0; motorsOff();
    addLog("disarmed"); server.send(200,"text/plain","off"); });
  // /move could only ever set ONE axis - each branch zeroed the other - so driving
  // and turning at the same time was impossible and every turn was a pivot in place.
  // /drive takes both axes at once, each -1..1, scaled by the tunable amounts.
  server.on("/drive",[]{
    lastCmdMs = millis();
    float v = server.hasArg("v") ? server.arg("v").toFloat() : 0.0f;
    float t = server.hasArg("t") ? server.arg("t").toFloat() : 0.0f;
    v = constrain(v, -1.0f, 1.0f);
    t = constrain(t, -1.0f, 1.0f);
    driveSpeed = -v * DRIVE_STEPS;      // same sign convention as /move fwd
    turnBias   =  t * TURN_AMOUNT;
    static float lv = 999, lt = 999;
    if (v != lv || t != lt){ lv = v; lt = t;
      addLog("drive v=" + String(v,1) + " t=" + String(t,1)); }
    server.send(200,"text/plain","ok");
  });

  server.on("/move",[]{
    lastCmdMs = millis();
    String d = server.arg("d");
    // Driving moves the position SETPOINT, so the LQR chases it. No lean hack, and the
    // position term keeps working the whole time instead of being switched off.
    if      (d=="fwd")  { driveSpeed = -DRIVE_STEPS; turnBias = 0; }
    else if (d=="back") { driveSpeed =  DRIVE_STEPS; turnBias = 0; }
    else if (d=="left") { driveSpeed = 0; turnBias = -TURN_AMOUNT; }
    else if (d=="right"){ driveSpeed = 0; turnBias =  TURN_AMOUNT; }
    else                { driveSpeed = 0; turnBias = 0; }
    static String lastD; if (d != lastD){ lastD = d; addLog("move: " + d); }
    server.send(200,"text/plain","ok");
  });
  server.on("/set",[]{
    if(server.hasArg("k1")) K1=server.arg("k1").toFloat();
    if(server.hasArg("k2")) K2=server.arg("k2").toFloat();
    if(server.hasArg("k3")) K3=server.arg("k3").toFloat();
    if(server.hasArg("k4")) K4=server.arg("k4").toFloat();
    if(server.hasArg("ph")) Kph=server.arg("ph").toFloat();
    if(server.hasArg("aa")) Kaa=server.arg("aa").toFloat();
    if(server.hasArg("at")) aaTau=constrain(server.arg("at").toFloat(), 0.002f, 0.2f);
    if(server.hasArg("bw")){ mpuBandHz=server.arg("bw").toInt();
                             if (mpuOK){ I2CLock lk; configureMPU(); }
                             addLog("MPU bandwidth " + String(mpuBandHz) + " Hz"); }
    if(server.hasArg("ta")) targetAngle=server.arg("ta").toFloat();
    // The number boxes are deliberately unbounded, so 0 or negative is reachable from
    // the UI. S = P[0][0] + R; a non-positive R makes S zero or negative, the Kalman
    // gain becomes inf/NaN, and the NaN lands in angle -> step rate.
    if(server.hasArg("qa")) Q_angle=fmaxf(server.arg("qa").toFloat(), 1e-7f);
    if(server.hasArg("qb")) Q_bias =fmaxf(server.arg("qb").toFloat(), 1e-7f);
    if(server.hasArg("rm")) R_meas =fmaxf(server.arg("rm").toFloat(), 1e-4f);
    if(server.hasArg("sg")) stepGain=constrain(server.arg("sg").toFloat(), 0.01f, 100.0f);
    if(server.hasArg("ma")) maxAccel=fmaxf(server.arg("ma").toFloat(), 0.0f);
    if(server.hasArg("ds")) DRIVE_STEPS=fmaxf(server.arg("ds").toFloat(), 0.0f);
    if(server.hasArg("tn")) TURN_AMOUNT=fmaxf(server.arg("tn").toFloat(), 0.0f);
    if(server.hasArg("mp")) MAX_POS_ERR=fmaxf(server.arg("mp").toFloat(), 100.0f);
    if(server.hasArg("ed")) ENGAGE_DEG=constrain(server.arg("ed").toFloat(), 0.2f, 20.0f);
    if(server.hasArg("dd")) DISENGAGE_DEG=constrain(server.arg("dd").toFloat(), 5.0f, 90.0f);
    clampEngageWindow();
    markCfgDirty();
    server.send(200,"text/plain","ok");
  });
  // The page must read its widget values from the board, never from hardcoded HTML,
  // otherwise a reload shows stale numbers and the next slider touch pushes them back.
  server.on("/cfg",[]{
    server.send(200,"application/json",
      "{\"k1\":"+String(K1,1)+",\"k2\":"+String(K2,2)+",\"k3\":"+String(K3,1)+",\"k4\":"+String(K4,1)+
      ",\"ta\":"+String(targetAngle,2)+",\"qa\":"+String(Q_angle,4)+
      ",\"qb\":"+String(Q_bias,4)+",\"rm\":"+String(R_meas,4)+
      ",\"sg\":"+String(stepGain,3)+
      ",\"ma\":"+String(maxAccel,0)+
      ",\"il\":"+(invLeft?"true":"false")+",\"io\":"+(invOut?"true":"false")+",\"ig\":"+(invGyro?"true":"false")+",\"ar\":"+(adaptiveR?"true":"false")+
      ",\"ds\":"+String(DRIVE_STEPS,0)+",\"tn\":"+String(TURN_AMOUNT,0)+
      ",\"mp\":"+String(MAX_POS_ERR,0)+",\"ed\":"+String(ENGAGE_DEG,1)+
      ",\"dd\":"+String(DISENGAGE_DEG,1)+",\"ph\":"+String(Kph,1)+",\"aa\":"+String(Kaa,1)+",\"at\":"+String(aaTau,3)+
      ",\"bw\":"+String(mpuBandHz)+",\"pk\":"+(posHold?"true":"false")+"}");
  });
  // Direction is deliberately NOT part of /defaults - it is a wiring fact, not a
  // tuning value, and clearing it on every tuning reset would be worse. But there
  // was no way back to a known state either, so here is one.
  server.on("/dirreset",[]{
    armed=false; engaged=false; testMode=false; motorsOff();
    invLeft=false; invOut=true; invGyro=false;
    markCfgDirty();
    addLog("DIRECTION reset: swapL=off invOut=ON invGyro=off");
    server.send(200,"text/plain","dirreset"); });
  // One text blob holding everything, in the same key names /set already accepts.
  // Saved, pasted into a message, or reloaded - all the same string.
  server.on("/cfgfile",[]{
    String j = "{\n";
    j += "  \"_name\": \"BalanceBot V2 LQR config\",\n";
    j += "  \"_uptime_s\": " + String(millis()/1000) + ",\n";
    j += "  \"k1\": " + String(K1,1) + ", \"k2\": " + String(K2,2) +
         ", \"k3\": " + String(K3,1) + ", \"k4\": " + String(K4,1) + ",\n";
    j += "  \"ph\": " + String(Kph,1) + ", \"pk\": " + String(posHold?"true":"false") +
         ", \"aa\": " + String(Kaa,1) + ", \"at\": " + String(aaTau,3) + ",\n";
    j += "  \"ta\": " + String(targetAngle,2) + ", \"sg\": " + String(stepGain,2) +
         ", \"ma\": " + String(maxAccel,0) + ",\n";
    j += "  \"qa\": " + String(Q_angle,4) + ", \"qb\": " + String(Q_bias,4) +
         ", \"rm\": " + String(R_meas,4) + ", \"bw\": " + String(mpuBandHz) + ",\n";
    j += "  \"ds\": " + String(DRIVE_STEPS,0) + ", \"tn\": " + String(TURN_AMOUNT,0) +
         ", \"mp\": " + String(MAX_POS_ERR,0) + ",\n";
    j += "  \"ed\": " + String(ENGAGE_DEG,1) + ", \"dd\": " + String(DISENGAGE_DEG,1) + ",\n";
    j += "  \"il\": " + String(invLeft?"true":"false") +
         ", \"io\": " + String(invOut?"true":"false") +
         ", \"ig\": " + String(invGyro?"true":"false") +
         ", \"ar\": " + String(adaptiveR?"true":"false") + "\n}";
    if (server.hasArg("dl"))
      server.sendHeader("Content-Disposition","attachment; filename=\"balancebot-v2.json\"");
    server.send(200,"application/json",j);
  });

  // Paste a saved file back in. Direction flags are applied too - if you are moving a
  // config between two robots, delete the il/io/ig lines first or you import someone
  // else's wiring.
  server.on("/loadcfg", HTTP_POST, []{
    String j = server.arg("plain");
    if (j.length() < 2){ server.send(400,"text/plain","empty"); return; }
    armed=false; engaged=false; testMode=false; motorsOff();
    float v; bool b; int n = 0;
    if (jNum(j,"k1",v)){ K1=v; n++; }            if (jNum(j,"k2",v)){ K2=v; n++; }
    if (jNum(j,"k3",v)){ K3=v; n++; }            if (jNum(j,"k4",v)){ K4=v; n++; }
    if (jNum(j,"ph",v)){ Kph=v; n++; }           if (jNum(j,"aa",v)){ Kaa=v; n++; }
    if (jNum(j,"at",v)){ aaTau=constrain(v,0.002f,0.2f); n++; }
    if (jNum(j,"ta",v)){ targetAngle=v; n++; }
    if (jNum(j,"sg",v)){ stepGain=constrain(v,0.01f,100.0f); n++; }
    if (jNum(j,"ma",v)){ maxAccel=fmaxf(v,0.0f); n++; }
    if (jNum(j,"qa",v)){ Q_angle=fmaxf(v,1e-7f); n++; }
    if (jNum(j,"qb",v)){ Q_bias =fmaxf(v,1e-7f); n++; }
    if (jNum(j,"rm",v)){ R_meas =fmaxf(v,1e-4f); n++; }
    if (jNum(j,"ds",v)){ DRIVE_STEPS=fmaxf(v,0.0f); n++; }
    if (jNum(j,"tn",v)){ TURN_AMOUNT=fmaxf(v,0.0f); n++; }
    if (jNum(j,"mp",v)){ MAX_POS_ERR=fmaxf(v,100.0f); n++; }
    if (jNum(j,"ed",v)){ ENGAGE_DEG=constrain(v,0.2f,20.0f); n++; }
    if (jNum(j,"dd",v)){ DISENGAGE_DEG=constrain(v,5.0f,90.0f); n++; }
    clampEngageWindow();
    if (jNum(j,"bw",v)){ mpuBandHz=(int)v; if(mpuOK){ I2CLock lk; configureMPU(); } n++; }
    if (jBool(j,"pk",b)){ posHold=b; n++; }      if (jBool(j,"il",b)){ invLeft=b; n++; }
    if (jBool(j,"io",b)){ invOut=b; n++; }       if (jBool(j,"ig",b)){ invGyro=b; n++; }
    if (jBool(j,"ar",b)){ adaptiveR=b; n++; }
    saveCfg();
    addLog("CONFIG LOADED - " + String(n) + " values, stood down");
    server.send(200,"text/plain", String("loaded ") + n + " values");
  });

  server.on("/defaults",[]{
    armed=false; engaged=false; testMode=false; motorsOff();
    K1=1200.0; targetAngle=0.0;
    Q_angle=0.001; Q_bias=0.003; R_meas=0.03; stepGain=0.83;
    K2=60.0; K3=1000.0; K4=400.0; Kph=0.0; Kaa=0.0; aaTau=0.02; mpuBandHz=44;
    // Was 30000 here while State.h, the number box and the slider all default to 0, so
    // a fresh flash and a RESET TUNING gave different slew limiting from the same UI.
    maxAccel=0;
    DRIVE_STEPS=1200; TURN_AMOUNT=120; MAX_POS_ERR=6000;
    ENGAGE_DEG=2.0; DISENGAGE_DEG=40.0;
    adaptiveR=false; posHold=true;   // posHold was the one flag /defaults never reset
    prefs.clear(); saveCfg();
    addLog("TUNING RESET - direction flags KEPT (use RESET DIRECTION)");
    server.send(200,"text/plain","defaults");
  });
  server.on("/data",[]{
    String l = logsAsJson();     // takes logMux and escapes; see Telemetry.cpp
    server.send(200,"application/json",
      "{\"a\":"+String(angle,1)+",\"b\":"+((engaged||testMode)?"true":"false")+
      ",\"s\":"+(mpuOK?"true":"false")+
      ",\"i\":\""+i2cInfo+"\",\"f\":"+(foundMPU?"true":"false")+
      ",\"n\":"+String(accNoise,2)+",\"e\":"+String(accErrRms,2)+
      ",\"l\":"+l+"}");
  });
}


// ==========================================================================
// 11. SETUP / LOOP / CONTROL TASK
// ==========================================================================

void setup() {
  Serial.begin(115200);
  prefs.begin("bal", false);
  loadCfg();
  // Latch EN high BEFORE the pin becomes an output. pinMode(OUTPUT) drives whatever is
  // already in the latch, which is 0 - so the old order enabled both drivers for the
  // instant between pinMode() and motorsOff(). (The window from power-on until this
  // line runs is a HARDWARE problem: A4988/DRV8825 ENABLE is active-low with an
  // internal pull-down, so the drivers hold current through the whole boot. Fit a 10k
  // pull-up from each EN to logic VDD - software cannot reach that window.)
  //
  // digitalWrite() CANNOT do this on arduino-esp32 3.x: __digitalWrite() is gated on
  // perimanGetPinBus(pin, ESP32_BUS_TYPE_GPIO) != NULL, and it is pinMode() that
  // registers that bus - so a write before pinMode() only logs "IO 27 is not set as
  // GPIO" and returns. Writing the output register directly has no such gate, and
  // pinMode() then drives the 1 that is already latched instead of the reset-state 0.
  REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << EN_PIN) | (1UL << EN2_PIN));
  pinMode(STEP_PIN,OUTPUT); pinMode(DIR_PIN,OUTPUT); pinMode(EN_PIN,OUTPUT);
  pinMode(STEP2_PIN,OUTPUT); pinMode(DIR2_PIN,OUTPUT); pinMode(EN2_PIN,OUTPUT);
  motorsOff();

  // Must exist before ANY I2C happens: I2CLock is a no-op while this is null.
  i2cMux = xSemaphoreCreateRecursiveMutex();

  stepTimer = timerBegin(1000000);                       // 1 MHz tick
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, 1000000 / ISR_HZ, true, 0);      // fire every 50us

  Wire.begin(21,22);
  Wire.setClock(I2C_HZ);
  // A bus wedged low by stepper noise used to block the control loop for the default
  // timeout, which is long enough to drop the robot. (400kHz also wants stronger
  // pull-ups than the 10k on a GY-521 breakout - fit 2.2k if you see MPU LOST.)
  Wire.setTimeOut(10);
  addLog("boot - I2C on SDA=D21 SCL=D22");
  scanI2C();

  mpuOK = mpu.begin();
  if (mpuOK){ configureMPU(); seedFilter(); addLog("MPU ready + Kalman"); }
  else addLog("MPU not found - retrying every 400ms");

  WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(true);
  // Modem sleep parks the radio between DTIM beacons, so every packet waits up to a
  // beacon interval. Measured RTT went 6ms floor / 150ms median / 475ms worst, which
  // is enough to stall the 17KB page: the browser opens ~6 sockets, the sync WebServer
  // serves one at a time, and the queue outlives the browser's own timeout.
  WiFi.setSleep(false);
  WiFi.begin(ssid,pass);
  // No blocking wait: this used to hold the robot hostage for up to 20s when the AP was
  // slow. The watchdog in loop() reports the address on association.
  // ALWAYS report this, plotter or not. It is printed once at boot, before any plot
  // data, so it cannot corrupt a trace - and without it there is no way to learn the
  // DHCP address, which is exactly how you end up typing a stale IP that times out.
  Serial.println();
  Serial.println("=== connecting to WiFi - the address prints as soon as it associates ===");
  addLog("WiFi connecting");

  setupWeb();
  server.begin();

  // Priority 5 beats loopTask's 1, so the control task preempts the web server the
  // moment its 5ms tick comes due. Core 1 keeps it off the core running the WiFi stack.
  xTaskCreatePinnedToCore(controlTask, "ctrl", 6144, NULL, 5, NULL, 1);
  lastLoop = micros();
}

void loop() {
  server.handleClient();

  // WiFi watchdog: setAutoReconnect only helps after a first successful association,
  // so a boot-time failure used to be permanent AND invisible.
  // NOT while engaged. An NVS commit can erase a flash sector, and code outside IRAM
  // cannot execute during a flash operation - but onStepTimer() IS in IRAM, so the step
  // ISR keeps pulsing at the last commanded rate while controlStep() is frozen. That is
  // the robot running open-loop with the motors driving, which is how touching a slider
  // mid-balance drops it. /loadcfg and /defaults already stand down before they write;
  // this one did not, and it is the one that fires while you are tuning.
  // Nothing is lost by waiting: /set applied the value to RAM already, and cfgDirty
  // stays set, so the write lands the moment it disengages.
  if (cfgDirty && !engaged && millis()-cfgTouched > 2000){ cfgDirty=false; saveCfg(); addLog("tuning saved"); }

  static unsigned long lastWifiChk = 0, lastAssoc = 0;
  static bool wasUp = false;
  if (millis() - lastWifiChk > 5000) {
    lastWifiChk = millis();
    bool up = (WiFi.status() == WL_CONNECTED);
    if (up && !wasUp) {
      Serial.println();
      Serial.print("=== OPEN: http://"); Serial.print(WiFi.localIP());
      Serial.println("  (or http://balance2.local) ===");
      MDNS.begin("balance2"); MDNS.addService("http","tcp",80);
      addLog("WiFi up " + WiFi.localIP().toString());
    } else if (!up && millis()-lastAssoc > 15000) {
      // Association + DHCP on a busy AP routinely needs more than 5s. Retrying on the
      // 5s tick tore down each attempt mid-handshake, so a board that was almost
      // connected could loop forever never finishing.
      lastAssoc = millis();
      WiFi.disconnect(); WiFi.begin(ssid,pass);
    }
    wasUp = up;
  }

  // Drive commands are momentary but arrive as one-shot requests, so they must expire
  // on their own. Balancing itself is NEVER cut on link loss - that would drop the
  // robot every time WiFi hiccups. Only the operator's intent expires.
  if ((driveSpeed != 0.0f || turnBias != 0.0f) && millis()-lastCmdMs > CMD_TIMEOUT_MS){
    driveSpeed = 0; turnBias = 0;
    addLog("link lost - drive command cleared");
  }
  if (testMode && millis()-lastPollMs > TEST_TIMEOUT_MS){
    testMode = false; motorsOff();
    addLog("link lost - speed test stopped");
  }

  delay(1);        // yield; the control loop is a separate task now
}

void controlTask(void*){
  lastLoop = micros();
  TickType_t last = xTaskGetTickCount();
  for(;;){
    controlStep();
    // vTaskDelayUntil returns IMMEDIATELY when its deadline has already passed. One
    // slow cycle - a 10ms I2C timeout is enough - puts this task permanently behind,
    // and it then spins at priority 5 without ever yielding, starving loopTask and
    // killing the web server outright. Always give up the CPU for at least one tick.
    TickType_t now = xTaskGetTickCount();
    if ((now - last) >= pdMS_TO_TICKS(CTRL_MS)){ last = now; vTaskDelay(1); }
    else vTaskDelayUntil(&last, pdMS_TO_TICKS(CTRL_MS));
  }
}
