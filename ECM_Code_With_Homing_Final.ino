// =====================================================
// PC-Controlled Jog + Drill (numeric jog, interruptible, timed)
// With:
//  - Top & bottom limit switches
//  - Homing command (up to top limit)
// NO motor power MOSFET control
//
// Commands (Serial Monitor @115200, any line ending):
//   uN = jog UP N times  (e.g. u50)
//   dN = jog DOWN N times (e.g. d6)
//   u  = jog UP once
//   d  = jog DOWN once
//   f  = finejog UP once
//   g  = finejog DOWN once
//   p  = runs pump alone
//   h  = home (run UP until top limit)
//   ' ' (space) = toggle DRILL (start/stop)
//   k  = KILL (stop all motion + pump)
//   r  = report status
// =====================================================

#include <Arduino.h>

// --------- Pins ---------
#define PUL_PIN   2
#define DIR_PIN   3
#define ENA_PIN   4
#define PUMP_PIN  10    // PWM-capable pin for pump

// Limit switch pins
#define LIMIT_TOP_PIN    6  // top limit switch (home)
#define LIMIT_BOTTOM_PIN 7  // bottom limit switch

// --------- Easy-to-change parameters ---------
const unsigned long STEPS_PER_JOG      = 25600; // 1mm
const unsigned int  PULSE_HIGH_US      = 10;
const unsigned int  JOG_PULSE_LOW_UP   = 100;
const unsigned int  JOG_PULSE_LOW_DOWN = 100;
// Fine jog: small nudge moves for setting electrode gap
const unsigned long STEPS_PER_FINE_JOG = 256;  // 0.01mm

unsigned int DRILL_PULSE_LOW_US = 27000;
uint8_t      DRILL_PUMP_PWM     = 75;

const bool ENABLE_ACTIVE_LOW = true;
const bool MOTOR_DIR_INVERT  = false;
const bool DRILL_DIR_HIGH    = false; // drilling electrical DIR level (usually DOWN)

// Homing parameters
const bool HOME_DIR_UP_LOGICAL = true;      // home by moving UP
const unsigned int HOME_PULSE_LOW_US = 100; // homing speed (slower = safer)

// Limit polarity: you said ACTIVE when pin is HIGH works for you
// If you ever change wiring to N.O. + INPUT_PULLUP, set this to false.
const bool LIMIT_ACTIVE_HIGH = true;

// --------- Stepper state ---------
bool   stepperActive      = false;
bool   stepperContinuous  = false; // false=jog, true=drill/home
bool   stepPinHigh        = false;
bool   currentDirHigh     = true;  // actual DIR pin state
bool   logicalDirUp       = true;  // logical direction: true=up, false=down

unsigned long stepsRemaining   = 0;
unsigned long nextEdgeUs       = 0;
unsigned int  currentPulseLowUs = 0;

// --------- Jog timing ---------
bool         jogTimingActive = false;
unsigned long jogStartMillis = 0;

// --------- Other state ---------
bool   drilling = false;
bool   homing   = false;
bool   homed    = false;

String cmdBuffer;

// --------- Helpers ---------
inline void setEnable(bool en){
  digitalWrite(ENA_PIN,
               (ENABLE_ACTIVE_LOW ? (en ? LOW : HIGH) : (en ? HIGH : LOW)));
}

inline void setDirRaw(bool dirHigh){
  currentDirHigh = MOTOR_DIR_INVERT ? !dirHigh : dirHigh;
  digitalWrite(DIR_PIN, currentDirHigh ? HIGH : LOW);
}

// Logical direction mapping: "up" vs "down"
inline void setDirLogical(bool up){
  logicalDirUp = up;
  // bool dirHigh = up ? true : false; // adjust here if logical up/down feels inverted
  bool dirHigh = false; // correction for broken stepper controller, if fixed comment out this line and uncomment above line
  setDirRaw(dirHigh);
}

// Limit switch helpers
inline bool topLimitActive() {
  int v = digitalRead(LIMIT_TOP_PIN);
  return LIMIT_ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
}

inline bool bottomLimitActive() {
  int v = digitalRead(LIMIT_BOTTOM_PIN);
  return LIMIT_ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
}
bool pumpManualOn = false;

// What to do if a limit is hit during motion
void handleLimitHit(const char* which){
  stepperActive     = false;
  stepperContinuous = false;
  drilling          = false;
  homing            = false;
  stepsRemaining    = 0;
  stepPinHigh       = false;
  jogTimingActive   = false;

  analogWrite(PUMP_PIN, 0);
  digitalWrite(PUL_PIN, LOW);

  Serial.print(F("[LIMIT] "));
  Serial.print(which);
  Serial.println(F(" switch triggered, motion stopped."));
}

// --------- Motion routines ---------
void startJog(bool dirUp, long times){
  if (times < 1) times = 1;

  if (drilling || homing) {
    Serial.println(F("[WARN] Cannot jog while drilling/homing. Stop first."));
    return;
  }

  // Prevent jogging further into an active limit
  if (dirUp && topLimitActive()) {
    Serial.println(F("[WARN] Top limit active; cannot jog further UP."));
    return;
  }
  if (!dirUp && bottomLimitActive()) {
    Serial.println(F("[WARN] Bottom limit active; cannot jog further DOWN."));
    return;
  }

  unsigned long totalSteps = STEPS_PER_JOG * (unsigned long)times;
  stepsRemaining    = totalSteps;
  stepperContinuous = false;
  currentPulseLowUs = dirUp ? JOG_PULSE_LOW_UP : JOG_PULSE_LOW_DOWN;

  setDirLogical(dirUp);
  delayMicroseconds(200);

  stepPinHigh      = false;
  stepperActive    = true;
  nextEdgeUs       = micros();

  jogTimingActive  = true;
  jogStartMillis   = millis();

  Serial.print(F("JOG "));
  Serial.print(dirUp ? F("UP x") : F("DOWN x"));
  Serial.println(times);
}
void startFineJog(bool dirUp, long times){
  if (drilling || homing) {
    Serial.println(F("[WARN] Cannot fine-jog while drilling/homing. Stop first."));
    return;
  }

  // Same limit checks as startJog()
  if (dirUp && topLimitActive()) {
    Serial.println(F("[WARN] Top limit active; cannot fine-jog further UP."));
    return;
  }
  if (!dirUp && bottomLimitActive()) {
    Serial.println(F("[WARN] Bottom limit active; cannot fine-jog further DOWN."));
    return;
  }

  // Just a fixed, small number of steps (instead of times * STEPS_PER_JOG)
  unsigned long totalSteps = STEPS_PER_FINE_JOG * (unsigned long)times;
  stepsRemaining    = totalSteps;
  stepperContinuous = false;
  currentPulseLowUs = dirUp ? JOG_PULSE_LOW_UP : JOG_PULSE_LOW_DOWN;

  setDirLogical(dirUp);
  delayMicroseconds(200);

  stepPinHigh      = false;
  stepperActive    = true;
  nextEdgeUs       = micros();

  // You CAN time it like a normal jog if you want:
  jogTimingActive  = true;
  jogStartMillis   = millis();

  Serial.print(F("FINE JOG "));
  Serial.println(dirUp ? F("UP") : F("DOWN"));
}

void startDrill(){
  if (homing) {
    Serial.println(F("[WARN] Cannot start drill while homing."));
    return;
  }

  // Interrupt any jog in progress
  stepsRemaining    = 0;
  stepperContinuous = true;
  drilling          = true;
  jogTimingActive   = false;

  // Drill direction is typically DOWN
  logicalDirUp = false;
  setDirRaw(DRILL_DIR_HIGH);
  delayMicroseconds(200);
  currentPulseLowUs = DRILL_PULSE_LOW_US;

  stepPinHigh   = false;
  stepperActive = true;
  nextEdgeUs    = micros();

  analogWrite(PUMP_PIN, DRILL_PUMP_PWM);

  Serial.println(F("[DRILL] START"));
}

void stopDrill(){
  drilling          = false;
  stepperContinuous = false;
  stepperActive     = false;
  stepsRemaining    = 0;
  stepPinHigh       = false;

  analogWrite(PUMP_PIN, 0);
  digitalWrite(PUL_PIN, LOW);

  Serial.println(F("[DRILL] STOP"));
}
void togglePump() {
  pumpManualOn = !pumpManualOn;  // flip the state
  if (pumpManualOn) {
    analogWrite(PUMP_PIN, DRILL_PUMP_PWM);   // turn pump on at set PWM
    Serial.println(F("[PUMP] Manual pump ON"));
  } else {
    analogWrite(PUMP_PIN, 0);                // turn pump off
    Serial.println(F("[PUMP] Manual pump OFF"));
  }
}

// Homing routine (UP until top limit)
void startHome(){
  if (drilling) {
    Serial.println(F("[WARN] Cannot home while drilling."));
    return;
  }
  if (homing) {
    Serial.println(F("[INFO] Already homing."));
    return;
  }

  // If already at top limit, just mark homed
  if (topLimitActive()) {
    homed  = true;
    homing = false;
    Serial.println(F("[HOME] Top limit already active, home set."));
    return;
  }

  homing            = true;
  homed             = false;
  stepperContinuous = true;
  stepsRemaining    = 0;
  jogTimingActive   = false;

  // Move UP for homing
  setDirLogical(true);  // logical "UP"
  delayMicroseconds(200);
  currentPulseLowUs = HOME_PULSE_LOW_US;

  stepPinHigh   = false;
  stepperActive = true;
  nextEdgeUs    = micros();

  Serial.println(F("[HOME] Starting homing UP to top limit..."));
}

void killAll(){
  drilling          = false;
  stepperActive     = false;
  stepperContinuous = false;
  stepsRemaining    = 0;
  stepPinHigh       = false;
  jogTimingActive   = false;
  homing            = false;

  analogWrite(PUMP_PIN, 0);
  digitalWrite(PUL_PIN, LOW);

  Serial.println(F("[KILL] All motion and pump stopped."));
}

// --------- Stepper service (shared by jog/drill/home) ---------
void stepperService(){
  if (!stepperActive) return;

  // Check limits BEFORE stepping
  if (topLimitActive() && logicalDirUp) {
    // If homing, this means home reached
    if (homing) {
      stepperActive     = false;
      stepperContinuous = false;
      homing            = false;
      homed             = true;
      stepPinHigh       = false;
      digitalWrite(PUL_PIN, LOW);
      Serial.println(F("[HOME] Top limit reached, home complete."));
      return;
    } else {
      handleLimitHit("TOP");
      return;
    }
  }
  if (bottomLimitActive() && !logicalDirUp) {
    handleLimitHit("BOTTOM");
    return;
  }

  unsigned long now = micros();
  if (now < nextEdgeUs) return;

  if (!stepPinHigh){
    digitalWrite(PUL_PIN, HIGH);
    stepPinHigh = true;
    nextEdgeUs  = now + PULSE_HIGH_US;
  } else {
    digitalWrite(PUL_PIN, LOW);
    stepPinHigh = false;
    nextEdgeUs  = now + currentPulseLowUs;

    // For jogs, count steps and end when done
    if (!stepperContinuous && stepsRemaining > 0) {
      stepsRemaining--;
      if (stepsRemaining == 0) {
        stepperActive = false;

        if (jogTimingActive) {
          unsigned long elapsed = millis() - jogStartMillis;
          jogTimingActive = false;
          Serial.print(F("DONE (elapsed "));
          Serial.print(elapsed);
          Serial.println(F(" ms)"));
        } else {
          Serial.println(F("DONE"));
        }
      }
    }
  }
}

void printReport(){
  Serial.println(F("---- STATUS ----"));
  Serial.print(F("Drilling: ")); Serial.println(drilling ? F("ON") : F("OFF"));
  Serial.print(F("Homing: ")); Serial.println(homing ? F("YES") : F("NO"));
  Serial.print(F("Homed: ")); Serial.println(homed ? F("YES") : F("NO"));
  Serial.print(F("Stepper active: ")); Serial.println(stepperActive ? F("YES") : F("NO"));
  Serial.print(F("Stepper continuous: ")); Serial.println(stepperContinuous ? F("YES") : F("NO"));
  Serial.print(F("Steps remaining (jog): ")); Serial.println(stepsRemaining);
  Serial.print(F("Jog timing active: ")); Serial.println(jogTimingActive ? F("YES") : F("NO"));
  Serial.print(F("Top limit: ")); Serial.println(topLimitActive() ? F("ACTIVE") : F("clear"));
  Serial.print(F("Bottom limit: ")); Serial.println(bottomLimitActive() ? F("ACTIVE") : F("clear"));
  Serial.print(F("Drill pulse low (us): ")); Serial.println(DRILL_PULSE_LOW_US);
  Serial.print(F("Pump PWM: ")); Serial.println(DRILL_PUMP_PWM);
  Serial.print(F("Jog steps per command: ")); Serial.println(STEPS_PER_JOG);
  Serial.println(F("----------------"));
}

// --------- Command parsing ---------
void handleCommand(String cmd){
  if (cmd.length() == 0) return;

  char c = cmd.charAt(0);
  if (c == 'p' || c == 'P') {
    togglePump();
    return;
  }

  if (c == ' ') {
    if (drilling) stopDrill();
    else          startDrill();
    return;
  }

  if (c=='k' || c=='K') {
    killAll();
    return;
  }

  if (c=='h' || c=='H') {
    startHome();
    return;
  }

  String rest = cmd.substring(1);
  rest.trim();
  long n = rest.length() > 0 ? rest.toInt() : 1;
  if (n < 1) n = 1;

  if (c=='u' || c=='U'){
    startJog(true, n);
  } else if (c=='d' || c=='D'){
    startJog(false, n);
  } else if (c == 'f' || c == 'F'){
    startFineJog(true, n);
  } else if (c == 'g' || c == 'G') {
    startFineJog(false, n);
  } else if (c=='r' || c=='R'){
    printReport();
  } else {
    Serial.print(F("[WARN] Unknown command: '"));
    Serial.print(c);
    Serial.println(F("'"));
  }
}

void setup(){
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  pinMode(LIMIT_TOP_PIN,    INPUT_PULLUP);
  pinMode(LIMIT_BOTTOM_PIN, INPUT_PULLUP);

  setEnable(true);
  analogWrite(PUMP_PIN, 0);
  digitalWrite(PUL_PIN, LOW);

  Serial.begin(115200);
  while(!Serial) {}
  Serial.println(F("Jog+Drill+Home with limits @115200"));
  Serial.println(F("Commands: u, d, u10, d5, h=home, [space]=drill toggle, k=kill, r=report, f=finejogup , g=finejogdown, p=pump"));
}

void loop(){
  stepperService();

  while (Serial.available()){
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdBuffer.length() > 0) {
        handleCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else {
      cmdBuffer += ch;
    }
  }
}
