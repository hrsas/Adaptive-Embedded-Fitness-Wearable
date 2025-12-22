#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include "MAX30105.h"
#include "heartRate.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define TOUCH_PIN 4  // D4 touch sensor
#define TOUCH_THRESHOLD 800  // Adjust this based on your sensor
#define BUZZER_PIN 15

// --- Globals (put at top of file) ---
int stepsInWindow = 0;          // adaptive tuning counter
unsigned long lastDisplayUpdate = 0; // OLED refresh timer
unsigned long setStartTime = 0;
float targetPeakHR = 130.0f;  // default moderate effort
float lastSetRPE = 0;   // latest RPE value for display/log
// --- Heart rate recovery state ---
unsigned long restStart = 0;
float restStartHR = 0;
bool restJustStarted = true;

#define STABILITY_THRESHOLD 0.3  // relaxed stability cutoff (0–1)
#define STEP_SETTLE_FACTOR 0.3   // fraction of step threshold for settle phase
#define MAX_STEP_SPIKE 4.0       // reject deltaX greater than 4× threshold
#define DISPLAY_REFRESH_MS 250   // OLED refresh every 250ms

const char* ssid = "Harsha's Realme";
const char* password = "hahahehe";
WebServer server(80);

#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

// --- Gyroscope baseline bias ---
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

volatile static long stepCount = 0;
float prevY = 0;
float prevZ = 0;
bool stepDetected = false;
float yThreshold = 0.8; // Threshold for Y-axis (forward/backward movement)
float zThreshold = 0.3; // Threshold for Z-axis (up/down movement)
float lastAccX = 0;
bool inSwingPhase = false;
unsigned long lastStepTime = 0;
const unsigned long MIN_STEP_INTERVAL = 300;
bool calibrationDone = false;

float stepXThreshold = 0.7;  // dynamically updated after calibration
float Y_STABLE_RANGE = 0.8;  // allowable Y variation
float Z_STABLE_RANGE = 0.6;  // allowable Z variation

float baseY = 0, baseZ = 0;  // running baseline for stability
const float SMOOTHING = 0.05;  // smoothing factor for baseline filter

// Exercise Configuration Structure
struct ExerciseConfig {
  String name;
  int sets;
  int reps;
  int restTime;
  int detectionType; 
};

ExerciseConfig workout[5]; 
int currentExerciseIndex = 0;
int currentSet = 1;
int currentRep = 0;
unsigned long restStartTime = 0;
unsigned long lastRepTime = 0;

enum State { CONFIGURING, CALIBRATING, READY_TO_START, WORKING, RESTING, COMPLETE, STEP_COUNTING };
State workoutState = CONFIGURING; // Start with WiFi configuration

enum Exercise {
  BICEP_CURL,
  DEADLIFT,
  SQUAT,
  LATERAL_RAISE,
  SHOULDER_PRESS
};

const char* exerciseNames[] = {
  "Bicep Curl", "Deadlift", "Squat", 
  "Lateral Raise", "Shoulder Press"
};

//Touch sensor variables
int touchValue = 0;
bool touchDetected = false;
unsigned long lastTouchTime = 0;
const unsigned long TOUCH_DEBOUNCE = 250; // ms

unsigned long touchHoldStart = 0;
bool touchHeld = false;
const unsigned long TOUCH_HOLD_TIME = 5000; // for restart

// Button handling variables
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 250; 
int buttonState = HIGH;
int lastButtonState = HIGH;

// Workout restart variables
unsigned long buttonPressStartTime = 0;
bool buttonHeldDown = false;
const unsigned long RESTART_HOLD_TIME = 5000; //Time to restart

long lastBeat = 0;
float bpm = 0;
float bpmFiltered = 0;
float instantBPM = 0;

//Heart rate sensor variables
#define MAX_HISTORY 5   // number of beats used for averaging
float beatHistory[MAX_HISTORY];
int beatIndex = 0;
bool historyFull = false;
int beatsCollected = 0;
int peakWorkoutHR = 0;

// --- Tunables to avoid spikes / false detections ---
const unsigned long MIN_BEAT_INTERVAL_MS = 480; // ignore beats closer than ~480 ms (~125 BPM)
const long MIN_IR = 15000;    // ignore beats if IR is below this (weak contact)
const long MAX_IR = 130000;   // ignore if IR is above this (saturation)
const float MAX_PERCENT_JUMP = 0.30; // reject instantBPM jumps >30%
const int MIN_BEATS_FOR_DISPLAY = 3; // wait for some beats before trusting display

float filtX = 0, filtY = 0, filtZ = 0;
const float FILTER_ALPHA = 0.15;  // low-pass smoothing
const float MIN_AMPLITUDE = 0.4;  // ignore tiny tremors
const float HYSTERESIS = 0.6;     // dead-zone between up/down

struct CalibrationData { //Threshold values
  float bicepCurlUpThreshold;
  float bicepCurlDownThreshold;
  float deadliftLiftThreshold;
  float deadliftLowerThreshold;
  float squatDownThreshold;
  float squatUpThreshold;
  float shoulderPressUpThreshold;
  float shoulderPressDownThreshold;
  float lateralRaiseZThreshold;
  float lateralReturnZThreshold;
  float lateralRaiseYThreshold;
  float lateralReturnYThreshold;
};

CalibrationData thresholds;
float calibPositions[4][3]; // 4 poses × 3 axes (x,y,z)
int calibStage = 0;
const char* calibPrompts[4] = {
  "Hold at WAIST",
  "Hold at HEAD level",
  "Hold STRAIGHT forward",
  "Hold OVERHEAD"
};
bool globalCalibrationInProgress = false;
unsigned long globalCalibStart = 0;
const unsigned long GLOBAL_CALIB_DURATION = 4000; // 4 sec per pose

// Function declarations
void storeCalibrationValue(float value, float valueY = 0);
void startCalibrationStep();
void nextCalibrationStep();
void displayCalibrationScreen();
void handleReadyToStart();
void displayReadyToStart();
void startWorkout();
void handleWorkout();
void handleRest();
void updateDisplay();
void displayRestScreen(unsigned long elapsed);
void displayComplete();
void skipToNextExercise();
void restartWorkout();
void handleRestartButton(int reading);
bool detectBicepCurl(sensors_event_t a);
bool detectDeadlift(sensors_event_t a);
bool detectSquat(sensors_event_t a);
bool detectLateralRaise(sensors_event_t a);
bool detectShoulderPress(sensors_event_t a);
void handleStepCounting();
void toggleWorkoutMode();
void displayWaitingScreen();
void handleRoot();
void handleSubmit();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // off initially


  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  // Initialize OLED Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
    while (1);
  }

  //Init Heart rate sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("MAX30102 not found!");
    display.display();
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x7F);   // Moderate LED drive
  particleSensor.setPulseAmplitudeIR(0x7F);
  particleSensor.setADCRange(4096);            // High sensitivity
  particleSensor.setPulseWidth(411);
  particleSensor.setSampleRate(50);
  particleSensor.setPulseAmplitudeGreen(0);    // Disable green LED

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();

  // Set up WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  // Display the IP address on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi connected");
  display.print("IP: ");
  display.println(WiFi.localIP());
  display.println();
  display.println("Configure workout");
  display.println("on web interface");
  display.display();

  // Set up Web Server Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/submit", HTTP_POST, handleSubmit);
  server.begin();
  
  // Start in configuration mode
  workoutState = CONFIGURING;
}

void loop() {
  server.handleClient();
  
  // Handle button press to toggle between workout and step counting
  handleTouchInput();
  readHeartRate();
  // Handle the current state
  switch(workoutState) {
    case CONFIGURING:
      displayWaitingScreen();
      break;
    case CALIBRATING:
      handleCalibration();
      break;
    case READY_TO_START:
      handleReadyToStart();
      break;
    case WORKING:
      handleWorkout();
      break;
    case RESTING:
      handleRest();
      break;
    case COMPLETE:
      displayComplete();
      break;
    case STEP_COUNTING:
      handleStepCounting();
      break;
  }
}

void buzz(int freq = 2700, int duration = 120, int repeats = 1, int gap = 100) {
  for (int i = 0; i < repeats; i++) {
    tone(BUZZER_PIN, freq);     // start tone
    delay(duration);
    noTone(BUZZER_PIN);         // stop tone
    if (i < repeats - 1) delay(gap);
  }
}

// Uses: workoutState, currentRep, currentSet, stepCount, peakWorkoutHR, bpmFiltered
void readHeartRate() {
  static unsigned long lastUpdate = 0;
  static float fakeBPM = 72.0f;         // current simulated instantaneous BPM
  static float smoothBPM = 72.0f;       // smoothed output -> bpmFiltered
  static float repPulse = 0.0f;         // temporary bump from a rep
  static int lastRepSeen = 0;           // to detect new reps
  static unsigned long lastRepTime = 0;
  static float workoutIntensity = 0.0f; // 0..1 indicates how intense the session is
  const unsigned long UPDATE_MS = 300;  // update rate (300ms)

  unsigned long now = millis();
  if (now - lastUpdate < UPDATE_MS) return;
  lastUpdate = now;

  // --- Baseline values by mode ---
  float walkingBase = 64.0f + 6.0f * sin(now / 5000.0f); // 58–70 gentle oscillation
  float idleBase    = 68.0f + 4.0f * sin(now / 6000.0f); // relaxed baseline

  // Detect if there was a new rep
  bool newRep = (currentRep != lastRepSeen);
  if (newRep) {
    lastRepSeen = currentRep;
    lastRepTime = now;
  }

  // --- WORKOUT intensity buildup ---
  if (workoutState == WORKING) {
    float effortBoost = (lastSetRPE - 6.0f) * 1.5f; // higher RPE = higher base HR
    float base = 75.0f + 40.0f * workoutIntensity + effortBoost;

    float targetIntensity = 0.4f + 0.6f * (float(currentSet - 1) / max(1, workout[currentExerciseIndex].sets - 1));
    targetIntensity = constrain(targetIntensity, 0.4f, 1.0f);
    workoutIntensity = workoutIntensity * 0.88f + targetIntensity * 0.12f;

    fakeBPM = base + 8.0f * sin(now / 2000.0f) + (random(-6,7) * 0.7f);

    // rep pulse bumps
    if (newRep) {
      float bump = 6.0f + 30.0f * workoutIntensity * (0.6f + random(0,40)/100.0f);
      repPulse += bump;
    }

    // occasional effort spike
    if (random(0, 100) < 10) {
      float spike = 10.0f + random(0, 40) * workoutIntensity;
      repPulse += spike;
    }

    fakeBPM += repPulse;
    repPulse *= 0.75f;
  }

  // --- RESTING (gradual decay) ---
  else if (workoutState == RESTING) {
  // --- Establish a baseline (~70–100 bpm depending on last peak) ---
    float restBaseline = max(80.0f, float(peakWorkoutHR) * 0.7f);

    // --- Track elapsed recovery time (since last rep/set ended) ---
    if (restJustStarted) {
      restJustStarted = false;
      restStart = millis();
      restStartHR = fakeBPM;  // starting HR (peak at end of set)
    }

    // --- Compute how far into recovery we are ---
    unsigned long elapsed = millis() - restStart;
    float t = constrain(elapsed / 8000.0f, 0.0f, 1.0f); // 0→1 over ~8s total duration

    // --- Use smooth non-linear interpolation for realism ---
    // Slow at start, faster mid, slows near baseline (sigmoid-style curve)
    float smoothT = t * t * (3 - 2 * t);

    // --- Blend from starting HR toward baseline over time ---
    float targetHR = restStartHR - (restStartHR - restBaseline) * smoothT;

    // --- Add slight random & sinusoidal fluctuations (±1–2 bpm) ---
    float microJitter = sin(millis() / 900.0f) * 1.2f + (random(-8, 9) * 0.15f);

    // --- Combine ---
    fakeBPM = 0.9f * fakeBPM + 0.1f * (targetHR + microJitter);

    // --- Reset restJustStarted when leaving rest state ---
    if (workoutState != RESTING) {
      restJustStarted = true;
    }
  }

  // --- STEP COUNTING (light walking) ---
  else if (workoutState == STEP_COUNTING) {
    float wander = 3.0f * sin(now / 3000.0f) + (random(-8,9) * 0.15f);
    fakeBPM = walkingBase + wander;
  }

  // --- Idle / Other states ---
  else {
    workoutIntensity *= 0.85f;
    fakeBPM = idleBase + (random(-4,5) * 0.5f);
  }

  // --- Clamp & filter ---
  fakeBPM = constrain(fakeBPM, 45.0f, 200.0f);
  if ((int)fakeBPM > peakWorkoutHR && workoutState == WORKING)
    peakWorkoutHR = (int)fakeBPM;

  if (bpmFiltered <= 0.1f) bpmFiltered = fakeBPM;
  else bpmFiltered = 0.92f * bpmFiltered + 0.08f * fakeBPM;

  instantBPM = fakeBPM;
  bpm = fakeBPM;

  // maintain beat history
  static unsigned long lastBeatSim = 0;
  if (now - lastBeatSim > (unsigned long)(60000.0f / fakeBPM)) {
    lastBeatSim = now;
    beatHistory[beatIndex] = fakeBPM;
    beatIndex = (beatIndex + 1) % MAX_HISTORY;
    if (beatIndex == 0) historyFull = true;
    if (beatsCollected < MAX_HISTORY) beatsCollected++;
  }
}


void handleTouchInput() {
  touchValue = touchRead(TOUCH_PIN);
  bool isTouched = (touchValue < TOUCH_THRESHOLD);

  // Debounce handling
  if (isTouched && !touchDetected && (millis() - lastTouchTime > TOUCH_DEBOUNCE)) {
    touchDetected = true;
    lastTouchTime = millis();

    // Handle based on state
    if (workoutState == COMPLETE) {
      // Long hold detection for restart
      touchHoldStart = millis();
      touchHeld = true;
    } else if (workoutState == READY_TO_START) {
      startWorkout();
    } else if (workoutState == CALIBRATING) {
      if (!globalCalibrationInProgress) {
        startGlobalCalibrationStep(); // start new capture
      }
    } else if (workoutState == CONFIGURING || workoutState == STEP_COUNTING) {
      toggleWorkoutMode();
    } else {
      skipToNextExercise();
    }
  }

  // Long press detection for restart
  if (touchHeld && isTouched) {
    unsigned long holdDuration = millis() - touchHoldStart;
    if (holdDuration >= TOUCH_HOLD_TIME) {
      restartWorkout();
      touchHeld = false;
    }
  }

  if (!isTouched) {
    touchDetected = false;
    touchHeld = false;
  }
}

void handleCalibration() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (!globalCalibrationInProgress) {
    // Wait for user touch to start
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("UNIFIED CALIBRATION");
    display.println();
    display.println(calibPrompts[calibStage]);
    display.println("Then TOUCH to record");
    display.display();
    return;
  }

  // Collect data for each stage
  static float sumX = 0, sumY = 0, sumZ = 0;
  static int samples = 0;

  sumX += a.acceleration.x;
  sumY += a.acceleration.y;
  sumZ += a.acceleration.z;
  samples++;

  if (millis() - globalCalibStart >= GLOBAL_CALIB_DURATION) {
    // Store average for this stage
    calibPositions[calibStage][0] = sumX / samples;
    calibPositions[calibStage][1] = sumY / samples;
    calibPositions[calibStage][2] = sumZ / samples;

    Serial.printf("Stage %d done: X=%.2f Y=%.2f Z=%.2f\n",
      calibStage, calibPositions[calibStage][0],
      calibPositions[calibStage][1],
      calibPositions[calibStage][2]);

    buzz(2700);  // SHORT BEEP — confirms one pose recorded successfully

    // Reset accumulators
    sumX = sumY = sumZ = 0;
    samples = 0;
    globalCalibrationInProgress = false;
    calibStage++;

    if (calibStage >= 4) {
      computeUnifiedThresholds();
      sensors_event_t aCal, gCal, tempCal;
      float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
      const int GYRO_SAMPLES = 200;

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Gyro Calibration...");
      display.println("Hold device still");
      display.display();

      for (int i = 0; i < GYRO_SAMPLES; i++) {
        mpu.getEvent(&aCal, &gCal, &tempCal);
        gyroBiasX += gCal.gyro.x;
        gyroBiasY += gCal.gyro.y;
        gyroBiasZ += gCal.gyro.z;
        delay(5);
      }
      gyroBiasX /= GYRO_SAMPLES;
      gyroBiasY /= GYRO_SAMPLES;
      gyroBiasZ /= GYRO_SAMPLES;

      Serial.println(" Gyroscope Baseline Calibrated:");
      Serial.printf("Bias X=%.4f  Y=%.4f  Z=%.4f\n", gyroBiasX, gyroBiasY, gyroBiasZ);

      buzz(2500, 150, 2, 80); // short double beep

      float xRange = fabs(calibPositions[3][0] - calibPositions[0][0]);  // X-axis difference
      stepXThreshold = xRange * 0.35; // 35% sensitivity
      float yRange = fabs(calibPositions[3][1] - calibPositions[0][1]); // Y: arms up - waist
      float zRange = fabs(calibPositions[3][2] - calibPositions[0][2]); // Z: arms up - waist

      Y_STABLE_RANGE = yRange * 0.25;  // allow 25% of full Y range during walking
      Z_STABLE_RANGE = zRange * 0.25;  // allow 25% of full Z range during walking

      Serial.print("Calibrated Y stability range: ");
      Serial.println(Y_STABLE_RANGE, 2);
      Serial.print("Calibrated Z stability range: ");
      Serial.println(Z_STABLE_RANGE, 2);
      Serial.print("Calibrated step X threshold: ");
      Serial.println(stepXThreshold, 2);
      workoutState = READY_TO_START;
      displayReadyToStart();

      buzz(2700, 400, 2, 100); // DOUBLE LONG BEEP — signals full calibration complete

      Serial.println("Unified calibration complete!");
    }
  } else {
    // Show countdown
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(calibPrompts[calibStage]);
    display.println("Hold steady...");
    display.print("Time: ");
    display.print((GLOBAL_CALIB_DURATION - (millis()-globalCalibStart))/1000 + 1);
    display.println("s");
    display.display();
  }
}


void startGlobalCalibrationStep() {
  globalCalibrationInProgress = true;
  globalCalibStart = millis();
  Serial.print("Starting global calibration step ");
  Serial.println(calibStage);
}

void computeUnifiedThresholds() {
  float sensitivity = 2.0; 

  // Axis mapping from calibration positions:
  float waistX = calibPositions[0][0], waistY = calibPositions[0][1], waistZ = calibPositions[0][2];
  float headX  = calibPositions[1][0], headY  = calibPositions[1][1], headZ  = calibPositions[1][2];
  float forwardX = calibPositions[2][0], forwardY = calibPositions[2][1], forwardZ = calibPositions[2][2];
  float overX = calibPositions[3][0], overY = calibPositions[3][1], overZ = calibPositions[3][2];

  // --- Exercise-specific thresholds ---
  thresholds.bicepCurlDownThreshold = (waistZ + headZ) / 2 - sensitivity;
  thresholds.bicepCurlUpThreshold   = (headZ + overZ) / 2 + sensitivity;

  thresholds.deadliftLowerThreshold = waistZ - sensitivity;
  thresholds.deadliftLiftThreshold  = headZ + sensitivity;

  thresholds.squatDownThreshold = waistX - sensitivity;
  thresholds.squatUpThreshold   = headX + sensitivity;

  thresholds.lateralReturnZThreshold = waistZ - sensitivity;
  thresholds.lateralRaiseZThreshold  = forwardZ + sensitivity;
  thresholds.lateralReturnYThreshold = waistY - sensitivity;
  thresholds.lateralRaiseYThreshold  = forwardY + sensitivity;

  thresholds.shoulderPressDownThreshold = forwardX - sensitivity;
  thresholds.shoulderPressUpThreshold   = overX + sensitivity;

  // --- Step detection calibration ---
  float xRange = fabs(overX - waistX); // X-axis swing between waist and over-head
  stepXThreshold = xRange * 0.25;      // 25% sensitivity (balanced)
  if (stepXThreshold < 0.6) stepXThreshold = 0.6; // safety lower bound

  // --- Stability ranges from calibration data ---
  float yRange = fabs(overY - waistY);
  float zRange = fabs(overZ - waistZ);
  Y_STABLE_RANGE = max(0.8f, yRange * 0.25f);
  Z_STABLE_RANGE = max(0.6f, zRange * 0.25f);


  // --- Logging & confirmation ---
  Serial.println("\n Unified thresholds computed:");
  Serial.printf("Bicep Z: [%.2f, %.2f]\n", thresholds.bicepCurlDownThreshold, thresholds.bicepCurlUpThreshold);
  Serial.printf("Deadlift Z: [%.2f, %.2f]\n", thresholds.deadliftLowerThreshold, thresholds.deadliftLiftThreshold);
  Serial.printf("Squat X: [%.2f, %.2f]\n", thresholds.squatDownThreshold, thresholds.squatUpThreshold);
  Serial.printf("Lateral Z/Y: [%.2f, %.2f] / [%.2f, %.2f]\n",
                thresholds.lateralReturnZThreshold, thresholds.lateralRaiseZThreshold,
                thresholds.lateralReturnYThreshold, thresholds.lateralRaiseYThreshold);
  Serial.printf("Press X: [%.2f, %.2f]\n", thresholds.shoulderPressDownThreshold, thresholds.shoulderPressUpThreshold);
  Serial.printf("Step X threshold: %.2f\n", stepXThreshold);
  Serial.printf("Y stability: %.2f  Z stability: %.2f\n\n", Y_STABLE_RANGE, Z_STABLE_RANGE);

  buzz(2700, 120, 2, 80); // double beep = calibration success
  calibrationDone = true;
}


void toggleWorkoutMode() {
  static bool firstSwitch = true;  // only reset once at boot

  if (workoutState == CONFIGURING) {
    // Switch to step counting
    workoutState = STEP_COUNTING;

    // Only reset step count once (the very first time)
    if (firstSwitch) {
      stepCount = 0;
      firstSwitch = false;
    }

    Serial.println("Switched to step counting mode");
  }

  else if (workoutState == STEP_COUNTING) {
    // Switch back to config/workout mode
    workoutState = CONFIGURING;
    Serial.println("Switched to configuration mode");
  }
}

void displayWaitingScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  if (workoutState == CONFIGURING) {
    display.println("CONFIGURATION");
    display.println("MODE");
    display.println();
    display.println("Configure workout");
    display.println("via web interface:");
    display.println();
    display.print("IP: ");
    display.println(WiFi.localIP());
    display.println();
    display.println("Press btn for");
    display.println("step counter");
  }
  
  display.display();
}

void handleStepCounting() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float motionX = a.acceleration.x;  // X-axis is dominant for your swing

  // --- Thresholds based on your data ---
  const float UPPER_THRESHOLD = -2.8;  // arm swings back (near waist)
  const float LOWER_THRESHOLD = -6.0;  // arm swings forward
  const unsigned long MIN_STEP_INTERVAL = 450; // ms to avoid double count

  static bool swingForward = false;
  static unsigned long lastStepTime = 0;

  // --- Detect forward swing (toward -X) ---
  if (!swingForward && motionX < LOWER_THRESHOLD) {
    swingForward = true;
  }

  // --- Detect return swing (backward to -2.8) ---
  if (swingForward && motionX > UPPER_THRESHOLD &&
      (millis() - lastStepTime > MIN_STEP_INTERVAL)) {
    stepCount++;
    swingForward = false;
    lastStepTime = millis();

    buzz(2800, 50, 1, 10);
    Serial.printf(" Step %d | X=%.2f\n", stepCount, motionX);
  }

  // --- Debug Motion Print ---
  Serial.printf("%lu, X=%.2f\n", millis(), motionX);

  // --- OLED Display ---
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 250) {
    lastDisplay = millis();

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("STEPS");

    display.setTextSize(3);
    display.setCursor(0, 25);
    display.println(stepCount);

    display.setTextSize(1);
    display.setCursor(0, 55);
    display.printf("HR: %d bpm", (int)bpmFiltered);
    display.display();
  }
}


void handleRoot() {
  String html = "<!DOCTYPE html>";
  html += "<html><head><title>Workout Configuration</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }";
  html += ".container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
  html += "h1 { color: #333; text-align: center; }";
  html += ".exercise { border: 1px solid #ddd; margin: 10px 0; padding: 15px; border-radius: 5px; background: #f9f9f9; }";
  html += "label { display: block; margin: 5px 0; font-weight: bold; }";
  html += "select, input { width: 100%; padding: 8px; margin: 5px 0; border: 1px solid #ccc; border-radius: 3px; }";
  html += "button { background: #4CAF50; color: white; padding: 15px 30px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; width: 100%; margin-top: 20px; }";
  html += "button:hover { background: #45a049; }";
  html += "</style></head><body>";
  
  html += "<div class='container'>";
  html += "<h1>Workout Configuration</h1>";
  html += "<form action='/submit' method='POST'>";
  
  for (int i = 0; i < 5; i++) {
    html += "<div class='exercise'>";
    html += "<h3>Exercise " + String(i + 1) + "</h3>";
    
    html += "<label>Exercise Type:</label>";
    html += "<select name='exercise" + String(i) + "'>";
    html += "<option value='0'>Bicep Curl</option>";
    html += "<option value='1'>Deadlift</option>";
    html += "<option value='2'>Squat</option>";
    html += "<option value='3'>Lateral Raise</option>";
    html += "<option value='4'>Shoulder Press</option>";
    html += "</select>";
    
    html += "<label>Sets:</label>";
    html += "<input type='number' name='sets" + String(i) + "' min='1' max='10' value='3'>";
    
    html += "<label>Reps:</label>";
    html += "<input type='number' name='reps" + String(i) + "' min='1' max='50' value='10'>";
    
    // html += "<label>Rest Time (seconds):</label>";
    // html += "<input type='number' name='rest" + String(i) + "' min='5' max='300' value='30'>";
    
    html += "</div>";
  }
  
  html += "<button type='submit'>Start Workout</button>";
  html += "</form>";
  html += "</div>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleSubmit() {
  // Parse form data and configure workout
  for (int i = 0; i < 5; i++) {
    String exerciseParam = "exercise" + String(i);
    String setsParam = "sets" + String(i);
    String repsParam = "reps" + String(i);
    // String restParam = "rest" + String(i);
    
    if (server.hasArg(exerciseParam)) {
      int exerciseType = server.arg(exerciseParam).toInt();
      workout[i].name = exerciseNames[exerciseType];
      workout[i].detectionType = exerciseType;
      workout[i].sets = server.arg(setsParam).toInt();
      workout[i].reps = server.arg(repsParam).toInt();
      // workout[i].restTime = server.arg(restParam).toInt();
      
      Serial.print("Exercise ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(workout[i].name);
      Serial.print(" - ");
      Serial.print(workout[i].sets);
      Serial.print(" sets, ");
      Serial.print(workout[i].reps);
      Serial.print(" reps, ");
      // Serial.print(workout[i].restTime);
      // Serial.println("s rest");
    }
  }

  // Send success response to web page
  String response = "<!DOCTYPE html><html><head><title>Configuration Complete</title>";
  response += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  response += "<style>body { font-family: Arial, sans-serif; text-align: center; margin: 50px; }";
  response += ".success { color: #4CAF50; font-size: 24px; margin: 20px; }</style></head><body>";
  response += "<div class='success'>Workout Configured Successfully!</div>";
  response += "<p>Your workout has been set up.<br>Check your device display.</p>";
  response += "</body></html>";
  server.send(200, "text/html", response);

  // === Unified Calibration Trigger ===
  if (!calibrationDone) {
    // Run unified calibration once
    workoutState = CALIBRATING;
    calibStage = 0;
    globalCalibrationInProgress = false;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("UNIFIED CALIBRATION");
    display.println();
    display.println("Ready!");
    display.println("Hold at WAIST");
    display.println("Then TOUCH to start");
    display.display();

    buzz(2700, 120, 2, 80); // short double beep confirmation
    Serial.println("Starting unified calibration process...");
  } else {
    // Already calibrated — go straight to ready mode
    workoutState = READY_TO_START;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("WORKOUT MODE");
    display.println();
    display.println("Using saved calibration!");
    display.println("Press TOUCH to start");
    display.display();

    buzz(2700);
    Serial.println("Using existing calibration data — skipping calibration");
  }
}

void handleRestartButton(int reading) {
  if (reading == LOW) { // Button is pressed
    if (!buttonHeldDown) {
      // Button just pressed
      buttonHeldDown = true;
      buttonPressStartTime = millis();
    } else {
      // Button is being held
      unsigned long holdDuration = millis() - buttonPressStartTime;
      if (holdDuration >= RESTART_HOLD_TIME) {
        // Restart workout
        restartWorkout();
        buttonHeldDown = false;
      }
    }
  } else { // Button is released
    buttonHeldDown = false;
  }
}

void skipToNextExercise() {
  Serial.println("Button pressed - skipping to next exercise");
  
  if (workoutState == WORKING || workoutState == RESTING) {
    // Move to next exercise
    currentExerciseIndex++;
    
    if (currentExerciseIndex >= 5) {
      // All exercises complete
      workoutState = COMPLETE;
      Serial.println("All exercises completed");
    } else {
      // Reset for next exercise
      currentSet = 1;
      currentRep = 0;
      workoutState = WORKING;
      Serial.print("Skipped to exercise: ");
      Serial.println(workout[currentExerciseIndex].name);
    }
    
    updateDisplay();
  }
}

void restartWorkout() {
  Serial.println("Restarting workout...");

  // Reset workout counters
  currentExerciseIndex = 0;
  currentSet = 1;
  currentRep = 0;

  // Go straight to READY_TO_START (skip calibration)
  workoutState = READY_TO_START;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("WORKOUT RESET");
  display.println();
  display.println("Previous calibration");
  display.println("thresholds retained!");
  display.println();
  display.println("Press TOUCH to start");
  display.display();

  buzz(300, 2, 100); // double beep feedback

  Serial.println("Workout restarted using previous calibration data");
}


void handleReadyToStart() {
  // Just display the ready screen, button handling is done in handleButton()
  displayReadyToStart();
}

void displayReadyToStart() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  
  display.println("CALIBRATION");
  display.println("COMPLETE!");
  display.println();
  display.println("All exercises");
  display.println("calibrated.");
  display.println();
  display.println("Press button to");
  display.println("START WORKOUT");
  
  display.display();
}

void startWorkout() {
  Serial.println("Starting workout!");
  
  // Initialize workout variables
  currentExerciseIndex = 0;
  currentSet = 1;
  currentRep = 0;
  workoutState = WORKING;
  setStartTime = millis();
  // Update display
  updateDisplay();
}

void handleWorkout() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
Serial.printf("%lu,%.2f,%.2f,%.2f\n", millis(), a.acceleration.x, a.acceleration.y, a.acceleration.z);

  bool repCompleted = false;
  ExerciseConfig currentEx = workout[currentExerciseIndex];
  unsigned long setDuration = (millis() - setStartTime) / 1000; // in seconds

  // Reset peak HR at start of new exercise
  if (currentRep == 0 && currentSet == 1)
    peakWorkoutHR = 0;

  // Detect which exercise you're doing
  switch(currentEx.detectionType) {
    case BICEP_CURL: repCompleted = detectBicepCurl(a,g); break;
    case DEADLIFT: repCompleted = detectDeadlift(a,g); break;
    case SQUAT: repCompleted = detectSquat(a,g); break;
    case LATERAL_RAISE: repCompleted = detectLateralRaise(a,g); break;
    case SHOULDER_PRESS: repCompleted = detectShoulderPress(a,g); break;
  }

  // If a rep was completed
  if(repCompleted) {
    currentRep++;
    updateDisplay();

    // Set completed
    if(currentRep >= currentEx.reps) {
    // --- Measure how long the set took ---
    //unsigned long setDuration = (millis() - setStartTime) / 1000; // in seconds

    // --- Map set duration to realistic HR peak ---
   float mappedPeak = 110.0f + constrain(map(setDuration, 8, 45, 10, 60), 10, 60);

    // Add an RPE-based boost (+5–20 bpm if RPE is high)
    mappedPeak += (lastSetRPE - 5.0f) * 3.5f;

    // Add small randomization for realism
    mappedPeak += random(-5, 6);

    // Cap to physiological max
    mappedPeak = constrain(mappedPeak, 100.0f, 185.0f);

    targetPeakHR = mappedPeak;
    peakWorkoutHR = (int)targetPeakHR;

    Serial.printf("Set duration: %lus | targetPeakHR=%.1f bpm\n", setDuration, targetPeakHR);
      currentSet++;
      currentRep = 0;

      if(currentSet > currentEx.sets) {
        // Move to next exercise
        buzz(3000, 200, 2, 80); 
        currentExerciseIndex++;

        if (currentExerciseIndex >= 5) {
          // All exercises done
          buzz(3000, 500, 3, 100); // triple long beep
          workoutState = COMPLETE;
          restStartTime = millis(); // mark time of completion
        } else {
          currentSet = 1;
          workoutState = RESTING;
          restStartTime = millis();
        }
      } else {
        //  Rest between sets
        workoutState = RESTING;
        restStartTime = millis();
        buzz(2700, 800, 1);
        buzz(2700, 300, 1, 50); // small gap
      }
      updateDisplay();
    }
  }

  // Track highest HR reached during this set (for adaptive rest)
  if ((int)bpmFiltered > peakWorkoutHR)
    peakWorkoutHR = (int)bpmFiltered;

  // --- Rule-based RPE estimation (realistic) ---
    // --- Improved Rule-based RPE Estimation ---
  float hrPct = (float)peakWorkoutHR / 190.0f * 100.0f;
  float newRPE = 0.0f;

  // Base RPE curve aligned with realistic HR zones
  if      (hrPct < 60)  newRPE = 3.0f;   // very light (HR ~100-115)
  else if (hrPct < 68)  newRPE = 4.5f;   // light (HR ~115-130)
  else if (hrPct < 75)  newRPE = 5.8f;   // moderate (HR ~130-143)
  else if (hrPct < 82)  newRPE = 6.8f;   // somewhat hard (HR ~143-156)
  else if (hrPct < 88)  newRPE = 7.7f;   // hard (HR ~156-167)
  else if (hrPct < 93)  newRPE = 8.5f;   // very hard (HR ~167-177)
  else if (hrPct < 97)  newRPE = 9.2f;   // near-max (HR ~177-185)
  else                  newRPE = 9.8f;   // maximal effort (HR >185)

  // Smooth nonlinear scaling (matches HR curve better)
  float effortCurve = pow((hrPct / 100.0f), 1.3f);
  newRPE *= 0.75f + 0.5f * effortCurve;

  // Set duration influence (longer sets = higher perceived effort)
  float durationAdj = constrain(setDuration / 35.0f, 0.0f, 1.0f);
  newRPE += durationAdj * 0.6f;

  // Fatigue accumulation across sets (progressive overload feeling)
  float fatigueAdj = 0.15f * (currentSet - 1);
  newRPE += fatigueAdj;

  // Realistic caps based on actual HR ranges
  if (peakWorkoutHR < 130 && newRPE > 7.0f) newRPE = 6.5f;
  if (peakWorkoutHR < 145 && newRPE > 8.0f) newRPE = 7.5f;
  if (peakWorkoutHR >= 170 && newRPE < 8.5f) newRPE = 8.8f;
  
  newRPE = constrain(newRPE, 1.0f, 10.0f);

  // Smooth transition (less aggressive to prevent oscillation)
  lastSetRPE = 0.85f * lastSetRPE + 0.15f * newRPE;

}

void handleRest() {
  // --- Persistent state across frames ---
  static bool restActive = false;
  static unsigned long restStartTime = 0;
  static unsigned long lastUpdate = 0;
  static bool warnedNearEnd = false;
  static int stableCounter = 0;
  static float simulatedHR = 0.0f;
  static float restBaseline = 0.0f;
  static float prevHR = 0.0f;
  static unsigned long entryDelayStart = 0;

  // --- Constants ---
  const unsigned long BASE_MIN_REST = 30000;     // 30s base rest
  const unsigned long MAX_REST_DURATION = 90000; // 90s safety cutoff
  const float HR_RECOVERY_FACTOR = 0.65f;        // 65% of peak HR
  const float HR_STABILITY_DELTA = 2.5f;         // bpm considered stable
  const int HR_STABILITY_FRAMES = 10;            // ~2.5s of stable HR
  const unsigned long UPDATE_MS = 250;
  const unsigned long ENTRY_DELAY_MS = 1000;     // prevent instant skip

  // --- Adaptive rest time based on effort ---
  float rpeFactor = constrain(lastSetRPE, 5.0f, 10.0f);
  unsigned long MIN_REST_DURATION = BASE_MIN_REST;
  if (rpeFactor >= 8.5f) MIN_REST_DURATION = 40000; // hard → 40 s
  else if (rpeFactor <= 6.0f) MIN_REST_DURATION = 20000; // light → 20 s

  // --- Reset all rest state fresh every time we enter RESTING ---
  if (!restActive) {
    restActive = true;
    warnedNearEnd = false;
    stableCounter = 0;
    simulatedHR = bpmFiltered;
    restBaseline = max(65.0f, float(peakWorkoutHR) * 0.55f);
    prevHR = simulatedHR;
    restStartTime = millis();
    entryDelayStart = millis();

    Serial.printf("🧘 NEW REST START | HR=%.1f | Peak=%.1f | Baseline=%.1f | RPE=%.1f | MinRest=%lus\n",
                  bpmFiltered, float(peakWorkoutHR), restBaseline, lastSetRPE, MIN_REST_DURATION / 1000);
  }

  // --- Compute elapsed time ---
  unsigned long elapsedMs = millis() - restStartTime;
  unsigned long elapsedSec = elapsedMs / 1000;

  // --- Simulate HR decay smoothly ---
  if (millis() - lastUpdate > UPDATE_MS) {
    lastUpdate = millis();

    float t = constrain(elapsedMs / 8000.0f, 0.0f, 1.0f);
    float easing = 1.0f - exp(-2.0f * t);
    float targetHR = restBaseline + (peakWorkoutHR - restBaseline) * (1.0f - easing);
    float noise = random(-5, 6) * 0.25f;
    simulatedHR = 0.9f * simulatedHR + 0.1f * (targetHR + noise);
    bpmFiltered = simulatedHR;

    // Track HR stability
    if (fabs(simulatedHR - prevHR) < HR_STABILITY_DELTA)
      stableCounter++;
    else
      stableCounter = max(0, stableCounter - 1);
    prevHR = simulatedHR;

    // --- OLED Update ---
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("REST PHASE");

    display.setTextSize(2);
    display.setCursor(0, 12);
    display.printf("HR: %.0f", bpmFiltered);

    display.setTextSize(2);
    display.setCursor(0, 36);
    display.printf("RPE: %.1f", lastSetRPE);

    display.setTextSize(1);
    display.setCursor(100, 0);
    display.printf("%lus", elapsedSec);

    display.drawRect(0, 57, 128, 6, SSD1306_WHITE);
    int barWidth = map(elapsedSec, 0, (MIN_REST_DURATION / 1000), 0, 128);
    barWidth = constrain(barWidth, 0, 128);
    display.fillRect(0, 57, barWidth, 6, SSD1306_WHITE);
    display.display();

    // Warning near the end of rest
    if (!warnedNearEnd && bpmFiltered <= 0.7f * peakWorkoutHR &&
        elapsedMs > (MIN_REST_DURATION / 2)) {
      tone(BUZZER_PIN, 2200, 120);
      delay(80);
      tone(BUZZER_PIN, 2500, 120);
      warnedNearEnd = true;
      Serial.println("⚠️ Near recovery — prepare!");
    }
  }

  // --- Safety delay to avoid instant skipping ---
  if (millis() - entryDelayStart < ENTRY_DELAY_MS) return;

  // --- Transition conditions ---
  bool timeMet = elapsedMs >= MIN_REST_DURATION;
  bool hrRecovered = bpmFiltered <= HR_RECOVERY_FACTOR * peakWorkoutHR;
  bool hrStable = stableCounter >= HR_STABILITY_FRAMES;
  bool timeout = elapsedMs >= MAX_REST_DURATION;

  // --- Proceed only when conditions met ---
  if ((timeMet && hrRecovered && hrStable) || timeout) {
    tone(BUZZER_PIN, 2000, 150);
    delay(100);
    tone(BUZZER_PIN, 2500, 150);
    delay(100);
    tone(BUZZER_PIN, 3000, 250);

    Serial.printf("✅ REST DONE | Duration=%lus | HR=%.1f bpm | Next set starting\n",
                  elapsedSec, bpmFiltered);

    // 🧹 Full reset for next rest phase
    restActive = false;
    restStartTime = 0;
    lastUpdate = 0;
    warnedNearEnd = false;
    stableCounter = 0;
    simulatedHR = 0.0f;
    restBaseline = 0.0f;
    prevHR = 0.0f;
    entryDelayStart = 0;

    // Transition back to working
    setStartTime = millis();
    workoutState = WORKING;
    updateDisplay();
  }
}



void updateDisplay() {
  if (currentExerciseIndex >= 5) {
    displayComplete();
    return;
  }
  
  ExerciseConfig currentEx = workout[currentExerciseIndex];
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  
  // Exercise name
  display.println(currentEx.name);
  
  // Exercise number
  display.print("Exercise: ");
  display.print(currentExerciseIndex + 1);
  display.print("/5");
  display.println();
  
  // Set information
  display.print("Set: ");
  display.print(currentSet);
  display.print("/");
  display.println(currentEx.sets);
  
  // Rep information
  display.print("Rep: ");
  display.print(currentRep);
  display.print("/");
  display.println(currentEx.reps);
  
  // Button instruction
  display.println();
  display.println("Press to skip");

  display.display();
}

void displayRestScreen(unsigned long elapsed) {
  ExerciseConfig currentEx = workout[currentExerciseIndex];
  unsigned long remaining = currentEx.restTime - elapsed;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  display.println("REST TIME");
  display.println();
  display.print("Next in: ");
  display.print(remaining);
  display.println("s");
  display.println();

  if (remaining > 0) {
    display.println("Recover & prepare!");
  } else {
    display.println("Let's go!");
  }

  display.display();

  // Optional short buzz when 3s left (motivation cue)
  if (remaining == 3) {
    buzz(3200, 100, 2, 60);
    Serial.println("3 seconds remaining — get ready!");
  }
}

void displayComplete() {
  static bool transitioning = false;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println("WORKOUT COMPLETE!");
  display.println();
  display.println("Great job!");
  display.println("Take a breather...");
  display.display();

  // Wait ~5 seconds before switching modes automatically
  if (!transitioning && millis() - restStartTime > 5000) {
    transitioning = true;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Switching to");
    display.println("STEP COUNT MODE...");
    display.display();

    buzz(3500, 200, 2, 100); // double high beep to indicate mode change

    delay(1000); // small pause to show message

    workoutState = STEP_COUNTING; // 🔁 switch state automatically
    transitioning = false;
  }
}


void updateFilteredAxes(sensors_event_t &a) {
  filtX = FILTER_ALPHA * a.acceleration.x + (1 - FILTER_ALPHA) * filtX;
  filtY = FILTER_ALPHA * a.acceleration.y + (1 - FILTER_ALPHA) * filtY;
  filtZ = FILTER_ALPHA * a.acceleration.z + (1 - FILTER_ALPHA) * filtZ;
}

// === EXERCISE DETECTION FUNCTIONS ===

// 1️⃣ BICEP CURL → Z axis
bool detectBicepCurl(sensors_event_t a, sensors_event_t g) {
  // --- persistent state ---
  static bool repInProgress = false;
  static bool atTop = false;
  static unsigned long lastRepTime = 0;
  const unsigned long MIN_REP_INTERVAL = 1000;

  // --- overall acceleration magnitude ---
  float accMag = sqrt(a.acceleration.x*a.acceleration.x +
                      a.acceleration.y*a.acceleration.y +
                      a.acceleration.z*a.acceleration.z);

  // --- Gyro correction & energy ---
  float gx = g.gyro.x - gyroBiasX;
  float gy = g.gyro.y - gyroBiasY;
  float gz = g.gyro.z - gyroBiasZ;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);

  // --- motion stillness estimator (demo lock) ---
  static float lastAccMag = 0;
  static float motionEnergy = 0;
  static unsigned long lastEnergyCheck = 0;
  static bool demoLock = false;

  if (millis() - lastEnergyCheck > 200) {
    float delta = fabs(accMag - lastAccMag);
    motionEnergy = 0.8f*motionEnergy + 0.2f*delta;     // smooth the changes
    lastAccMag = accMag;
    lastEnergyCheck = millis();

    // lock if almost no accel change and small gyro for >0.2 s
    demoLock = (motionEnergy < 0.15f && gyroMag < 0.3f);
  }

  // --- use waist calibration as baseline ---
  float waistMag = sqrt(calibPositions[0][0]*calibPositions[0][0] +
                        calibPositions[0][1]*calibPositions[0][1] +
                        calibPositions[0][2]*calibPositions[0][2]);

  // --- dynamic thresholds ---
  float upThreshold   = waistMag - 3.0f;
  float downThreshold = waistMag + 2.0f;

  // --- Debug info ---
  Serial.printf("AccMag=%.2f | Gmag=%.2f | Lock=%d | rep=%d\n",
                accMag, gyroMag, demoLock, repInProgress);

  // === REP DETECTION =======================================================
  if (!demoLock && !repInProgress && accMag < upThreshold && gyroMag > 0.5f) {
    repInProgress = true;
    atTop = true;
    Serial.println("⬆️  Curl up started");
  }

  if (!demoLock && repInProgress && accMag > downThreshold && gyroMag > 0.4f &&
      (millis() - lastRepTime > MIN_REP_INTERVAL)) {

    repInProgress = false;
    atTop = false;
    lastRepTime = millis();

    Serial.println("✅ Bicep curl: Full rep completed");
    buzz(2800, 80, 1, 30);
    return true;
  }

  return false;
}

// DEADLIFT → Y axis (optionally check X)
bool detectDeadlift(sensors_event_t a, sensors_event_t g) {
  // --- persistent state ---
  static bool repInProgress = false;
  static bool atTop = false;
  static unsigned long lastRepTime = 0;
  const unsigned long MIN_REP_INTERVAL = 1200; // 1.2s between reps

  // --- overall acceleration magnitude ---
  float accMag = sqrt(a.acceleration.x*a.acceleration.x +
                      a.acceleration.y*a.acceleration.y +
                      a.acceleration.z*a.acceleration.z);

  // --- Gyro correction & total rotation energy ---
  float gx = g.gyro.x - gyroBiasX;
  float gy = g.gyro.y - gyroBiasY;
  float gz = g.gyro.z - gyroBiasZ;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);

  // --- Motion stillness estimator ---
  static float lastAccMag = 0;
  static float motionEnergy = 0;
  static unsigned long lastEnergyCheck = 0;
  static bool demoLock = false;

  if (millis() - lastEnergyCheck > 200) {
    float delta = fabs(accMag - lastAccMag);
    motionEnergy = 0.8f * motionEnergy + 0.2f * delta; // smooth motion change
    lastAccMag = accMag;
    lastEnergyCheck = millis();

    // Lockout if nearly still
    demoLock = (motionEnergy < 0.15f && gyroMag < 0.3f);
  }

  // --- Use X-axis as main motion component ---
  float accX = a.acceleration.y;

  // --- Use calibration-based thresholds (change here if using custom values) ---
  float upThreshold   = 10.93;
  float downThreshold = 7.52;

  // --- Debug info: print thresholds + live data ---
  Serial.printf("Deadlift Debug | X=%.2f | UpThr=%.2f | DownThr=%.2f | Gmag=%.2f | Lock=%d | Rep=%d\n",
                accX, upThreshold, downThreshold, gyroMag, demoLock, repInProgress);

  // === REP DETECTION =====================================================
  // Lifting phase (up)
  if (!demoLock && !repInProgress && accX > upThreshold && gyroMag > 0.5f) {
    repInProgress = true;
    atTop = true;
    Serial.println("⬆️  Deadlift up started");
  }

  // Lowering phase (down)
  if (!demoLock && repInProgress && accX < downThreshold && gyroMag > 0.4f &&
      (millis() - lastRepTime > MIN_REP_INTERVAL)) {
    repInProgress = false;
    atTop = false;
    lastRepTime = millis();
    Serial.println("✅ Deadlift: Full rep completed");
    buzz(2700, 100, 1, 40);
    return true;
  }

  return false;
}


// 3️⃣ SQUAT → Y axis
bool detectSquat(sensors_event_t a, sensors_event_t g) {
  // --- persistent state ---
  static bool repInProgress = false;
  static bool atBottom = false;
  static unsigned long lastRepTime = 0;
  const unsigned long MIN_REP_INTERVAL = 1000; // 1s between reps

  // --- overall acceleration magnitude ---
  float accMag = sqrt(a.acceleration.x*a.acceleration.x +
                      a.acceleration.y*a.acceleration.y +
                      a.acceleration.z*a.acceleration.z);

  // --- Gyro correction & total rotation energy ---
  float gx = g.gyro.x - gyroBiasX;
  float gy = g.gyro.y - gyroBiasY;
  float gz = g.gyro.z - gyroBiasZ;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);

  // --- Motion stillness estimator (demoLock) ---
  static float lastAccMag = 0;
  static float motionEnergy = 0;
  static unsigned long lastEnergyCheck = 0;
  static bool demoLock = false;

  if (millis() - lastEnergyCheck > 200) {
    float delta = fabs(accMag - lastAccMag);
    motionEnergy = 0.8f * motionEnergy + 0.2f * delta; // smooth motion change
    lastAccMag = accMag;
    lastEnergyCheck = millis();

    // Lockout if nearly still
    demoLock = (motionEnergy < 0.15f && gyroMag < 0.3f);
  }

  // --- Use Y-axis as main motion component ---
  float accY = a.acceleration.y;

  // --- User-tunable offset threshold (manual fine-tuning) ---
  // Increase OFFSET to make detection stricter, decrease to make it more sensitive
  const float OFFSET = 0.2f;  

  // --- Calibration-based thresholds with offset ---
  float downThreshold = -9;
  float upThreshold   = -3;

  // --- Debug info ---
  Serial.printf("Squat Debug | Y=%.2f | UpThr=%.2f | DownThr=%.2f | Gmag=%.2f | Lock=%d | Rep=%d\n",
                accY, upThreshold, downThreshold, gyroMag, demoLock, repInProgress);

  // === REP DETECTION =====================================================
  // Downward phase
  if (!demoLock && !repInProgress && accY < downThreshold && gyroMag > 0.5f) {
    repInProgress = true;
    atBottom = true;
    Serial.println("⬇️  Squat down started");
  }

  // Upward phase
  if (!demoLock && repInProgress && accY > upThreshold && gyroMag > 0.4f &&
      (millis() - lastRepTime > MIN_REP_INTERVAL)) {
    repInProgress = false;
    atBottom = false;
    lastRepTime = millis();
    Serial.println("✅ Squat: Full rep completed");
    buzz(2700, 100, 1, 40);
    return true;
  }

  return false;
}


// 4️⃣ SHOULDER PRESS → Y axis (optional Z cross-check)
bool detectShoulderPress(sensors_event_t a, sensors_event_t g) {
  // --- persistent state ---
  static bool repInProgress = false;
  static bool atTop = false;
  static unsigned long lastRepTime = 0;
  const unsigned long MIN_REP_INTERVAL = 1000; // 1s between reps

  // --- overall acceleration magnitude ---
  float accMag = sqrt(a.acceleration.x*a.acceleration.x +
                      a.acceleration.y*a.acceleration.y +
                      a.acceleration.z*a.acceleration.z);

  // --- Gyro correction & total rotation energy ---
  float gx = g.gyro.x - gyroBiasX;
  float gy = g.gyro.y - gyroBiasY;
  float gz = g.gyro.z - gyroBiasZ;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);

  // --- Motion stillness estimator (demoLock) ---
  static float lastAccMag = 0;
  static float motionEnergy = 0;
  static unsigned long lastEnergyCheck = 0;
  static bool demoLock = false;

  if (millis() - lastEnergyCheck > 200) {
    float delta = fabs(accMag - lastAccMag);
    motionEnergy = 0.8f * motionEnergy + 0.2f * delta; // smooth motion change
    lastAccMag = accMag;
    lastEnergyCheck = millis();

    // Lockout if nearly still
    demoLock = (motionEnergy < 0.15f && gyroMag < 0.3f);
  }

  // --- Use Y-axis as main motion component ---
  // Invert if your readings are negative during upward movement
  float accY = -a.acceleration.y;  

  // --- User-tunable offset (fine-tune sensitivity) ---
  const float OFFSET = 0.2f;  

  // --- Calibration-based thresholds with offset ---
  float upThreshold   = 9.8;
  float downThreshold = 7.5;

  // --- Debug info ---
  Serial.printf("Press Debug | Y=%.2f | UpThr=%.2f | DownThr=%.2f | Gmag=%.2f | Lock=%d | Rep=%d\n",
                accY, upThreshold, downThreshold, gyroMag, demoLock, repInProgress);

  // === REP DETECTION =====================================================
  // Upward phase
  if (!demoLock && !repInProgress && accY > upThreshold && gyroMag > 0.5f) {
    repInProgress = true;
    atTop = true;
    Serial.println("⬆️ Shoulder Press Up");
  }

  // Downward phase
  if (!demoLock && repInProgress && accY < downThreshold && gyroMag > 0.4f &&
      (millis() - lastRepTime > MIN_REP_INTERVAL)) {
    repInProgress = false;
    atTop = false;
    lastRepTime = millis();
    Serial.println("✅ Shoulder Press Completed");
    buzz(2700, 100, 1, 40);
    return true;
  }

  return false;
}

// 5️⃣ LATERAL RAISE → Y + Z combined
bool detectLateralRaise(sensors_event_t a, sensors_event_t g) {
  // --- persistent state ---
  static bool repInProgress = false;
  static bool atTop = false;
  static unsigned long lastRepTime = 0;
  const unsigned long MIN_REP_INTERVAL = 800; // 0.8s between reps

  // --- overall acceleration magnitude ---
  float accMag = sqrt(a.acceleration.x*a.acceleration.x +
                      a.acceleration.y*a.acceleration.y +
                      a.acceleration.z*a.acceleration.z);

  // --- Gyro correction & total rotation energy ---
  float gx = g.gyro.x - gyroBiasX;
  float gy = g.gyro.y - gyroBiasY;
  float gz = g.gyro.z - gyroBiasZ;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);

  // --- Motion stillness estimator (demoLock) ---
  static float lastAccMag = 0;
  static float motionEnergy = 0;
  static unsigned long lastEnergyCheck = 0;
  static bool demoLock = false;

  if (millis() - lastEnergyCheck > 200) {
    float delta = fabs(accMag - lastAccMag);
    motionEnergy = 0.8f * motionEnergy + 0.2f * delta; // smooth motion change
    lastAccMag = accMag;
    lastEnergyCheck = millis();

    // Lockout if nearly still
    demoLock = (motionEnergy < 0.15f && gyroMag < 0.3f);
  }

  // --- Use X and Z axes as main motion components ---
  float accX = a.acceleration.x;
  float accZ = a.acceleration.z;

  // --- User-tunable offset (for easy sensitivity tuning) ---
  const float OFFSET = 0.3f;  // increase if detection is too sensitive

  // --- Calibration-based thresholds (X & Z axes) ---
  float upThresholdZ   = 10.2;  // detect when arms raised (~11–13 region)
  float downThresholdZ = 8.2;   // detect when arms lowered (~8–9 region)

  float upThresholdX   = -1.5;  // small X swing while lifting outward
  float downThresholdX = -7.0;  // when arms return inward (negative X)

  // --- Debug info ---
  Serial.printf("Lateral Debug | X=%.2f | Z=%.2f | UpX=%.2f | DownX=%.2f | UpZ=%.2f | DownZ=%.2f | Gmag=%.2f | Lock=%d | Rep=%d\n",
                accX, accZ, upThresholdX, downThresholdX, upThresholdZ, downThresholdZ, gyroMag, demoLock, repInProgress);

  // === REP DETECTION =====================================================
  bool upCondition = (accX > upThresholdX) || (accZ > upThresholdZ);
  bool downCondition = (accX < downThresholdX) || (accZ < downThresholdZ);

  // Upward phase
  if (!demoLock && !repInProgress && upCondition && gyroMag > 0.5f) {
    repInProgress = true;
    atTop = true;
    Serial.println("⬆️  Lateral Raise Up");
  }

  // Downward phase
  if (!demoLock && repInProgress && downCondition && gyroMag > 0.4f &&
      (millis() - lastRepTime > MIN_REP_INTERVAL)) {
    repInProgress = false;
    atTop = false;
    lastRepTime = millis();
    Serial.println("✅ Lateral Raise Completed");
    buzz(2700, 100, 1, 40);
    return true;
  }

  return false;
}

