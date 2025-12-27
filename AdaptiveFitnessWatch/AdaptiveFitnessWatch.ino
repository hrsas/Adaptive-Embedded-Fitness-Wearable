#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include "MAX30105.h"
#include "heartRate.h"

//--TinyML requirements---
#include "rpe_model_quant.h"
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>

#define NUM_INPUTS 7
#define NUM_OUTPUTS 1
#define ARENA_SIZE 60000
#define NUMBER_OF_OPS 50

Eloquent::TF::Sequential<NUMBER_OF_OPS, ARENA_SIZE> tf;

const float scaler_mean[NUM_INPUTS] = {
  130.7151, 143.1066, 115.2316,
  25.2831, -0.9292, 3.7733, 0.2201
};

const float scaler_std[NUM_INPUTS] = {
  16.5989, 16.1716, 20.5403,
  6.9640, 0.2520, 1.2911, 0.0762
};


struct RPEFeatures {
  float avg_bpm;
  float peak_bpm;
  float min_bpm_rest;
  float hr_recovery_30s;
  float recovery_slope;
  float hr_variability;
  float session_load_index;
};

RPEFeatures rpeFeatures;

float bpmSum = 0;
int bpmSamples = 0;
float hrVarSum = 0;
int hrVarSamples = 0;
float prevHRForVar = -1;
bool restBaselineCaptured = false;
bool recoveryCaptured = false;


//----------------------end of TinyML requiremtns


/**************** HEART RATE MODULE ****************/

// ---- Sensor ----
MAX30105 hrSensor;

// ---- Beat detection (SparkFun) ----
const byte HR_RATE_SIZE = 4;
byte hrRates[HR_RATE_SIZE];
byte hrRateSpot = 0;

long hrLastBeat = 0;
float hrBPMInstant = 0;
int hrBPMAvg = 0;

// ---- RR + confidence ----
#define HR_RR_BUF 6
unsigned long hrRR[HR_RR_BUF];
byte hrRRIndex = 0;
bool hrRRFilled = false;

// ---- Gated BPM ----
int hrBPMUsed = 0;
int hrLastGoodBPM = 0;
float hrBPMTrend = 0;

// ---- Confidence ----
int hrConfidence = 0;

//---------------------------------------------


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
int peakWorkoutHR = 0;
float lastSetRPE = 0;   // latest RPE value for display/log
// --- Heart rate recovery state ---
unsigned long restStart = 0;
float restStartHR = 0;
bool restJustStarted = true;

#define STABILITY_THRESHOLD 0.3  // relaxed stability cutoff (0–1)
#define STEP_SETTLE_FACTOR 0.3   // fraction of step threshold for settle phase
#define MAX_STEP_SPIKE 4.0       // reject deltaX greater than 4× threshold
#define DISPLAY_REFRESH_MS 250   // OLED refresh every 250ms

const char* ssid = "Nidish’s iPhone";
const char* password = "helloman";
WebServer server(80);

#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_MPU6050 mpu;

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
void HR_Init();
void HR_Update();
int HR_GetBPM();
int HR_GetConfidence();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // off initially

  HR_Init(); //init heart rate sensor

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

    // ===== ML MODEL INIT =====
  tf.setNumInputs(NUM_INPUTS);
  tf.setNumOutputs(NUM_OUTPUTS);

  tf.resolver.AddFullyConnected();

  while (!tf.begin(rpe_model).isOk()) {
    Serial.println(tf.exception.toString());
    delay(1000);
  }

  Serial.println("✅ RPE ML model loaded");
}

void loop() {
  server.handleClient();
  
  // Handle button press to toggle between workout and step counting
  handleTouchInput();
  HR_Update();
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
    display.printf("HR: %d bpm", (int)HR_GetBPM());
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

  // // Reset peak HR at start of new exercise
  // static bool hrResetDone = false;

  // if (currentRep == 0 && currentSet == 1 && !hrResetDone) {
  //   peakWorkoutHR = 0;
  //   bpmSum = 0;
  //   bpmSamples = 0;
  //   hrResetDone = true;
  //   hrVarSum = 0;
  //   hrVarSamples = 0;
  //   prevHRForVar = -1;
  // }

  // // reset flag when exercise changes
  // if (currentRep == 0 && currentSet > 1) {
  //   hrResetDone = false;
  // }

      static int lastSetSeen = -1;

      if (currentSet != lastSetSeen) {
        lastSetSeen = currentSet;

        // Reset per-set HR features
        peakWorkoutHR = 0;
        bpmSum = 0;
        bpmSamples = 0;
        hrVarSum = 0;
        hrVarSamples = 0;
        prevHRForVar = -1;

        // Reset rest-related capture flags
        restBaselineCaptured = false;
        recoveryCaptured = false;

        Serial.printf("🔄 HR features reset for Set %d\n", currentSet);
      }


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

    rpeFeatures.avg_bpm = bpmSum / max(1, bpmSamples);
    rpeFeatures.peak_bpm = peakWorkoutHR;
    rpeFeatures.hr_variability = (hrVarSamples > 0) ? (hrVarSum / hrVarSamples) : 0;


    // ================= SESSION LOAD INDEX =================
    float durationMin = setDuration / 60.0f;
    float intensity = rpeFeatures.avg_bpm / 190.0f;

    rpeFeatures.session_load_index = intensity * durationMin;

    //reset rest flags
    restBaselineCaptured = false;
    recoveryCaptured = false;

    Serial.printf("LoadIndex=%.3f | avgHR=%.1f | duration=%.1f min\n",
      rpeFeatures.session_load_index,
      rpeFeatures.avg_bpm,
      durationMin);


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

  // ===== HR FEATURE ACCUMULATION (confidence-gated) =====
  if (HR_GetConfidence() > 50) {
    bpmSum += HR_GetBPM();
    bpmSamples++;
  }

    // ===== HR VARIABILITY (confidence-gated) =====
  if (HR_GetConfidence() > 50 && prevHRForVar >= 0) {
    hrVarSum += abs(HR_GetBPM() - prevHRForVar);
    hrVarSamples++;
  }
  prevHRForVar = HR_GetBPM();



  // Track highest HR reached during this set (for adaptive rest)
  if (HR_GetConfidence() > 60 && HR_GetBPM() > peakWorkoutHR)
    peakWorkoutHR = HR_GetBPM();

}

void handleRest() {
  // --- Persistent state across frames ---
  static bool restActive = false;
  static unsigned long lastUpdate = 0;
  static bool warnedNearEnd = false;
  static int stableCounter = 0;
  static float prevHR = 0.0f;
  static unsigned long entryDelayStart = 0;

  // --- Constants ---
  const unsigned long BASE_MIN_REST = 30000;     // 30s base rest
  const unsigned long MAX_REST_DURATION = 90000; // 90s safety cutoff
  const float HR_RECOVERY_FACTOR = 0.65f;        // 65% of peak HR
  const float HR_STABILITY_DELTA = 2.5f;         // bpm considered stable
  const int HR_STABILITY_FRAMES = 10;            // ~2.5s stable HR
  const unsigned long UPDATE_MS = 250;
  const unsigned long ENTRY_DELAY_MS = 1000;

  // --- Adaptive rest time based on effort ---
  float rpeFactor = constrain(lastSetRPE, 5.0f, 10.0f);
  unsigned long MIN_REST_DURATION = BASE_MIN_REST;
  if (rpeFactor >= 8.5f)      MIN_REST_DURATION = 40000;
  else if (rpeFactor <= 6.0f) MIN_REST_DURATION = 20000;

  // --- Entering REST state ---
  if (!restActive) {
    restActive = true;
    warnedNearEnd = false;
    stableCounter = 0;
    prevHR = HR_GetBPM();
    restStartTime = millis();
    restBaselineCaptured = false;
    recoveryCaptured = false;
    entryDelayStart = millis();

    Serial.printf(
      "🧘 REST START | HR=%.1f | Peak=%.1f | RPE=%.1f | MinRest=%lus\n",
      HR_GetBPM(), float(peakWorkoutHR), lastSetRPE,
      MIN_REST_DURATION / 1000
    );
  }

  // --- Time tracking ---
  unsigned long elapsedMs  = millis() - restStartTime;
  unsigned long elapsedSec = elapsedMs / 1000;

  // --- Periodic update ---
  if (millis() - lastUpdate > UPDATE_MS) {
    lastUpdate = millis();

    // ===== ML FEATURE CAPTURE =====

    // Baseline HR after 10s (confidence-gated)
    if (!restBaselineCaptured &&
        elapsedMs > 10000 &&
        HR_GetConfidence() > 50) {

      rpeFeatures.min_bpm_rest = HR_GetBPM();
      restBaselineCaptured = true;
    }

    // 30s HR recovery (confidence-gated)
    if (!recoveryCaptured &&
        elapsedMs > 30000 &&
        HR_GetConfidence() > 50) {

      rpeFeatures.hr_recovery_30s =
        rpeFeatures.peak_bpm - HR_GetBPM();
      recoveryCaptured = true;
    }

    // HR stability detection (confidence-gated)
    if (HR_GetConfidence() > 50 &&
        fabs(HR_GetBPM() - prevHR) < HR_STABILITY_DELTA) {
      stableCounter++;
    } else {
      stableCounter = max(0, stableCounter - 1);
    }

    prevHR = HR_GetBPM();

    // --- OLED ---
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("REST PHASE");

    display.setTextSize(2);
    display.setCursor(0, 12);
    display.printf("HR: %.0f", HR_GetBPM());

    display.setTextSize(2);
    display.setCursor(0, 36);
    display.printf("RPE: %.1f", lastSetRPE);

    display.setTextSize(1);
    display.setCursor(100, 0);
    display.printf("%lus", elapsedSec);

    display.drawRect(0, 57, 128, 6, SSD1306_WHITE);
    int barWidth = map(elapsedSec, 0,
                       (MIN_REST_DURATION / 1000), 0, 128);
    barWidth = constrain(barWidth, 0, 128);
    display.fillRect(0, 57, barWidth, 6, SSD1306_WHITE);
    display.display();

    // Warning near end
    if (!warnedNearEnd &&
        HR_GetBPM() <= 0.7f * peakWorkoutHR &&
        elapsedMs > (MIN_REST_DURATION / 2)) {

      tone(BUZZER_PIN, 2200, 120);
      delay(80);
      tone(BUZZER_PIN, 2500, 120);
      warnedNearEnd = true;
    }
  }

  // --- Safety delay ---
  if (millis() - entryDelayStart < ENTRY_DELAY_MS)
    return;

  // --- Exit conditions ---
  bool timeMet     = elapsedMs >= MIN_REST_DURATION;
  bool hrRecovered = HR_GetBPM() <= HR_RECOVERY_FACTOR * peakWorkoutHR;
  bool hrStable    = stableCounter >= HR_STABILITY_FRAMES;
  bool timeout     = elapsedMs >= MAX_REST_DURATION;

  if ((timeMet && hrRecovered && hrStable) || timeout) {

    // ===== RECOVERY SLOPE =====
    float restDurationSec = elapsedMs / 1000.0f;
    rpeFeatures.recovery_slope =
      (HR_GetBPM() - rpeFeatures.peak_bpm) / restDurationSec;

    // ===== ML RPE =====
    float mlRPE = runRPEModel();
    lastSetRPE = 0.85f * lastSetRPE + 0.15f * mlRPE;

    Serial.printf("🧠 ML RPE: %.2f | Smoothed: %.2f\n",
                  mlRPE, lastSetRPE);

    tone(BUZZER_PIN, 2000, 150);
    delay(100);
    tone(BUZZER_PIN, 2500, 150);
    delay(100);
    tone(BUZZER_PIN, 3000, 250);

    // --- Reset rest state ---
    restActive = false;
    lastUpdate = 0;
    warnedNearEnd = false;
    stableCounter = 0;
    prevHR = 0.0f;
    entryDelayStart = 0;

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


void HR_Init() {
  if (!hrSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("❌ MAX30102 not found");
    while (1);
  }

  hrSensor.setup();
  hrSensor.setPulseAmplitudeRed(0x7F);
  hrSensor.setPulseAmplitudeIR(0x7F);
  hrSensor.setPulseAmplitudeGreen(0);

  hrSensor.setADCRange(4096);
  hrSensor.setPulseWidth(411);
  hrSensor.setSampleRate(50);

  Serial.println("✅ Heart Rate sensor initialized");
}


void HR_Update() {
  long ir = hrSensor.getIR();

  if (checkForBeat(ir)) {
    unsigned long now = millis();
    unsigned long rr = now - hrLastBeat;
    hrLastBeat = now;

    hrBPMInstant = 60.0f / (rr / 1000.0f);

    if (hrBPMInstant > 20 && hrBPMInstant < 255) {
      hrRates[hrRateSpot++] = (byte)hrBPMInstant;
      hrRateSpot %= HR_RATE_SIZE;

      hrBPMAvg = 0;
      for (byte i = 0; i < HR_RATE_SIZE; i++)
        hrBPMAvg += hrRates[i];
      hrBPMAvg /= HR_RATE_SIZE;

      hrRR[hrRRIndex++] = rr;
      if (hrRRIndex >= HR_RR_BUF) {
        hrRRIndex = 0;
        hrRRFilled = true;
      }
    }
  }

  // ---- Confidence ----
  if (hrRRFilled) {
    float mean = 0;
    for (int i = 0; i < HR_RR_BUF; i++) mean += hrRR[i];
    mean /= HR_RR_BUF;

    float var = 0;
    for (int i = 0; i < HR_RR_BUF; i++) {
      float d = hrRR[i] - mean;
      var += d * d;
    }
    var /= HR_RR_BUF;

    float stdDev = sqrt(var);
    hrConfidence = constrain(100 - (stdDev / mean) * 100, 0, 100);
  } else {
    hrConfidence = 0;
  }

  // ---- Confidence gating ----
  if (hrConfidence >= 70) {
    hrBPMUsed = hrBPMAvg;
    hrLastGoodBPM = hrBPMUsed;
    hrBPMTrend = hrBPMUsed;
  }
  else if (hrConfidence >= 40) {
    hrBPMTrend += 0.1f * (hrBPMAvg - hrBPMTrend);
    hrBPMUsed = hrBPMTrend;
  }
  else {
    hrBPMUsed = hrLastGoodBPM;
  }
}

int HR_GetBPM() {
  return hrBPMUsed;
}

int HR_GetConfidence() {
  return hrConfidence;
}



float runRPEModel() {
  float input[NUM_INPUTS] = {
    rpeFeatures.avg_bpm,
    rpeFeatures.peak_bpm,
    rpeFeatures.min_bpm_rest,
    rpeFeatures.hr_recovery_30s,
    rpeFeatures.recovery_slope,
    rpeFeatures.hr_variability,
    rpeFeatures.session_load_index
  };

  for (int i = 0; i < NUM_INPUTS; i++) {
    input[i] = (input[i] - scaler_mean[i]) / scaler_std[i];
  }

  if (!tf.predict(input).isOk()) {
    Serial.println(tf.exception.toString());
    return lastSetRPE;
  }

  float rpe = tf.outputs[0];   // ✅ regression output
  rpe = constrain(rpe, 1.0f, 10.0f);
  return rpe;
}                

