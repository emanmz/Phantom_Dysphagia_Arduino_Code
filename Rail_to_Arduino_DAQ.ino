// =============================================================
// PIN & SETTINGS
// =============================================================

// Vertical actuator (100mm stroke)
#define dirPinV 9
#define stepPinV 8
const float steps_per_mm_vertical = 555.56;
const float max_position_mm_vertical = 100.0;
long currentPositionStepsV = 0;

// Horizontal actuator (13.04 mm stroke)
#define dirPinH 2
#define stepPinH 3
const float steps_per_mm_horizontal = 613.50;
//const float max_position_mm_horizontal = 13.04;
const float max_position_mm_horizontal = 25.00;
long currentPositionStepsH = 0;


// =============================================================
// LVDT CALIBRATION
// LVDT connected to Analog Pin A0
// Constants from your Python fit: mm = m * bit + b
// Your Python output: Slope (m) = -0.00518, Intercept (b) = 4.84308
// =============================================================
const int LVDT_PIN = A0;
const float CAL_LVDT_SLOPE_MM_PER_BIT = -0.00518; // m (mm/bit)
const float CAL_LVDT_INTERCEPT_MM = 4.84308; // b (mm)

// LVDT Filtering Limits: Data must be STRICTLY between these values.
const float LVDT_MAX_DEFLECTION = 3.7; // anything above or at this ignore
const float LVDT_MIN_DEFLECTION = 1.7; // anything below or at this ignore
const float CAL_Y_MAX_MM = 2.5; // Max vertical stroke used for calibration (LVDT limit)

// =============================================================
// CALIBRATION COEFFICIENTS
// PLACE CALIBRATION RESULTS HERE AFTER RUNNING CALIBRATION WITH ARDUINO CODE IF USING
// // =============================================================
float cal_slope_X = 0.900650; // Horizontal: real mm = slope * commanded_mm + offset
float cal_offset_X = -0.776723;
//Slope (m): 0.900650
//Intercept (b): -0.776723
float cal_slope_Y = 1.639743; // Vertical: real mm = slope * commanded_mm + offset
float cal_offset_Y = -1.543572;
//New Cal Slope (mm/mm): 1.639743
//New Cal Offset (mm): -1.543572

// =============================================================
// FUNCTION HEADERS
// =============================================================
void moveBoth(float realHorizMM, float horizSpeed_mmps,
              float realVertMM, float vertSpeed_mmps);
void moveBoth_Without_Calib(float horizMM, float horizSpeed_mmps, float vertMM, float vertSpeed_mmps);
float readLVDT_mm();
void calibrate_x();
void calibrate_y();
void linearRegression(float *x, float *y, int n, float &slope, float &intercept);


// =============================================================
// SETUP
// =============================================================
void setup() {
  pinMode(stepPinV, OUTPUT);
  pinMode(dirPinV, OUTPUT);
  pinMode(stepPinH, OUTPUT);
  pinMode(dirPinH, OUTPUT);
  pinMode(LVDT_PIN, INPUT); // A0 is default INPUT

  Serial.begin(9600);
  delay(300);

  Serial.println("Ready. Use 'X', 'Y', or 'horizMM,horizSpeed,vertMM,vertSpeed' command.");
}


// =============================================================
// LOOP (MODIFIED FOR PYTHON)
// =============================================================
void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  
  // NOTE: Python automation sends commands directly.
  
  if (input.equalsIgnoreCase("X")) { 
    Serial.println("Starting X calibration via serial command...");
    calibrate_x(); 
    return; 
  }
  if (input.equalsIgnoreCase("Y")) {
    Serial.println("Starting Y calibration via serial command...");
    calibrate_y(); 
    return; 
  }

  // --- direct movement (triggered by the Python script or manual input) ----
  if (input.indexOf(',') != -1) {
    int p1 = input.indexOf(',');
    int p2 = input.indexOf(',', p1 + 1);
    int p3 = input.indexOf(',', p2 + 1);
    
    // Check for all three commas
    if (p1 < 0 || p2 < 0 || p3 < 0) {
      Serial.println("Invalid format. Expected: horizMM,horizSpeed,vertMM,vertSpeed");
      return;
    }

    float horizMM = input.substring(0, p1).toFloat();
    float horizSpeed = input.substring(p1 + 1, p2).toFloat();
    float vertMM = input.substring(p2 + 1, p3).toFloat();
    float vertSpeed = input.substring(p3 + 1).toFloat();

    // safe speeds (constraints ensure movement is within reasonable limits)
    horizSpeed = constrain(horizSpeed, 5.0, 25.0);
    vertSpeed = constrain(vertSpeed, 20.0, 80.0);

    moveBoth(horizMM, horizSpeed, vertMM, vertSpeed);

    Serial.println("Returning to zero...");
    moveBoth(0, horizSpeed, 0, vertSpeed);
  }
}


// =============================================================
// LVDT READING FUNCTION
// =============================================================
float readLVDT_mm() {
  long sum_bits = 0;
  for (int i = 0; i < 200; i++) sum_bits += analogRead(LVDT_PIN); // Read 200 samples (0-1023)
  float avg_bit = sum_bits / 200.0;
  
  // Apply LVDT calibration: mm_measured = m * avg_bit + b
  float mm_measured = CAL_LVDT_SLOPE_MM_PER_BIT * avg_bit + CAL_LVDT_INTERCEPT_MM;
  
  // Filtering logic: only accept data STRICTLY between min and max
  if (mm_measured >= LVDT_MAX_DEFLECTION || mm_measured <= LVDT_MIN_DEFLECTION) {
    return -999.0; // Sentinel value for bad data
  }
  
  return mm_measured;
}
// =============================================================
// MOVE FUNCTION (USES CALIBRATION) (MODIFIED FOR PYTHON)
// =============================================================
void moveBoth(float realHorizMM, float horizSpeed_mmps,
              float realVertMM, float vertSpeed_mmps)
{
  // 1. Convert REAL (Desired) MM to COMMANDED MM using calibration
  // Real = Slope * Commanded + Offset   →   Commanded = (Real - Offset) / Slope
  float horizMM = (realHorizMM - cal_offset_X) / cal_slope_X;
  float vertMM = (realVertMM - cal_offset_Y) / cal_slope_Y;
  
  // --- CSV Data Output for Runtime Movement (Python Target) ---
  Serial.println("\n#MOVE_CALC_DATA_START");
  Serial.println("Desired_Horiz_MM,Commanded_Horiz_MM,X_Slope,X_Offset,Desired_Vert_MM,Commanded_Vert_MM,Y_Slope,Y_Offset");
  
  // Data Line
  Serial.print(realHorizMM, 6); Serial.print(",");
  Serial.print(horizMM, 6); Serial.print(",");
  Serial.print(cal_slope_X, 6); Serial.print(",");
  Serial.print(cal_offset_X, 6); Serial.print(",");
  
  Serial.print(realVertMM, 6); Serial.print(",");
  Serial.print(vertMM, 6); Serial.print(",");
  Serial.print(cal_slope_Y, 6); Serial.print(",");
  Serial.println(cal_offset_Y, 6);
  
  Serial.println("#MOVE_CALC_DATA_END\n");
  // ------------------------------------------------------------
  

  // 2. Constrain Commanded MM to physical limits
  horizMM = constrain(horizMM, 0.0, max_position_mm_horizontal);
  vertMM = constrain(vertMM, 0.0, max_position_mm_vertical);

  // 3. Convert Commanded MM to Target Steps
  long targetH = round(horizMM * steps_per_mm_horizontal);
  long targetV = round(vertMM * steps_per_mm_vertical);
  
  // 4. Calculate steps to move (delta) and direction (dir)
  long deltaH = abs(targetH - currentPositionStepsH);
  long deltaV = abs(targetV - currentPositionStepsV);

  bool dirH = (targetH > currentPositionStepsH);
  bool dirV = (targetV > currentPositionStepsV);

  // Horizontal direction is FLIPPED (LEFT/Positive MM = LOW)
  digitalWrite(dirPinH, dirH ? LOW : HIGH);
  // Vertical direction (DOWN/Positive MM = LOW) 
  digitalWrite(dirPinV, dirV ? LOW : HIGH);

  // 5. Calculate step delay (micro-seconds) from speed (mm/s)
  // Delay (us) = 1,000,000 / (Speed (mm/s) * Steps/mm)
  int delayH = max(80, int(1000000.0 / (horizSpeed_mmps * steps_per_mm_horizontal)));
  int delayV = max(80, int(1000000.0 / (vertSpeed_mmps * steps_per_mm_vertical)));

  // 6. Perform the movement (Blocking)
  long sH = 0; // Steps executed for Horizontal
  long sV = 0; // Steps executed for Vertical
  
  // Loop until both actuators have reached their target steps
  while (sH < deltaH || sV < deltaV) {
    
    // Horizontal Stepping
    if (sH < deltaH) {
      digitalWrite(stepPinH, HIGH); 
      delayMicroseconds(delayH);
      digitalWrite(stepPinH, LOW); 
      delayMicroseconds(delayH);
      sH++;
    }

    // Vertical Stepping
    if (sV < deltaV) {
      digitalWrite(stepPinV, HIGH); 
      delayMicroseconds(delayV);
      digitalWrite(stepPinV, LOW); 
      delayMicroseconds(delayV);
      sV++;
    }
  }

  // 7. Update current position
  Serial.println(" ");
  currentPositionStepsH = targetH;
  Serial.print("Current Pos H: ");
  Serial.println(currentPositionStepsH);
  currentPositionStepsV = targetV;
  Serial.print("Current Pos V: ");
  Serial.println(currentPositionStepsV);
}

void moveBoth_Without_Calib(float horizMM, float horizSpeed_mmps, float vertMM, float vertSpeed_mmps) {
  // Target steps can be negative if horizMM is negative
  long targetH = horizMM * steps_per_mm_horizontal;
  long targetV = vertMM * steps_per_mm_vertical;

  // delta is always the absolute step difference needed
  long deltaH = abs(targetH - currentPositionStepsH);
  long deltaV = abs(targetV - currentPositionStepsV);

  // dirH/dirV is true if moving to a higher (more positive) step count
  bool dirH = targetH > currentPositionStepsH;
  bool dirV = targetV > currentPositionStepsV;
  
  // Set directions
  // Horizontal direction is FLIPPED (LEFT/Positive MM = LOW)
  digitalWrite(dirPinH, dirH ? LOW : HIGH);
  // Vertical direction (DOWN/Positive MM = LOW) 
  digitalWrite(dirPinV, dirV ? LOW : HIGH); 

  // Calculate step delay (micro-seconds) from speed (mm/s)
  int delayH = (int)(1000000.0 / (horizSpeed_mmps * steps_per_mm_horizontal));
  int delayV = (int)(1000000.0 / (vertSpeed_mmps * steps_per_mm_vertical));
  // Ensure minimum delay (max speed limit)
  delayH = max(80, delayH);
  delayV = max(80, delayV);

  long stepsH = 0, stepsV = 0;

  // Core movement loop
  while (stepsH < deltaH || stepsV < deltaV) {
    if (stepsH < deltaH) {
      digitalWrite(stepPinH, HIGH);
      delayMicroseconds(delayH);
      digitalWrite(stepPinH, LOW);
      delayMicroseconds(delayH);
      stepsH++;
    }

    if (stepsV < deltaV) {
      digitalWrite(stepPinV, HIGH);
      delayMicroseconds(delayV);
      digitalWrite(stepPinV, LOW);
      delayMicroseconds(delayV);
      stepsV++;
    }
  }

  // Update current position to the new (potentially negative) target
  Serial.println(" ");
  currentPositionStepsH = targetH;
  Serial.print("Current Pos H: ");
  Serial.println(currentPositionStepsH);
  currentPositionStepsV = targetV;
  Serial.print("Current Pos V: ");
  Serial.println(currentPositionStepsV);
}