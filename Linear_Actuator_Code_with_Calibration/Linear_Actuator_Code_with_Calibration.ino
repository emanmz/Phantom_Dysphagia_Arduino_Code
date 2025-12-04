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
// (updated automatically after calibration)
// Resulting equation: real mm = slope * commanded_mm + offset
// =============================================================

//float cal_slope_X = 0.900650; 
//float cal_offset_X = -0.776723;
float cal_slope_X = 1; 
float cal_offset_X = 0;

float cal_slope_Y = 1.639743; 
float cal_offset_Y = 0;

//float cal_slope_Y = 1; 
//float cal_offset_Y = 0;

// =============================================================
// FUNCTION HEADERS
// =============================================================
void moveBoth(float realHorizMM, float horizSpeed_mmps,
              float realVertMM, float vertSpeed_mmps);
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

  Serial.println("Ready.");
  Serial.println("Commands:");
  Serial.println(" X → calibrate X");
  Serial.println(" Y → calibrate Y");
  Serial.println(" Move → enter XY move mode");
}


// =============================================================
// LOOP
// =============================================================
void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.equalsIgnoreCase("X")) { 
    Serial.println("Are you ready for X calibration? (Type 'Ready' to begin)");
    // Wait for the next input, which should be 'Ready'
    while (!Serial.available()); 
    String readyCheck = Serial.readStringUntil('\n');
    readyCheck.trim();

    if (readyCheck.equalsIgnoreCase("Ready")){
      calibrate_x(); 
    }
    return; 
  }
  if (input.equalsIgnoreCase("Y")) {
    Serial.println("Are you ready for Y calibration? (Type 'Ready' to begin)");
    // Wait for the next input, which should be 'Ready'
    while (!Serial.available());
    String readyCheck = Serial.readStringUntil('\n');
    readyCheck.trim();
    
    if (readyCheck.equalsIgnoreCase("Ready")){
      calibrate_y(); 
    }  
    return; 
  }

  if (input.equalsIgnoreCase("Move")) {
    Serial.println("Send: horiz_mm,horiz_speed,vert_mm,vert_speed");
    return;
  }

  // --- direct movement ----
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

    Serial.println("Moving...");
    moveBoth(horizMM, horizSpeed, vertMM, vertSpeed);

    Serial.println("Returning to zero...");
    //moveBoth(0, horizSpeed, 0, vertSpeed);
  }
}


// =============================================================
// LVDT READING FUNCTION
// Returns -999.0 if the reading is outside the min/max limits.
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
// LINEAR REGRESSION FUNCTION
// =============================================================
// Fits: y = slope * x + intercept
void linearRegression(float *x, float *y, int n, float &slope, float &intercept) {
  float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;

  for (int i = 0; i < n; i++) {
    sumX += x[i];
    sumY += y[i];
    sumXY += x[i] * y[i];
    sumXX += x[i] * x[i];
  }

  float denom = (n * sumXX - sumX * sumX);
  if (denom == 0) {
    // Avoid division by zero, set to default no-calibration
    slope = 1.0;
    intercept = 0.0;
    return;
  }

  slope = (n * sumXY - sumX * sumY) / denom;
  intercept = (sumY - slope * sumX) / n;
}


// =============================================================
// X CALIBRATION
// Uses 0.1 mm steps and filters LVDT data.
// =============================================================
void calibrate_x() {
  Serial.println("Moving Y axis for LVDT to sit flush with calibration block...");
  float y_coord_for_calib = 5.0;
  moveBoth_Without_Calib(0, 0, y_coord_for_calib, 10);
  Serial.println("Starting X calibration...");

  // Calibrate over the entire horizontal stroke (13.04 mm) with 0.1 mm steps
  float stepSize = 0.1;
  const int N_MAX = round(max_position_mm_horizontal / stepSize) + 1; 
  float mm_commanded_List[N_MAX]; // X-data (Commanded Actuator mm)
  float mm_measured_List[N_MAX]; // Y-data (LVDT Measured mm)

  int idx = 0; // Loop counter
  int valid_count = 0; // Actual number of valid points collected

  // Loop through all commanded positions
  for (float mm_cmd = 0.0; valid_count < 15; mm_cmd += stepSize) {
    //Serial.println(idx);
    //Serial.println(N_MAX);
    // Stop if commanded position exceeds max stroke
    if (mm_cmd > max_position_mm_horizontal){
      Serial.print("HERE!");
      break;
    }

    // Move horizontal actuator to mm_cmd, vertical actuator to y_coord_for_calib.
    moveBoth_Without_Calib(mm_cmd, 10, y_coord_for_calib, 1);
    delay(300); // Wait for movement to settle

    float mm_measured = readLVDT_mm(); // Read and filter LVDT data

    if (mm_measured != -999.0) { 
      mm_commanded_List[valid_count] = mm_cmd; // Record the commanded mm
      mm_measured_List[valid_count] = mm_measured; // Record the measured mm

      Serial.print("VALID Data ("); Serial.print(valid_count); Serial.print(") - Cmd: ");
      Serial.print(mm_cmd, 1);
      Serial.print("mm, Measured: ");
      Serial.println(mm_measured, 6);
      valid_count++;
    } else {
      Serial.print("IGNORED Data - Cmd: ");
      Serial.print(mm_cmd, 1);
      Serial.println("mm. LVDT reading out of limits.");
    }
    idx++; // Increment loop counter regardless of data validity
  }
  Serial.println("out of for loop");
  // Compute regression
  if (valid_count < 2) {
    Serial.println("ERROR: Insufficient valid data for X calibration. Keeping default coefficients.");
  } else {
    linearRegression(mm_commanded_List, mm_measured_List, valid_count, cal_slope_X, cal_offset_X);

    Serial.println("X Actuator Calibration Complete:");
    Serial.print("Valid Data Points: ");
    Serial.println(valid_count);
    Serial.print("New Cal Slope (mm/mm): ");
    Serial.println(cal_slope_X, 6);
    Serial.print("New Cal Offset (mm): ");
    Serial.println(cal_offset_X, 6);
  
  }
  
  // Return to zero position
  Serial.println("Returning to zero...");
  moveBoth_Without_Calib(0, 10, 0, 10);
}


// =============================================================
// Y CALIBRATION
// Uses 0.1 mm steps and filters LVDT data over a limited range.
// =============================================================
void calibrate_y() {
  Serial.println("Moving X axis for LVDT to sit flush with calibration block...");
  float x_coord_for_calib = 0.0;
  //moveBoth_Without_Calib(x_coord_for_calib, 10, 0, 0);
  Serial.println("Starting Y calibration...");

  // Calibrate over a limited vertical stroke (CAL_Y_MAX_MM) with 0.1 mm steps
  float stepSize = 0.1;
  const int N_MAX = round(CAL_Y_MAX_MM / stepSize) + 1; 
  float mm_commanded_List[N_MAX]; // X-data (Commanded Actuator mm)
  float mm_measured_List[N_MAX]; // Y-data (LVDT Measured mm)

  int idx = 0;
  int valid_count = 0;

  for (float mm_cmd = 0.0; valid_count < 15; mm_cmd += stepSize) {
    // Stop if commanded position exceeds calibration limit
    if (mm_cmd > CAL_Y_MAX_MM) break;

    // Move only vertical actuator to mm_cmd, horizontal actuator to 0.
    //moveBoth(0, 10, mm_cmd, 40); 
    moveBoth_Without_Calib(x_coord_for_calib, 0, mm_cmd, 10);
    delay(300);

    float mm_measured = readLVDT_mm(); // Read and filter LVDT data

    if (mm_measured != -999.0) {
      mm_commanded_List[valid_count] = mm_cmd;
      mm_measured_List[valid_count] = mm_measured;

      Serial.print("VALID Data ("); Serial.print(valid_count); Serial.print(") - Cmd: ");
      Serial.print(mm_cmd, 1);
      Serial.print("mm, Measured: ");
      Serial.println(mm_measured, 6);
      valid_count++;
    } else {
      Serial.print("IGNORED Data - Cmd: ");
      Serial.print(mm_cmd, 1);
      Serial.println("mm. LVDT reading out of limits.");
    }
    idx++;
  }

  // Compute regression
  if (valid_count < 2) {
    Serial.println("ERROR: Insufficient valid data for Y calibration. Keeping default coefficients.");
  } else {
    linearRegression(mm_commanded_List, mm_measured_List, valid_count, cal_slope_Y, cal_offset_Y);

    Serial.println("Y Actuator Calibration Complete:");
    Serial.print("Valid Data Points: ");
    Serial.println(valid_count);
    Serial.print("New Cal Slope (mm/mm): ");
    Serial.println(cal_slope_Y, 6);
    Serial.print("New Cal Offset (mm): ");
    Serial.println(cal_offset_Y, 6);
  }
  
  // Return to zero position
  Serial.println("Returning to zero...");
  moveBoth_Without_Calib(0, 10, 0, 40);
}


// =============================================================
// MOVE FUNCTION (USES CALIBRATION)
// Moves both actuators simultaneously using a simple blocking loop.
// =============================================================
void moveBoth(float realHorizMM, float horizSpeed_mmps,
              float realVertMM, float vertSpeed_mmps)
{
  // 1. Convert REAL (Desired) MM to COMMANDED MM using calibration
  // Real = Slope * Commanded + Offset   →   Commanded = (Real - Offset) / Slope
  float horizMM = realHorizMM / cal_slope_X;
  float vertMM = realVertMM / cal_slope_Y;
  Serial.println("Horizontal Movement");
  Serial.print(realHorizMM);
  Serial.print(" mm (Desired Measurment) | ");
  Serial.print(horizMM);
  Serial.print(" mm (Commanded Measurment)\n");
  
  Serial.println("Vertical Movement");
  Serial.print(realVertMM);
  Serial.print(" mm (Desired Measurment) | ");
  Serial.print(vertMM);
  Serial.print(" mm (Commanded Measurment)\n");
  

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