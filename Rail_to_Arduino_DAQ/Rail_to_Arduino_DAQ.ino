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
const float max_position_mm_horizontal = 25.00;
long currentPositionStepsH = 0;


// =============================================================
// LVDT CALIBRATION
// LVDT connected to Analog Pin A0
// =============================================================
const int LVDT_PIN = A0;
const float CAL_LVDT_SLOPE_MM_PER_BIT = -0.00518; // m (mm/bit)
const float CAL_LVDT_INTERCEPT_MM = 4.84308; // b (mm)

// LVDT Filtering Limits
const float LVDT_MAX_DEFLECTION = 3.7; 
const float LVDT_MIN_DEFLECTION = 1.7; 
const float CAL_Y_MAX_MM = 2.5; 

// =============================================================
// CALIBRATION COEFFICIENTS
// These map Commanded MM (calculated from steps) to Real MM (estimated position)
// Real_MM = Commanded_MM * Slope + Offset
// Commanded_MM = (Real_MM - Offset) / Slope
// =============================================================
float cal_slope_X = 0.900650; 
float cal_offset_X = -0.776723;
float cal_slope_Y = 1.639743; 
float cal_offset_Y = -1.543572;

// =============================================================
// FUNCTION HEADERS
// =============================================================
void moveBoth(float realHorizMM, float horizSpeed_mmps,
              float realVertMM, float vertSpeed_mmps);
void moveBoth_Without_Calib(float horizMM, float horizSpeed_mmps, float vertMM, float vertSpeed_mmps);
float readLVDT_mm();


// =============================================================
// GLOBAL LOGGING FLAG
// Used to print the header only once before streaming starts.
bool hasPrintedHeader = false;
const int LOG_INTERVAL_STEPS = 50; 

// =============================================================
// SETUP
// =============================================================
void setup() {
  pinMode(stepPinV, OUTPUT);
  pinMode(dirPinV, OUTPUT);
  pinMode(stepPinH, OUTPUT);
  pinMode(dirPinH, OUTPUT);
  pinMode(LVDT_PIN, INPUT); 

  Serial.begin(9600);
  delay(300);

  Serial.println("Ready. Use 'horizMM,horizSpeed,vertMM,vertSpeed' command.");
}


// =============================================================
// LOOP (MODIFIED FOR PYTHON)
// =============================================================
void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  
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
    moveBoth_Without_Calib(0, horizSpeed, 0, vertSpeed);
  }
}


// =============================================================
// LVDT READING FUNCTION
// =============================================================
float readLVDT_mm() {
  long sum_bits = 0;
  // Increase sampling for better average (e.g., 200 samples)
  for (int i = 0; i < 200; i++) sum_bits += analogRead(LVDT_PIN); 
  float avg_bit = sum_bits / 200.0;
  
  // Apply LVDT calibration: mm_measured = m * avg_bit + b
  float mm_measured = CAL_LVDT_SLOPE_MM_PER_BIT * avg_bit + CAL_LVDT_INTERCEPT_MM;
  
  
  return mm_measured;
}

// =============================================================
// MOVE FUNCTION (USES CALIBRATION) (REAL-TIME STREAMING)
// =============================================================
void moveBoth(float realHorizMM, float horizSpeed_mmps,
              float realVertMM, float vertSpeed_mmps)
{
  // 1. Convert REAL (Desired) MM to COMMANDED MM using inverse calibration
  float horizMM_cmd_target = (realHorizMM - cal_offset_X) / cal_slope_X;
  float vertMM_cmd_target = (realVertMM - cal_offset_Y) / cal_slope_Y;
  
  // 2. Constrain Commanded MM to physical limits
  horizMM_cmd_target = constrain(horizMM_cmd_target, 0.0, max_position_mm_horizontal);
  vertMM_cmd_target = constrain(vertMM_cmd_target, 0.0, max_position_mm_vertical);

  // 3. Convert Commanded MM to Target Steps
  long targetH = round(horizMM_cmd_target * steps_per_mm_horizontal);
  long targetV = round(vertMM_cmd_target * steps_per_mm_vertical);
  
  // 4. Calculate steps to move (delta) and direction (dir)
  long totalDeltaH = abs(targetH - currentPositionStepsH);
  long totalDeltaV = abs(targetV - currentPositionStepsV);

  bool dirH = (targetH > currentPositionStepsH);
  bool dirV = (targetV > currentPositionStepsV);

  // Horizontal direction is FLIPPED (LEFT/Positive MM = LOW)
  digitalWrite(dirPinH, dirH ? LOW : HIGH);
  // Vertical direction (DOWN/Positive MM = LOW) 
  digitalWrite(dirPinV, dirV ? LOW : HIGH);

  // 5. Calculate step delay (micro-seconds) from speed (mm/s)
  int delayH = max(80, int(1000000.0 / (horizSpeed_mmps * steps_per_mm_horizontal)));
  int delayV = max(80, int(1000000.0 / (vertSpeed_mmps * steps_per_mm_vertical)));

  // 6. Perform the movement (Blocking)
  long sH = 0; // Steps executed for Horizontal in this move
  long sV = 0; // Steps executed for Vertical in this move
  int log_counter = 0; // Counter for step logging interval
  
  // Initial Header Print (Only print header once at the start of the first movement)
  if (!hasPrintedHeader) {
    // UPDATED HEADER with Commanded MM and Real MM for both axes
    Serial.println("Time_ms,Steps_H,Commanded_H_mm,Real_H_mm,Steps_V,Commanded_V_mm,Real_V_mm,LVDT_Output_mm"); 
    hasPrintedHeader = true;
  }
  
  // Loop until both actuators have reached their delta steps
  while (sH < totalDeltaH || sV < totalDeltaV) {
    
    // Horizontal Stepping
    if (sH < totalDeltaH) {
      digitalWrite(stepPinH, HIGH); 
      delayMicroseconds(delayH);
      digitalWrite(stepPinH, LOW); 
      delayMicroseconds(delayH);
      sH++;
    }

    // Vertical Stepping
    if (sV < totalDeltaV) {
      digitalWrite(stepPinV, HIGH); 
      delayMicroseconds(delayV);
      digitalWrite(stepPinV, LOW); 
      delayMicroseconds(delayV);
      sV++;
    }
    
    // --- REAL-TIME LVDT SAMPLING AND STREAMING ---
    // Log data every LOG_INTERVAL_STEPS (50 steps)
    if (log_counter >= LOG_INTERVAL_STEPS) {
      unsigned long current_time_ms = millis();
      float current_lvdt_mm = readLVDT_mm();
      
      // Calculate current absolute position in steps
      long current_steps_H = currentPositionStepsH + (dirH ? sH : -sH);
      long current_steps_V = currentPositionStepsV + (dirV ? sV : -sV);

      // Calculate the Commanded MM (based on physical steps)
      float current_cmd_mm_H = current_steps_H / steps_per_mm_horizontal;
      float current_cmd_mm_V = current_steps_V / steps_per_mm_vertical;

      // Calculate the Estimated Real MM (based on calibration)
      float current_real_mm_H = current_cmd_mm_H * cal_slope_X + cal_offset_X;
      float current_real_mm_V = current_cmd_mm_V * cal_slope_Y + cal_offset_Y; // The one to compare with LVDT
      
      // Stream the data line (8 columns now)
      Serial.print(current_time_ms); Serial.print(",");
      Serial.print(current_steps_H); Serial.print(","); 
      Serial.print(current_cmd_mm_H, 6); Serial.print(","); // NEW
      Serial.print(current_real_mm_H, 6); Serial.print(","); // NEW
      Serial.print(current_steps_V); Serial.print(","); 
      Serial.print(current_cmd_mm_V, 6); Serial.print(","); // NEW
      Serial.print(current_real_mm_V, 6); Serial.print(","); // NEW
      Serial.print(current_lvdt_mm, 6); Serial.println();
      
      log_counter = 0; // Reset counter
    }
    log_counter++; // Increment counter for every step cycle
  }

  // 7. Update current position (steps 6 & 7 from original logic)
  currentPositionStepsH = targetH;
  currentPositionStepsV = targetV;
  
  // 8. FINAL DATA LOGGING (Using original markers for Python compatibility)
  
  // Capture final time and LVDT reading at the absolute stop point
  unsigned long time_at_stop_ms = millis();
  float final_lvdt_mm = readLVDT_mm();
  
  // --- FINAL POSITION LOG (Uses markers for Python to capture the 'official' result) ---
  // This section remains the same as it already captures the required calibration parameters.
  Serial.println("\n#MOVE_CALC_DATA_START");
  Serial.println("Time_ms,Desired_Horiz_MM,Commanded_Horiz_MM,X_Slope,X_Offset,Desired_Vert_MM,Commanded_Vert_MM,Y_Slope,Y_Offset,LVDT_Output");
  
  // Data Line
  Serial.print(time_at_stop_ms); Serial.print(","); 
  Serial.print(realHorizMM, 6); Serial.print(",");
  Serial.print(horizMM_cmd_target, 6); Serial.print(",");
  Serial.print(cal_slope_X, 6); Serial.print(",");
  Serial.print(cal_offset_X, 6); Serial.print(",");
  
  Serial.print(realVertMM, 6); Serial.print(",");
  Serial.print(vertMM_cmd_target, 6); Serial.print(",");
  Serial.print(cal_slope_Y, 6); Serial.print(",");
  Serial.print(cal_offset_Y, 6); Serial.print(",");
  
  Serial.print(final_lvdt_mm, 6);
  Serial.println(); 

  
  Serial.println("#MOVE_CALC_DATA_END\n");
  // ------------------------------------------------------------

  Serial.println(" ");
  Serial.print("Current Pos H: ");
  Serial.println(currentPositionStepsH);
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