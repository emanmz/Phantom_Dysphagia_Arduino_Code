// =============================================================
//                      PIN & SETTINGS
// =============================================================

// Vertical actuator (100mm stroke)
#define dirPinV 9
#define stepPinV 8
const float steps_per_mm_vertical = 555.56;
const float max_position_mm_vertical = 100.0;
// Note: currentPositionSteps must be a signed type (long) to track negative positions.
long currentPositionStepsV = 0; 

// Horizontal actuator (13.04 mm stroke)
#define dirPinH 2
#define stepPinH 3
const float steps_per_mm_horizontal = 613.50;
const float max_position_mm_horizontal = 13.04;
long currentPositionStepsH = 0; // Note: long is a signed type

// Define the maximum allowed negative travel for horizontal axis (adjust this value)
const float min_position_mm_horizontal = -5.0;


// =============================================================
//                      FUNCTION HEADERS
// =============================================================
void moveBoth(float horizMM, float horizSpeed_mmps, float vertMM, float vertSpeed_mmps);


// =============================================================
//                           SETUP
// =============================================================
void setup() {
  pinMode(stepPinV, OUTPUT);
  pinMode(dirPinV, OUTPUT);
  pinMode(stepPinH, OUTPUT);
  pinMode(dirPinH, OUTPUT);
  Serial.begin(9600);
  Serial.println("Ready. Send input as: horiz_mm,horiz_mmps,vert_mm,vert_mmps");
  Serial.print("Horizontal limits: ");
  Serial.print(min_position_mm_horizontal);
  Serial.print("mm to ");
  Serial.print(max_position_mm_horizontal);
  Serial.println("mm");
}


// =============================================================
//                            LOOP
// =============================================================
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.indexOf(',') != -1) {
      int p1 = input.indexOf(',');
      int p2 = input.indexOf(',', p1 + 1);
      int p3 = input.indexOf(',', p2 + 1);
      
      if (p1 < 0 || p2 < 0 || p3 < 0) {
        Serial.println("Invalid format. Use: horizMM,horizSpeed,vertMM,vertSpeed");
        return;
      }

      float horizMM = input.substring(0, p1).toFloat();
      float horizSpeed_mmps = input.substring(p1 + 1, p2).toFloat();
      float vertMM = input.substring(p2 + 1, p3).toFloat();
      float vertSpeed_mmps = input.substring(p3 + 1).toFloat();

      // --- CHANGE: Horizontal position constraint updated to allow negative movement ---
      horizMM = constrain(horizMM, min_position_mm_horizontal, max_position_mm_horizontal);
      // Vertical constraint remains positive (0.0 to max)
      vertMM = constrain(vertMM, 0.0, max_position_mm_vertical); 
      
      // Safe speed constraints
      horizSpeed_mmps = constrain(horizSpeed_mmps, 5.0, 25.0);
      vertSpeed_mmps = constrain(vertSpeed_mmps, 20.0, 80.0);

      Serial.print("Moving to H: "); Serial.print(horizMM); Serial.print("mm, V: "); Serial.print(vertMM); Serial.println("mm");
      
      // Move out
      moveBoth(horizMM, horizSpeed_mmps, vertMM, vertSpeed_mmps);
      
      Serial.println("Returning to zero...");
      // Then return to zero (0.0mm)
      //moveBoth(0.0, horizSpeed_mmps, 0.0, vertSpeed_mmps);
    }
  }
}


// =============================================================
//                MOVE FUNCTION (ABSOLUTE POSITIONING)
// =============================================================
void moveBoth(float horizMM, float horizSpeed_mmps, float vertMM, float vertSpeed_mmps) {
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
  currentPositionStepsH = targetH;
  currentPositionStepsV = targetV;
}