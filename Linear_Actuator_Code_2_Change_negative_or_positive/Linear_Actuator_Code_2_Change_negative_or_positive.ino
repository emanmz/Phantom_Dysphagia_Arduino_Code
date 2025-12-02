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
const float min_position_mm_vertical = -5.0;



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

      horizMM = constrain(horizMM, min_position_mm_horizontal, max_position_mm_horizontal);
      // Vertical constraint remains positive (0.0 to max)
      vertMM = constrain(vertMM, min_position_mm_vertical, max_position_mm_vertical); 
      
      // Safe speed constraints
      horizSpeed_mmps = constrain(horizSpeed_mmps, 5.0, 25.0);
      vertSpeed_mmps = constrain(vertSpeed_mmps, 2