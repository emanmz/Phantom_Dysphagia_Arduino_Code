## ⚙️ README: Linear Actuator Control Code

This README provides an overview and instructions for the Arduino code used to control two stepper-motor-driven linear actuators. This is includes multiple files:
Linear_Actuator_Code_2_Change_negative_or_positive
Linear_Actuator_Code_with_Calibration
Rail_to_Arduino_DAQ
Adams_Apple_Movement.py
Linear_Regression.py

## 1st **Code File:** (Implied: `Linear_Actuator_Code_2_Change_negative_or_positive.ino`)

-----

## ⚠️ WARNING: Free Use / Uncalibrated Mode

This code is designed for "free use," allowing absolute positioning of the actuators without applying calibration coefficients.

  * **DANGER:** Use caution. The position commands directly translate to motor steps, meaning if the physical system is not homed or if the motor loses steps, the reported position will be incorrect, potentially leading to the actuator hitting its physical end stops.
  * **Recommendation:** Only use this code if you fully understand the mechanics and stepper motor operation.

-----

## 📌 Actuator & Axis Definitions

This system uses standard CNC-style axis naming conventions:

| Axis | Actuator | Movement | Pins | Position Range |
| :--- | :--- | :--- | :--- | :--- |
| **Z** | Horizontal | Forward & Backwards | `dirPinH` (2), `stepPinH` (3) | **Negative and Positive** |
| **Y** | Vertical | Up & Down | `dirPinV` (9), `stepPinV` (8) | **Negative and Positive** |

-----

## 🚀 How to Use (Serial Interface)

The code runs in a continuous loop, monitoring the Serial Monitor for position and speed commands.

### 1\. Startup

1.  Connect the Arduino to the PC and open the **Serial Monitor** at **9600 baud**.
2.  The system will output: `Ready. Send input as: z_mm,z_mmps,vert_mm,vert_mmps`

### 2\. Sending Commands

Commands must be sent in the following comma-separated format:

```
z_mm,z_mmps,vert_mm,vert_mmps
```

| Parameter | Description | Units | Range |
| :--- | :--- | :--- | :--- |
| **`z_mm`** | Target absolute position for the **Z (Horizontal)** axis. | mm | Constrained by `min_position_mm_z` (e.g., -5.0) and `max_position_mm_z` (e.g., 25.0). |
| **`z_mmps`** | Desired speed for the **Z (Horizontal)** axis. | mm/s | Constrained to **5.0 to 25.0 mm/s**. |
| **`vert_mm`** | Target absolute position for the **Y (Vertical)** axis. | mm | Constrained by `min_position_mm_vertical` (e.g., -5.0) and `max_position_mm_vertical` (e.g., 100.0). |
| **`vert_mmps`**| Desired speed for the **Y (Vertical)** axis. | mm/s | Constrained to **20.0 to 80.0 mm/s**. |

**Example Command:**

To move the Z-axis to 10.0 mm at 15 mm/s and the Vertical (Y) axis to 50.0 mm at 40 mm/s, send:

```
10.0,15,50.0,40
```

-----

## 🛠️ Key Code Settings

The main configurable values are located at the top of the file under `PIN & SETTINGS`.

### Horizontal (Z) Settings

| Constant | Value | Description |
| :--- | :--- | :--- |
| `steps_per_mm_horizontal` | `613.50` | The physical steps required for 1 mm of Z-axis travel. **Crucial for accuracy.** |
| `max_position_mm_z` | `25.00` | Max positive position allowed for Z (soft limit). |
| `min_position_mm_z` | `-5.0` | Max negative position allowed for Z (soft limit). |

### Vertical (Y) Settings

| Constant | Value | Description |
| :--- | :--- | :--- |
| `steps_per_mm_vertical` | `555.56` | The physical steps required for 1 mm of Y-axis travel. **Crucial for accuracy.** |
| `max_position_mm_vertical` | `100.0` | Max positive position allowed for Y (soft limit). |
| `min_position_mm_vertical` | `-5.0` | Max negative position allowed for Y (soft limit). |

-----

## 💻 Core Functions

### `moveBoth(float zMM, float zSpeed_mmps, float vertMM, float vertSpeed_mmps)`

  * **Purpose:** Moves both actuators simultaneously to the absolute target positions specified by `zMM` and `vertMM`.
  * **Logic:**
    1.  Calculates the required **target steps** (`targetH`, `targetV`) by multiplying the input `mm` by the respective `steps_per_mm`.
    2.  Calculates the **delta steps** (`deltaH`, `deltaV`) needed from the current position.
    3.  Determines the direction (`dirH`, `dirV`) based on whether the target step count is higher or lower than the current step count.
    4.  Calculates the **step delay** from the input speed (`mm/s`).
    5.  Steps both motors in a blocking `while` loop until both reach their respective targets.
    6.  Updates the global position variables (`currentPositionStepsH`, `currentPositionStepsV`).

## 2nd Code: Linear_Actuator_Code_with_Calibration Calibrated 2-Axis Actuator Control with LVDT

This code is an Arduino sketch designed for **precise, closed-loop positioning** of two stepper motor actuators (Horizontal/Z and Vertical/Y) by applying **calibration coefficients** derived from an **LVDT (Linear Variable Differential Transformer)** sensor.

-----

## ⚠️ Operation Mode

This code is designed for laboratory or experimental use where the relationship between the commanded motor travel and the real-world measured travel must be continuously corrected.

  * **Axis Naming:**

      * **Horizontal (H) Actuator** = **Z-Axis** (Forward/Backward)
      * **Vertical (V) Actuator** = **Y-Axis** (Up/Down)

  * **Core Function:** It performs two main tasks:

    1.  **Calibration:** Runs routines (`calibrate_z`, `calibrate_y`) to determine the correction factors (slope and offset) between commanded motor movement and the actual movement measured by the LVDT.
    2.  **Movement:** Uses the calculated correction factors to convert a **Desired Real-World position** (`realHorizMM`) into the **necessary Commanded Motor Steps**.

-----

## 🛠️ Key Components & Setup

### 1\. Actuators and Kinematics

| Axis | Pin Settings | Max Position | Steps/mm |
| :--- | :--- | :--- | :--- |
| **Z** (Horizontal) | `dirPinH` (2), `stepPinH` (3) | `25.00 mm` | `613.50` |
| **Y** (Vertical) | `dirPinV` (9), `stepPinV` (8) | `100.0 mm` | `555.56` |

### 2\. LVDT Sensor (Measurement)

The LVDT measures real-world displacement and is critical for calibration.

| Setting | Value | Description |
| :--- | :--- | :--- |
| `LVDT_PIN` | `A0` | Analog Pin for LVDT input (0-1023 bits). |
| `CAL_LVDT_SLOPE_MM_PER_BIT` | `-0.00518` | Slope ($m$) from the LVDT's initial characterization: $mm = m \cdot bit + b$. |
| `CAL_LVDT_INTERCEPT_MM` | `4.84308` | Intercept ($b$) from the LVDT's initial characterization. |
| `LVDT_MAX/MIN_DEFLECTION` | `3.7` / `1.7` | Values used to **filter out bad/unstable readings** during data collection. |

### 3\. Calibration Coefficients

These values are updated after running the calibration routines (`Z` or `Y`). They define the final correction formula:

$$
\text{Real mm} = \text{Slope} \cdot \text{Commanded mm} + \text{Offset}
$$

| Axis | Variable | Initial Value |
| :--- | :--- | :--- |
| **Z** | `cal_slope_z` | `0` |
| **Z** | `cal_offset_z` | `-0.776723` |
| **Y** | `cal_slope_Y` | `1.639743` |
| **Y** | `cal_offset_Y` | `0` |

-----

## ⌨️ Serial Commands

All interaction is done via the **Serial Monitor** at **9600 baud**.

### 1\. Calibration

To initiate a calibration routine:

  * **Z-Axis (Horizontal):** Type `Z`
  * **Y-Axis (Vertical):** Type `Y`

The code will prompt you to confirm with `Ready` before starting the movement and data collection process.

### 2\. Movement (Using Calibration)

To move the actuators to a **real-world desired position** after calibration:

1.  Type `Move` (This simply prints the command format).
2.  Send the position and speed command in the following format:

<!-- end list -->

```
horiz_mm,horiz_speed,vert_mm,vert_speed
```

| Parameter | Description |
| :--- | :--- |
| `horiz_mm` | **Desired Real-World Z position (mm)**. |
| `horiz_speed` | **Z-Axis speed (mm/s)**. |
| `vert_mm` | **Desired Real-World Y position (mm)**. |
| `vert_speed` | **Y-Axis speed (mm/s)**. |

The `moveBoth` function will automatically apply the calculated **`cal_slope`** and **`cal_offset`** to determine the actual steps needed to hit the `realHorizMM` target.

-----

## ⚙️ Core Functionality


### **Core Functionality: `moveBoth(...)`**

* **Correction:** It inverts the calibration equation to find the required motor command using the formula above.
* **Execution:** The motor is then commanded to move to the calculated **Commanded mm** position using a non-blocking step algorithm.

### `moveBoth(...)`

This function handles the main movement using the calibration coefficients.

### **Inverse Calibration Formula**

The `moveBoth` function uses this formula to calculate the necessary motor command:

$$
\text{Commanded mm} = \frac{\text{Desired Real mm} - \text{Offset}}{\text{Slope}}
$$

  * **Execution:** The motor is then commanded to move to the `Commanded mm` position using a non-blocking step algorithm.

### `readLVDT_mm()`

Reads the LVDT sensor, averages **200 analog samples**, applies the LVDT's intrinsic calibration constants, and returns the measured displacement in mm. It uses the `LVDT_MAX/MIN_DEFLECTION` limits to return a sentinel value of `-999.0` for noisy or invalid readings, ensuring only quality data is used for the actuator calibration.

### `linearRegression(...)`

Performs a standard linear fit to calculate the **slope** and **intercept (offset)** based on the collected data points (Commanded vs. Measured positions). This result is stored in the global `cal_slope` and `cal_offset` variables.

This system is a sophisticated **Data Acquisition (DAQ)** platform that controls two linear actuators using pre-determined calibration coefficients and streams real-time LVDT sensor data back to a controlling **Python script** for logging and analysis.

---

#### 3rd & 4th **Code Files:** Calibrated 2-Axis Motion and Data Streaming (Rail\_to\_Arduino\_DAQ & Adams\_Apple\_Movement)

This project consists of two linked files:

1.  **Arduino Code (Rail\_to\_Arduino\_DAQ):** Controls the stepper motors, reads the LVDT, applies calibration corrections for movement, and streams real-time movement data.
2.  **Python Code (`Adams_Apple_Movement.py`):** Acts as the **Host Controller** to send movement commands to the Arduino and captures the streaming data into a CSV log file.

---

## 📌 Actuator & Axis Definitions

This system uses a horizontal actuator (Z) and a vertical actuator (Y).

| Axis | Actuator | Movement | Max Position (Commanded) |
| :--- | :--- | :--- | :--- |
| **H / X-Axis** | Horizontal | Forward & Backwards | `25.00 mm` |
| **V / Y-Axis** | Vertical | Up & Down | `100.0 mm` |

### Calibration Model

The core of the movement logic is the **inverse calibration** applied in the `moveBoth` function:

$$
\text{Commanded mm} = \frac{\text{Desired Real mm} - \text{Offset}}{\text{Slope}}
$$

This equation correctly solves for the necessary motor command to achieve the $\text{Desired Real mm}$ based on the calculated $\text{Slope}$ and $\text{Offset}$ coefficients.
| Axis | Coefficient | Value | Description |
| :--- | :--- | :--- | :--- |
| **X (H)** | `cal_slope_X` | `0.900650` | Slope correction for horizontal travel. |
| **X (H)** | `cal_offset_X` | `-0.776723` | Offset correction for horizontal travel. |
| **Y (V)** | `cal_slope_Y` | `1.639743` | Slope correction for vertical travel. |
| **Y (V)** | `cal_offset_Y` | `-1.543572` | Offset correction for vertical travel. |

---

## ⚡️ Arduino Sketch: Motion Control & Data Streaming

The Arduino sketch focuses on executing a movement, applying calibration, and streaming the resulting data over the serial port.

### Key Functionality

1.  **Movement (`moveBoth`):** Accepts **Desired Real-World MM** values, converts them to the required **Commanded Motor MM** using the calibration coefficients, and executes the movement by calculating the necessary steps and timing delays.
2.  **Real-Time Logging:** While the motors are stepping, data is logged every $\mathbf{50}$ **steps** (`LOG\_INTERVAL\_STEPS`). Each line streams:
    * Time (ms)
    * Current Steps (H & V)
    * Commanded MM (H & V, based on steps)
    * Estimated Real MM (H & V, based on calibration)
    * Real LVDT Reading (mm)
3.  **End-of-Move Summary:** After the motion is complete and the LVDT is sampled at the final position, a summary block is printed, demarcated by `#MOVE_CALC_DATA_START` and `#MOVE_CALC_DATA_END`. This allows the Python script to easily isolate the final, definitive data points for a single move.
4.  **Homing:** After every `moveBoth` command, the code immediately calls `moveBoth_Without_Calib(0, horizSpeed, 0, vertSpeed)` to **return the actuators to the zero position**.

---

## 🐍 Python Host Script: Data Acquisition (DAQ)

The Python script manages the experiment sequence, sends commands, and captures all serial output, separating the streaming data from the summary data.

### Configuration

* **`ARDUINO_PORT`:** Must be changed to match your Arduino's serial port (e.g., `'COM3'` on Windows, `'/dev/ttyACM0'` on Linux/Mac).
* **`OUTPUT_CSV_FILE`:** The file name where all real-time stream data will be logged (e.g., `'movement_log6.csv'`).
* **`MOVEMENT_PLAN`:** A list of moves to execute in sequence. Each element is `[realHorizMM, horizSpeed_mmps, realVertMM, vertSpeed_mmps]`.

### Data Logging Logic

The script uses a **state machine** to handle the varying output from the Arduino:

1.  **Stream Data:** All comma-separated lines *outside* the summary block are treated as real-time movement data and are written directly to the CSV file.
2.  **Summary Block:** Lines between `#MOVE_CALC_DATA_START` and `#MOVE_CALC_DATA_END` are read and ignored in the main CSV stream, as they serve as an end-of-move marker for more targeted data extraction if needed.
3.  **Sequence Completion:** The script waits until the Arduino prints a final position report (`"Current Pos V: "`) to confirm the entire sequence (move out + return to zero) is complete before proceeding to the next move in `MOVEMENT_PLAN`.
