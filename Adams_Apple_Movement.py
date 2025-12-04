import serial
import time
import csv

# --- Configuration ---
# Windows: 'COM3', 'COM4', etc.
# Mac/Linux: '/dev/ttyACM0' or '/dev/ttyUSB0' or similar (often starts with /dev/tty.)
ARDUINO_PORT = '/dev/cu.usbmodem141401'  # <-- CHANGE THIS TO YOUR ARDUINO PORT
BAUD_RATE = 9600
OUTPUT_CSV_FILE = 'movement_log6.csv'

# Define a list of movements to execute (in mm and mm/s)
# Format: [realHorizMM, horizSpeed_mmps, realVertMM, vertSpeed_mmps]
MOVEMENT_PLAN = [
    [0.0, 10, 0.5, 40]]

# --- Main Logic ---

def run_movements_and_log():
    try:
        # 1. Establish Serial connection
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=5)
        print(f"Connected to Arduino on {ARDUINO_PORT}")
        time.sleep(2) # Give Arduino time to reset and initialize Serial

        # Wait for "Ready." message from Arduino before sending commands
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "Ready" in line:
                print(f"Arduino acknowledged: {line}")
                break
        
        # 2. Prepare CSV file
        with open(OUTPUT_CSV_FILE, 'w', newline='') as csvfile:
            log_writer = csv.writer(csvfile)
            
            # The Arduino will print the stream header once globally.
            header_written = False
            
            # Flag to ignore the specific final summary block output
            is_in_summary_block = False 

            for i, move in enumerate(MOVEMENT_PLAN):
                # Construct the command string: "horizMM,horizSpeed,vertMM,vertSpeed\n"
                command = f"{move[0]},{move[1]},{move[2]},{move[3]}\n"
                print(f"\n--- Running Move {i+1}: {command.strip()} ---")
                
                # Send command
                ser.write(command.encode())
                
                # 3. Read the Serial response continuously until the move sequence finishes
                start_time = time.time()
                move_sequence_complete = False
                
                # We expect the full sequence (move to target + return to zero) 
                # to take no more than 30 seconds.
                while time.time() - start_time < 30: 
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not line:
                        continue
                    
                    # --- State Machine for Summary Block ---
                    if line == "#MOVE_CALC_DATA_START":
                        is_in_summary_block = True
                        continue
                    
                    if line == "#MOVE_CALC_DATA_END":
                        is_in_summary_block = False
                        # We also skip the next line, which is usually a blank line
                        ser.readline() 
                        continue
                    
                    # --- State Detection for Sequence End ---
                    # The Arduino prints "Current Pos V: " at the end of each moveBoth call.
                    if line.startswith("Current Pos V:"):
                        print(f"[Arduino]: {line}")
                        # We check if the next command in the loop should run 
                        # (i.e., this move is done)
                        if "Returning to zero" not in line: 
                            move_sequence_complete = True
                            break 
                    
                    # --- Data Logging Logic ---
                    # We check if the line is not empty, not a summary block, and contains commas (is data)
                    if not is_in_summary_block and ',' in line:
                        # Stream Header: "Time_ms,Steps_H,Steps_V,LVDT_Output_mm"
                        if line.startswith("Time_ms,Steps_H"):
                            if not header_written:
                                log_writer.writerow(line.split(','))
                                header_written = True
                        
                        # Data Line (Starts with a number and has commas)
                        elif header_written:
                            try:
                                # Attempt to cast the first element to float to confirm it's a data row
                                float(line.split(',')[0]) 
                                log_writer.writerow(line.split(','))
                            except ValueError:
                                # It's a line with commas but not data (e.g., error message)
                                print(f"[Arduino]: {line}") 
                        
                    # Print all other non-data, non-marker lines (debug messages)
                    elif not is_in_summary_block:
                         print(f"[Arduino]: {line}") 

                if not move_sequence_complete:
                    print(f"Warning: Move sequence {i+1} timed out or did not complete cleanly.")


    except serial.SerialException as e:
        print(f"\nERROR: Could not connect to port {ARDUINO_PORT}. Please check the port name and ensure the Arduino IDE Serial Monitor is closed.")
        print(f"Detail: {e}")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("\nSerial connection closed.")
            print(f"Data saved to {OUTPUT_CSV_FILE}")

if __name__ == "__main__":
    run_movements_and_log()