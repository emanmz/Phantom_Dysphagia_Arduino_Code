import serial
import time
import csv

# --- Configuration ---
# Windows: 'COM3', 'COM4', etc.
# Mac/Linux: '/dev/ttyACM0' or '/dev/ttyUSB0' or similar (often starts with /dev/tty.)
ARDUINO_PORT = 'COM3'  # <-- CHANGE THIS TO YOUR ARDUINO PORT
BAUD_RATE = 9600
OUTPUT_CSV_FILE = 'movement_log.csv'

# Define a list of movements to execute (in mm and mm/s)
# Format: [realHorizMM, horizSpeed_mmps, realVertMM, vertSpeed_mmps]
MOVEMENT_PLAN = [
    [1.0, 10, 0.5, 40],
    [2.5, 15, 1.0, 50],
    [0.5, 10, 2.0, 40],
    [2.0, 15, 1.5, 50],
    [0.0, 10, 0.0, 40] # Return to zero 
]

# --- Main Logic ---

def run_movements_and_log():
    try:
        # 1. Establish Serial connection
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=5)
        print(f"Connected to Arduino on {ARDUINO_PORT}")
        time.sleep(2) # Give Arduino time to reset and initialize Serial

        # Send 'Move' command to enter the move-listening state
        ser.write(b'Move\n')
        time.sleep(0.5)

        # 2. Prepare CSV file
        with open(OUTPUT_CSV_FILE, 'w', newline='') as csvfile:
            log_writer = csv.writer(csvfile)
            
            # Write a header row that matches the Arduino output structure
            header_written = False
            
            for i, move in enumerate(MOVEMENT_PLAN):
                # Construct the command string: "horizMM,horizSpeed,vertMM,vertSpeed\n"
                command = f"{move[0]},{move[1]},{move[2]},{move[3]}\n"
                print(f"\n--- Running Move {i+1}: {command.strip()} ---")
                
                # Send command
                ser.write(command.encode())
                
                # 3. Read the Serial response and capture the CSV data
                csv_data_found = False
                csv_line_read = ""
                
                start_time = time.time()
                while time.time() - start_time < 20: # Timeout after 20 seconds per move
                    line = ser.readline().decode('utf-8').strip()
                    
                    if not line:
                        continue
                    
                    if line == "#MOVE_CALC_DATA_START":
                        # Reading the header line
                        header_line = ser.readline().decode('utf-8').strip()
                        
                        # Reading the data line
                        data_line = ser.readline().decode('utf-8').strip()
                        
                        # Reading the end marker
                        end_marker = ser.readline().decode('utf-8').strip()

                        if data_line and end_marker == "#MOVE_CALC_DATA_END":
                            csv_line_read = data_line
                            csv_data_found = True
                            
                            # Write header only once
                            if not header_written:
                                log_writer.writerow(header_line.split(','))
                                header_written = True
                            
                            # Write data
                            log_writer.writerow(csv_line_read.split(','))
                            print("Data Captured and Logged.")
                            break # Move on to the next command
                        
                    print(f"[Arduino]: {line}") 
                
                if not csv_data_found:
                    print(f"Warning: CSV data not found for move {i+1}.")


    except serial.SerialException as e:
        print(f"\nERROR: Could not connect to port {ARDUINO_PORT}. Please check the port name and ensure the Arduino IDE Serial Monitor is closed.")
        print(f"Detail: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("\nSerial connection closed.")
            print(f"Data saved to {OUTPUT_CSV_FILE}")

if __name__ == "__main__":
    run_movements_and_log()