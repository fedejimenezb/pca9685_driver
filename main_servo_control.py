# main_servo_control.py
# Example script demonstrating how to use the PCA9685_PinPong class
# from the pca9685_driver.py library file to control TWO servos.

import time
# Import necessary PinPong components
try:
    from pinpong.board import Board, I2C
except ImportError:
    print("Error: PinPong library not found. Please ensure it is installed.")
    exit()

# Import the PCA9685 driver class from the other file
# Requires pca9685_driver.py to be in the same directory or Python path
try:
    from pca9685_driver import PCA9685_PinPong
except ImportError:
    print("Error: Could not import PCA9685_PinPong from pca9685_driver.py.")
    print("Ensure pca9685_driver.py is saved in the same directory.")
    exit()

# --- Configuration ---
SERVO_CHANNEL_1 = 0  # Control servo connected to channel 0
SERVO_CHANNEL_2 = 15 # Control servo connected to channel 15

# --- Servo Calibration ---
# IMPORTANT: Adjust these values to match YOUR specific servos.
#            You might need different values for channel 0 and channel 15.
#            Set USE_CUSTOM_CALIBRATION to False to use library defaults (500-2500us / 0-180deg)
USE_CUSTOM_CALIBRATION = True
# Calibration for Servo 1 (Channel 0)
SERVO1_MIN_PULSE = 600  # Example: Pulse width (us) for min angle
SERVO1_MAX_PULSE = 2400 # Example: Pulse width (us) for max angle
SERVO1_MIN_DEG = 0      # Example: Min angle servo can physically reach
SERVO1_MAX_DEG = 180    # Example: Max angle servo can physically reach

# Calibration for Servo 2 (Channel 15) - Assuming same calibration for this example
# Adjust if your second servo is different!
SERVO2_MIN_PULSE = 600  # Example: Pulse width (us) for min angle
SERVO2_MAX_PULSE = 2400 # Example: Pulse width (us) for max angle
SERVO2_MIN_DEG = 0      # Example: Min angle servo can physically reach
SERVO2_MAX_DEG = 180    # Example: Max angle servo can physically reach


# --- Main Execution ---
pca_device = None # Define outside try for cleanup access

try:
    # 1. Initialize PinPong Board and I2C
    try:
        Board("UNIHIKER").begin()
        i2c_bus = I2C()
        print("PinPong Board and I2C Initialized.")
    except Exception as board_err:
        print(f"Error initializing PinPong Board/I2C: {board_err}")
        print("Please ensure the script runs in the correct environment.")
        exit()

    # 2. Create an instance of the PCA9685 driver
    pca_device = PCA9685_PinPong(i2c_bus) # Uses default address 0x40, freq 50Hz

    # 3. (Optional) Set custom calibration if needed
    if USE_CUSTOM_CALIBRATION:
        print("Setting custom calibration...")
        # Set calibration for Servo 1
        pca_device.set_servo_calibration(
            SERVO_CHANNEL_1,
            min_pulse=SERVO1_MIN_PULSE,
            max_pulse=SERVO1_MAX_PULSE,
            min_angle=SERVO1_MIN_DEG,
            max_angle=SERVO1_MAX_DEG
        )
        # Set calibration for Servo 2
        pca_device.set_servo_calibration(
            SERVO_CHANNEL_2,
            min_pulse=SERVO2_MIN_PULSE,
            max_pulse=SERVO2_MAX_PULSE,
            min_angle=SERVO2_MIN_DEG,
            max_angle=SERVO2_MAX_DEG
        )

    # 4. Control the servos
    print("\nStarting servo movement for channels {} and {}...".format(SERVO_CHANNEL_1, SERVO_CHANNEL_2))
    print("Make sure the separate servo power (V+) is connected and ON.")
    print("Press Ctrl+C to stop.")

    # Define angles to move to (using Servo 1's range for this example)
    center_angle = (SERVO1_MAX_DEG + SERVO1_MIN_DEG) / 2
    angles_to_test = [center_angle, SERVO1_MIN_DEG, SERVO1_MAX_DEG, center_angle] # Example sequence

    for target_angle in angles_to_test:
        print(f"Moving servos to {target_angle} degrees...")
        # Set angle for Servo 1
        pca_device.set_servo_angle(SERVO_CHANNEL_1, target_angle)
        # Set angle for Servo 2 (using the same target angle)
        pca_device.set_servo_angle(SERVO_CHANNEL_2, target_angle)

        # Wait for the servos to reach the position
        time.sleep(1.5) # Adjust delay as needed

    print("\nMovement sequence complete.")


except KeyboardInterrupt:
    print("\nKeyboard interrupt detected. Stopping servos and exiting.")
except RuntimeError as e:
    print(f"Runtime Error: {e}") # Catches errors like device not found during init
except Exception as e:
    print(f"\nAn unexpected error occurred: {e}")
finally:
    # --- Cleanup ---
    # Ensure PWM is turned off for the servo channels on exit
    if pca_device: # Check if the object was successfully created
        print("Cleaning up...")
        pca_device.cleanup(SERVO_CHANNEL_1) # Disable channel 1
        pca_device.cleanup(SERVO_CHANNEL_2) # Disable channel 2
        # Alternatively, disable all channels: pca_device.cleanup()
    print("Program finished.")

