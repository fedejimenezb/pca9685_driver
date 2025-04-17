# pca9685_driver.py (Save this code as a Python file)
# Description: A library module to control a PCA9685 PWM/Servo Driver
#              using the PinPong library on UNIHIKER M10.

import time
import math
# Import Board and I2C classes from the pinpong library
# Ensure pinpong library is available in the environment where this is run.
try:
    from pinpong.board import Board, I2C
except ImportError:
    print("Error: PinPong library not found. Please ensure it is installed.")
    print("This library requires PinPong for hardware interaction.")
    # Define dummy classes if PinPong is missing, allowing import but failing later
    class Board: pass
    class I2C: pass
    # Or re-raise the error: raise

# Default Constants
DEFAULT_PCA9685_ADDRESS = 0x40
DEFAULT_PWM_FREQUENCY = 50
DEFAULT_SERVO_MIN_PULSE = 500
DEFAULT_SERVO_MAX_PULSE = 2500
DEFAULT_SERVO_MIN_ANGLE = 0
DEFAULT_SERVO_MAX_ANGLE = 180
NUM_CHANNELS = 16

class PCA9685_PinPong:
    """
    Class to control a PCA9685 PWM/Servo Driver using the PinPong library
    on a UNIHIKER M10 (or similar PinPong-supported board).
    Provides a higher-level interface for setting servo angles.
    """

    # --- Register Definitions (from Datasheet) ---
    _REG_MODE1       = 0x00
    _REG_MODE2       = 0x01
    _REG_PRE_SCALE   = 0xFE
    _REG_LED0_ON_L   = 0x06 # Base address for channel 0
    # ... (other registers could be defined if needed)

    # MODE1 Bits
    _MODE1_RESTART = 0x80
    _MODE1_SLEEP   = 0x10
    _MODE1_ALLCALL = 0x01
    # ... (other MODE1 bits)

    # MODE2 Bits
    _MODE2_OUTDRV  = 0x04 # 1 = Totem Pole (default), 0 = Open-Drain
    # ... (other MODE2 bits)

    def __init__(self, i2c_bus, address=DEFAULT_PCA9685_ADDRESS, freq=DEFAULT_PWM_FREQUENCY):
        """
        Initialize the PCA9685 controller.

        Args:
            i2c_bus: An initialized pinpong.board.I2C object.
            address (int): The I2C address of the PCA9685 board.
            freq (float): The initial PWM frequency to set (Hz).
        """
        if i2c_bus is None or not isinstance(i2c_bus, I2C):
             # Check if I2C is the dummy class due to import error
            if type(i2c_bus).__name__ == 'I2C' and 'pinpong.board' not in str(type(i2c_bus)):
                 raise ImportError("PinPong library failed to import correctly.")
            raise ValueError("An initialized PinPong I2C object is required.")

        self.i2c = i2c_bus
        self.address = address
        self._frequency = 0 # Will be set by set_pwm_frequency

        # Initialize servo calibration data storage
        self._servo_calibration = []
        for _ in range(NUM_CHANNELS):
            self._servo_calibration.append({
                "min_pulse": DEFAULT_SERVO_MIN_PULSE,
                "max_pulse": DEFAULT_SERVO_MAX_PULSE,
                "min_angle": DEFAULT_SERVO_MIN_ANGLE,
                "max_angle": DEFAULT_SERVO_MAX_ANGLE,
            })

        print(f"\nChecking for PCA9685 at address 0x{self.address:02X}...")
        if self._read_reg(self._REG_MODE1) is None:
            raise RuntimeError(f"PCA9685 not found or not responding at address 0x{self.address:02X}.")
        else:
            print("PCA9685 detected successfully!")

        # Perform initial chip setup
        self.begin()
        if not self.set_pwm_frequency(freq):
             # Attempt frequency set again or raise error?
             print("Warning: Initial frequency setting failed. Retrying...")
             time.sleep(0.1)
             if not self.set_pwm_frequency(freq):
                  raise RuntimeError("Failed to set initial PWM frequency.")

    # --- Low-Level I2C Communication Wrappers ---
    def _write_reg(self, reg, value):
        """Internal helper to write a single byte to a register."""
        try:
            self.i2c.writeto_mem(self.address, reg, [value & 0xFF])
        except Exception as e:
            print(f"Error writing 0x{value:02X} to I2C Register 0x{reg:02X}: {e}")
            raise

    def _read_reg(self, reg):
        """Internal helper to read a single byte from a register."""
        try:
            read_data = self.i2c.readfrom_mem(self.address, reg, 1)
            return read_data[0] if read_data and len(read_data) > 0 else None
        except Exception:
            return None

    # --- Core Control Methods ---
    def begin(self, mode2_value=None):
        """Initialize the PCA9685 chip: Wake up and set MODE2."""
        print("Initializing PCA9685 chip...")
        mode1 = self._read_reg(self._REG_MODE1)
        if mode1 is not None and (mode1 & self._MODE1_SLEEP):
            print("  Chip was asleep, waking up...")
            self._write_reg(self._REG_MODE1, mode1 & ~self._MODE1_SLEEP)
            time.sleep(0.005)
        elif mode1 is None: # If read failed, still try to wake
             print("  Warning: Could not read MODE1, attempting wake...")
             self._write_reg(self._REG_MODE1, 0x00)
             time.sleep(0.005)

        if mode2_value is None: mode2_value = self._MODE2_OUTDRV
        print(f"  Setting MODE2 to 0x{mode2_value:02X}")
        self._write_reg(self._REG_MODE2, mode2_value)
        print("PCA9685 begin() complete.")

    def set_pwm_frequency(self, freq_hz):
        """Sets the PWM frequency for all channels."""
        print(f"Setting PWM frequency to {freq_hz} Hz...")
        self._frequency = float(freq_hz)
        prescale = int(math.floor(25000000.0 / (4096.0 * self._frequency) - 0.5))
        prescale = max(3, min(prescale, 255))
        print(f"  Calculated Prescaler: {prescale} (0x{prescale:02X})")

        old_mode1 = self._read_reg(self._REG_MODE1)
        if old_mode1 is None: return False

        try:
            self._write_reg(self._REG_MODE1, (old_mode1 & ~self._MODE1_RESTART) | self._MODE1_SLEEP)
            time.sleep(0.005)
            self._write_reg(self._REG_PRE_SCALE, prescale)
            time.sleep(0.005)
            self._write_reg(self._REG_MODE1, old_mode1 & ~self._MODE1_SLEEP)
            time.sleep(0.005)
            print("  Frequency set complete.")
            return True
        except Exception as e:
            print(f"  Error during frequency setting: {e}")
            # Attempt recovery
            if old_mode1 is not None:
                try: self._write_reg(self._REG_MODE1, old_mode1 & ~self._MODE1_SLEEP)
                except Exception: pass
            return False

    def set_pwm(self, channel, on_tick, off_tick):
        """Sets the raw ON/OFF ticks for a channel (writes bytes individually)."""
        if not (0 <= channel < NUM_CHANNELS): return
        reg_on_l = self._REG_LED0_ON_L + 4 * channel
        on_tick = max(0, min(int(on_tick), 4095))
        off_tick = max(0, min(int(off_tick), 4095))
        try:
            self._write_reg(reg_on_l    , on_tick & 0xFF)
            self._write_reg(reg_on_l + 1, (on_tick >> 8) & 0x0F)
            self._write_reg(reg_on_l + 2, off_tick & 0xFF)
            self._write_reg(reg_on_l + 3, (off_tick >> 8) & 0x0F)
        except Exception as e:
            print(f"Error writing PWM data for channel {channel}: {e}")

    def set_servo_pulse(self, channel, pulse_width_us):
        """Sets servo position based on raw pulse width (microseconds)."""
        if self._frequency <= 0: return # Frequency must be set
        if not (0 <= channel < NUM_CHANNELS): return
        ticks = int(round((float(pulse_width_us) * self._frequency * 4096.0) / 1000000.0))
        self.set_pwm(channel, 0, ticks) # Start at tick 0

    # --- Public User Methods ---
    def set_servo_calibration(self, channel, min_pulse=DEFAULT_SERVO_MIN_PULSE, max_pulse=DEFAULT_SERVO_MAX_PULSE, min_angle=DEFAULT_SERVO_MIN_ANGLE, max_angle=DEFAULT_SERVO_MAX_ANGLE):
         """
         Sets the calibration parameters for a specific servo channel.

         Args:
             channel (int): The servo channel number (0-15).
             min_pulse (int): Pulse width (us) for min_angle.
             max_pulse (int): Pulse width (us) for max_angle.
             min_angle (float): Minimum angle (degrees) for this servo.
             max_angle (float): Maximum angle (degrees) for this servo.
         """
         if not (0 <= channel < NUM_CHANNELS):
             print(f"Error: Invalid channel {channel} for calibration.")
             return
         self._servo_calibration[channel] = {
             "min_pulse": min_pulse,
             "max_pulse": max_pulse,
             "min_angle": min_angle,
             "max_angle": max_angle,
         }
         print(f"Calibration set for channel {channel}: Pulse={min_pulse}-{max_pulse}us, Angle={min_angle}-{max_angle}deg")

    def set_servo_angle(self, channel, angle):
        """
        Moves a servo on a specific channel to the desired angle (degrees),
        using the stored calibration data for that channel.

        Args:
            channel (int): The servo channel number (0-15).
            angle (float): The target angle in degrees.
        """
        if not (0 <= channel < NUM_CHANNELS):
            print(f"Error: Invalid servo channel {channel}.")
            return
        if self._frequency <= 0:
            print("Error: PWM frequency not set. Call set_pwm_frequency() first.")
            return

        # Retrieve calibration for this channel
        cal = self._servo_calibration[channel]
        min_angle = cal["min_angle"]
        max_angle = cal["max_angle"]
        min_pulse = cal["min_pulse"]
        max_pulse = cal["max_pulse"]

        # Clamp angle to calibrated limits
        angle = max(min_angle, min(float(angle), max_angle))

        # Perform linear interpolation to map angle to pulse width
        pulse_range = float(max_pulse - min_pulse)
        angle_range = float(max_angle - min_angle)

        if angle_range == 0: # Avoid division by zero
            pulse_width = min_pulse + pulse_range / 2.0
        else:
            pulse_width = min_pulse + ((angle - min_angle) * pulse_range / angle_range)

        # Set the servo position using the calculated pulse width
        # print(f"Setting Ch {channel} to {angle}deg -> {int(pulse_width)}us pulse") # Debug
        self.set_servo_pulse(channel, int(pulse_width))

    def cleanup(self, channel=None):
        """Disables PWM on a specific channel or all channels."""
        if channel is not None:
             if 0 <= channel < NUM_CHANNELS:
                 print(f"Disabling PWM on channel {channel}...")
                 self.set_pwm(channel, 0, 0) # Set duty cycle to 0%
             else:
                 print(f"Warning: Invalid channel {channel} specified for cleanup.")
        else:
            print("Disabling PWM on all channels...")
            for ch in range(NUM_CHANNELS):
                 self.set_pwm(ch, 0, 0)

# --- End of PCA9685_PinPong Class ---


# --- Example Usage Block ---
# This block only runs when the script is executed directly (python pca9685_driver.py)
# It will NOT run when this file is imported as a module into another script.
if __name__ == "__main__":

    print("--- PCA9685 PinPong Library Example ---")

    # --- Configuration for Example ---
    SERVO_CHANNEL_TO_TEST = 0 # Which channel is the servo connected to?

    # --- Servo Calibration Parameters ---
    # !!! ADJUST THESE FOR YOUR SPECIFIC SERVO !!!
    # You might need different values for different servos
    MY_SERVO_MIN_PULSE = 600  # Example: Pulse width (us) for min angle
    MY_SERVO_MAX_PULSE = 2400 # Example: Pulse width (us) for max angle
    MY_SERVO_MIN_DEG = 0      # Example: Min angle servo can physically reach
    MY_SERVO_MAX_DEG = 180    # Example: Max angle servo can physically reach

    pca9685_device = None # Define outside try block for cleanup

    try:
        # --- Setup ---
        # Initialize PinPong Board and I2C (Required before creating PCA9685 object)
        # Ensure UNIHIKER board is selected if running on UNIHIKER
        try:
             Board("UNIHIKER").begin()
             i2c_bus = I2C()
             print("PinPong Board and I2C Initialized.")
        except Exception as board_err:
             print(f"Error initializing PinPong Board/I2C: {board_err}")
             print("Please ensure the script runs in the correct environment.")
             exit()

        # Instantiate the PCA9685 device object
        # This automatically detects, wakes, sets default MODE2, and sets default frequency (50Hz)
        pca9685_device = PCA9685_PinPong(i2c_bus) # Uses default address 0x40, freq 50Hz

        # --- Optional: Set Custom Calibration ---
        # If your servo doesn't use the defaults (500-2500us, 0-180deg), set them here:
        pca9685_device.set_servo_calibration(
            SERVO_CHANNEL_TO_TEST,
            min_pulse=MY_SERVO_MIN_PULSE,
            max_pulse=MY_SERVO_MAX_PULSE,
            min_angle=MY_SERVO_MIN_DEG,
            max_angle=MY_SERVO_MAX_DEG
        )
        # You can set calibration for other channels too if needed

        # --- Optional: Set different frequency if needed ---
        # pca9685_device.set_pwm_frequency(60) # Example: Set to 60Hz

        # --- Control the Servo ---
        print("\nStarting servo movement using class methods...")
        print("Make sure the separate servo power (V+) is connected and ON.")
        print("Press Ctrl+C to stop.")

        # Move to center angle
        center_angle = (MY_SERVO_MAX_DEG + MY_SERVO_MIN_DEG) / 2
        print(f"Moving servo {SERVO_CHANNEL_TO_TEST} to center ({center_angle} degrees)...")
        pca9685_device.set_servo_angle(SERVO_CHANNEL_TO_TEST, center_angle)
        time.sleep(2)

        # Move to min angle
        print(f"Moving servo {SERVO_CHANNEL_TO_TEST} to min ({MY_SERVO_MIN_DEG} degrees)...")
        pca9685_device.set_servo_angle(SERVO_CHANNEL_TO_TEST, MY_SERVO_MIN_DEG)
        time.sleep(2)

        # Move to max angle
        print(f"Moving servo {SERVO_CHANNEL_TO_TEST} to max ({MY_SERVO_MAX_DEG} degrees)...")
        pca9685_device.set_servo_angle(SERVO_CHANNEL_TO_TEST, MY_SERVO_MAX_DEG)
        time.sleep(2)

        # Move back to center
        print(f"Moving servo {SERVO_CHANNEL_TO_TEST} back to center ({center_angle} degrees)...")
        pca9685_device.set_servo_angle(SERVO_CHANNEL_TO_TEST, center_angle)
        time.sleep(2)

        print("\nMovement sequence complete.")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Stopping servo and exiting.")
    except RuntimeError as e:
        print(f"Runtime Error: {e}") # Catch errors like device not found
    except Exception as e:
        print(f"\nAn unexpected error occurred in main example: {e}")
    finally:
        # --- Cleanup ---
        if pca9685_device: # Check if object was successfully created
            pca9685_device.cleanup(SERVO_CHANNEL_TO_TEST) # Turn off the test servo channel
        print("Program finished.")

