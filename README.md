# PCA9685 Servo Driver Library for UNIHIKER (using PinPong)

## Overview

This Python library provides a class (`PCA9685_PinPong`) for controlling a PCA9685 16-channel PWM/Servo Driver board using the native `pinpong` library on a UNIHIKER M10 single-board computer.

It simplifies the process of setting servo angles by encapsulating the low-level I2C communication and register manipulation required by the PCA9685 chip, based on its datasheet.

This library was developed as an alternative to using generic Linux libraries like Adafruit-Blinka, leveraging the built-in `pinpong` environment of the UNIHIKER.

## Features

* Provides a Python class (`PCA9685_PinPong`) for easy integration.
* Uses the UNIHIKER's native `pinpong` library for I2C communication.
* Initializes the PCA9685 chip (detects, wakes, sets frequency).
* Allows setting PWM frequency (defaults to 50Hz for servos).
* Provides a simple `set_servo_angle(channel, angle)` method.
* Includes per-channel calibration for servo pulse width and angle range.
* Includes default calibration values (500-2500µs pulse, 0-180 degrees).
* Includes a `cleanup()` method to disable PWM output.

## Requirements

* **Hardware:**
    * UNIHIKER M10 board.
    * PCA9685 16-Channel PWM/Servo Driver board.
    * Servo motor(s).
    * Appropriate **separate power supply** for the servos (e.g., 5V or 6V, with sufficient current capacity) connected to the PCA9685 `V+` input.
    * Jumper wires or suitable connectors.
* **Software:**
    * UNIHIKER M10 operating system with Python 3 and the `pinpong` library installed (usually pre-installed).
    * The `pca9685_driver.py` file (containing the `PCA9685_PinPong` class).

## Installation / Setup

1.  Save the Python code containing the `PCA9685_PinPong` class definition into a file named `pca9685_driver.py` on your UNIHIKER M10.
2.  Ensure the `pinpong` library is functional in your UNIHIKER's Python environment.

## Hardware Connection

1.  **Power Off:** Ensure UNIHIKER and servo power supply are off before connecting.
2.  **I2C Connection:**
    * PCA9685 `SCL` <-> UNIHIKER Pin 5 (`SCL`)
    * PCA9685 `SDA` <-> UNIHIKER Pin 3 (`SDA`)
3.  **Logic Power:**
    * PCA9685 `VCC` <-> UNIHIKER Pin 1 or 17 (`3.3V`)
    * PCA9685 logic `GND` <-> UNIHIKER Pin 6 or other `GND`
4.  **Servo Power (CRITICAL):**
    * Connect your **separate** servo power supply positive (+) terminal to PCA9685 `V+`.
    * Connect your **separate** servo power supply negative (-) terminal (Ground) to PCA9685 power `GND`.
    * Ensure this ground is common with the UNIHIKER ground (the logic GND connection usually ensures this via the PCA9685 board).
5.  **Servo(s):**
    * Plug servo(s) into the desired channel headers (0-15) on the PCA9685.
    * **Verify orientation:** Match servo wires (Signal, V+, GND) to the header pins (`PWM`/`SIG`, `V+`, `GND`).

## How to Use (API)

1.  **Save the Library:** Ensure `pca9685_driver.py` is accessible.
2.  **Import:** In your main Python script, import the necessary components:
    ```python
    import time
    from pinpong.board import Board, I2C
    from pca9685_driver import PCA9685_PinPong
    ```
3.  **Initialize PinPong:** Set up the board and I2C bus first:
    ```python
    try:
        Board("UNIHIKER").begin()
        i2c_bus = I2C()
        print("PinPong Board and I2C Initialized.")
    except Exception as board_err:
        print(f"Error initializing PinPong Board/I2C: {board_err}")
        exit()
    ```
4.  **Instantiate the Driver:** Create an object of the class. This automatically detects the chip, wakes it, and sets the default frequency (50Hz).
    ```python
    try:
        # Use default address 0x40 and default freq 50Hz
        pca_device = PCA9685_PinPong(i2c_bus)
        # Or specify address and frequency:
        # pca_device = PCA9685_PinPong(i2c_bus, address=0x41, freq=60)
    except RuntimeError as e:
        print(f"Error creating PCA9685 device: {e}")
        exit()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        exit()
    ```
5.  **Set Servo Calibration (Optional but Recommended):** If your servo's range differs from the default (500-2500µs pulse for 0-180 degrees), calibrate it:
    ```python
    # Example calibration for servo on channel 0
    pca_device.set_servo_calibration(
        channel=0,
        min_pulse=600,    # Pulse width for min angle (us)
        max_pulse=2400,   # Pulse width for max angle (us)
        min_angle=0,      # Min angle servo reaches (degrees)
        max_angle=180     # Max angle servo reaches (degrees)
    )
    ```
6.  **Set Servo Angle:** Use the primary method to move the servo:
    ```python
    channel = 0
    target_angle = 90.0 # degrees
    pca_device.set_servo_angle(channel, target_angle)
    time.sleep(1) # Allow time for servo to move
    ```
7.  **Cleanup:** When finished, disable PWM on the servo channel(s) or all channels:
    ```python
    # Disable a specific channel
    pca_device.cleanup(channel=0)
    # Or disable all channels
    # pca_device.cleanup()
    ```

## Example Script (`main_servo_control.py`) Overview

The accompanying example script (`main_servo_control.py`, found in the `servo_control_example_script` document) demonstrates the following:

* Imports the `PCA9685_PinPong` class from `pca9685_driver.py`.
* Initializes the `pinpong` board and I2C bus.
* Creates an instance of the `PCA9685_PinPong` driver.
* Sets custom calibration values for two servos connected to channel 0 and channel 15.
* Enters a loop that calls `pca_device.set_servo_angle()` for both channels, moving them together through a sequence of angles (center, min, max, center).
* Includes `try...except...finally` for error handling and ensures `cleanup()` is called to disable the servos upon exit.

This example provides a practical demonstration of how to use the library class in a separate application file.

## Troubleshooting

* **Servo Not Moving:**
    * Verify the **separate V+ servo power supply** (voltage, current capacity, connections, is it ON?). This is the most common issue.
    * Double-check **servo wiring orientation** on the PCA9685 channel header (Signal, V+, GND).
    * Ensure the correct **channel number** is used in the code.
    * Verify the **servo calibration** (`min_pulse`, `max_pulse`) matches your specific servo. Try different ranges if unsure.
    * Run `sudo i2cdetect -y 4` to confirm the PCA9685 is detected (usually at `0x40`).
    * Check the script's console output for any error messages.
    * Try a different servo or a different PCA9685 channel to rule out hardware faults.
* **ImportError:** Ensure `pca9685_driver.py` is saved in the same directory as your main script, or in a location included in Python's path. Ensure the `pinpong` library is correctly installed and working.

