// To upload the code to each of the target microcontrollers, modify the target microcontroller name on line 26 and
// use platformio to compile and upload the project. When uploading the code, make sure only one
// Teensy 4.1 is connected to the host-PC at the same time.

// The microcontrollers assembled and configured as part of this project are designed to work with the Python
// interfaces available from the sl-experiment project: https://github.com/Sun-Lab-NBB/sl-experiment.
// See https://github.com/Sun-Lab-NBB/sl-micro-controllers for more details on assembling the hardware and
// installing the project.
// API documentation: https://sl-micro-controllers-api-docs.netlify.app/.
// Author: Ivan Kondratyev (Inkaros).

// Dependencies
#include <Arduino.h>
#include <communication.h>
#include <kernel.h>
#include <module.h>

// Initializes the serial communication class.
Communication axmc_communication(Serial);  // NOLINT(*-interfaces-global-init)

// Defines the target microcontroller. Our VR system currently has 3 valid targets: ACTOR, SENSOR and ENCODER. 
// WJ note: only ACTOR is currently used.

// Resolves microcontroller-specific module configuration and layout
// Use the same controller as for valve and lick

#include "valve_module.h"
#include "lick_module.h"
#include "servo_module.h"
// #include "analog_module.h"
// #include "sine_module.h"

constexpr uint8_t kControllerID = 111;
constexpr uint32_t kKeepAliveInterval = 1000;  // 1 second == 1000 ms

ValveModule<16, true> left_valve(1, 1, axmc_communication);
ValveModule<9,  true> right_valve(1, 2, axmc_communication);

LickModule<22> left_lick_sensor(2, 1, axmc_communication);
LickModule<23>  right_lick_sensor(2, 2, axmc_communication);

// Moves the lick spout in and out. Pin 2 carries the servo signal; power the servo from a separate 5-6 V supply.
ServoModule<2> spout_servo(5, 1, axmc_communication);

// AnalogModule<14> analog_signal(3, 1, axmc_communication);

// SineModule<0x60> sine_wave_test(4, 1, axmc_communication);  // 0x60: MCP4725 default address (A0 tied low).

Module* modules[] = {
    &left_valve,
    // &right_valve,
    &left_lick_sensor,
    &spout_servo
    // &right_lick_sensor,
    // &analog_signal,
    // &sine_wave_test
};

// Instantiates the Kernel class using the assets instantiated above.
Kernel axmc_kernel(kControllerID, axmc_communication, modules, kKeepAliveInterval);

void setup()
{
    Serial.begin(115200);  // The baudrate is ignored for teensy boards.

    // Sets ADC resolution to 12 bits. Teensy boards can support up to 16 bits, but 12 often produces cleaner readouts.
    analogReadResolution(12);

    axmc_kernel.Setup();  // Carries out the rest of the setup depending on the module configuration.
}

void loop()
{
    axmc_kernel.RuntimeCycle();
}
