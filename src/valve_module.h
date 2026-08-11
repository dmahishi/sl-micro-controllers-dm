/**
 * @file
 * @brief The header-only file for the ValveModule class. This class allows interfacing with a solenoid valve to
 * controllably dispense precise amounts of fluid.
 *
 * @section vlv_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_VALVE_MODULE_H
#define AXMC_VALVE_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Sends digital signals to dispense precise amounts of fluid via the managed solenoid valve.
 *
 * This module is specifically designed to send digital signals that trigger Field-Effect-Transistor (FET) gated relay
 * hardware to deliver voltage that opens or closes the controlled valve. Depending on configuration, this module is
 * designed to work with both Normally Closed (NC) and Normally Open (NO) valves.
 *
 *
 * @note This class was calibrated to work with fluid valves that deliver microliter-precise amounts of fluid under
 * gravitational driving force. The current class implementation may not work as intended for other use cases.
 * Additionally, the class is designed for dispensing predetermined amounts of fluid and not for continuous flow rate
 * control, which would require a PWM-based approach similar to the one used by the BreakModule class.
 *
 * @tparam kPin the digital pin connected to the valve's FET-gated relay.
 * @tparam kNormallyClosed determines whether the managed valve is opened or closed when unpowered. This is
 * used to adjust the class behavior so that toggle OFF always means the valve is closed and toggle ON means the
 * valve is open.
 * @tparam kStartClosed determines the initial state of the valve during class initialization. This works
 * together with kNormallyClosed parameter to deliver the desired initial voltage level for the valve to either be
 * opened or closed after hardware initialization.
 */
template <
    const uint8_t kValvePin,
    const bool kNormallyClosed,
    const bool kStartClosed = true>
    
class ValveModule final : public Module
{
        // Ensures that the valve pin does not interfere with the LED pin.
        static_assert(
            kValvePin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different valve pin for the ValveModule "
            "instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kOpen                     = 51,  ///< The valve is currently open.
            kClosed                   = 52,  ///< The valve is currently closed.
            kCalibrated               = 53,  ///< The valve calibration cycle has been completed.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kSendPulse = 1,  ///< Deliver a precise amount of fluid by cycling valve open and close states.
            kToggleOn  = 2,  ///< Sets the valve to be permanently open.
            kToggleOff = 3,  ///< Sets the valve to be permanently closed.
            kCalibrate = 4,  ///< Repeatedly pulses the valve to map different pulse_durations to dispensed fluid volumes.
        };

        /// Initializes the class by subclassing the base Module class.
        ValveModule(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication
        ) :
            Module(module_type, module_id, communication)
        {}

        /// Overwrites the custom_parameters structure memory with the data extracted from the Communication
        /// reception buffer.
        bool SetCustomParameters() override
        {
            // Attempts to extract the received parameters
            return ExtractParameters(_custom_parameters);
        }

        /// Resolves and executes the currently active command.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(get_active_command()))
            {
                // Pulse
                case kModuleCommands::kSendPulse: Pulse(); return true;
                // Open
                case kModuleCommands::kToggleOn: Open(); return true;
                // Close
                case kModuleCommands::kToggleOff: Close(); return true;
                // Calibrate
                case kModuleCommands::kCalibrate: Calibrate(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Sets the valve state based on the configuration of the valve's FET gate and the desired initial
            // state.
            pinModeFast(kValvePin, OUTPUT);
            if (kStartClosed)
            {
                digitalWriteFast(kValvePin, kClose);  // Ensures the valve is closed.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kClosed));
            }
            else
            {
                digitalWriteFast(kValvePin, kOpen);  // Ensures the valve is open.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOpen));
            }

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pulse_duration    = 35000;  // ~ 5.0 uL of water in the current Sun lab system.
            _custom_parameters.calibration_count = 200;    // The valve is pulsed 500 times during calibration.

            return true;
        }

        ~ValveModule() override = default;

    private:
        /// Stores the instance's addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration    = 35000;   ///< The time, in microseconds, to keep the valve open.
                uint16_t calibration_count = 200;     ///< The number of times to pulse the valve during calibration.
        } PACKED_STRUCT _custom_parameters;

        /// Stores the digital signal that needs to be sent to the valve pin to open the valve.
        static constexpr bool kOpen = kNormallyClosed ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the digital signal that needs to be sent to the valve pin to close the valve.
        static constexpr bool kClose = kNormallyClosed ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the time, in microseconds, that must separate any two consecutive pulses during the valve
        /// calibration. The value for this attribute is hardcoded for the system's safety, as pulsing the
        /// valve too fast may generate undue stress in the calibrated hydraulic system.
        static constexpr uint32_t kCalibrationDelay = 300000;

        /// Cycles opening and closing the valve to deliver the precise amount of fluid.
        void Pulse()
        {
            switch (get_command_stage())
            {
                // Opens the valve
                case 1:
                    digitalWriteFast(kValvePin, kOpen);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOpen));

                    AdvanceCommandStage();
                    return;

                // Waits for the requested valve pulse duration of microseconds to pass.
                case 2:
                    if (!WaitForMicros(_custom_parameters.pulse_duration)) return;
                    AdvanceCommandStage();
                    return;

                // Closes the valve
                case 3:
                    digitalWriteFast(kValvePin, kClose);
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kClosed));
                    CompleteCommand();
                    return;

                default: AbortCommand();
            }
        }

        /// Opens the valve.
        void Open()
        {
            digitalWriteFast(kValvePin, kOpen);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kOpen));
            CompleteCommand();
        }

        /// Closes the valve.
        void Close()
        {
            digitalWriteFast(kValvePin, kClose);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kClosed));
            CompleteCommand();
        }

        /// Opens the valve for the requested pulse_duration microseconds and repeats the procedure for the
        /// calibration_count repetitions without blocking or (majorly) delaying.
        void Calibrate()
        {
            // Essentially runs the modified Pulse() command for the requested number of repetitions.
            for (uint16_t i = 0; i < _custom_parameters.calibration_count; ++i)
            {
                // Opens the valve
                digitalWriteFast(kValvePin, kOpen);

                // Blocks in-place until the pulse duration passes.
                delayMicroseconds(_custom_parameters.pulse_duration);

                // Closes the valve
                digitalWriteFast(kValvePin, kClose);

                // Blocks for kCalibrationDelay of microseconds to ensure the valve closes before initiating the next
                // cycle.
                delayMicroseconds(kCalibrationDelay);
            }

            // This command completes after running the requested number of cycles.
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kCalibrated));
            CompleteCommand();
        }

};

#endif  //AXMC_VALVE_MODULE_H
