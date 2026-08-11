/**
 * @file
 * @brief The header-only file for the LickModule class. This class allows interfacing with a custom conductive lick
 * sensor used to monitor the licking behavior of animals.
 *
 * This class was specifically designed to monitor the licking behavior of laboratory mice during experiments. This
 * class will likely work for other purposes that benefit from detecting wet or dry contacts, such as touch or lick
 * sensors.
 *
 * @section lck_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_LICK_MODULE_H
#define AXMC_LICK_MODULE_H

#include <cstdint>
#include <Arduino.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Monitors the state of a custom conductive lick sensor for significant state changes and notifies the PC when
 * such changes occur.
 *
 * This module is designed to work with a custom lick sensor. The sensor works by directly injecting a small amount of
 * current through one contact surface and monitoring the pin wired to the second contact surface. When the animal,
 * such as a laboratory mouse, licks the sensor while making contact with the current injector surface, the sensor
 * detects a positive change in voltage across the sensor. The detection threshold can be configured to distinguish
 * between dry and wet touch, which is used to separate limb contacts from tongue contacts.
 *
 * @note This class was calibrated to work for and tested on C57BL6J Wild-type and transgenic mice.
 *
 * @tparam kPin the analog pin whose state will be monitored to detect licks.
 */
template <const uint8_t kPin>
class LickModule final : public Module
{
        // Ensures that the pin does not interfere with LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for LickModule instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kChanged = 51,  /// The signal received by the monitored pin has significantly changed since the last check.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Checks the state of the input pin, and if necessary informs the PC of any changes.
        };

        /// Initializes the TTLModule class by subclassing the base Module class.
        LickModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
            Module(module_type, module_id, communication)
        {}

        /// Overwrites the custom_parameters structure memory with the data extracted from the Communication
        /// reception buffer.
        bool SetCustomParameters() override
        {
            // Extracts the received parameters into the _custom_parameters structure of the class. If extraction fails,
            // returns false. This instructs the Kernel to execute the necessary steps to send an error message to the
            // PC.
            return ExtractParameters(_custom_parameters);
        }

        /// Executes the currently active command.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(get_active_command()))
            {
                // CheckState
                case kModuleCommands::kCheckState: CheckState(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Sets pin to Input mode.
            pinModeFast(kPin, INPUT_PULLDOWN);

            // Resets the custom_parameters structure fields to their default values. Assumes 12-bit ADC resolution.
            _custom_parameters.signal_threshold  = 200;  // Ideally should be just high enough to filter out noise
            _custom_parameters.delta_threshold   = 180;  // Ideally should be at least half of the minimal threshold
            _custom_parameters.average_pool_size = 0;    // Better to have at 0 because Teensy already does this

            // Notifies the PC about the initial sensor state. Primarily, this is needed to support data source
            // time-alignment during post-processing.
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kChanged), static_cast<uint16_t>(0));

            return true;
        }

        ~LickModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint16_t signal_threshold = 200;  ///< The lower boundary for signals to be reported to PC.
                uint16_t delta_threshold  = 180;  ///< The minimum difference between checks to be reported to PC.
                uint8_t average_pool_size = 0;    ///< The number of readouts to average into pin state value.
        } PACKED_STRUCT _custom_parameters;

        /// Checks the signal received by the input pin and, if necessary, reports it to the PC.
        void CheckState()
        {
            // Stores the previous readout of the analog pin. This is used to limit the number of messages sent to the
            // PC by only reporting significant changes of the pin state (signal). The level that constitutes
            // significant change can be adjusted through the custom_parameters structure.
            static uint16_t previous_readout = 0;

            // Tracks whether the previous message sent to the PC included a zero signal value.
            static bool previous_zero = true;  // A zero-message is sent at class initialization.

            // Evaluates the state of the pin. Averages the requested number of readouts to produce the final
            // analog signal value. Note, since we statically configure the controller to use 10-14 bit ADC resolution,
            // this value should not use the full range of the 16-bit uint variable.
            const uint16_t signal = AnalogRead<kPin>(_custom_parameters.average_pool_size);

            // Calculates the absolute difference between the current signal and the previous readout. This is used
            // to ensure only significant signal changes are reported to the PC. Note, although we are casting both to
            // int32 to support the delta calculation, the resultant delta will always be within the uint_16 range.
            // Therefore, it is fine to cast it back to uint16 to avoid unnecessary future casting in the 'if'
            // statements.
            const auto delta =
                static_cast<uint16_t>(abs(static_cast<int32_t>(signal) - static_cast<int32_t>(previous_readout)));

            // Prevents reporting signals that are not significantly different from the previous readout value.
            if (delta <= _custom_parameters.delta_threshold)
            {
                CompleteCommand();
                return;
            }

            previous_readout = signal;  // Overwrites the previous readout with the current signal

            // If the signal is above the threshold, sends it to the PC
            if (signal >= _custom_parameters.signal_threshold)
            {
                // Sends the detected signal to the PC.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kChanged), signal);
                previous_zero = false;
            }

            // If the signal is below the threshold, pulls it to 0 and notifies the PC
            else
            {
                if (!previous_zero)
                {
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kChanged), static_cast<uint16_t>(0));
                    previous_zero = true;
                }
            }

            // Completes command execution
            CompleteCommand();
        }
};

#endif  //AXMC_LICK_MODULE_H
