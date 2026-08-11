/**
 * This class is designed to interface with an analog input pin on the microcontroller. It reads the analog
 * signal from the specified pin, checks if the signal exceeds a defined threshold (default 0), and reports
 * the status to the PC.    -- WJ
 */

 
#ifndef ANALOG_MODULE_WJ
#define ANALOG_MODULE_WJ

#include <cstdint>
#include <Arduino.h>
#include <digitalWriteFast.h>
#include <module.h>

template <const uint8_t kPin>
class AnalogModule final : public Module 
{
        // Ensures that the pin does not interfere with LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for AnalogModule instance."
        );

    public:

        enum class kCustomStatusCodes : uint8_t
        {
            kNonZero = 51,  /// The signal received by the monitored pin is above threshold (zero).
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Checks the state of the input pin, and if necessary informs the PC of any changes.
        };

        /// Initializes the AnalogModule class by subclassing the base Module class.
        AnalogModule(
            const uint8_t module_type, const uint8_t module_id, Communication& communication) :
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
            _custom_parameters.signal_threshold  = 0;  // Set to zero so that any photometry signal can be detected. Change this to filter out noise
            _custom_parameters.average_pool_size = 0;    // Better to have at 0 because Teensy already does this

            // Notifies the PC about the initial analog state input. Primarily, this is needed to support data source
            // time-alignment during post-processing.
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kNonZero), static_cast<uint16_t>(0));

            return true;
        }

        ~AnalogModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint16_t signal_threshold = 0;  ///< The lower boundary for signals to be reported to PC.
                uint8_t average_pool_size = 0;    ///< The number of readouts to average into pin state value.
        } PACKED_STRUCT _custom_parameters;

        /// Checks the signal received by the input pin and, if necessary, reports it to the PC.
        void CheckState()
        {
            // Evaluates the state of the pin. Averages the requested number of readouts to produce the final
            // analog signal value. Note, since we statically configure the controller to use 10-14 bit ADC resolution,
            // this value should not use the full range of the 16-bit uint variable.
            const uint16_t signal = AnalogRead<kPin>(_custom_parameters.average_pool_size);

            // Prevents reporting signals that are below threshold (default is zero).
            if (signal <= _custom_parameters.signal_threshold)
            {
                CompleteCommand();
                return;
            }

            // If the signal is above the threshold, sends it to the PC
            if (signal >= _custom_parameters.signal_threshold)
            {
                // Sends the detected signal to the PC.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kNonZero), signal);
            }

            // Completes command execution
            CompleteCommand();
        }
};
#endif  //ANALOG_MODULE_WJ
