/**
 * @file
 * @brief The header-only file for the SineModule class. This class outputs a PWM-approximated sine wave on a single
 * pin, stepping through one lookup-table sample per activation.
 *
 * @section sine_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - cmath for the sinf() function used to compute lookup table samples.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 */

#ifndef AXMC_SINE_MODULE_H
#define AXMC_SINE_MODULE_H

#include <cmath>
#include <Arduino.h>
#include <module.h>

/**
 * @brief Outputs a PWM-approximated sine wave on the managed pin, advancing by one lookup-table sample each time the
 * kNextSample command is activated.
 *
 * This module does not time its own wave period. Instead, the PC drives the wave by repeatedly queueing the
 * kNextSample command as a recurrent command with a cycle_delay of period_duration / sample_count microseconds.
 * Each activation advances the module's internal phase index by one step and updates the PWM duty cycle to the
 * corresponding sine value, so the resulting wave's period is entirely determined by the requested cycle_delay
 * and the sample_count parameter, not by anything tracked inside the module. The output pin still carries a PWM
 * carrier wave, so an external RC low-pass filter is required to recover a smooth analog sine voltage from the
 * pin.
 *
 * @tparam kPin the PWM-capable pin used to output the wave.
 */
template <const uint8_t kPin>
class SineModule final : public Module
{
        // Ensures that the pin does not interfere with the LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for SineModule instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kSampleSet = 51,  ///< The PWM duty cycle has been updated to the current sample's sine value.
            kStopped   = 52,  ///< The wave output has been stopped and the phase index reset.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kNextSample = 1,  ///< Advances the phase index by one step and updates the PWM duty cycle.
            kStop       = 2,  ///< Stops the wave output and resets the phase index to 0.
        };

        /// Initializes the SineModule class by subclassing the base Module class.
        SineModule(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
            Module(module_type, module_id, communication)
        {}

        /// Overwrites the custom_parameters structure memory with the data extracted from the Communication
        /// reception buffer.
        bool SetCustomParameters() override
        {
            return ExtractParameters(_custom_parameters);
        }

        /// Executes the currently active command.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(get_active_command()))
            {
                // NextSample
                case kModuleCommands::kNextSample: NextSample(); return true;
                // Stop
                case kModuleCommands::kStop: Stop(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Configures the pin for PWM output and sets the carrier frequency well above the expected sample
            // update rate, so an external RC filter can smooth the output into an analog sine wave.
            pinMode(kPin, OUTPUT);
            analogWriteFrequency(kPin, kPwmFrequency);
            analogWrite(kPin, 0);

            _phase_index = 0;

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.sample_count = 200;  // Steps per wave period. Set cycle_delay to period / this value.
            _custom_parameters.amplitude    = 127;  // Peak deviation from offset, in 8-bit PWM duty units.
            _custom_parameters.offset       = 128;  // The duty cycle that corresponds to the sine wave's zero level.

            return true;
        }

        ~SineModule() override = default;

    private:
        /// Stores the PWM carrier frequency, in Hz, used to output the wave. This has to be well above the sample
        /// update rate, so the external RC filter sees a fast carrier riding on top of the slow, intended waveform.
        static constexpr uint32_t kPwmFrequency = 5000;

        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint16_t sample_count = 200;  ///< The number of lookup-table samples per wave period.
                uint8_t amplitude     = 127;  ///< The peak deviation from offset, in 8-bit PWM duty units.
                uint8_t offset        = 128;  ///< The duty cycle that corresponds to the sine wave's zero level.
        } PACKED_STRUCT _custom_parameters;

        /// Stores the index of the next lookup-table sample to output, wrapping back to 0 after sample_count.
        uint16_t _phase_index = 0;

        /// Computes and outputs the sine value for the current phase index, then advances to the next index.
        void NextSample()
        {
            // Guards against a division by zero if sample_count was set to 0 by the PC, since sample_count is a
            // runtime parameter not otherwise validated by ExtractParameters().
            const uint16_t sample_count = _custom_parameters.sample_count > 0 ? _custom_parameters.sample_count : 1;

            const auto angle =
                static_cast<float>(TWO_PI) * static_cast<float>(_phase_index) / static_cast<float>(sample_count);

            // Scales the [-1.0, 1.0] sine value by the requested amplitude and centers it on the requested offset.
            const auto duty = static_cast<int16_t>(
                static_cast<float>(_custom_parameters.offset) + sinf(angle) * static_cast<float>(_custom_parameters.amplitude)
            );

            // Clamps the computed duty cycle to the valid 8-bit PWM range, in case the requested amplitude and offset
            // combination would otherwise overflow or underflow it.
            const auto clamped_duty = static_cast<uint8_t>(constrain(duty, 0, 255));

            analogWrite(kPin, clamped_duty);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kSampleSet), clamped_duty);

            // Advances to the next sample, wrapping back to 0 after completing a full period.
            _phase_index++;
            if (_phase_index >= sample_count) _phase_index = 0;

            CompleteCommand();
        }

        /// Stops the wave output and resets the phase index, so the next kNextSample command restarts the period.
        void Stop()
        {
            analogWrite(kPin, 0);
            _phase_index = 0;
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kStopped));
            CompleteCommand();
        }
};

#endif  //AXMC_SINE_MODULE_H
