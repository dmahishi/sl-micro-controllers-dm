/**
 * @file
 * @brief The header-only file for the SineModule class. This class outputs a sine wave through an MCP4725 I2C DAC,
 * stepping through one lookup-table sample per activation.
 *
 * @section sine_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - Wire.h for the I2C bus used to communicate with the MCP4725 DAC.
 * - cmath for the sinf() function used to compute lookup table samples.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 */

#ifndef AXMC_SINE_MODULE_H
#define AXMC_SINE_MODULE_H

#include <cmath>
#include <Arduino.h>
#include <Wire.h>
#include <module.h>

/**
 * @brief Outputs a sine wave through an MCP4725 I2C DAC, advancing by one lookup-table sample each time the
 * kNextSample command is activated.
 *
 * This module does not time its own wave period. Instead, the PC drives the wave by repeatedly queueing the
 * kNextSample command as a recurrent command with a cycle_delay of period_duration / sample_count microseconds.
 * Each activation advances the module's internal phase index by one step and writes the corresponding sine value
 * to the DAC over I2C, so the resulting wave's period is entirely determined by the requested cycle_delay and the
 * sample_count parameter, not by anything tracked inside the module. The DAC outputs a true analog voltage
 * directly, so unlike a PWM-based approach, no external RC filter is required. This module uses the default I2C
 * bus (pins 18/SDA and 19/SCL on Teensy 4.0).
 *
 * @tparam kAddress the 7-bit I2C address of the target MCP4725 DAC (0x60 or 0x61, depending on the breakout's A0
 * pin).
 */
template <const uint8_t kAddress>
class SineModule final : public Module
{
        // Ensures that the configured address is a valid, non-reserved 7-bit I2C address.
        static_assert(
            kAddress >= 0x08 && kAddress <= 0x77,
            "Configured I2C address falls within the reserved address range. Select a different address for "
            "SineModule instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kSampleSet = 51,  ///< The DAC output has been updated to the current sample's sine value.
            kStopped   = 52,  ///< The wave output has been stopped and the phase index reset.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kNextSample = 1,  ///< Advances the phase index by one step and updates the DAC output.
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
            Wire.begin();

            _phase_index = 0;
            WriteDAC(0);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.sample_count = 200;   // Steps per wave period. Set cycle_delay to period / this
                                                       // value.
            _custom_parameters.amplitude    = 2047;  // Peak deviation from offset, in 12-bit DAC units.
            _custom_parameters.offset       = 2048;  // The DAC value that corresponds to the sine wave's zero
                                                       // level.

            return true;
        }

        ~SineModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint16_t sample_count = 200;   ///< The number of lookup-table samples per wave period.
                uint16_t amplitude    = 2047;  ///< The peak deviation from offset, in 12-bit DAC units.
                uint16_t offset       = 2048;  ///< The DAC value that corresponds to the sine wave's zero level.
        } PACKED_STRUCT _custom_parameters;

        /// Stores the index of the next lookup-table sample to output, wrapping back to 0 after sample_count.
        uint16_t _phase_index = 0;

        /// Writes a 12-bit value to the MCP4725 DAC using its fast-mode write command, which updates the volatile
        /// DAC register directly without touching the chip's EEPROM.
        void WriteDAC(const uint16_t value)
        {
            const uint16_t clamped_value = value > 4095 ? 4095 : value;

            Wire.beginTransmission(kAddress);
            Wire.write(static_cast<uint8_t>(clamped_value >> 8));    // Upper 4 bits, top nibble is 0 (fast mode).
            Wire.write(static_cast<uint8_t>(clamped_value & 0xFF));  // Lower 8 bits.
            Wire.endTransmission();
        }

        /// Computes and outputs the sine value for the current phase index, then advances to the next index.
        void NextSample()
        {
            // Guards against a division by zero if sample_count was set to 0 by the PC, since sample_count is a
            // runtime parameter not otherwise validated by ExtractParameters().
            const uint16_t sample_count = _custom_parameters.sample_count > 0 ? _custom_parameters.sample_count : 1;

            const auto angle =
                static_cast<float>(TWO_PI) * static_cast<float>(_phase_index) / static_cast<float>(sample_count);

            // Scales the [-1.0, 1.0] sine value by the requested amplitude and centers it on the requested offset.
            const auto dac_value = static_cast<int32_t>(
                static_cast<float>(_custom_parameters.offset) + sinf(angle) * static_cast<float>(_custom_parameters.amplitude)
            );

            // Clamps the computed value to the valid 12-bit DAC range, in case the requested amplitude and offset
            // combination would otherwise overflow or underflow it.
            const auto clamped_value = static_cast<uint16_t>(constrain(dac_value, 0, 4095));

            WriteDAC(clamped_value);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kSampleSet), clamped_value);

            // Advances to the next sample, wrapping back to 0 after completing a full period.
            _phase_index++;
            if (_phase_index >= sample_count) _phase_index = 0;

            CompleteCommand();
        }

        /// Stops the wave output and resets the phase index, so the next kNextSample command restarts the period.
        void Stop()
        {
            WriteDAC(0);
            _phase_index = 0;
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kStopped));
            CompleteCommand();
        }
};

#endif  //AXMC_SINE_MODULE_H
