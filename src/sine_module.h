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
 * @section sine_mod_gaps Blanking gaps
 * When the gap_interval parameter is not 0, the module periodically holds its output at the wave's zero level for a
 * whole number of periods instead of emitting the sine. These blanking gaps make the emitted wave aperiodic, so
 * that a separate recording of the wave can be aligned to this module's log without the period ambiguity a
 * featureless sine carries. Each gap runs one period longer than the gap before it, which allows every gap emitted
 * during a runtime to be identified from its own duration alone, without observing the start of the wave.
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

            // Runs the I2C bus in fast mode. Each DAC write blocks the controller's runtime cycle for the duration
            // of the transfer, so the ~270 us a 3-byte write costs at the 100 kHz default consumes a large share of
            // the per-sample budget at higher wave frequencies and delays every other module managed by the Kernel.
            Wire.setClock(400000);

            ResetWaveState();
            WriteDAC(0);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.sample_count = 40;    // Steps per wave period. Set cycle_delay to period / this
                                                       // value.
            _custom_parameters.amplitude    = 2047;  // Peak deviation from offset, in 12-bit DAC units.
            _custom_parameters.offset       = 2048;  // The DAC value that corresponds to the sine wave's zero
                                                       // level.
            _custom_parameters.gap_interval = 0;     // Periods between blanking gaps. 0 disables gap insertion.

            return true;
        }

        ~SineModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint16_t sample_count = 40;    ///< The number of lookup-table samples per wave period.
                uint16_t amplitude    = 2047;  ///< The peak deviation from offset, in 12-bit DAC units.
                uint16_t offset       = 2048;  ///< The DAC value that corresponds to the sine wave's zero level.
                uint16_t gap_interval = 0;     ///< The number of wave periods between blanking gaps. 0 disables
                                               ///< gap insertion.
        } PACKED_STRUCT _custom_parameters;

        /// Stores the index of the next lookup-table sample to output, wrapping back to 0 after sample_count.
        uint16_t _phase_index = 0;

        /// Stores the number of complete wave periods emitted since the end of the last blanking gap.
        uint16_t _periods_since_gap = 0;

        /// Stores the number of periods left in the active blanking gap. Holds 0 while the wave runs normally.
        uint16_t _gap_periods_left = 0;

        /// Stores the length, in periods, of the next blanking gap. Grows by one after each gap, so that every gap
        /// emitted during a runtime is uniquely identifiable from its own duration.
        uint16_t _next_gap_length = 1;

        /// Returns the phase index and the blanking-gap schedule to the state they hold at the start of a wave.
        void ResetWaveState()
        {
            _phase_index       = 0;
            _periods_since_gap = 0;
            _gap_periods_left  = 0;
            _next_gap_length   = 1;
        }

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

        /// Advances the blanking-gap schedule by one completed wave period.
        void AdvanceGapSchedule()
        {
            if (_custom_parameters.gap_interval == 0) return;  // Gap insertion is disabled.

            // Counts down an active gap before evaluating whether a new gap is due, so that the two never overlap.
            if (_gap_periods_left > 0)
            {
                _gap_periods_left--;
                return;
            }

            _periods_since_gap++;
            if (_periods_since_gap >= _custom_parameters.gap_interval)
            {
                _periods_since_gap = 0;
                _gap_periods_left  = _next_gap_length;

                // Each gap runs one period longer than the gap before it. This makes the sequence of gaps aperiodic
                // and allows the PC to identify any single gap from its duration, without having to observe the
                // start of the wave.
                _next_gap_length++;
            }
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

            // During a blanking gap, holds the output at the wave's zero level instead of emitting the sine. The
            // phase index still advances below, so the wave resumes a gap exactly where it would have been had the
            // gap never happened, which leaves the PC-side phase model continuous across the gap. Holding at offset
            // rather than at 0 keeps a gap distinguishable from a stopped wave or a disconnected output.
            const uint16_t output_value = _gap_periods_left > 0 ? _custom_parameters.offset : clamped_value;

            WriteDAC(output_value);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kSampleSet), output_value);

            // Advances to the next sample, wrapping back to 0 after completing a full period.
            _phase_index++;
            if (_phase_index >= sample_count)
            {
                _phase_index = 0;
                AdvanceGapSchedule();
            }

            CompleteCommand();
        }

        /// Stops the wave output and resets the phase index and the gap schedule, so the next kNextSample command
        /// restarts both the period and the blanking-gap sequence.
        void Stop()
        {
            WriteDAC(0);
            ResetWaveState();
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kStopped));
            CompleteCommand();
        }
};

#endif  //AXMC_SINE_MODULE_H
