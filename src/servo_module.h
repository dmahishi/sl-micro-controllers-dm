/**
 * @file
 * @brief The header-only file for the ServoModule class. This class allows moving the lick spout in front of the
 * animal and away from it with a hobby servo motor.
 *
 * @section srv_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - Servo.h for generating the servo control (PWM) signal.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 */

#ifndef AXMC_SERVO_MODULE_H
#define AXMC_SERVO_MODULE_H

#include <Arduino.h>
#include <Servo.h>
#include <module.h>

/**
 * @brief Moves a hobby servo between the 'in' (spout in front of the animal) and 'out' (spout retracted) angles.
 *
 * Hobby servos do not report their position, so the module reports the start of each movement and, after the
 * configured settle time, the arrival at the target angle. The PC timestamps both messages with the same clock as all
 * other logged data.
 *
 * @note The servo must be powered from a separate 5-6 V supply that shares ground with the Teensy. Powering the
 * servo from the Teensy's pins can brown out the board.
 *
 * @tparam kServoPin the PWM-capable digital pin connected to the servo's signal wire.
 */
template <const uint8_t kServoPin>
class ServoModule final : public Module
{
        // Ensures that the servo pin does not interfere with the LED pin.
        static_assert(
            kServoPin != LED_BUILTIN,
            "The LED-connected pin is reserved for LED manipulation. Select a different servo pin for the ServoModule "
            "instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kMovingIn  = 51,  ///< The servo started moving the spout in front of the animal.
            kMovingOut = 52,  ///< The servo started retracting the spout.
            kIn        = 53,  ///< The spout reached the 'in' position (after the settle time).
            kOut       = 54,  ///< The spout reached the 'out' position (after the settle time).
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kMoveIn  = 1,  ///< Moves the spout in front of the animal.
            kMoveOut = 2,  ///< Retracts the spout.
        };

        /// Initializes the class by subclassing the base Module class.
        ServoModule(
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
            return ExtractParameters(_custom_parameters);
        }

        /// Resolves and executes the currently active command.
        bool RunActiveCommand() override
        {
            switch (static_cast<kModuleCommands>(get_active_command()))
            {
                case kModuleCommands::kMoveIn: Move(_custom_parameters.in_angle, true); return true;
                case kModuleCommands::kMoveOut: Move(_custom_parameters.out_angle, false); return true;
                default: return false;
            }
        }

        /// Sets up module hardware parameters. The spout starts retracted.
        bool SetupModule() override
        {
            // Unlike other modules, keeps the parameters received from the PC instead of resetting them to defaults.
            // The kernel re-runs this method at the end of each session, and resetting the 'out' angle could swing
            // the spout to an uncalibrated position. The defaults in CustomRuntimeParameters apply until the PC first
            // sends the rig's angles.
            if (!_servo.attached()) _servo.attach(kServoPin);
            _current_angle = _custom_parameters.out_angle;
            _servo.write(_current_angle);
            SendData(static_cast<uint8_t>(kCustomStatusCodes::kOut));

            return true;
        }

        ~ServoModule() override = default;

    private:
        /// Stores the instance's addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                uint8_t in_angle     = 90;      ///< The servo angle, in degrees, that puts the spout in reach.
                uint8_t out_angle    = 0;       ///< The servo angle, in degrees, that retracts the spout.
                uint32_t step_delay  = 5000;    ///< The time, in microseconds, between 1-degree steps. 0 jumps.
                uint32_t settle_time = 200000;  ///< The time, in microseconds, to wait after the last step.
        } PACKED_STRUCT _custom_parameters;

        /// Generates the servo control signal.
        Servo _servo;

        /// Stores the angle, in degrees, the servo was last commanded to.
        uint8_t _current_angle = 0;

        /// Measures the time since the last 1-degree step.
        elapsedMicros _step_timer;

        /// Moves the servo to the target angle in 1-degree steps, then waits for the settle time before reporting the
        /// arrival. The movement does not block other modules, such as the lick sensor.
        void Move(const uint8_t target_angle, const bool moving_in)
        {
            switch (get_command_stage())
            {
                // Reports the start of the movement.
                case 1:
                    SendData(
                        static_cast<uint8_t>(
                            moving_in ? kCustomStatusCodes::kMovingIn : kCustomStatusCodes::kMovingOut
                        )
                    );
                    _step_timer = 0;
                    AdvanceCommandStage();
                    return;

                // Steps toward the target angle, one degree every step_delay microseconds.
                case 2:
                    if (_custom_parameters.step_delay == 0)
                    {
                        _current_angle = target_angle;
                        _servo.write(_current_angle);
                    }
                    else if (_step_timer >= _custom_parameters.step_delay && _current_angle != target_angle)
                    {
                        _current_angle += _current_angle < target_angle ? 1 : -1;
                        _servo.write(_current_angle);
                        _step_timer = 0;
                    }
                    if (_current_angle == target_angle) AdvanceCommandStage();
                    return;

                // Waits for the spout to settle and reports the arrival.
                case 3:
                    if (!WaitForMicros(_custom_parameters.settle_time)) return;
                    SendData(static_cast<uint8_t>(moving_in ? kCustomStatusCodes::kIn : kCustomStatusCodes::kOut));
                    CompleteCommand();
                    return;

                default: AbortCommand();
            }
        }
};

#endif  //AXMC_SERVO_MODULE_H
