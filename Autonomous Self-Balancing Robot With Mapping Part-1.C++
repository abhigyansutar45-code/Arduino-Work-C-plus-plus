/*
 * ================================================================
 *  AUTONOMOUS SELF-BALANCING ROBOT — ASBR
 * ================================================================
 *
 *  Target:
 *      Arduino Uno
 *
 *  Language:
 *      C++ / Arduino-compatible C++
 *
 *  PART 1:
 *      - Core configuration
 *      - Robot state
 *      - PID controller
 *      - IMU abstraction
 *      - Encoder abstraction
 *      - Motor abstraction
 *      - Ultrasonic distance sensors
 *      - Battery monitor
 *      - Safety manager
 *      - Complementary sensor fusion
 *
 *  IMPORTANT:
 *      This firmware is designed as an educational robotics
 *      foundation. Pin assignments and sensor drivers must be
 *      matched to the actual hardware before connecting motors.
 *
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/*
 * ================================================================
 * SECTION 1 — PIN CONFIGURATION
 * ================================================================
 */

namespace Pins
{
    // Motor driver pins
    constexpr uint8_t LEFT_MOTOR_PWM  = 5;
    constexpr uint8_t LEFT_MOTOR_DIR  = 4;

    constexpr uint8_t RIGHT_MOTOR_PWM = 6;
    constexpr uint8_t RIGHT_MOTOR_DIR = 7;

    // Encoder pins
    constexpr uint8_t LEFT_ENCODER_A  = 2;
    constexpr uint8_t RIGHT_ENCODER_A = 3;

    // Ultrasonic sensors
    constexpr uint8_t FRONT_TRIG = 8;
    constexpr uint8_t FRONT_ECHO = 9;

    constexpr uint8_t LEFT_TRIG  = 10;
    constexpr uint8_t LEFT_ECHO  = 11;

    constexpr uint8_t RIGHT_TRIG = 12;
    constexpr uint8_t RIGHT_ECHO = 13;

    // Battery analog input
    constexpr uint8_t BATTERY_PIN = A0;
}

/*
 * ================================================================
 * SECTION 2 — ROBOT CONFIGURATION
 * ================================================================
 */

struct RobotConfig
{
    float wheelRadius;
    float wheelSeparation;

    float robotMass;
    float centerOfMassHeight;

    float maximumMotorCommand;
    float maximumAcceleration;

    float maximumTiltAngle;
    float emergencyTiltAngle;

    long encoderTicksPerRevolution;

    float controlFrequency;
    float sensorFrequency;
    float navigationFrequency;
    float mappingFrequency;

    float batteryLowPercentage;
    float batteryCriticalPercentage;

    float ultrasonicMaximumDistance;

    float mapResolution;

    int mapWidth;
    int mapHeight;
};

RobotConfig config =
{
    0.0325f,       // wheel radius
    0.145f,        // wheel separation

    0.85f,         // robot mass
    0.085f,        // center of mass height

    255.0f,        // maximum motor command
    400.0f,        // acceleration limit

    25.0f,         // maximum normal tilt
    40.0f,         // emergency tilt

    360,           // encoder ticks

    100.0f,        // control Hz
    50.0f,         // sensor Hz
    10.0f,         // navigation Hz
    5.0f,          // mapping Hz

    25.0f,         // low battery
    10.0f,         // critical battery

    300.0f,        // ultrasonic range cm

    0.10f,         // map resolution

    80,            // map width
    80             // map height
};

/*
 * ================================================================
 * SECTION 3 — BASIC MATHEMATICS
 * ================================================================
 */

constexpr float PI_VALUE = 3.14159265358979323846f;

float degreesToRadians(float degrees)
{
    return degrees * PI_VALUE / 180.0f;
}

float radiansToDegrees(float radians)
{
    return radians * 180.0f / PI_VALUE;
}

float normalizeAngle(float angle)
{
    while (angle > PI_VALUE)
    {
        angle -= 2.0f * PI_VALUE;
    }

    while (angle < -PI_VALUE)
    {
        angle += 2.0f * PI_VALUE;
    }

    return angle;
}

float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

/*
 * ================================================================
 * SECTION 4 — VECTOR TYPES
 * ================================================================
 */

struct Vector2
{
    float x;
    float y;

    Vector2()
        : x(0.0f), y(0.0f)
    {
    }

    Vector2(float xValue, float yValue)
        : x(xValue), y(yValue)
    {
    }
};

float vectorMagnitude(const Vector2& vector)
{
    return sqrt(
        vector.x * vector.x +
        vector.y * vector.y
    );
}

/*
 * ================================================================
 * SECTION 5 — ROBOT POSE
 * ================================================================
 */

struct Pose2D
{
    float x;
    float y;
    float theta;

    Pose2D()
        : x(0.0f),
          y(0.0f),
          theta(0.0f)
    {
    }
};

struct RobotState
{
    Pose2D pose;

    float velocity;
    float angularVelocity;

    float pitch;
    float roll;
    float yaw;

    float pitchRate;
    float rollRate;
    float yawRate;

    float batteryPercentage;
    float batteryVoltage;

    float leftMotorCommand;
    float rightMotorCommand;

    bool imuHealthy;
    bool encoderHealthy;
    bool distanceSensorsHealthy;
    bool batteryHealthy;

    bool balanced;
    bool emergencyStop;

    RobotState()
        : velocity(0.0f),
          angularVelocity(0.0f),
          pitch(0.0f),
          roll(0.0f),
          yaw(0.0f),
          pitchRate(0.0f),
          rollRate(0.0f),
          yawRate(0.0f),
          batteryPercentage(100.0f),
          batteryVoltage(12.0f),
          leftMotorCommand(0.0f),
          rightMotorCommand(0.0f),
          imuHealthy(false),
          encoderHealthy(false),
          distanceSensorsHealthy(false),
          batteryHealthy(false),
          balanced(false),
          emergencyStop(false)
    {
    }
};

/*
 * ================================================================
 * SECTION 6 — PID CONTROLLER
 * ================================================================
 */

class PIDController
{
private:

    float kp;
    float ki;
    float kd;

    float setpoint;

    float integral;

    float previousError;

    float outputMinimum;
    float outputMaximum;

    float integralMinimum;
    float integralMaximum;

    float deadZone;

    float derivativeFilter;

    float filteredDerivative;

    bool firstUpdate;

public:

    PIDController()
        : kp(0.0f),
          ki(0.0f),
          kd(0.0f),
          setpoint(0.0f),
          integral(0.0f),
          previousError(0.0f),
          outputMinimum(-255.0f),
          outputMaximum(255.0f),
          integralMinimum(-100.0f),
          integralMaximum(100.0f),
          deadZone(0.0f),
          derivativeFilter(0.8f),
          filteredDerivative(0.0f),
          firstUpdate(true)
    {
    }

    PIDController(
        float proportional,
        float integralGain,
        float derivativeGain
    )
        : kp(proportional),
          ki(integralGain),
          kd(derivativeGain),
          setpoint(0.0f),
          integral(0.0f),
          previousError(0.0f),
          outputMinimum(-255.0f),
          outputMaximum(255.0f),
          integralMinimum(-100.0f),
          integralMaximum(100.0f),
          deadZone(0.0f),
          derivativeFilter(0.8f),
          filteredDerivative(0.0f),
          firstUpdate(true)
    {
    }

    void setGains(
        float proportional,
        float integralGain,
        float derivativeGain
    )
    {
        kp = proportional;
        ki = integralGain;
        kd = derivativeGain;
    }

    void setSetpoint(float value)
    {
        setpoint = value;
    }

    float getSetpoint() const
    {
        return setpoint;
    }

    void setOutputLimits(
        float minimum,
        float maximum
    )
    {
        outputMinimum = minimum;
        outputMaximum = maximum;
    }

    void setIntegralLimits(
        float minimum,
        float maximum
    )
    {
        integralMinimum = minimum;
        integralMaximum = maximum;
    }

    void setDeadZone(float value)
    {
        deadZone = fabs(value);
    }

    void setDerivativeFilter(float value)
    {
        derivativeFilter =
            clampFloat(value, 0.0f, 0.99f);
    }

    void reset()
    {
        integral = 0.0f;
        previousError = 0.0f;
        filteredDerivative = 0.0f;
        firstUpdate = true;
    }

    float update(
        float measurement,
        float deltaTime
    )
    {
        if (deltaTime <= 0.0f)
        {
            return 0.0f;
        }

        float error =
            setpoint - measurement;

        if (fabs(error) < deadZone)
        {
            error = 0.0f;
        }

        integral += error * deltaTime;

        integral = clampFloat(
            integral,
            integralMinimum,
            integralMaximum
        );

        float derivative = 0.0f;

        if (!firstUpdate)
        {
            derivative =
                (error - previousError) /
                deltaTime;
        }

        firstUpdate = false;

        filteredDerivative =
            derivativeFilter *
            filteredDerivative
            +
            (1.0f - derivativeFilter) *
            derivative;

        float output =
            kp * error
            +
            ki * integral
            +
            kd * filteredDerivative;

        /*
         * Anti-windup:
         * If the output is saturated, reduce integral
         * accumulation in the direction of saturation.
         */

        if (output > outputMaximum)
        {
            output = outputMaximum;

            if (error > 0.0f)
            {
                integral -= error * deltaTime;
            }
        }

        if (output < outputMinimum)
        {
            output = outputMinimum;

            if (error < 0.0f)
            {
                integral -= error * deltaTime;
            }
        }

        previousError = error;

        return output;
    }
};

/*
 * ================================================================
 * SECTION 7 — IMU DATA
 * ================================================================
 */

struct IMUData
{
    float accelerometerX;
    float accelerometerY;
    float accelerometerZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float pitch;
    float roll;
    float yaw;

    bool valid;

    IMUData()
        : accelerometerX(0.0f),
          accelerometerY(0.0f),
          accelerometerZ(1.0f),
          gyroX(0.0f),
          gyroY(0.0f),
          gyroZ(0.0f),
          pitch(0.0f),
          roll(0.0f),
          yaw(0.0f),
          valid(false)
    {
    }
};

/*
 * ================================================================
 * SECTION 8 — IMU INTERFACE
 * ================================================================
 */

class IIMUSensor
{
public:

    virtual bool begin() = 0;

    virtual bool update(
        float deltaTime
    ) = 0;

    virtual IMUData getData() const = 0;

    virtual bool isHealthy() const = 0;

    virtual ~IIMUSensor()
    {
    }
};

/*
 * ================================================================
 * SECTION 9 — SIMPLIFIED IMU
 * ================================================================
 *
 * This implementation provides a hardware-independent baseline.
 *
 * A real MPU6050/MPU6500/etc. driver can later implement the same
 * IIMUSensor interface.
 */

class SimulatedIMU : public IIMUSensor
{
private:

    IMUData data;

    float simulatedPitch;

    float simulatedRoll;

    float simulatedYaw;

    unsigned long lastUpdate;

public:

    SimulatedIMU()
        : simulatedPitch(0.0f),
          simulatedRoll(0.0f),
          simulatedYaw(0.0f),
          lastUpdate(0)
    {
    }

    bool begin() override
    {
        data = IMUData();

        simulatedPitch = 0.0f;
        simulatedRoll = 0.0f;
        simulatedYaw = 0.0f;

        lastUpdate = millis();

        data.valid = true;

        return true;
    }

    bool update(
        float deltaTime
    ) override
    {
        if (deltaTime <= 0.0f)
        {
            return false;
        }

        /*
         * Small deterministic simulated motion.
         * This is deliberately labeled simulation and should
         * not be interpreted as actual IMU measurements.
         */

        float timeSeconds =
            millis() / 1000.0f;

        simulatedPitch =
            0.8f *
            sin(timeSeconds * 0.7f);

        simulatedRoll =
            0.4f *
            sin(timeSeconds * 0.5f);

        simulatedYaw +=
            0.01f * deltaTime;

        data.pitch =
            simulatedPitch;

        data.roll =
            simulatedRoll;

        data.yaw =
            normalizeAngle(simulatedYaw);

        data.pitchRate =
            0.8f *
            0.7f *
            cos(timeSeconds * 0.7f);

        data.rollRate =
            0.4f *
            0.5f *
            cos(timeSeconds * 0.5f);

        data.yawRate =
            0.01f;

        data.accelerometerX =
            sin(degreesToRadians(data.pitch));

        data.accelerometerY =
            sin(degreesToRadians(data.roll));

        data.accelerometerZ =
            cos(
                degreesToRadians(data.pitch)
            );

        data.gyroX =
            data.rollRate;

        data.gyroY =
            data.pitchRate;

        data.gyroZ =
            data.yawRate;

        data.valid = true;

        return true;
    }

    IMUData getData() const override
    {
        return data;
    }

    bool isHealthy() const override
    {
        return data.valid;
    }
};

/*
 * ================================================================
 * SECTION 10 — ENCODER DATA
 * ================================================================
 */

struct EncoderData
{
    long leftTicks;
    long rightTicks;

    long previousLeftTicks;
    long previousRightTicks;

    float leftDistance;
    float rightDistance;

    bool valid;

    EncoderData()
        : leftTicks(0),
          rightTicks(0),
          previousLeftTicks(0),
          previousRightTicks(0),
          leftDistance(0.0f),
          rightDistance(0.0f),
          valid(false)
    {
    }
};

/*
 * ================================================================
 * SECTION 11 — ENCODER INTERFACE
 * ================================================================
 */

class IEncoder
{
public:

    virtual bool begin() = 0;

    virtual void update() = 0;

    virtual long getLeftTicks() const = 0;

    virtual long getRightTicks() const = 0;

    virtual void reset() = 0;

    virtual bool isHealthy() const = 0;

    virtual ~IEncoder()
    {
    }
};

/*
 * ================================================================
 * SECTION 12 — ENCODER IMPLEMENTATION
 * ================================================================
 */

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

void leftEncoderISR()
{
    leftEncoderTicks++;
}

void rightEncoderISR()
{
    rightEncoderTicks++;
}

class ArduinoEncoder : public IEncoder
{
private:

    bool healthy;

public:

    ArduinoEncoder()
        : healthy(false)
    {
    }

    bool begin() override
    {
        pinMode(
            Pins::LEFT_ENCODER_A,
            INPUT_PULLUP
        );

        pinMode(
            Pins::RIGHT_ENCODER_A,
            INPUT_PULLUP
        );

        attachInterrupt(
            digitalPinToInterrupt(
                Pins::LEFT_ENCODER_A
            ),
            leftEncoderISR,
            RISING
        );

        attachInterrupt(
            digitalPinToInterrupt(
                Pins::RIGHT_ENCODER_A
            ),
            rightEncoderISR,
            RISING
        );

        healthy = true;

        return true;
    }

    void update() override
    {
        /*
         * Hardware interrupts update the counters.
         * No additional processing is required here.
         */
    }

    long getLeftTicks() const override
    {
        noInterrupts();

        long value =
            leftEncoderTicks;

        interrupts();

        return value;
    }

    long getRightTicks() const override
    {
        noInterrupts();

        long value =
            rightEncoderTicks;

        interrupts();

        return value;
    }

    void reset() override
    {
        noInterrupts();

        leftEncoderTicks = 0;
        rightEncoderTicks = 0;

        interrupts();
    }

    bool isHealthy() const override
    {
        return healthy;
    }
};

/*
 * ================================================================
 * SECTION 13 — MOTOR COMMAND
 * ================================================================
 */

struct MotorCommand
{
    float left;
    float right;

    MotorCommand()
        : left(0.0f),
          right(0.0f)
    {
    }
};

/*
 * ================================================================
 * SECTION 14 — MOTOR INTERFACE
 * ================================================================
 */

class IMotorController
{
public:

    virtual bool begin() = 0;

    virtual void setCommand(
        float left,
        float right
    ) = 0;

    virtual void stop() = 0;

    virtual MotorCommand getCommand() const = 0;

    virtual ~IMotorController()
    {
    }
};

/*
 * ================================================================
 * SECTION 15 — MOTOR CONTROLLER
 * ================================================================
 */

class ArduinoMotorController :
    public IMotorController
{
private:

    MotorCommand command;

    float accelerationLimit;

    unsigned long lastUpdate;

    float previousLeft;

    float previousRight;

    int convertToPWM(float value)
    {
        value =
            clampFloat(
                value,
                -255.0f,
                255.0f
            );

        return (int)fabs(value);
    }

    void applyMotor(
        uint8_t pwmPin,
        uint8_t directionPin,
        float commandValue
    )
    {
        if (commandValue >= 0.0f)
        {
            digitalWrite(
                directionPin,
                HIGH
            );
        }
        else
        {
            digitalWrite(
                directionPin,
                LOW
            );
        }

        analogWrite(
            pwmPin,
            convertToPWM(commandValue)
        );
    }

public:

    ArduinoMotorController()
        : accelerationLimit(400.0f),
          lastUpdate(0),
          previousLeft(0.0f),
          previousRight(0.0f)
    {
    }

    bool begin() override
    {
        pinMode(
            Pins::LEFT_MOTOR_PWM,
            OUTPUT
        );

        pinMode(
            Pins::LEFT_MOTOR_DIR,
            OUTPUT
        );

        pinMode(
            Pins::RIGHT_MOTOR_PWM,
            OUTPUT
        );

        pinMode(
            Pins::RIGHT_MOTOR_DIR,
            OUTPUT
        );

        stop();

        lastUpdate =
            millis();

        return true;
    }

    void setAccelerationLimit(
        float limit
    )
    {
        accelerationLimit =
            fabs(limit);
    }

    void setCommand(
        float left,
        float right
    ) override
    {
        unsigned long now =
            millis();

        float dt =
            (now - lastUpdate) /
            1000.0f;

        if (dt <= 0.0f)
        {
            dt = 0.001f;
        }

        float maximumChange =
            accelerationLimit * dt;

        float leftDifference =
            left - previousLeft;

        float rightDifference =
            right - previousRight;

        leftDifference =
            clampFloat(
                leftDifference,
                -maximumChange,
                maximumChange
            );

        rightDifference =
            clampFloat(
                rightDifference,
                -maximumChange,
                maximumChange
            );

        previousLeft +=
            leftDifference;

        previousRight +=
            rightDifference;

        command.left =
            clampFloat(
                previousLeft,
                -255.0f,
                255.0f
            );

        command.right =
            clampFloat(
                previousRight,
                -255.0f,
                255.0f
            );

        applyMotor(
            Pins::LEFT_MOTOR_PWM,
            Pins::LEFT_MOTOR_DIR,
            command.left
        );

        applyMotor(
            Pins::RIGHT_MOTOR_PWM,
            Pins::RIGHT_MOTOR_DIR,
            command.right
        );

        lastUpdate = now;
    }

    void stop() override
    {
        command.left = 0.0f;
        command.right = 0.0f;

        previousLeft = 0.0f;
        previousRight = 0.0f;

        analogWrite(
            Pins::LEFT_MOTOR_PWM,
            0
        );

        analogWrite(
            Pins::RIGHT_MOTOR_PWM,
            0
        );
    }

    MotorCommand getCommand() const override
    {
        return command;
    }
};

/*
 * ================================================================
 * SECTION 16 — DISTANCE SENSOR
 * ================================================================
 */

class IDistanceSensor
{
public:

    virtual bool begin() = 0;

    virtual float readDistanceCM() = 0;

    virtual bool isHealthy() const = 0;

    virtual ~IDistanceSensor()
    {
    }
};

/*
 * ================================================================
 * SECTION 17 — ULTRASONIC SENSOR
 * ================================================================
 */

class UltrasonicSensor :
    public IDistanceSensor
{
private:

    uint8_t triggerPin;

    uint8_t echoPin;

    float maximumDistance;

    bool healthy;

public:

    UltrasonicSensor(
        uint8_t trigger,
        uint8_t echo,
        float maximumRange
    )
        : triggerPin(trigger),
          echoPin(echo),
          maximumDistance(maximumRange),
          healthy(false)
    {
    }

    bool begin() override
    {
        pinMode(
            triggerPin,
            OUTPUT
        );

        pinMode(
            echoPin,
            INPUT
        );

        digitalWrite(
            triggerPin,
            LOW
        );

        healthy = true;

        return true;
    }

    float readDistanceCM() override
    {
        digitalWrite(
            triggerPin,
            LOW
        );

        delayMicroseconds(2);

        digitalWrite(
            triggerPin,
            HIGH
        );

        delayMicroseconds(10);

        digitalWrite(
            triggerPin,
            LOW
        );

        unsigned long duration =
            pulseIn(
                echoPin,
                HIGH,
                30000UL
            );

        if (duration == 0)
        {
            return maximumDistance;
        }

        float distance =
            duration * 0.0343f / 2.0f;

        if (distance < 0.0f)
        {
            return maximumDistance;
        }

        if (distance > maximumDistance)
        {
            return maximumDistance;
        }

        return distance;
    }

    bool isHealthy() const override
    {
        return healthy;
    }
};

/*
 * ================================================================
 * SECTION 18 — BATTERY MONITOR
 * ================================================================
 */

class IBatteryMonitor
{
public:

    virtual bool begin() = 0;

    virtual float getVoltage() = 0;

    virtual float getPercentage() = 0;

    virtual bool isHealthy() const = 0;

    virtual ~IBatteryMonitor()
    {
    }
};

class BatteryMonitor :
    public IBatteryMonitor
{
private:

    float minimumVoltage;

    float maximumVoltage;

    float dividerRatio;

    bool healthy;

public:

    BatteryMonitor()
        : minimumVoltage(9.0f),
          maximumVoltage(12.6f),
          dividerRatio(3.0f),
          healthy(false)
    {
    }

    bool begin() override
    {
        pinMode(
            Pins::BATTERY_PIN,
            INPUT
        );

        healthy = true;

        return true;
    }

    float getVoltage() override
    {
        int raw =
            analogRead(
                Pins::BATTERY_PIN
            );

        float measured =
            raw * (5.0f / 1023.0f);

        float voltage =
            measured * dividerRatio;

        return voltage;
    }

    float getPercentage() override
    {
        float voltage =
            getVoltage();

        float percentage =
            (
                voltage -
                minimumVoltage
            )
            /
            (
                maximumVoltage -
                minimumVoltage
            )
            *
            100.0f;

        return clampFloat(
            percentage,
            0.0f,
            100.0f
        );
    }

    bool isHealthy() const override
    {
        return healthy;
    }
};

/*
 * ================================================================
 * SECTION 19 — SAFETY STATES
 * ================================================================
 */

enum class SafetyState
{
    NORMAL,
    WARNING,
    LIMITED,
    EMERGENCY_STOP
};

/*
 * ================================================================
 * SECTION 20 — SAFETY MANAGER
 * ================================================================
 */

class SafetyManager
{
private:

    SafetyState state;

    float maximumTilt;

    float emergencyTilt;

    float lowBattery;

    float criticalBattery;

    bool sensorFailure;

    bool communicationFailure;

public:

    SafetyManager()
        : state(SafetyState::NORMAL),
          maximumTilt(25.0f),
          emergencyTilt(40.0f),
          lowBattery(25.0f),
          criticalBattery(10.0f),
          sensorFailure(false),
          communicationFailure(false)
    {
    }

    void configure(
        float normalTilt,
        float emergencyTiltAngle,
        float lowBatteryLevel,
        float criticalBatteryLevel
    )
    {
        maximumTilt =
            fabs(normalTilt);

        emergencyTilt =
            fabs(emergencyTiltAngle);

        lowBattery =
            lowBatteryLevel;

        criticalBattery =
            criticalBatteryLevel;
    }

    void update(
        const RobotState& robot
    )
    {
        float absolutePitch =
            fabs(
                robot.pitch
            );

        if (absolutePitch >= emergencyTilt)
        {
            state =
                SafetyState::EMERGENCY_STOP;

            return;
        }

        if (
            robot.batteryPercentage
            <= criticalBattery
        )
        {
            state =
                SafetyState::EMERGENCY_STOP;

            return;
        }

        if (
            !robot.imuHealthy ||
            !robot.encoderHealthy
        )
        {
            state =
                SafetyState::EMERGENCY_STOP;

            return;
        }

        if (
            robot.batteryPercentage
            <= lowBattery
        )
        {
            state =
                SafetyState::LIMITED;

            return;
        }

        if (
            absolutePitch >= maximumTilt
        )
        {
            state =
                SafetyState::WARNING;

            return;
        }

        state =
            SafetyState::NORMAL;
    }

    SafetyState getState() const
    {
        return state;
    }

    bool isEmergency() const
    {
        return state ==
            SafetyState::EMERGENCY_STOP;
    }

    bool isLimited() const
    {
        return state ==
            SafetyState::LIMITED;
    }
};

/*
 * ================================================================
 * SECTION 21 — BALANCE CONTROLLER
 * ================================================================
 */

class BalanceController
{
private:

    PIDController anglePID;

    PIDController velocityPID;

    float targetPitch;

    float targetVelocity;

    float maximumOutput;

public:

    BalanceController()
        : targetPitch(0.0f),
          targetVelocity(0.0f),
          maximumOutput(255.0f)
    {
        /*
         * These are starting educational gains only.
         * Real hardware must be tuned experimentally and safely.
         */

        anglePID.setGains(
            75.0f,
            0.0f,
            1.5f
        );

        velocityPID.setGains(
            1.0f,
            0.0f,
            0.05f
        );

        anglePID.setOutputLimits(
            -255.0f,
            255.0f
        );

        velocityPID.setOutputLimits(
            -10.0f,
            10.0f
        );
    }

    void setTargetPitch(
        float pitch
    )
    {
        targetPitch = pitch;

        anglePID.setSetpoint(
            pitch
        );
    }

    void setTargetVelocity(
        float velocity
    )
    {
        targetVelocity =
            velocity;

        velocityPID.setSetpoint(
            velocity
        );
    }

    void reset()
    {
        anglePID.reset();
        velocityPID.reset();
    }

    float update(
        float pitch,
        float velocity,
        float deltaTime
    )
    {
        float velocityCorrection =
            velocityPID.update(
                velocity,
                deltaTime
            );

        float commandedPitch =
            targetPitch +
            velocityCorrection;

        anglePID.setSetpoint(
            commandedPitch
        );

        float motorOutput =
            anglePID.update(
                pitch,
                deltaTime
            );

        return clampFloat(
            motorOutput,
            -maximumOutput,
            maximumOutput
        );
    }
};

/*
 * ================================================================
 * SECTION 22 — DIFFERENTIAL ODOMETRY
 * ================================================================
 */

class Odometry
{
private:

    Pose2D pose;

    long previousLeftTicks;

    long previousRightTicks;

    bool initialized;

public:

    Odometry()
        : previousLeftTicks(0),
          previousRightTicks(0),
          initialized(false)
    {
    }

    void reset(
        const Pose2D& initialPose,
        long leftTicks,
        long rightTicks
    )
    {
        pose =
            initialPose;

        previousLeftTicks =
            leftTicks;

        previousRightTicks =
            rightTicks;

        initialized = true;
    }

    void update(
        long leftTicks,
        long rightTicks
    )
    {
        if (!initialized)
        {
            reset(
                Pose2D(),
                leftTicks,
                rightTicks
            );

            return;
        }

        long leftDelta =
            leftTicks -
            previousLeftTicks;

        long rightDelta =
            rightTicks -
            previousRightTicks;

        previousLeftTicks =
            leftTicks;

        previousRightTicks =
            rightTicks;

        float wheelCircumference =
            2.0f *
            PI_VALUE *
            config.wheelRadius;

        float leftDistance =
            (
                (float)leftDelta /
                (float)config.encoderTicksPerRevolution
            )
            *
            wheelCircumference;

        float rightDistance =
            (
                (float)rightDelta /
                (float)config.encoderTicksPerRevolution
            )
            *
            wheelCircumference;

        float centerDistance =
            (
                leftDistance +
                rightDistance
            )
            * 0.5f;

        float deltaTheta =
            (
                rightDistance -
                leftDistance
            )
            /
            config.wheelSeparation;

        float midpointHeading =
            pose.theta +
            deltaTheta * 0.5f;

        pose.x +=
            centerDistance *
            cos(midpointHeading);

        pose.y +=
            centerDistance *
            sin(midpointHeading);

        pose.theta =
            normalizeAngle(
                pose.theta +
                deltaTheta
            );
    }

    Pose2D getPose() const
    {
        return pose;
    }
};

/*
 * ================================================================
 * SECTION 23 — SENSOR FUSION
 * ================================================================
 */

class SensorFusion
{
private:

    float fusedPitch;

    float fusedRoll;

    float fusedYaw;

    float alpha;

public:

    SensorFusion()
        : fusedPitch(0.0f),
          fusedRoll(0.0f),
          fusedYaw(0.0f),
          alpha(0.98f)
    {
    }

    void setAlpha(
        float value
    )
    {
        alpha =
            clampFloat(
                value,
                0.0f,
                1.0f
            );
    }

    void update(
        const IMUData& imu,
        float deltaTime
    )
    {
        if (!imu.valid)
        {
            return;
        }

        /*
         * Complementary filter:
         *
         * angle =
         *     alpha * gyro-integrated-angle
         *     +
         *     (1-alpha) * accelerometer-angle
         *
         * This implementation is intentionally compact for
         * Arduino-class hardware.
         */

        float gyroPitch =
            fusedPitch +
            imu.gyroY *
            deltaTime;

        float gyroRoll =
            fusedRoll +
            imu.gyroX *
            deltaTime;

        float accelerometerPitch =
            radiansToDegrees(
                atan2(
                    imu.accelerometerX,
                    imu.accelerometerZ
                )
            );

        float accelerometerRoll =
            radiansToDegrees(
                atan2(
                    imu.accelerometerY,
                    imu.accelerometerZ
                )
            );

        fusedPitch =
            alpha * gyroPitch
            +
            (1.0f - alpha) *
            accelerometerPitch;

        fusedRoll =
            alpha * gyroRoll
            +
            (1.0f - alpha) *
            accelerometerRoll;

        fusedYaw =
            normalizeAngle(
                fusedYaw +
                imu.gyroZ *
                deltaTime
            );
    }

    float getPitch() const
    {
        return fusedPitch;
    }

    float getRoll() const
    {
        return fusedRoll;
    }

    float getYaw() const
    {
        return fusedYaw;
    }
};

/*
 * ================================================================
 * SECTION 24 — OBSTACLE DATA
 * ================================================================
 */

struct ObstacleMeasurement
{
    float distance;
    float angle;
    float confidence;
    bool valid;

    ObstacleMeasurement()
        : distance(0.0f),
          angle(0.0f),
          confidence(0.0f),
          valid(false)
    {
    }
};

struct DistanceSensorArray
{
    float front;
    float left;
    float right;

    bool frontValid;
    bool leftValid;
    bool rightValid;

    DistanceSensorArray()
        : front(0.0f),
          left(0.0f),
          right(0.0f),
          frontValid(false),
          leftValid(false),
          rightValid(false)
    {
    }
};

/*
 * ================================================================
 * SECTION 25 — GLOBAL ROBOT OBJECTS
 * ================================================================
 */

SimulatedIMU imu;

ArduinoEncoder encoders;

ArduinoMotorController motors;

UltrasonicSensor frontSensor(
    Pins::FRONT_TRIG,
    Pins::FRONT_ECHO,
    config.ultrasonicMaximumDistance
);

UltrasonicSensor leftSensor(
    Pins::LEFT_TRIG,
    Pins::LEFT_ECHO,
    config.ultrasonicMaximumDistance
);

UltrasonicSensor rightSensor(
    Pins::RIGHT_TRIG,
    Pins::RIGHT_ECHO,
    config.ultrasonicMaximumDistance
);

BatteryMonitor battery;

BalanceController balanceController;

SensorFusion sensorFusion;

Odometry odometry;

SafetyManager safetyManager;

RobotState robotState;

DistanceSensorArray distanceSensors;

/*
 * ================================================================
 * SECTION 26 — TIMING
 * ================================================================
 */

unsigned long previousControlTime = 0;

unsigned long previousSensorTime = 0;

unsigned long previousNavigationTime = 0;

unsigned long previousMappingTime = 0;

unsigned long previousTelemetryTime = 0;

/*
 * ================================================================
 * SECTION 27 — SENSOR UPDATE
 * ================================================================
 */

void updateSensors(
    float deltaTime
)
{
    imu.update(
        deltaTime
    );

    encoders.update();

    distanceSensors.front =
        frontSensor.readDistanceCM();

    distanceSensors.left =
        leftSensor.readDistanceCM();

    distanceSensors.right =
        rightSensor.readDistanceCM();

    distanceSensors.frontValid =
        frontSensor.isHealthy();

    distanceSensors.leftValid =
        leftSensor.isHealthy();

    distanceSensors.rightValid =
        rightSensor.isHealthy();

    robotState.batteryVoltage =
        battery.getVoltage();

    robotState.batteryPercentage =
        battery.getPercentage();

    robotState.imuHealthy =
        imu.isHealthy();

    robotState.encoderHealthy =
        encoders.isHealthy();

    robotState.distanceSensorsHealthy =
        distanceSensors.frontValid &&
        distanceSensors.leftValid &&
        distanceSensors.rightValid;

    robotState.batteryHealthy =
        battery.isHealthy();
}

/*
 * ================================================================
 * SECTION 28 — STATE ESTIMATION
 * ================================================================
 */

void updateStateEstimation(
    float deltaTime
)
{
    IMUData imuData =
        imu.getData();

    sensorFusion.update(
        imuData,
        deltaTime
    );

    odometry.update(
        encoders.getLeftTicks(),
        encoders.getRightTicks()
    );

    robotState.pose =
        odometry.getPose();

    robotState.pitch =
        sensorFusion.getPitch();

    robotState.roll =
        sensorFusion.getRoll();

    robotState.yaw =
        sensorFusion.getYaw();

    robotState.pitchRate =
        imuData.pitchRate;

    robotState.rollRate =
        imuData.rollRate;

    robotState.yawRate =
        imuData.yawRate;
}

/*
 * ================================================================
 * SECTION 29 — BALANCE UPDATE
 * ================================================================
 */

void updateBalance(
    float deltaTime
)
{
    if (safetyManager.isEmergency())
    {
        motors.stop();

        robotState.leftMotorCommand =
            0.0f;

        robotState.rightMotorCommand =
            0.0f;

        return;
    }

    float balanceOutput =
        balanceController.update(
            robotState.pitch,
            robotState.velocity,
            deltaTime
        );

    /*
     * At this stage both wheels receive the balancing command.
     *
     * Later navigation logic will add differential steering:
     *
     * left  = balance - steering
     * right = balance + steering
     */

    motors.setCommand(
        balanceOutput,
        balanceOutput
    );

    MotorCommand command =
        motors.getCommand();

    robotState.leftMotorCommand =
        command.left;

    robotState.rightMotorCommand =
        command.right;
}

/*
 * ================================================================
 * SECTION 30 — OBSTACLE CHECK
 * ================================================================
 */

bool obstacleDetected()
{
    constexpr float SAFE_DISTANCE =
        35.0f;

    if (
        distanceSensors.frontValid &&
        distanceSensors.front <
        SAFE_DISTANCE
    )
    {
        return true;
    }

    if (
        distanceSensors.leftValid &&
        distanceSensors.left <
        20.0f
    )
    {
        return true;
    }

    if (
        distanceSensors.rightValid &&
        distanceSensors.right <
        20.0f
    )
    {
        return true;
    }

    return false;
}

/*
 * ================================================================
 * SECTION 31 — SAFETY UPDATE
 * ================================================================
 */

void updateSafety()
{
    safetyManager.update(
        robotState
    );

    if (
        safetyManager.isEmergency()
    )
    {
        robotState.emergencyStop =
            true;

        motors.stop();
    }
    else
    {
        robotState.emergencyStop =
            false;
    }

    robotState.balanced =
        fabs(
            robotState.pitch
        )
        <
        config.maximumTiltAngle;
}

/*
 * ================================================================
 * SECTION 32 — TELEMETRY
 * ================================================================
 */

void printTelemetry()
{
    Serial.println(
        F("================================")
    );

    Serial.println(
        F("ASBR TELEMETRY")
    );

    Serial.print(
        F("X: ")
    );

    Serial.println(
        robotState.pose.x,
        3
    );

    Serial.print(
        F("Y: ")
    );

    Serial.println(
        robotState.pose.y,
        3
    );

    Serial.print(
        F("Heading: ")
    );

    Serial.println(
        radiansToDegrees(
            robotState.pose.theta
        ),
        2
    );

    Serial.print(
        F("Pitch: ")
    );

    Serial.println(
        robotState.pitch,
        2
    );

    Serial.print(
        F("Roll: ")
    );

    Serial.println(
        robotState.roll,
        2
    );

    Serial.print(
        F("Yaw: ")
    );

    Serial.println(
        robotState.yaw,
        2
    );

    Serial.print(
        F("Front: ")
    );

    Serial.print(
        distanceSensors.front
    );

    Serial.println(
        F(" cm")
    );

    Serial.print(
        F("Left: ")
    );

    Serial.print(
        distanceSensors.left
    );

    Serial.println(
        F(" cm")
    );

    Serial.print(
        F("Right: ")
    );

    Serial.print(
        distanceSensors.right
    );

    Serial.println(
        F(" cm")
    );

    Serial.print(
        F("Battery: ")
    );

    Serial.print(
        robotState.batteryPercentage,
        1
    );

    Serial.println(
        F("%")
    );

    Serial.print(
        F("Motor L: ")
    );

    Serial.println(
        robotState.leftMotorCommand,
        1
    );

    Serial.print(
        F("Motor R: ")
    );

    Serial.println(
        robotState.rightMotorCommand,
        1
    );

    Serial.print(
        F("Obstacle: ")
    );

    Serial.println(
        obstacleDetected()
            ? F("YES")
            : F("NO")
    );

    Serial.print(
        F("Emergency: ")
    );

    Serial.println(
        robotState.emergencyStop
            ? F("YES")
            : F("NO")
    );

    Serial.println(
        F("================================")
    );
}

/*
 * ================================================================
 * SECTION 33 — COMMAND INTERFACE
 * ================================================================
 */

void printHelp()
{
    Serial.println(
        F("")
    );

    Serial.println(
        F("ASBR COMMAND INTERFACE")
    );

    Serial.println(
        F("----------------------")
    );

    Serial.println(
        F("help       - Show commands")
    );

    Serial.println(
        F("status     - Robot status")
    );

    Serial.println(
        F("pose       - Robot pose")
    );

    Serial.println(
        F("sensors    - Sensor readings")
    );

    Serial.println(
        F("battery    - Battery status")
    );

    Serial.println(
        F("motors     - Motor commands")
    );

    Serial.println(
        F("stop       - Emergency stop")
    );

    Serial.println(
        F("reset      - Reset emergency state")
    );

    Serial.println(
        F("quit       - Stop motors")
    );

    Serial.println(
        F("")
    );
}

void processCommand(
    String command
)
{
    command.trim();

    command.toLowerCase();

    if (command == "help")
    {
        printHelp();

        return;
    }

    if (command == "status")
    {
        printTelemetry();

        return;
    }

    if (command == "pose")
    {
        Serial.print(
            F("X = ")
        );

        Serial.println(
            robotState.pose.x,
            3
        );

        Serial.print(
            F("Y = ")
        );

        Serial.println(
            robotState.pose.y,
            3
        );

        Serial.print(
            F("Theta = ")
        );

        Serial.println(
            robotState.pose.theta,
            3
        );

        return;
    }

    if (command == "sensors")
    {
        Serial.print(
            F("Front: ")
        );

        Serial.println(
            distanceSensors.front
        );

        Serial.print(
            F("Left: ")
        );

        Serial.println(
            distanceSensors.left
        );

        Serial.print(
            F("Right: ")
        );

        Serial.println(
            distanceSensors.right
        );

        return;
    }

    if (command == "battery")
    {
        Serial.print(
            F("Voltage: ")
        );

        Serial.println(
            robotState.batteryVoltage,
            2
        );

        Serial.print(
            F("Percentage: ")
        );

        Serial.println(
            robotState.batteryPercentage,
            1
        );

        return;
    }

    if (command == "motors")
    {
        Serial.print(
            F("Left: ")
        );

        Serial.println(
            robotState.leftMotorCommand
        );

        Serial.print(
            F("Right: ")
        );

        Serial.println(
            robotState.rightMotorCommand
        );

        return;
    }

    if (
        command == "stop" ||
        command == "quit"
    )
    {
        motors.stop();

        robotState.emergencyStop =
            true;

        Serial.println(
            F("MOTORS STOPPED")
        );

        return;
    }

    if (command == "reset")
    {
        robotState.emergencyStop =
            false;

        safetyManager =
            SafetyManager();

        Serial.println(
            F("Safety state reset")
        );

        return;
    }

    Serial.print(
        F("Unknown command: ")
    );

    Serial.println(
        command
    );
}

/*
 * ================================================================
 * SECTION 34 — SERIAL COMMAND PROCESSING
 * ================================================================
 */

void updateCommandInterface()
{
    static String commandBuffer;

    while (Serial.available() > 0)
    {
        char character =
            Serial.read();

        if (
            character == '\n' ||
            character == '\r'
        )
        {
            if (
                commandBuffer.length()
                > 0
            )
            {
                processCommand(
                    commandBuffer
                );

                commandBuffer =
                    "";
            }
        }
        else
        {
            if (
                commandBuffer.length()
                < 64
            )
            {
                commandBuffer +=
                    character;
            }
        }
    }
}

/*
 * ================================================================
 * SECTION 35 — SYSTEM INITIALIZATION
 * ================================================================
 */

void initializeSystem()
{
    Serial.begin(
        115200
    );

    delay(500);

    Serial.println();

    Serial.println(
        F("======================================")
    );

    Serial.println(
        F(" AUTONOMOUS SELF-BALANCING ROBOT")
    );

    Serial.println(
        F(" ASBR C++ FIRMWARE")
    );

    Serial.println(
        F("======================================")
    );

    Serial.println(
        F("Initializing IMU...")
    );

    if (imu.begin())
    {
        Serial.println(
            F("IMU: OK")
        );
    }
    else
    {
        Serial.println(
            F("IMU: FAILED")
        );
    }

    Serial.println(
        F("Initializing encoders...")
    );

    if (encoders.begin())
    {
        Serial.println(
            F("Encoders: OK")
        );
    }
    else
    {
        Serial.println(
            F("Encoders: FAILED")
        );
    }

    Serial.println(
        F("Initializing motors...")
    );

    if (motors.begin())
    {
        Serial.println(
            F("Motors: OK")
        );
    }
    else
    {
        Serial.println(
            F("Motors: FAILED")
        );
    }

    Serial.println(
        F("Initializing distance sensors...")
    );

    frontSensor.begin();
    leftSensor.begin();
    rightSensor.begin();

    Serial.println(
        F("Distance sensors: OK")
    );

    Serial.println(
        F("Initializing battery monitor...")
    );

    battery.begin();

    Serial.println(
        F("Battery monitor: OK")
    );

    safetyManager.configure(
        config.maximumTiltAngle,
        config.emergencyTiltAngle,
        config.batteryLowPercentage,
        config.batteryCriticalPercentage
    );

    balanceController.setTargetPitch(
        0.0f
    );

    balanceController.setTargetVelocity(
        0.0f
    );

    Pose2D initialPose;

    odometry.reset(
        initialPose,
        encoders.getLeftTicks(),
        encoders.getRightTicks()
    );

    robotState.imuHealthy =
        imu.isHealthy();

    robotState.encoderHealthy =
        encoders.isHealthy();

    robotState.batteryHealthy =
        battery.isHealthy();

    Serial.println();

    Serial.println(
        F("System initialization complete.")
    );

    Serial.println(
        F("Type 'help' for commands.")
    );

    Serial.println();
}

/*
 * ================================================================
 * SECTION 36 — ARDUINO SETUP
 * ================================================================
 */

void setup()
{
    initializeSystem();

    previousControlTime =
        micros();

    previousSensorTime =
        micros();

    previousNavigationTime =
        millis();

    previousMappingTime =
        millis();

    previousTelemetryTime =
        millis();
}

/*
 * ================================================================
 * SECTION 37 — CONTROL LOOP
 * ================================================================
 */

void updateControlLoop()
{
    unsigned long currentMicros =
        micros();

    unsigned long elapsed =
        currentMicros -
        previousControlTime;

    float deltaTime =
        elapsed / 1000000.0f;

    float desiredPeriod =
        1.0f /
        config.controlFrequency;

    if (
        deltaTime <
        desiredPeriod
    )
    {
        return;
    }

    previousControlTime =
        currentMicros;

    updateSensors(
        deltaTime
    );

    updateStateEstimation(
        deltaTime
    );

    updateSafety();

    if (
        !robotState.emergencyStop
    )
    {
        updateBalance(
            deltaTime
        );
    }
    else
    {
        motors.stop();
    }
}

/*
 * ================================================================
 * SECTION 38 — TELEMETRY LOOP
 * ================================================================
 */

void updateTelemetry()
{
    unsigned long now =
        millis();

    if (
        now -
        previousTelemetryTime
        <
        1000
    )
    {
        return;
    }

    previousTelemetryTime =
        now;

    printTelemetry();
}

/*
 * ================================================================
 * SECTION 39 — MAIN LOOP
 * ================================================================
 */

void loop()
{
    updateCommandInterface();

    updateControlLoop();

    updateTelemetry();
}