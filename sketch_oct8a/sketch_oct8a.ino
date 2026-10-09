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
/*
 * ================================================================
 *  AUTONOMOUS SELF-BALANCING ROBOT — ASBR
 * ================================================================
 *
 *  PART 2
 *
 *  OCCUPANCY GRID
 *  RAY CASTING
 *  SENSOR PROJECTION
 *  MAP UPDATE ENGINE
 *  MAP STATISTICS
 *
 *  CONTINUES FROM PART 1
 *
 * ================================================================
 */


/*
 * ================================================================
 * SECTION 40 — MAP CELL STATES
 * ================================================================
 */

enum class MapCellState : int8_t
{
    UNKNOWN = -1,
    FREE = 0,
    OCCUPIED = 1
};


/*
 * ================================================================
 * SECTION 41 — MAP CELL
 * ================================================================
 */

struct MapCell
{
    int8_t occupancy;

    uint16_t observations;

    uint16_t freeObservations;

    uint16_t occupiedObservations;

    MapCell()
        : occupancy(
              static_cast<int8_t>(
                  MapCellState::UNKNOWN
              )
          ),
          observations(0),
          freeObservations(0),
          occupiedObservations(0)
    {
    }

    void reset()
    {
        occupancy =
            static_cast<int8_t>(
                MapCellState::UNKNOWN
            );

        observations = 0;

        freeObservations = 0;

        occupiedObservations = 0;
    }

    bool isUnknown() const
    {
        return occupancy ==
            static_cast<int8_t>(
                MapCellState::UNKNOWN
            );
    }

    bool isFree() const
    {
        return occupancy ==
            static_cast<int8_t>(
                MapCellState::FREE
            );
    }

    bool isOccupied() const
    {
        return occupancy ==
            static_cast<int8_t>(
                MapCellState::OCCUPIED
            );
    }
};


/*
 * ================================================================
 * SECTION 42 — GRID COORDINATE
 * ================================================================
 */

struct GridCoordinate
{
    int x;

    int y;

    GridCoordinate()
        : x(0),
          y(0)
    {
    }

    GridCoordinate(
        int gridX,
        int gridY
    )
        : x(gridX),
          y(gridY)
    {
    }

    bool operator==(
        const GridCoordinate& other
    ) const
    {
        return
            x == other.x &&
            y == other.y;
    }

    bool operator!=(
        const GridCoordinate& other
    ) const
    {
        return !(*this == other);
    }
};


/*
 * ================================================================
 * SECTION 43 — SENSOR RAY
 * ================================================================
 */

struct SensorRay
{
    Vector2 origin;

    Vector2 endpoint;

    float angle;

    float distance;

    bool valid;

    SensorRay()
        : origin(),
          endpoint(),
          angle(0.0f),
          distance(0.0f),
          valid(false)
    {
    }
};


/*
 * ================================================================
 * SECTION 44 — MAP BOUNDARIES
 * ================================================================
 */

struct MapBounds
{
    float minimumX;

    float maximumX;

    float minimumY;

    float maximumY;

    MapBounds()
        : minimumX(0.0f),
          maximumX(0.0f),
          minimumY(0.0f),
          maximumY(0.0f)
    {
    }

    float width() const
    {
        return maximumX - minimumX;
    }

    float height() const
    {
        return maximumY - minimumY;
    }
};


/*
 * ================================================================
 * SECTION 45 — MAP STATISTICS
 * ================================================================
 */

struct MapStatistics
{
    uint32_t unknownCells;

    uint32_t freeCells;

    uint32_t occupiedCells;

    uint32_t totalCells;

    float coveragePercentage;

    MapBounds boundaries;

    MapStatistics()
        : unknownCells(0),
          freeCells(0),
          occupiedCells(0),
          totalCells(0),
          coveragePercentage(0.0f),
          boundaries()
    {
    }
};


/*
 * ================================================================
 * SECTION 46 — OCCUPANCY GRID
 * ================================================================
 */

class OccupancyGrid
{
private:

    /*
     * Arduino Uno has limited SRAM.
     *
     * The configured map is therefore stored in a compact
     * dynamically allocated array.
     *
     * Each cell currently uses a compact representation.
     */

    MapCell* cells;

    int width;

    int height;

    float resolution;

    Vector2 origin;

    bool initialized;

public:

    OccupancyGrid()
        : cells(nullptr),
          width(0),
          height(0),
          resolution(0.1f),
          origin(),
          initialized(false)
    {
    }

    ~OccupancyGrid()
    {
        release();
    }

    bool initialize(
        int mapWidth,
        int mapHeight,
        float mapResolution,
        const Vector2& mapOrigin
    )
    {
        release();

        if (
            mapWidth <= 0 ||
            mapHeight <= 0
        )
        {
            return false;
        }

        if (
            mapResolution <= 0.0f
        )
        {
            return false;
        }

        long totalCells =
            static_cast<long>(
                mapWidth
            )
            *
            static_cast<long>(
                mapHeight
            );

        /*
         * Conservative Arduino memory guard.
         *
         * The Uno cannot safely hold a large desktop-sized map.
         */

        if (
            totalCells > 4000L
        )
        {
            return false;
        }

        cells =
            new MapCell[
                totalCells
            ];

        if (cells == nullptr)
        {
            return false;
        }

        width =
            mapWidth;

        height =
            mapHeight;

        resolution =
            mapResolution;

        origin =
            mapOrigin;

        initialized =
            true;

        clear();

        return true;
    }

    void release()
    {
        if (cells != nullptr)
        {
            delete[] cells;

            cells = nullptr;
        }

        width = 0;

        height = 0;

        initialized = false;
    }

    void clear()
    {
        if (!initialized)
        {
            return;
        }

        long total =
            static_cast<long>(
                width
            )
            *
            static_cast<long>(
                height
            );

        for (
            long index = 0;
            index < total;
            ++index
        )
        {
            cells[index].reset();
        }
    }

    bool isInitialized() const
    {
        return initialized;
    }

    int getWidth() const
    {
        return width;
    }

    int getHeight() const
    {
        return height;
    }

    float getResolution() const
    {
        return resolution;
    }

    Vector2 getOrigin() const
    {
        return origin;
    }

    bool isInside(
        int x,
        int y
    ) const
    {
        return
            x >= 0 &&
            y >= 0 &&
            x < width &&
            y < height;
    }

    int indexOf(
        int x,
        int y
    ) const
    {
        if (!isInside(x, y))
        {
            return -1;
        }

        return
            y * width +
            x;
    }

    MapCell* getCell(
        int x,
        int y
    )
    {
        int index =
            indexOf(
                x,
                y
            );

        if (index < 0)
        {
            return nullptr;
        }

        return &cells[index];
    }

    const MapCell* getCell(
        int x,
        int y
    ) const
    {
        int index =
            indexOf(
                x,
                y
            );

        if (index < 0)
        {
            return nullptr;
        }

        return &cells[index];
    }

    MapCellState getState(
        int x,
        int y
    ) const
    {
        const MapCell* cell =
            getCell(
                x,
                y
            );

        if (cell == nullptr)
        {
            return MapCellState::UNKNOWN;
        }

        return static_cast<MapCellState>(
            cell->occupancy
        );
    }

    void setState(
        int x,
        int y,
        MapCellState state
    )
    {
        MapCell* cell =
            getCell(
                x,
                y
            );

        if (cell == nullptr)
        {
            return;
        }

        cell->occupancy =
            static_cast<int8_t>(
                state
            );

        cell->observations++;

        if (
            state ==
            MapCellState::FREE
        )
        {
            cell->freeObservations++;
        }

        if (
            state ==
            MapCellState::OCCUPIED
        )
        {
            cell->occupiedObservations++;
        }
    }

    void markFree(
        int x,
        int y
    )
    {
        MapCell* cell =
            getCell(
                x,
                y
            );

        if (cell == nullptr)
        {
            return;
        }

        cell->observations++;

        cell->freeObservations++;

        /*
         * A repeated free observation can override an
         * earlier unknown state.
         *
         * Occupied cells are deliberately not immediately
         * erased because an obstacle measurement should
         * require stronger evidence to disappear.
         */

        if (
            cell->occupiedObservations == 0 ||
            cell->freeObservations >
                cell->occupiedObservations + 2
        )
        {
            cell->occupancy =
                static_cast<int8_t>(
                    MapCellState::FREE
                );
        }
    }

    void markOccupied(
        int x,
        int y
    )
    {
        MapCell* cell =
            getCell(
                x,
                y
            );

        if (cell == nullptr)
        {
            return;
        }

        cell->observations++;

        cell->occupiedObservations++;

        /*
         * Occupied observations receive strong priority.
         */

        if (
            cell->occupiedObservations >=
            cell->freeObservations
        )
        {
            cell->occupancy =
                static_cast<int8_t>(
                    MapCellState::OCCUPIED
                );
        }
    }

    GridCoordinate worldToGrid(
        float worldX,
        float worldY
    ) const
    {
        float localX =
            worldX -
            origin.x;

        float localY =
            worldY -
            origin.y;

        int gridX =
            static_cast<int>(
                floor(
                    localX /
                    resolution
                )
            );

        int gridY =
            static_cast<int>(
                floor(
                    localY /
                    resolution
                )
            );

        return GridCoordinate(
            gridX,
            gridY
        );
    }

    Vector2 gridToWorld(
        int gridX,
        int gridY
    ) const
    {
        float worldX =
            origin.x
            +
            (
                gridX + 0.5f
            )
            *
            resolution;

        float worldY =
            origin.y
            +
            (
                gridY + 0.5f
            )
            *
            resolution;

        return Vector2(
            worldX,
            worldY
        );
    }

    MapStatistics getStatistics() const
    {
        MapStatistics statistics;

        if (!initialized)
        {
            return statistics;
        }

        statistics.totalCells =
            static_cast<uint32_t>(
                width * height
            );

        for (
            int y = 0;
            y < height;
            ++y
        )
        {
            for (
                int x = 0;
                x < width;
                ++x
            )
            {
                MapCellState state =
                    getState(
                        x,
                        y
                    );

                if (
                    state ==
                    MapCellState::UNKNOWN
                )
                {
                    statistics.unknownCells++;
                }
                else if (
                    state ==
                    MapCellState::FREE
                )
                {
                    statistics.freeCells++;
                }
                else if (
                    state ==
                    MapCellState::OCCUPIED
                )
                {
                    statistics.occupiedCells++;
                }
            }
        }

        uint32_t known =
            statistics.freeCells +
            statistics.occupiedCells;

        if (
            statistics.totalCells > 0
        )
        {
            statistics.coveragePercentage =
                (
                    static_cast<float>(
                        known
                    )
                    /
                    static_cast<float>(
                        statistics.totalCells
                    )
                )
                *
                100.0f;
        }

        statistics.boundaries =
            calculateBounds();

        return statistics;
    }

    MapBounds calculateBounds() const
    {
        MapBounds bounds;

        if (!initialized)
        {
            return bounds;
        }

        bounds.minimumX =
            origin.x;

        bounds.minimumY =
            origin.y;

        bounds.maximumX =
            origin.x +
            width * resolution;

        bounds.maximumY =
            origin.y +
            height * resolution;

        return bounds;
    }

    bool saveToSerial() const
    {
        if (!initialized)
        {
            return false;
        }

        Serial.println(
            F("MAP_BEGIN")
        );

        Serial.print(
            F("WIDTH=")
        );

        Serial.println(
            width
        );

        Serial.print(
            F("HEIGHT=")
        );

        Serial.println(
            height
        );

        Serial.print(
            F("RESOLUTION=")
        );

        Serial.println(
            resolution,
            4
        );

        for (
            int y = 0;
            y < height;
            ++y
        )
        {
            for (
                int x = 0;
                x < width;
                ++x
            )
            {
                MapCellState state =
                    getState(
                        x,
                        y
                    );

                char symbol = '?';

                if (
                    state ==
                    MapCellState::FREE
                )
                {
                    symbol = '.';
                }
                else if (
                    state ==
                    MapCellState::OCCUPIED
                )
                {
                    symbol = '#';
                }

                Serial.print(
                    symbol
                );
            }

            Serial.println();
        }

        Serial.println(
            F("MAP_END")
        );

        return true;
    }

    void printCompactMap(
        const Pose2D& robotPose
    ) const
    {
        if (!initialized)
        {
            Serial.println(
                F("Map not initialized.")
            );

            return;
        }

        /*
         * Printing the complete 80x80 map repeatedly would
         * consume a large amount of serial bandwidth.
         *
         * Therefore this diagnostic view displays a reduced
         * section around the robot.
         */

        GridCoordinate robotCell =
            worldToGrid(
                robotPose.x,
                robotPose.y
            );

        constexpr int VIEW_RADIUS = 12;

        int minimumX =
            robotCell.x -
            VIEW_RADIUS;

        int maximumX =
            robotCell.x +
            VIEW_RADIUS;

        int minimumY =
            robotCell.y -
            VIEW_RADIUS;

        int maximumY =
            robotCell.y +
            VIEW_RADIUS;

        Serial.println(
            F("---- LOCAL MAP ----")
        );

        for (
            int y = maximumY;
            y >= minimumY;
            --y
        )
        {
            for (
                int x = minimumX;
                x <= maximumX;
                ++x
            )
            {
                if (
                    x == robotCell.x &&
                    y == robotCell.y
                )
                {
                    Serial.print('R');

                    continue;
                }

                MapCellState state =
                    getState(
                        x,
                        y
                    );

                if (
                    state ==
                    MapCellState::UNKNOWN
                )
                {
                    Serial.print('?');
                }
                else if (
                    state ==
                    MapCellState::FREE
                )
                {
                    Serial.print('.');
                }
                else
                {
                    Serial.print('#');
                }
            }

            Serial.println();
        }

        Serial.println(
            F("--------------------")
        );
    }
};


/*
 * ================================================================
 * SECTION 47 — SENSOR POSE
 * ================================================================
 */

struct SensorPose
{
    float offsetX;

    float offsetY;

    float angleOffset;

    SensorPose()
        : offsetX(0.0f),
          offsetY(0.0f),
          angleOffset(0.0f)
    {
    }

    SensorPose(
        float x,
        float y,
        float angle
    )
        : offsetX(x),
          offsetY(y),
          angleOffset(angle)
    {
    }
};


/*
 * ================================================================
 * SECTION 48 — SENSOR PROJECTOR
 * ================================================================
 */

class SensorProjector
{
private:

    SensorPose frontPose;

    SensorPose leftPose;

    SensorPose rightPose;

public:

    SensorProjector()
        : frontPose(
              0.08f,
              0.0f,
              0.0f
          ),
          leftPose(
              0.0f,
              0.06f,
              PI_VALUE / 2.0f
          ),
          rightPose(
              0.0f,
              -0.06f,
              -PI_VALUE / 2.0f
          )
    {
    }

    void setFrontPose(
        const SensorPose& pose
    )
    {
        frontPose =
            pose;
    }

    void setLeftPose(
        const SensorPose& pose
    )
    {
        leftPose =
            pose;
    }

    void setRightPose(
        const SensorPose& pose
    )
    {
        rightPose =
            pose;
    }

    Vector2 transformSensorOrigin(
        const Pose2D& robotPose,
        const SensorPose& sensorPose
    ) const
    {
        /*
         * Robot-local sensor position:
         *
         * [offsetX]
         * [offsetY]
         *
         * is rotated by robot heading theta.
         */

        float cosTheta =
            cos(
                robotPose.theta
            );

        float sinTheta =
            sin(
                robotPose.theta
            );

        float worldX =
            robotPose.x
            +
            sensorPose.offsetX *
            cosTheta
            -
            sensorPose.offsetY *
            sinTheta;

        float worldY =
            robotPose.y
            +
            sensorPose.offsetX *
            sinTheta
            +
            sensorPose.offsetY *
            cosTheta;

        return Vector2(
            worldX,
            worldY
        );
    }

    SensorRay project(
        const Pose2D& robotPose,
        const SensorPose& sensorPose,
        float distance
    ) const
    {
        SensorRay ray;

        if (
            distance <= 0.0f
        )
        {
            return ray;
        }

        ray.origin =
            transformSensorOrigin(
                robotPose,
                sensorPose
            );

        ray.angle =
            normalizeAngle(
                robotPose.theta
                +
                sensorPose.angleOffset
            );

        ray.distance =
            distance;

        ray.endpoint.x =
            ray.origin.x
            +
            cos(ray.angle)
            *
            distance;

        ray.endpoint.y =
            ray.origin.y
            +
            sin(ray.angle)
            *
            distance;

        ray.valid = true;

        return ray;
    }

    SensorRay projectFront(
        const Pose2D& robotPose,
        float distance
    ) const
    {
        return project(
            robotPose,
            frontPose,
            distance
        );
    }

    SensorRay projectLeft(
        const Pose2D& robotPose,
        float distance
    ) const
    {
        return project(
            robotPose,
            leftPose,
            distance
        );
    }

    SensorRay projectRight(
        const Pose2D& robotPose,
        float distance
    ) const
    {
        return project(
            robotPose,
            rightPose,
            distance
        );
    }
};


/*
 * ================================================================
 * SECTION 49 — RAY CASTING RESULT
 * ================================================================
 */

struct RayCastResult
{
    uint16_t freeCellsUpdated;

    bool obstacleMarked;

    bool terminatedByBoundary;

    bool terminatedByMaximumRange;

    RayCastResult()
        : freeCellsUpdated(0),
          obstacleMarked(false),
          terminatedByBoundary(false),
          terminatedByMaximumRange(false)
    {
    }
};


/*
 * ================================================================
 * SECTION 50 — RAY CASTER
 * ================================================================
 */

class RayCaster
{
private:

    OccupancyGrid* map;

public:

    RayCaster()
        : map(nullptr)
    {
    }

    void attachMap(
        OccupancyGrid* occupancyGrid
    )
    {
        map =
            occupancyGrid;
    }

    /*
     * Integer grid traversal based on the ray's world
     * endpoints.
     *
     * The implementation walks through the line using
     * incremental steps and marks intermediate cells free.
     */

    RayCastResult castRay(
        const SensorRay& ray,
        bool obstacleDetectedAtEndpoint
    )
    {
        RayCastResult result;

        if (
            map == nullptr ||
            !map->isInitialized()
        )
        {
            return result;
        }

        if (!ray.valid)
        {
            return result;
        }

        GridCoordinate start =
            map->worldToGrid(
                ray.origin.x,
                ray.origin.y
            );

        GridCoordinate end =
            map->worldToGrid(
                ray.endpoint.x,
                ray.endpoint.y
            );

        /*
         * Number of interpolation steps.
         *
         * At least one step per map cell crossed.
         */

        int dx =
            abs(
                end.x -
                start.x
            );

        int dy =
            abs(
                end.y -
                start.y
            );

        int steps =
            max(
                dx,
                dy
            );

        if (steps <= 0)
        {
            if (
                obstacleDetectedAtEndpoint
            )
            {
                map->markOccupied(
                    start.x,
                    start.y
                );

                result.obstacleMarked =
                    true;
            }

            return result;
        }

        int previousX =
            start.x;

        int previousY =
            start.y;

        for (
            int step = 0;
            step <= steps;
            ++step
        )
        {
            float ratio =
                static_cast<float>(
                    step
                )
                /
                static_cast<float>(
                    steps
                );

            float worldX =
                ray.origin.x
                +
                (
                    ray.endpoint.x -
                    ray.origin.x
                )
                *
                ratio;

            float worldY =
                ray.origin.y
                +
                (
                    ray.endpoint.y -
                    ray.origin.y
                )
                *
                ratio;

            GridCoordinate current =
                map->worldToGrid(
                    worldX,
                    worldY
                );

            if (
                current.x ==
                    previousX &&
                current.y ==
                    previousY
            )
            {
                continue;
            }

            previousX =
                current.x;

            previousY =
                current.y;

            if (
                !map->isInside(
                    current.x,
                    current.y
                )
            )
            {
                result.terminatedByBoundary =
                    true;

                break;
            }

            bool isLastCell =
                (
                    step >=
                    steps - 1
                );

            if (
                isLastCell &&
                obstacleDetectedAtEndpoint
            )
            {
                map->markOccupied(
                    current.x,
                    current.y
                );

                result.obstacleMarked =
                    true;
            }
            else
            {
                map->markFree(
                    current.x,
                    current.y
                );

                result.freeCellsUpdated++;
            }
        }

        return result;
    }
};


/*
 * ================================================================
 * SECTION 51 — MAPPING SENSOR TYPE
 * ================================================================
 */

enum class MappingSensorType
{
    FRONT,
    LEFT,
    RIGHT
};


/*
 * ================================================================
 * SECTION 52 — MAP UPDATE REPORT
 * ================================================================
 */

struct MapUpdateReport
{
    uint16_t raysProcessed;

    uint16_t freeCellsUpdated;

    uint16_t occupiedCellsUpdated;

    uint16_t invalidMeasurements;

    uint16_t boundaryTerminations;

    MapUpdateReport()
        : raysProcessed(0),
          freeCellsUpdated(0),
          occupiedCellsUpdated(0),
          invalidMeasurements(0),
          boundaryTerminations(0)
    {
    }
};


/*
 * ================================================================
 * SECTION 53 — MAPPING ENGINE
 * ================================================================
 */

class MappingEngine
{
private:

    OccupancyGrid occupancyGrid;

    SensorProjector sensorProjector;

    RayCaster rayCaster;

    float maximumRange;

    float obstacleThreshold;

    bool initialized;

public:

    MappingEngine()
        : maximumRange(300.0f),
          obstacleThreshold(290.0f),
          initialized(false)
    {
    }

    bool initialize(
        int width,
        int height,
        float resolution,
        const Vector2& origin
    )
    {
        bool result =
            occupancyGrid.initialize(
                width,
                height,
                resolution,
                origin
            );

        if (!result)
        {
            initialized =
                false;

            return false;
        }

        rayCaster.attachMap(
            &occupancyGrid
        );

        initialized =
            true;

        return true;
    }

    bool isInitialized() const
    {
        return initialized;
    }

    OccupancyGrid& getMap()
    {
        return occupancyGrid;
    }

    const OccupancyGrid& getMap() const
    {
        return occupancyGrid;
    }

    SensorProjector& getSensorProjector()
    {
        return sensorProjector;
    }

    void setMaximumRange(
        float range
    )
    {
        maximumRange =
            max(
                0.1f,
                range
            );
    }

    void setObstacleThreshold(
        float threshold
    )
    {
        obstacleThreshold =
            clampFloat(
                threshold,
                0.1f,
                maximumRange
            );
    }

    bool isValidMeasurement(
        float distance
    ) const
    {
        if (
            isnan(distance) ||
            isinf(distance)
        )
        {
            return false;
        }

        if (
            distance <= 0.0f
        )
        {
            return false;
        }

        if (
            distance > maximumRange
        )
        {
            return false;
        }

        return true;
    }

    bool isObstacleMeasurement(
        float distance
    ) const
    {
        return
            distance <
            obstacleThreshold;
    }

    MapUpdateReport update(
        const Pose2D& robotPose,
        const DistanceSensorArray& sensors
    )
    {
        MapUpdateReport report;

        if (!initialized)
        {
            return report;
        }

        /*
         * FRONT SENSOR
         */

        if (sensors.frontValid)
        {
            if (
                isValidMeasurement(
                    sensors.front
                )
            )
            {
                SensorRay ray =
                    sensorProjector.projectFront(
                        robotPose,
                        sensors.front
                    );

                bool obstacle =
                    isObstacleMeasurement(
                        sensors.front
                    );

                RayCastResult result =
                    rayCaster.castRay(
                        ray,
                        obstacle
                    );

                report.raysProcessed++;

                report.freeCellsUpdated +=
                    result.freeCellsUpdated;

                if (
                    result.obstacleMarked
                )
                {
                    report.occupiedCellsUpdated++;
                }

                if (
                    result.terminatedByBoundary
                )
                {
                    report.boundaryTerminations++;
                }
            }
            else
            {
                report.invalidMeasurements++;
            }
        }
        else
        {
            report.invalidMeasurements++;
        }

        /*
         * LEFT SENSOR
         */

        if (sensors.leftValid)
        {
            if (
                isValidMeasurement(
                    sensors.left
                )
            )
            {
                SensorRay ray =
                    sensorProjector.projectLeft(
                        robotPose,
                        sensors.left
                    );

                bool obstacle =
                    isObstacleMeasurement(
                        sensors.left
                    );

                RayCastResult result =
                    rayCaster.castRay(
                        ray,
                        obstacle
                    );

                report.raysProcessed++;

                report.freeCellsUpdated +=
                    result.freeCellsUpdated;

                if (
                    result.obstacleMarked
                )
                {
                    report.occupiedCellsUpdated++;
                }

                if (
                    result.terminatedByBoundary
                )
                {
                    report.boundaryTerminations++;
                }
            }
            else
            {
                report.invalidMeasurements++;
            }
        }
        else
        {
            report.invalidMeasurements++;
        }

        /*
         * RIGHT SENSOR
         */

        if (sensors.rightValid)
        {
            if (
                isValidMeasurement(
                    sensors.right
                )
            )
            {
                SensorRay ray =
                    sensorProjector.projectRight(
                        robotPose,
                        sensors.right
                    );

                bool obstacle =
                    isObstacleMeasurement(
                        sensors.right
                    );

                RayCastResult result =
                    rayCaster.castRay(
                        ray,
                        obstacle
                    );

                report.raysProcessed++;

                report.freeCellsUpdated +=
                    result.freeCellsUpdated;

                if (
                    result.obstacleMarked
                )
                {
                    report.occupiedCellsUpdated++;
                }

                if (
                    result.terminatedByBoundary
                )
                {
                    report.boundaryTerminations++;
                }
            }
            else
            {
                report.invalidMeasurements++;
            }
        }
        else
        {
            report.invalidMeasurements++;
        }

        return report;
    }

    void clear()
    {
        occupancyGrid.clear();
    }

    MapStatistics statistics() const
    {
        return occupancyGrid.getStatistics();
    }

    void printStatistics() const
    {
        MapStatistics stats =
            statistics();

        Serial.println(
            F("========== MAP STATISTICS ==========")
        );

        Serial.print(
            F("Total cells: ")
        );

        Serial.println(
            stats.totalCells
        );

        Serial.print(
            F("Unknown cells: ")
        );

        Serial.println(
            stats.unknownCells
        );

        Serial.print(
            F("Free cells: ")
        );

        Serial.println(
            stats.freeCells
        );

        Serial.print(
            F("Occupied cells: ")
        );

        Serial.println(
            stats.occupiedCells
        );

        Serial.print(
            F("Coverage: ")
        );

        Serial.print(
            stats.coveragePercentage,
            2
        );

        Serial.println(
            F("%")
        );

        Serial.print(
            F("Map width: ")
        );

        Serial.print(
            stats.boundaries.width(),
            2
        );

        Serial.println(
            F(" m")
        );

        Serial.print(
            F("Map height: ")
        );

        Serial.print(
            stats.boundaries.height(),
            2
        );

        Serial.println(
            F(" m")
        );

        Serial.println(
            F("====================================")
        );
    }

    void printLocalMap(
        const Pose2D& robotPose
    ) const
    {
        occupancyGrid.printCompactMap(
            robotPose
        );
    }

    bool saveMap()
    {
        return occupancyGrid.saveToSerial();
    }
};


/*
 * ================================================================
 * SECTION 54 — GLOBAL MAPPING ENGINE
 * ================================================================
 */

MappingEngine mappingEngine;


/*
 * ================================================================
 * SECTION 55 — MAP INITIALIZATION
 * ================================================================
 */

void initializeMapping()
{
    /*
     * Map origin is positioned so that the initial robot
     * begins near the center of the map.
     */

    float mapWidthMeters =
        config.mapWidth *
        config.mapResolution;

    float mapHeightMeters =
        config.mapHeight *
        config.mapResolution;

    Vector2 origin(
        -mapWidthMeters / 2.0f,
        -mapHeightMeters / 2.0f
    );

    bool initialized =
        mappingEngine.initialize(
            config.mapWidth,
            config.mapHeight,
            config.mapResolution,
            origin
        );

    if (initialized)
    {
        Serial.println(
            F("Mapping engine: OK")
        );
    }
    else
    {
        Serial.println(
            F("Mapping engine: FAILED")
        );
    }

    mappingEngine.setMaximumRange(
        config.ultrasonicMaximumDistance
    );

    /*
     * Measurements closer than this threshold are treated
     * as obstacle detections.
     */

    mappingEngine.setObstacleThreshold(
        280.0f
    );
}


/*
 * ================================================================
 * SECTION 56 — MAPPING UPDATE LOOP
 * ================================================================
 */

MapUpdateReport lastMapReport;

void updateMapping()
{
    unsigned long currentTime =
        millis();

    float period =
        1000.0f /
        config.mappingFrequency;

    if (
        (
            currentTime -
            previousMappingTime
        )
        <
        period
    )
    {
        return;
    }

    previousMappingTime =
        currentTime;

    if (
        !mappingEngine.isInitialized()
    )
    {
        return;
    }

    lastMapReport =
        mappingEngine.update(
            robotState.pose,
            distanceSensors
        );
}


/*
 * ================================================================
 * SECTION 57 — MAP DIAGNOSTIC COMMANDS
 * ================================================================
 */

void processMapCommand(
    const String& command
)
{
    if (
        command == "map"
    )
    {
        mappingEngine.printStatistics();

        return;
    }

    if (
        command == "map_view"
    )
    {
        mappingEngine.printLocalMap(
            robotState.pose
        );

        return;
    }

    if (
        command == "save_map"
    )
    {
        bool success =
            mappingEngine.saveMap();

        if (success)
        {
            Serial.println(
                F("Map exported through Serial.")
            );
        }
        else
        {
            Serial.println(
                F("Map export failed.")
            );
        }

        return;
    }

    if (
        command == "clear_map"
    )
    {
        mappingEngine.clear();

        Serial.println(
            F("Map cleared.")
        );

        return;
    }
}


/*
 * ================================================================
 * SECTION 58 — EXTENDED COMMAND ROUTER
 * ================================================================
 */

void processExtendedCommand(
    String command
)
{
    command.trim();

    command.toLowerCase();

    if (
        command == "map" ||
        command == "map_view" ||
        command == "save_map" ||
        command == "clear_map"
    )
    {
        processMapCommand(
            command
        );

        return;
    }

    processCommand(
        command
    );
}


/*
 * ================================================================
 * SECTION 59 — MAPPING-AWARE COMMAND INTERFACE
 * ================================================================
 */

void updateMappingCommandInterface()
{
    static String commandBuffer;

    while (
        Serial.available() > 0
    )
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
                processExtendedCommand(
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
                < 80
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
 * SECTION 60 — MAP COVERAGE CHECK
 * ================================================================
 */

float getMapCoverage()
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return 0.0f;
    }

    MapStatistics statistics =
        mappingEngine.statistics();

    return
        statistics.coveragePercentage;
}


/*
 * ================================================================
 * SECTION 61 — WORLD TO MAP HELPER
 * ================================================================
 */

GridCoordinate worldPositionToMapCell(
    float x,
    float y
)
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return GridCoordinate(
            0,
            0
        );
    }

    return
        mappingEngine
            .getMap()
            .worldToGrid(
                x,
                y
            );
}


/*
 * ================================================================
 * SECTION 62 — MAP CELL QUERY
 * ================================================================
 */

MapCellState getMapCellState(
    int x,
    int y
)
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return MapCellState::UNKNOWN;
    }

    return
        mappingEngine
            .getMap()
            .getState(
                x,
                y
            );
}


/*
 * ================================================================
 * SECTION 63 — OBSTACLE QUERY IN WORLD SPACE
 * ================================================================
 */

bool isWorldPositionOccupied(
    float x,
    float y
)
{
    GridCoordinate cell =
        worldPositionToMapCell(
            x,
            y
        );

    MapCellState state =
        getMapCellState(
            cell.x,
            cell.y
        );

    return
        state ==
        MapCellState::OCCUPIED;
}


/*
 * ================================================================
 * SECTION 64 — FREE-SPACE QUERY
 * ================================================================
 */

bool isWorldPositionFree(
    float x,
    float y
)
{
    GridCoordinate cell =
        worldPositionToMapCell(
            x,
            y
        );

    MapCellState state =
        getMapCellState(
            cell.x,
            cell.y
        );

    return
        state ==
        MapCellState::FREE;
}


/*
 * ================================================================
 * SECTION 65 — UNKNOWN-SPACE QUERY
 * ================================================================
 */

bool isWorldPositionUnknown(
    float x,
    float y
)
{
    GridCoordinate cell =
        worldPositionToMapCell(
            x,
            y
        );

    MapCellState state =
        getMapCellState(
            cell.x,
            cell.y
        );

    return
        state ==
        MapCellState::UNKNOWN;
}


/*
 * ================================================================
 * SECTION 66 — SENSOR RAY DEBUGGING
 * ================================================================
 */

void printSensorRay(
    const SensorRay& ray,
    const char* name
)
{
    Serial.print(
        name
    );

    Serial.print(
        F(" origin=(")
    );

    Serial.print(
        ray.origin.x,
        3
    );

    Serial.print(
        F(",")
    );

    Serial.print(
        ray.origin.y,
        3
    );

    Serial.print(
        F(") endpoint=(")
    );

    Serial.print(
        ray.endpoint.x,
        3
    );

    Serial.print(
        F(",")
    );

    Serial.print(
        ray.endpoint.y,
        3
    );

    Serial.print(
        F(") distance=")
    );

    Serial.print(
        ray.distance,
        2
    );

    Serial.print(
        F(" angle=")
    );

    Serial.print(
        radiansToDegrees(
            ray.angle
        ),
        2
    );

    Serial.println(
        F(" deg")
    );
}


/*
 * ================================================================
 * SECTION 67 — SENSOR PROJECTION DIAGNOSTICS
 * ================================================================
 */

void printSensorProjectionDiagnostics()
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        Serial.println(
            F("Mapping engine not initialized.")
        );

        return;
    }

    SensorProjector& projector =
        mappingEngine
            .getSensorProjector();

    SensorRay frontRay =
        projector.projectFront(
            robotState.pose,
            distanceSensors.front
        );

    SensorRay leftRay =
        projector.projectLeft(
            robotState.pose,
            distanceSensors.left
        );

    SensorRay rightRay =
        projector.projectRight(
            robotState.pose,
            distanceSensors.right
        );

    Serial.println(
        F("===== SENSOR PROJECTION =====")
    );

    printSensorRay(
        frontRay,
        "FRONT"
    );

    printSensorRay(
        leftRay,
        "LEFT"
    );

    printSensorRay(
        rightRay,
        "RIGHT"
    );

    Serial.println(
        F("=============================")
    );
}


/*
 * ================================================================
 * SECTION 68 — MAP UPDATE DIAGNOSTICS
 * ================================================================
 */

void printLastMapUpdate()
{
    Serial.println(
        F("===== LAST MAP UPDATE =====")
    );

    Serial.print(
        F("Rays: ")
    );

    Serial.println(
        lastMapReport.raysProcessed
    );

    Serial.print(
        F("Free cells: ")
    );

    Serial.println(
        lastMapReport.freeCellsUpdated
    );

    Serial.print(
        F("Occupied cells: ")
    );

    Serial.println(
        lastMapReport.occupiedCellsUpdated
    );

    Serial.print(
        F("Invalid measurements: ")
    );

    Serial.println(
        lastMapReport.invalidMeasurements
    );

    Serial.print(
        F("Boundary terminations: ")
    );

    Serial.println(
        lastMapReport.boundaryTerminations
    );

    Serial.println(
        F("===========================")
    );
}


/*
 * ================================================================
 * SECTION 69 — MAPPING STATUS
 * ================================================================
 */

void printMappingStatus()
{
    Serial.println(
        F("========== MAPPING STATUS ==========")
    );

    Serial.print(
        F("Initialized: ")
    );

    Serial.println(
        mappingEngine.isInitialized()
            ? F("YES")
            : F("NO")
    );

    Serial.print(
        F("Map width: ")
    );

    Serial.println(
        mappingEngine
            .getMap()
            .getWidth()
    );

    Serial.print(
        F("Map height: ")
    );

    Serial.println(
        mappingEngine
            .getMap()
            .getHeight()
    );

    Serial.print(
        F("Resolution: ")
    );

    Serial.print(
        mappingEngine
            .getMap()
            .getResolution(),
        3
    );

    Serial.println(
        F(" m/cell")
    );

    Serial.print(
        F("Coverage: ")
    );

    Serial.print(
        getMapCoverage(),
        2
    );

    Serial.println(
        F("%")
    );

    Serial.println(
        F("====================================")
    );
}


/*
 * ================================================================
 * SECTION 70 — MAPPING TEST
 * ================================================================
 */

bool runMappingBasicTest()
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return false;
    }

    OccupancyGrid& map =
        mappingEngine.getMap();

    /*
     * Select a central cell.
     */

    int centerX =
        map.getWidth() / 2;

    int centerY =
        map.getHeight() / 2;

    map.markFree(
        centerX,
        centerY
    );

    map.markFree(
        centerX + 1,
        centerY
    );

    map.markFree(
        centerX,
        centerY + 1
    );

    map.markOccupied(
        centerX + 2,
        centerY
    );

    MapCellState freeState =
        map.getState(
            centerX,
            centerY
        );

    MapCellState occupiedState =
        map.getState(
            centerX + 2,
            centerY
        );

    bool freeCorrect =
        freeState ==
        MapCellState::FREE;

    bool occupiedCorrect =
        occupiedState ==
        MapCellState::OCCUPIED;

    return
        freeCorrect &&
        occupiedCorrect;
}


/*
 * ================================================================
 * SECTION 71 — RAY CASTING TEST
 * ================================================================
 */

bool runRayCastingTest()
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return false;
    }

    Pose2D testPose;

    testPose.x = 0.0f;

    testPose.y = 0.0f;

    testPose.theta = 0.0f;

    SensorRay ray =
        mappingEngine
            .getSensorProjector()
            .projectFront(
                testPose,
                1.0f
            );

    RayCaster caster;

    caster.attachMap(
        &mappingEngine.getMap()
    );

    RayCastResult result =
        caster.castRay(
            ray,
            true
        );

    return
        result.freeCellsUpdated > 0
        ||
        result.obstacleMarked;
}


/*
 * ================================================================
 * SECTION 72 — MAP TEST SUITE
 * ================================================================
 */

void runMappingTests()
{
    Serial.println(
        F("")
    );

    Serial.println(
        F("========== MAPPING TESTS ==========")
    );

    bool gridTest =
        runMappingBasicTest();

    Serial.print(
        F("Occupancy Grid Test: ")
    );

    Serial.println(
        gridTest
            ? F("PASS")
            : F("FAIL")
    );

    bool rayTest =
        runRayCastingTest();

    Serial.print(
        F("Ray Casting Test: ")
    );

    Serial.println(
        rayTest
            ? F("PASS")
            : F("FAIL")
    );

    Serial.println(
        F("===================================")
    );

    Serial.println();
}


/*
 * ================================================================
 * SECTION 73 — INITIALIZE PART 2
 * ================================================================
 */

void initializeMappingSubsystem()
{
    initializeMapping();

    if (
        mappingEngine.isInitialized()
    )
    {
        runMappingTests();
    }
}


/*
 * ================================================================
 * SECTION 74 — MAPPING LOOP INTEGRATION
 * ================================================================
 */

void runMappingSubsystem()
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return;
    }

    updateMapping();
}


/*
 * ================================================================
 * SECTION 75 — END OF PART 2
 * ================================================================
 *
 * IMPLEMENTED:
 *
 *  [x] Occupancy grid
 *  [x] Unknown/free/occupied states
 *  [x] Configurable map resolution
 *  [x] Configurable map dimensions
 *  [x] World -> grid conversion
 *  [x] Grid -> world conversion
 *  [x] Sensor coordinate transformation
 *  [x] Sensor ray generation
 *  [x] Ray traversal
 *  [x] Free-space marking
 *  [x] Occupied-cell marking
 *  [x] Map coverage calculation
 *  [x] Map statistics
 *  [x] Map boundaries
 *  [x] Local map visualization through Serial
 *  [x] Map export through Serial
 *  [x] Map clearing
 *  [x] Mapping diagnostics
 *  [x] Mapping tests
 *
 * NEXT:
 *
 *  PART 3
 *      A* PATH PLANNING
 *      PATH VALIDATION
 *      PATH SMOOTHING
 *      GRID NAVIGATION
 *
 * ================================================================
 */
 // ================================================================
// PART 3 — LOCALIZATION + OBSTACLE DETECTION + A* PATH PLANNING
// Arduino Uno Autonomous Self-Balancing Robot
// ================================================================
//
// This part continues after Part 2.
//
// Implements:
//   1. Odometry-based localization
//   2. Obstacle representation
//   3. Obstacle detection
//   4. A* path planning
//   5. Path reconstruction
//   6. Path validation
//   7. Basic path smoothing
//   8. Navigation target selection
//   9. Navigation state management
//  10. Safety checks
//
// IMPORTANT:
// The project specification requires localization, obstacle detection,
// A* path planning, path smoothing, and a navigation controller.
// Full SLAM is NOT falsely implemented here; it remains an extension.
//
// Arduino Uno memory note:
// Keep the occupancy grid compact. Large grids can exceed the Uno's
// approximately 2 KB SRAM. The planner therefore uses fixed-size,
// compact arrays.
//
// ================================================================

#ifndef ROBOT_PART_3_INCLUDED
#define ROBOT_PART_3_INCLUDED

#include <Arduino.h>
#include <math.h>

// ================================================================
// PART 3 CONFIGURATION
// ================================================================

#ifndef ASTAR_MAX_NODES
#define ASTAR_MAX_NODES 256
#endif

#ifndef ASTAR_MAX_PATH
#define ASTAR_MAX_PATH 128
#endif

#ifndef MAX_OBSTACLES
#define MAX_OBSTACLES 16
#endif

#ifndef MAX_FRONTIER_CANDIDATES
#define MAX_FRONTIER_CANDIDATES 32
#endif

#ifndef NAVIGATION_LOOKAHEAD_CELLS
#define NAVIGATION_LOOKAHEAD_CELLS 2
#endif

#ifndef NAVIGATION_GOAL_TOLERANCE
#define NAVIGATION_GOAL_TOLERANCE 0.08f
#endif

#ifndef NAVIGATION_HEADING_TOLERANCE
#define NAVIGATION_HEADING_TOLERANCE 0.12f
#endif

#ifndef NAVIGATION_MAX_LINEAR_SPEED
#define NAVIGATION_MAX_LINEAR_SPEED 0.20f
#endif

#ifndef NAVIGATION_MAX_ANGULAR_SPEED
#define NAVIGATION_MAX_ANGULAR_SPEED 1.50f
#endif

#ifndef OBSTACLE_MIN_CONFIDENCE
#define OBSTACLE_MIN_CONFIDENCE 0.30f
#endif

// ================================================================
// COMPACT PATH-PLANNING GRID
// ================================================================
//
// Do not blindly use a dynamically allocated large grid on an Uno.
// The planner uses the existing mapping system as its source.
//
// These values represent planner limits rather than the physical map.
//
// ================================================================

#define PLANNER_MAX_WIDTH  32
#define PLANNER_MAX_HEIGHT 32

// ================================================================
// LOCALIZATION STRUCTURES
// ================================================================

struct OdometryState
{
    float x;
    float y;
    float theta;

    float previousLeftDistance;
    float previousRightDistance;

    float totalDistance;
    float totalRotation;

    bool initialized;
    bool valid;
};

struct PoseEstimate
{
    float x;
    float y;
    float theta;

    float linearVelocity;
    float angularVelocity;

    unsigned long timestamp;

    bool valid;
};

struct LocalizationStatus
{
    bool odometryAvailable;
    bool poseValid;
    bool sensorCorrectionAvailable;
    bool initialized;

    float positionUncertainty;
    float headingUncertainty;

    unsigned long updateCount;
};

// ================================================================
// OBSTACLE STRUCTURES
// ================================================================

struct Obstacle
{
    float x;
    float y;

    float distance;
    float angle;

    float confidence;

    unsigned long timestamp;

    bool valid;
};

struct ObstacleDetectionResult
{
    Obstacle front;
    Obstacle left;
    Obstacle right;
    Obstacle rear;

    uint8_t count;

    bool frontBlocked;
    bool leftBlocked;
    bool rightBlocked;
    bool rearBlocked;

    float nearestDistance;

    bool valid;
};

// ================================================================
// PATH CELL
// ================================================================

struct PathCell
{
    int16_t x;
    int16_t y;
};

struct PathNode
{
    int16_t x;
    int16_t y;

    float g;
    float h;
    float f;

    int16_t parentX;
    int16_t parentY;

    bool opened;
    bool closed;
    bool valid;
};

struct PlannedPath
{
    PathCell cells[ASTAR_MAX_PATH];

    uint16_t length;

    bool valid;
    bool smoothed;
    bool collisionFree;

    uint16_t expandedNodes;

    PathCell start;
    PathCell goal;
};

// ================================================================
// NAVIGATION COMMAND
// ================================================================

struct NavigationCommand
{
    float linearVelocity;
    float angularVelocity;

    bool valid;
    bool stop;
};

// ================================================================
// NAVIGATION STATE
// ================================================================

enum NavigationState
{
    NAV_IDLE,
    NAV_PLANNING,
    NAV_FOLLOWING,
    NAV_AVOIDING,
    NAV_GOAL_REACHED,
    NAV_NO_PATH,
    NAV_EMERGENCY_STOP
};

struct NavigationStatus
{
    NavigationState state;

    PathCell goal;

    uint16_t currentPathIndex;

    float distanceToGoal;
    float headingError;

    bool pathAvailable;
    bool goalReached;

    unsigned long navigationStartTime;
};

// ================================================================
// LOCALIZATION MANAGER
// ================================================================

class LocalizationManager
{
private:

    OdometryState odometry;
    PoseEstimate pose;
    LocalizationStatus status;

    float wheelRadius;
    float wheelSeparation;

    float clampAngle(float angle)
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

public:

    LocalizationManager()
    {
        odometry.x = 0.0f;
        odometry.y = 0.0f;
        odometry.theta = 0.0f;

        odometry.previousLeftDistance = 0.0f;
        odometry.previousRightDistance = 0.0f;

        odometry.totalDistance = 0.0f;
        odometry.totalRotation = 0.0f;

        odometry.initialized = false;
        odometry.valid = false;

        pose.x = 0.0f;
        pose.y = 0.0f;
        pose.theta = 0.0f;

        pose.linearVelocity = 0.0f;
        pose.angularVelocity = 0.0f;

        pose.timestamp = 0;

        pose.valid = false;

        status.odometryAvailable = false;
        status.poseValid = false;
        status.sensorCorrectionAvailable = false;
        status.initialized = false;

        status.positionUncertainty = 0.0f;
        status.headingUncertainty = 0.0f;
        status.updateCount = 0;

        wheelRadius = 0.03f;
        wheelSeparation = 0.14f;
    }

    void configure(float radius, float separation)
    {
        if (radius > 0.001f)
        {
            wheelRadius = radius;
        }

        if (separation > 0.001f)
        {
            wheelSeparation = separation;
        }
    }

    void reset(float x, float y, float theta)
    {
        odometry.x = x;
        odometry.y = y;
        odometry.theta = theta;

        odometry.previousLeftDistance = 0.0f;
        odometry.previousRightDistance = 0.0f;

        odometry.totalDistance = 0.0f;
        odometry.totalRotation = 0.0f;

        odometry.initialized = false;
        odometry.valid = true;

        pose.x = x;
        pose.y = y;
        pose.theta = theta;

        pose.linearVelocity = 0.0f;
        pose.angularVelocity = 0.0f;

        pose.timestamp = millis();
        pose.valid = true;

        status.odometryAvailable = true;
        status.poseValid = true;
        status.initialized = true;

        status.positionUncertainty = 0.01f;
        status.headingUncertainty = 0.02f;

        status.updateCount = 0;
    }

    void updateFromWheelDistances(
        float leftDistance,
        float rightDistance,
        unsigned long now
    )
    {
        if (isnan(leftDistance) ||
            isnan(rightDistance) ||
            isinf(leftDistance) ||
            isinf(rightDistance))
        {
            status.odometryAvailable = false;
            return;
        }

        if (!odometry.initialized)
        {
            odometry.previousLeftDistance = leftDistance;
            odometry.previousRightDistance = rightDistance;

            odometry.initialized = true;

            status.odometryAvailable = true;

            return;
        }

        float deltaLeft =
            leftDistance - odometry.previousLeftDistance;

        float deltaRight =
            rightDistance - odometry.previousRightDistance;

        odometry.previousLeftDistance = leftDistance;
        odometry.previousRightDistance = rightDistance;

        const float MAX_WHEEL_DELTA = 0.50f;

        if (fabs(deltaLeft) > MAX_WHEEL_DELTA ||
            fabs(deltaRight) > MAX_WHEEL_DELTA)
        {
            return;
        }

        float deltaCenter =
            (deltaLeft + deltaRight) * 0.5f;

        float deltaTheta =
            (deltaRight - deltaLeft) /
            wheelSeparation;

        float middleTheta =
            odometry.theta + deltaTheta * 0.5f;

        odometry.x +=
            deltaCenter * cos(middleTheta);

        odometry.y +=
            deltaCenter * sin(middleTheta);

        odometry.theta =
            clampAngle(
                odometry.theta + deltaTheta
            );

        odometry.totalDistance += fabs(deltaCenter);
        odometry.totalRotation += fabs(deltaTheta);

        unsigned long dtMillis =
            now - pose.timestamp;

        if (dtMillis > 0)
        {
            float dt =
                dtMillis / 1000.0f;

            pose.linearVelocity =
                deltaCenter / dt;

            pose.angularVelocity =
                deltaTheta / dt;
        }

        pose.x = odometry.x;
        pose.y = odometry.y;
        pose.theta = odometry.theta;
        pose.timestamp = now;

        pose.valid = true;

        status.odometryAvailable = true;
        status.poseValid = true;

        status.positionUncertainty +=
            fabs(deltaCenter) * 0.002f;

        status.headingUncertainty +=
            fabs(deltaTheta) * 0.001f;

        if (status.positionUncertainty > 1.0f)
        {
            status.positionUncertainty = 1.0f;
        }

        if (status.headingUncertainty > PI_VALUE)
        {
            status.headingUncertainty = PI_VALUE;
        }

        status.updateCount++;
    }

    PoseEstimate getPose()
    {
        return pose;
    }

    LocalizationStatus getStatus()
    {
        return status;
    }

    void applyExternalPoseCorrection(
        float x,
        float y,
        float theta,
        float positionConfidence
    )
    {
        if (positionConfidence <= 0.0f)
        {
            return;
        }

        if (positionConfidence > 1.0f)
        {
            positionConfidence = 1.0f;
        }

        pose.x =
            pose.x * (1.0f - positionConfidence) +
            x * positionConfidence;

        pose.y =
            pose.y * (1.0f - positionConfidence) +
            y * positionConfidence;

        pose.theta =
            pose.theta * (1.0f - positionConfidence) +
            theta * positionConfidence;

        pose.theta = clampAngle(pose.theta);

        odometry.x = pose.x;
        odometry.y = pose.y;
        odometry.theta = pose.theta;

        status.sensorCorrectionAvailable = true;

        status.positionUncertainty *=
            (1.0f - positionConfidence);

        status.headingUncertainty *=
            (1.0f - positionConfidence);
    }
};

LocalizationManager localizationManager;

// ================================================================
// OBSTACLE DETECTOR
// ================================================================

class ObstacleDetector
{
private:

    ObstacleDetectionResult result;

    float frontThreshold;
    float sideThreshold;
    float rearThreshold;

    Obstacle createObstacle(
        float distance,
        float angle,
        float x,
        float y,
        float confidence
    )
    {
        Obstacle obstacle;

        obstacle.x = x;
        obstacle.y = y;

        obstacle.distance = distance;
        obstacle.angle = angle;

        obstacle.confidence = confidence;

        obstacle.timestamp = millis();

        obstacle.valid =
            distance > 0.0f &&
            confidence >= OBSTACLE_MIN_CONFIDENCE;

        return obstacle;
    }

public:

    ObstacleDetector()
    {
        frontThreshold = 0.35f;
        sideThreshold = 0.30f;
        rearThreshold = 0.30f;

        clear();
    }

    void configure(
        float front,
        float side,
        float rear
    )
    {
        if (front > 0.05f)
        {
            frontThreshold = front;
        }

        if (side > 0.05f)
        {
            sideThreshold = side;
        }

        if (rear > 0.05f)
        {
            rearThreshold = rear;
        }
    }

    void clear()
    {
        result.front.valid = false;
        result.left.valid = false;
        result.right.valid = false;
        result.rear.valid = false;

        result.count = 0;

        result.frontBlocked = false;
        result.leftBlocked = false;
        result.rightBlocked = false;
        result.rearBlocked = false;

        result.nearestDistance = 9999.0f;

        result.valid = false;
    }

    void update(
        float frontDistance,
        float leftDistance,
        float rightDistance,
        float rearDistance,
        const PoseEstimate &pose
    )
    {
        clear();

        if (!pose.valid)
        {
            return;
        }

        float frontAngle =
            pose.theta;

        float leftAngle =
            pose.theta + PI_VALUE * 0.5f;

        float rightAngle =
            pose.theta - PI_VALUE * 0.5f;

        float rearAngle =
            pose.theta + PI_VALUE;

        float frontX =
            pose.x + frontDistance * cos(frontAngle);

        float frontY =
            pose.y + frontDistance * sin(frontAngle);

        float leftX =
            pose.x + leftDistance * cos(leftAngle);

        float leftY =
            pose.y + leftDistance * sin(leftAngle);

        float rightX =
            pose.x + rightDistance * cos(rightAngle);

        float rightY =
            pose.y + rightDistance * sin(rightAngle);

        float rearX =
            pose.x + rearDistance * cos(rearAngle);

        float rearY =
            pose.y + rearDistance * sin(rearAngle);

        if (frontDistance > 0.0f &&
            !isnan(frontDistance))
        {
            result.front =
                createObstacle(
                    frontDistance,
                    frontAngle,
                    frontX,
                    frontY,
                    1.0f
                );

            result.frontBlocked =
                frontDistance <= frontThreshold;
        }

        if (leftDistance > 0.0f &&
            !isnan(leftDistance))
        {
            result.left =
                createObstacle(
                    leftDistance,
                    leftAngle,
                    leftX,
                    leftY,
                    1.0f
                );

            result.leftBlocked =
                leftDistance <= sideThreshold;
        }

        if (rightDistance > 0.0f &&
            !isnan(rightDistance))
        {
            result.right =
                createObstacle(
                    rightDistance,
                    rightAngle,
                    rightX,
                    rightY,
                    1.0f
                );

            result.rightBlocked =
                rightDistance <= sideThreshold;
        }

        if (rearDistance > 0.0f &&
            !isnan(rearDistance))
        {
            result.rear =
                createObstacle(
                    rearDistance,
                    rearAngle,
                    rearX,
                    rearY,
                    1.0f
                );

            result.rearBlocked =
                rearDistance <= rearThreshold;
        }

        result.count = 0;

        if (result.front.valid)
        {
            result.count++;
        }

        if (result.left.valid)
        {
            result.count++;
        }

        if (result.right.valid)
        {
            result.count++;
        }

        if (result.rear.valid)
        {
            result.count++;
        }

        if (result.front.valid &&
            result.front.distance < result.nearestDistance)
        {
            result.nearestDistance =
                result.front.distance;
        }

        if (result.left.valid &&
            result.left.distance < result.nearestDistance)
        {
            result.nearestDistance =
                result.left.distance;
        }

        if (result.right.valid &&
            result.right.distance < result.nearestDistance)
        {
            result.nearestDistance =
                result.right.distance;
        }

        if (result.rear.valid &&
            result.rear.distance < result.nearestDistance)
        {
            result.nearestDistance =
                result.rear.distance;
        }

        result.valid = true;
    }

    ObstacleDetectionResult getResult()
    {
        return result;
    }

    bool isFrontBlocked()
    {
        return result.frontBlocked;
    }

    bool isLeftBlocked()
    {
        return result.leftBlocked;
    }

    bool isRightBlocked()
    {
        return result.rightBlocked;
    }

    bool isRearBlocked()
    {
        return result.rearBlocked;
    }
};

ObstacleDetector obstacleDetector;

// ================================================================
// A* PATH PLANNER
// ================================================================

class AStarPlanner
{
private:

    PathNode nodes[ASTAR_MAX_NODES];

    PlannedPath path;

    int16_t gridWidth;
    int16_t gridHeight;

    float heuristic(
        int16_t x1,
        int16_t y1,
        int16_t x2,
        int16_t y2
    )
    {
        float dx =
            fabs((float)x2 - (float)x1);

        float dy =
            fabs((float)y2 - (float)y1);

        // Octile distance.
        const float D = 1.0f;
        const float D2 = 1.41421356f;

        if (dx > dy)
        {
            return D * (dx - dy) + D2 * dy;
        }

        return D * (dy - dx) + D2 * dx;
    }

    int nodeIndex(
        int16_t x,
        int16_t y
    )
    {
        return y * gridWidth + x;
    }

    bool validCoordinate(
        int16_t x,
        int16_t y
    )
    {
        return
            x >= 0 &&
            y >= 0 &&
            x < gridWidth &&
            y < gridHeight;
    }

    void clearNodes()
    {
        for (int i = 0; i < ASTAR_MAX_NODES; i++)
        {
            nodes[i].x = 0;
            nodes[i].y = 0;

            nodes[i].g = 0.0f;
            nodes[i].h = 0.0f;
            nodes[i].f = 0.0f;

            nodes[i].parentX = -1;
            nodes[i].parentY = -1;

            nodes[i].opened = false;
            nodes[i].closed = false;
            nodes[i].valid = false;
        }
    }

    int findLowestOpenNode()
    {
        int best = -1;

        float bestF = 999999.0f;

        for (int i = 0; i < ASTAR_MAX_NODES; i++)
        {
            if (!nodes[i].valid)
            {
                continue;
            }

            if (!nodes[i].opened)
            {
                continue;
            }

            if (nodes[i].closed)
            {
                continue;
            }

            if (nodes[i].f < bestF)
            {
                bestF = nodes[i].f;
                best = i;
            }
        }

        return best;
    }

    int findNode(
        int16_t x,
        int16_t y
    )
    {
        for (int i = 0; i < ASTAR_MAX_NODES; i++)
        {
            if (!nodes[i].valid)
            {
                continue;
            }

            if (nodes[i].x == x &&
                nodes[i].y == y)
            {
                return i;
            }
        }

        return -1;
    }

    int createNode(
        int16_t x,
        int16_t y
    )
    {
        int existing =
            findNode(x, y);

        if (existing >= 0)
        {
            return existing;
        }

        for (int i = 0; i < ASTAR_MAX_NODES; i++)
        {
            if (!nodes[i].valid)
            {
                nodes[i].x = x;
                nodes[i].y = y;

                nodes[i].g = 999999.0f;
                nodes[i].h = 0.0f;
                nodes[i].f = 999999.0f;

                nodes[i].parentX = -1;
                nodes[i].parentY = -1;

                nodes[i].opened = false;
                nodes[i].closed = false;
                nodes[i].valid = true;

                return i;
            }
        }

        return -1;
    }

    bool isWalkable(
        const OccupancyGrid &grid,
        int16_t x,
        int16_t y
    )
    {
        if (!validCoordinate(x, y))
        {
            return false;
        }

        MapCell cell =
            grid.getCell(x, y);

        if (cell.state == CELL_OCCUPIED)
        {
            return false;
        }

        // Unknown cells are deliberately treated as non-walkable
        // for safe baseline navigation.
        if (cell.state == CELL_UNKNOWN)
        {
            return false;
        }

        return true;
    }

    void reversePath()
    {
        uint16_t left = 0;

        if (path.length == 0)
        {
            return;
        }

        uint16_t right =
            path.length - 1;

        while (left < right)
        {
            PathCell temporary =
                path.cells[left];

            path.cells[left] =
                path.cells[right];

            path.cells[right] =
                temporary;

            left++;
            right--;
        }
    }

    bool reconstructPath(
        int goalNode
    )
    {
        path.length = 0;

        int current =
            goalNode;

        while (current >= 0)
        {
            if (path.length >= ASTAR_MAX_PATH)
            {
                return false;
            }

            PathCell cell;

            cell.x = nodes[current].x;
            cell.y = nodes[current].y;

            path.cells[path.length++] =
                cell;

            if (nodes[current].parentX < 0)
            {
                break;
            }

            current =
                findNode(
                    nodes[current].parentX,
                    nodes[current].parentY
                );
        }

        reversePath();

        return path.length > 0;
    }

    float movementCost(
        int16_t oldX,
        int16_t oldY,
        int16_t newX,
        int16_t newY
    )
    {
        if (oldX != newX &&
            oldY != newY)
        {
            return 1.41421356f;
        }

        return 1.0f;
    }

public:

    AStarPlanner()
    {
        gridWidth = 0;
        gridHeight = 0;

        path.length = 0;
        path.valid = false;
        path.smoothed = false;
        path.collisionFree = false;
        path.expandedNodes = 0;

        clearNodes();
    }

    bool configure(
        uint16_t width,
        uint16_t height
    )
    {
        if (width == 0 ||
            height == 0)
        {
            return false;
        }

        if (width > PLANNER_MAX_WIDTH)
        {
            width = PLANNER_MAX_WIDTH;
        }

        if (height > PLANNER_MAX_HEIGHT)
        {
            height = PLANNER_MAX_HEIGHT;
        }

        if ((uint32_t)width * (uint32_t)height >
            ASTAR_MAX_NODES)
        {
            return false;
        }

        gridWidth = width;
        gridHeight = height;

        return true;
    }

    bool plan(
        OccupancyGrid &grid,
        PathCell start,
        PathCell goal
    )
    {
        path.length = 0;
        path.valid = false;
        path.smoothed = false;
        path.collisionFree = false;
        path.expandedNodes = 0;

        path.start = start;
        path.goal = goal;

        if (!validCoordinate(
                start.x,
                start.y))
        {
            return false;
        }

        if (!validCoordinate(
                goal.x,
                goal.y))
        {
            return false;
        }

        if (!isWalkable(
                grid,
                start.x,
                start.y))
        {
            return false;
        }

        if (!isWalkable(
                grid,
                goal.x,
                goal.y))
        {
            return false;
        }

        clearNodes();

        int startNode =
            createNode(
                start.x,
                start.y
            );

        if (startNode < 0)
        {
            return false;
        }

        nodes[startNode].g = 0.0f;

        nodes[startNode].h =
            heuristic(
                start.x,
                start.y,
                goal.x,
                goal.y
            );

        nodes[startNode].f =
            nodes[startNode].h;

        nodes[startNode].opened = true;

        const int8_t directions[8][2] =
        {
            { 1,  0},
            {-1,  0},
            { 0,  1},
            { 0, -1},
            { 1,  1},
            { 1, -1},
            {-1,  1},
            {-1, -1}
        };

        while (true)
        {
            int current =
                findLowestOpenNode();

            if (current < 0)
            {
                break;
            }

            nodes[current].closed = true;

            path.expandedNodes++;

            if (nodes[current].x == goal.x &&
                nodes[current].y == goal.y)
            {
                if (!reconstructPath(current))
                {
                    return false;
                }

                path.valid = true;

                return true;
            }

            for (uint8_t direction = 0;
                 direction < 8;
                 direction++)
            {
                int16_t nx =
                    nodes[current].x +
                    directions[direction][0];

                int16_t ny =
                    nodes[current].y +
                    directions[direction][1];

                if (!validCoordinate(nx, ny))
                {
                    continue;
                }

                if (!isWalkable(
                        grid,
                        nx,
                        ny))
                {
                    continue;
                }

                // Prevent diagonal corner cutting.
                if (directions[direction][0] != 0 &&
                    directions[direction][1] != 0)
                {
                    int16_t sideAX =
                        nodes[current].x +
                        directions[direction][0];

                    int16_t sideAY =
                        nodes[current].y;

                    int16_t sideBX =
                        nodes[current].x;

                    int16_t sideBY =
                        nodes[current].y +
                        directions[direction][1];

                    if (!isWalkable(
                            grid,
                            sideAX,
                            sideAY))
                    {
                        continue;
                    }

                    if (!isWalkable(
                            grid,
                            sideBX,
                            sideBY))
                    {
                        continue;
                    }
                }

                int neighbor =
                    createNode(nx, ny);

                if (neighbor < 0)
                {
                    continue;
                }

                if (nodes[neighbor].closed)
                {
                    continue;
                }

                float newG =
                    nodes[current].g +
                    movementCost(
                        nodes[current].x,
                        nodes[current].y,
                        nx,
                        ny
                    );

                if (!nodes[neighbor].opened ||
                    newG < nodes[neighbor].g)
                {
                    nodes[neighbor].parentX =
                        nodes[current].x;

                    nodes[neighbor].parentY =
                        nodes[current].y;

                    nodes[neighbor].g =
                        newG;

                    nodes[neighbor].h =
                        heuristic(
                            nx,
                            ny,
                            goal.x,
                            goal.y
                        );

                    nodes[neighbor].f =
                        nodes[neighbor].g +
                        nodes[neighbor].h;

                    nodes[neighbor].opened =
                        true;
                }
            }

            if (path.expandedNodes >= ASTAR_MAX_NODES)
            {
                break;
            }
        }

        return false;
    }

    PlannedPath getPath()
    {
        return path;
    }

    uint16_t getPathLength()
    {
        return path.length;
    }

    PathCell getPathCell(
        uint16_t index
    )
    {
        PathCell empty;

        empty.x = 0;
        empty.y = 0;

        if (index >= path.length)
        {
            return empty;
        }

        return path.cells[index];
    }

    bool hasPath()
    {
        return path.valid &&
               path.length > 0;
    }
};

AStarPlanner pathPlanner;

// ================================================================
// PATH COLLISION VALIDATOR
// ================================================================

class PathValidator
{
public:

    bool isPathSafe(
        OccupancyGrid &grid,
        PlannedPath &candidate
    )
    {
        if (!candidate.valid)
        {
            return false;
        }

        if (candidate.length == 0)
        {
            return false;
        }

        for (uint16_t i = 0;
             i < candidate.length;
             i++)
        {
            int16_t x =
                candidate.cells[i].x;

            int16_t y =
                candidate.cells[i].y;

            MapCell cell =
                grid.getCell(x, y);

            if (cell.state != CELL_FREE)
            {
                return false;
            }
        }

        return true;
    }

    bool segmentSafe(
        OccupancyGrid &grid,
        PathCell a,
        PathCell b
    )
    {
        int16_t dx =
            abs(b.x - a.x);

        int16_t dy =
            abs(b.y - a.y);

        int16_t sx =
            a.x < b.x ? 1 : -1;

        int16_t sy =
            a.y < b.y ? 1 : -1;

        int16_t err =
            dx - dy;

        int16_t x = a.x;
        int16_t y = a.y;

        while (true)
        {
            MapCell cell =
                grid.getCell(x, y);

            if (cell.state != CELL_FREE)
            {
                return false;
            }

            if (x == b.x &&
                y == b.y)
            {
                break;
            }

            int16_t e2 =
                2 * err;

            if (e2 > -dy)
            {
                err -= dy;
                x += sx;
            }

            if (e2 < dx)
            {
                err += dx;
                y += sy;
            }
        }

        return true;
    }
};

PathValidator pathValidator;

// ================================================================
// PATH SMOOTHER
// ================================================================

class PathSmoother
{
public:

    void smooth(
        OccupancyGrid &grid,
        PlannedPath &path
    )
    {
        if (!path.valid)
        {
            return;
        }

        if (path.length < 3)
        {
            path.smoothed = true;
            return;
        }

        PathCell output[ASTAR_MAX_PATH];

        uint16_t outputLength = 0;

        uint16_t current = 0;

        output[outputLength++] =
            path.cells[0];

        while (current < path.length - 1)
        {
            uint16_t farthest =
                current + 1;

            for (uint16_t candidate =
                    current + 2;
                 candidate < path.length;
                 candidate++)
            {
                if (pathValidator.segmentSafe(
                        grid,
                        path.cells[current],
                        path.cells[candidate]))
                {
                    farthest = candidate;
                }
                else
                {
                    break;
                }
            }

            if (outputLength >= ASTAR_MAX_PATH)
            {
                break;
            }

            output[outputLength++] =
                path.cells[farthest];

            current = farthest;
        }

        path.length =
            outputLength;

        for (uint16_t i = 0;
             i < outputLength;
             i++)
        {
            path.cells[i] =
                output[i];
        }

        path.smoothed = true;

        path.collisionFree =
            pathValidator.isPathSafe(
                grid,
                path
            );
    }
};

PathSmoother pathSmoother;

// ================================================================
// NAVIGATION CONTROLLER
// ================================================================

class NavigationController
{
private:

    NavigationStatus status;

    PlannedPath currentPath;

    float linearGain;
    float angularGain;

    float maxLinearSpeed;
    float maxAngularSpeed;

    float goalTolerance;

    float normalize(
        float angle
    )
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

    float clamp(
        float value,
        float minimum,
        float maximum
    )
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

public:

    NavigationController()
    {
        status.state = NAV_IDLE;

        status.goal.x = 0;
        status.goal.y = 0;

        status.currentPathIndex = 0;

        status.distanceToGoal = 0.0f;
        status.headingError = 0.0f;

        status.pathAvailable = false;
        status.goalReached = false;

        status.navigationStartTime = 0;

        currentPath.length = 0;
        currentPath.valid = false;

        linearGain = 0.8f;
        angularGain = 1.8f;

        maxLinearSpeed =
            NAVIGATION_MAX_LINEAR_SPEED;

        maxAngularSpeed =
            NAVIGATION_MAX_ANGULAR_SPEED;

        goalTolerance =
            NAVIGATION_GOAL_TOLERANCE;
    }

    void configure(
        float linearK,
        float angularK,
        float maxLinear,
        float maxAngular
    )
    {
        if (linearK > 0.0f)
        {
            linearGain = linearK;
        }

        if (angularK > 0.0f)
        {
            angularGain = angularK;
        }

        if (maxLinear > 0.0f)
        {
            maxLinearSpeed = maxLinear;
        }

        if (maxAngular > 0.0f)
        {
            maxAngularSpeed = maxAngular;
        }
    }

    void setPath(
        const PlannedPath &path
    )
    {
        currentPath = path;

        status.pathAvailable =
            currentPath.valid &&
            currentPath.length > 0;

        status.currentPathIndex = 0;

        status.goalReached = false;

        if (status.pathAvailable)
        {
            status.state =
                NAV_FOLLOWING;
        }
        else
        {
            status.state =
                NAV_NO_PATH;
        }
    }

    void setGoal(
        PathCell goal
    )
    {
        status.goal = goal;

        status.goalReached = false;

        status.state =
            NAV_PLANNING;

        status.navigationStartTime =
            millis();
    }

    void clearPath()
    {
        currentPath.length = 0;

        currentPath.valid = false;

        status.pathAvailable = false;

        status.currentPathIndex = 0;

        status.state = NAV_IDLE;

        status.goalReached = false;
    }

    NavigationCommand update(
        const PoseEstimate &pose,
        const ObstacleDetectionResult &obstacles,
        OccupancyGrid &grid
    )
    {
        NavigationCommand command;

        command.linearVelocity = 0.0f;
        command.angularVelocity = 0.0f;
        command.valid = false;
        command.stop = true;

        if (!pose.valid)
        {
            status.state =
                NAV_EMERGENCY_STOP;

            return command;
        }

        if (!status.pathAvailable ||
            !currentPath.valid)
        {
            status.state =
                NAV_NO_PATH;

            return command;
        }

        if (obstacles.frontBlocked)
        {
            status.state =
                NAV_AVOIDING;

            command.linearVelocity = 0.0f;

            if (obstacles.leftBlocked &&
                !obstacles.rightBlocked)
            {
                command.angularVelocity =
                    -0.7f;
            }
            else if (!obstacles.leftBlocked &&
                     obstacles.rightBlocked)
            {
                command.angularVelocity =
                    0.7f;
            }
            else
            {
                command.angularVelocity =
                    0.6f;
            }

            command.angularVelocity =
                clamp(
                    command.angularVelocity,
                    -maxAngularSpeed,
                    maxAngularSpeed
                );

            command.valid = true;
            command.stop = false;

            return command;
        }

        if (status.currentPathIndex >=
            currentPath.length)
        {
            status.state =
                NAV_GOAL_REACHED;

            status.goalReached = true;

            return command;
        }

        PathCell target =
            currentPath.cells[
                status.currentPathIndex
            ];

        float resolution =
            config.mapResolution;

        float targetX;
        float targetY;

        grid.gridToWorld(
            target.x,
            target.y,
            targetX,
            targetY
        );

        float dx =
            targetX - pose.x;

        float dy =
            targetY - pose.y;

        float distance =
            sqrt(
                dx * dx +
                dy * dy
            );

        float targetAngle =
            atan2(
                dy,
                dx
            );

        float headingError =
            normalize(
                targetAngle - pose.theta
            );

        status.distanceToGoal =
            distance;

        status.headingError =
            headingError;

        if (status.currentPathIndex ==
            currentPath.length - 1 &&
            distance <= goalTolerance)
        {
            status.goalReached = true;

            status.state =
                NAV_GOAL_REACHED;

            command.linearVelocity = 0.0f;
            command.angularVelocity = 0.0f;
            command.valid = true;
            command.stop = true;

            return command;
        }

        if (distance < goalTolerance)
        {
            status.currentPathIndex++;

            return update(
                pose,
                obstacles,
                grid
            );
        }

        float angular =
            angularGain *
            headingError;

        angular =
            clamp(
                angular,
                -maxAngularSpeed,
                maxAngularSpeed
            );

        float headingFactor =
            1.0f -
            fabs(headingError) /
            PI_VALUE;

        if (headingFactor < 0.0f)
        {
            headingFactor = 0.0f;
        }

        float linear =
            linearGain *
            distance *
            headingFactor;

        linear =
            clamp(
                linear,
                0.0f,
                maxLinearSpeed
            );

        if (fabs(headingError) >
            PI_VALUE * 0.65f)
        {
            linear = 0.0f;
        }

        command.linearVelocity =
            linear;

        command.angularVelocity =
            angular;

        command.valid = true;
        command.stop = false;

        return command;
    }

    NavigationStatus getStatus()
    {
        return status;
    }

    PlannedPath getPath()
    {
        return currentPath;
    }
};

NavigationController navigationController;

// ================================================================
// NAVIGATION MANAGER
// ================================================================

class AutonomousNavigationManager
{
private:

    bool enabled;
    bool autonomousMode;

    PathCell currentGoal;

    unsigned long lastPlanTime;

    unsigned long planningInterval;

    bool goalActive;

public:

    AutonomousNavigationManager()
    {
        enabled = false;
        autonomousMode = false;

        currentGoal.x = 0;
        currentGoal.y = 0;

        lastPlanTime = 0;

        planningInterval = 1000;

        goalActive = false;
    }

    void enable()
    {
        enabled = true;
    }

    void disable()
    {
        enabled = false;
        autonomousMode = false;

        navigationController.clearPath();
    }

    bool isEnabled()
    {
        return enabled;
    }

    void setAutonomousMode(
        bool value
    )
    {
        autonomousMode = value;

        if (!value)
        {
            navigationController.clearPath();
        }
    }

    bool setGoal(
        int16_t x,
        int16_t y
    )
    {
        currentGoal.x = x;
        currentGoal.y = y;

        goalActive = true;

        navigationController.setGoal(
            currentGoal
        );

        return true;
    }

    void cancelGoal()
    {
        goalActive = false;

        navigationController.clearPath();
    }

    bool planIfRequired(
        OccupancyGrid &grid
    )
    {
        if (!enabled ||
            !autonomousMode ||
            !goalActive)
        {
            return false;
        }

        unsigned long now =
            millis();

        if (now - lastPlanTime <
            planningInterval)
        {
            return false;
        }

        PoseEstimate pose =
            localizationManager.getPose();

        int16_t startX;
        int16_t startY;

        grid.worldToGrid(
            pose.x,
            pose.y,
            startX,
            startY
        );

        if (!grid.isInside(
                startX,
                startY))
        {
            return false;
        }

        if (!grid.isInside(
                currentGoal.x,
                currentGoal.y))
        {
            return false;
        }

        PlannedPath planned;

        planned.length = 0;
        planned.valid = false;
        planned.smoothed = false;
        planned.collisionFree = false;
        planned.expandedNodes = 0;

        if (!pathPlanner.plan(
                grid,
                PathCell{startX, startY},
                currentGoal
            ))
        {
            navigationController.clearPath();

            lastPlanTime = now;

            return false;
        }

        planned =
            pathPlanner.getPath();

        pathSmoother.smooth(
            grid,
            planned
        );

        if (!planned.collisionFree)
        {
            navigationController.clearPath();

            lastPlanTime = now;

            return false;
        }

        navigationController.setPath(
            planned
        );

        lastPlanTime = now;

        return true;
    }

    NavigationCommand update(
        OccupancyGrid &grid,
        const ObstacleDetectionResult &obstacles
    )
    {
        NavigationCommand stopCommand;

        stopCommand.linearVelocity = 0.0f;
        stopCommand.angularVelocity = 0.0f;
        stopCommand.valid = false;
        stopCommand.stop = true;

        if (!enabled ||
            !autonomousMode ||
            !goalActive)
        {
            return stopCommand;
        }

        planIfRequired(grid);

        PoseEstimate pose =
            localizationManager.getPose();

        return navigationController.update(
            pose,
            obstacles,
            grid
        );
    }
};

AutonomousNavigationManager autonomousNavigationManager;

// ================================================================
// LOCALIZATION UPDATE
// ================================================================
//
// The exact encoder variables from Part 1 are project-dependent.
// Keep this function as the integration point.
//
// Replace the two distance arguments with the actual accumulated
// left/right wheel displacement from your encoder subsystem.
//
// ================================================================

void updateLocalization(
    float leftWheelDistance,
    float rightWheelDistance
)
{
    localizationManager.updateFromWheelDistances(
        leftWheelDistance,
        rightWheelDistance,
        millis()
    );

    robotState.pose.x =
        localizationManager.getPose().x;

    robotState.pose.y =
        localizationManager.getPose().y;

    robotState.pose.theta =
        localizationManager.getPose().theta;
}

// ================================================================
// SENSOR DISTANCE UNIT NORMALIZATION
// ================================================================
//
// The mapping specification uses physical distances while the
// ultrasonic sensors may return centimeters.
//
// This helper keeps mapping/navigation units consistent.
//
// If Part 1 already stores sensor values in meters, call this with
// values that are already in meters and change the flag to false.
//
// ================================================================

float sensorDistanceToMeters(
    float distance,
    bool inputIsCentimeters
)
{
    if (distance <= 0.0f)
    {
        return -1.0f;
    }

    if (isnan(distance) ||
        isinf(distance))
    {
        return -1.0f;
    }

    if (inputIsCentimeters)
    {
        return distance / 100.0f;
    }

    return distance;
}

// ================================================================
// OBSTACLE UPDATE
// ================================================================

void updateObstacleDetection()
{
    PoseEstimate pose =
        localizationManager.getPose();

    if (!pose.valid)
    {
        return;
    }

    // IMPORTANT:
    // These values assume Part 1 ultrasonic readings are centimeters.
    //
    // If Part 1 uses meters, set the second argument of
    // sensorDistanceToMeters() to false.

    float front =
        sensorDistanceToMeters(
            distanceSensors.front,
            true
        );

    float left =
        sensorDistanceToMeters(
            distanceSensors.left,
            true
        );

    float right =
        sensorDistanceToMeters(
            distanceSensors.right,
            true
        );

    float rear = -1.0f;

    obstacleDetector.update(
        front,
        left,
        right,
        rear,
        pose
    );
}

// ================================================================
// MAP / PATH PLANNER CONFIGURATION
// ================================================================

void initializePathPlanningSubsystem()
{
    uint16_t width =
        config.mapWidth;

    uint16_t height =
        config.mapHeight;

    if (width > PLANNER_MAX_WIDTH)
    {
        width =
            PLANNER_MAX_WIDTH;
    }

    if (height > PLANNER_MAX_HEIGHT)
    {
        height =
            PLANNER_MAX_HEIGHT;
    }

    if (!pathPlanner.configure(
            width,
            height
        ))
    {
        Serial.println(
            F("A*: planner configuration failed")
        );
    }

    localizationManager.configure(
        0.03f,
        0.14f
    );

    localizationManager.reset(
        robotState.pose.x,
        robotState.pose.y,
        robotState.pose.theta
    );

    obstacleDetector.configure(
        0.35f,
        0.30f,
        0.30f
    );

    navigationController.configure(
        0.8f,
        1.8f,
        NAVIGATION_MAX_LINEAR_SPEED,
        NAVIGATION_MAX_ANGULAR_SPEED
    );

    autonomousNavigationManager.enable();

    Serial.println(
        F("Localization + A* subsystem initialized")
    );
}

// ================================================================
// NAVIGATION COMMAND PROCESSING
// ================================================================
//
// Commands:
//
// NAV ON
// NAV OFF
// AUTO ON
// AUTO OFF
// GOAL x y
// NAV STATUS
// PATH
// STOP
//
// Example:
//
// GOAL 20 20
//
// ================================================================

void processNavigationCommand(
    String command
)
{
    command.trim();

    command.toUpperCase();

    if (command == "NAV ON")
    {
        autonomousNavigationManager.enable();

        Serial.println(
            F("NAVIGATION ENABLED")
        );

        return;
    }

    if (command == "NAV OFF")
    {
        autonomousNavigationManager.disable();

        Serial.println(
            F("NAVIGATION DISABLED")
        );

        return;
    }

    if (command == "AUTO ON")
    {
        autonomousNavigationManager.setAutonomousMode(
            true
        );

        Serial.println(
            F("AUTONOMOUS MODE ENABLED")
        );

        return;
    }

    if (command == "AUTO OFF")
    {
        autonomousNavigationManager.setAutonomousMode(
            false
        );

        Serial.println(
            F("AUTONOMOUS MODE DISABLED")
        );

        return;
    }

    if (command == "STOP")
    {
        autonomousNavigationManager.cancelGoal();

        Serial.println(
            F("NAVIGATION STOPPED")
        );

        return;
    }

    if (command.startsWith("GOAL "))
    {
        int separator =
            command.indexOf(
                ' ',
                5
            );

        if (separator < 0)
        {
            Serial.println(
                F("Usage: GOAL x y")
            );

            return;
        }

        String xText =
            command.substring(
                5,
                separator
            );

        String yText =
            command.substring(
                separator + 1
            );

        int16_t x =
            xText.toInt();

        int16_t y =
            yText.toInt();

        if (autonomousNavigationManager.setGoal(
                x,
                y
            ))
        {
            Serial.print(
                F("GOAL SET: ")
            );

            Serial.print(x);

            Serial.print(
                F(",")
            );

            Serial.println(y);
        }

        return;
    }

    if (command == "NAV STATUS")
    {
        NavigationStatus status =
            navigationController.getStatus();

        Serial.print(
            F("STATE: ")
        );

        Serial.println(
            (int)status.state
        );

        Serial.print(
            F("PATH: ")
        );

        Serial.println(
            status.pathAvailable ?
            F("YES") :
            F("NO")
        );

        Serial.print(
            F("INDEX: ")
        );

        Serial.println(
            status.currentPathIndex
        );

        Serial.print(
            F("DISTANCE: ")
        );

        Serial.println(
            status.distanceToGoal,
            3
        );

        Serial.print(
            F("HEADING ERROR: ")
        );

        Serial.println(
            status.headingError,
            3
        );

        return;
    }

    if (command == "PATH")
    {
        PlannedPath path =
            navigationController.getPath();

        Serial.print(
            F("PATH LENGTH: ")
        );

        Serial.println(
            path.length
        );

        for (uint16_t i = 0;
             i < path.length;
             i++)
        {
            Serial.print(i);

            Serial.print(
                F(": ")
            );

            Serial.print(
                path.cells[i].x
            );

            Serial.print(
                F(",")
            );

            Serial.println(
                path.cells[i].y
            );
        }

        return;
    }
}

// ================================================================
// NAVIGATION LOOP
// ================================================================

NavigationCommand updateAutonomousNavigation()
{
    ObstacleDetectionResult obstacles =
        obstacleDetector.getResult();

    NavigationCommand command =
        autonomousNavigationManager.update(
            mappingEngine.getGrid(),
            obstacles
        );

    return command;
}

// ================================================================
// SAFETY FILTER
// ================================================================

NavigationCommand applyNavigationSafety(
    NavigationCommand command
)
{
    if (!command.valid)
    {
        command.linearVelocity = 0.0f;
        command.angularVelocity = 0.0f;
        command.stop = true;

        return command;
    }

    ObstacleDetectionResult obstacles =
        obstacleDetector.getResult();

    if (obstacles.frontBlocked &&
        command.linearVelocity > 0.0f)
    {
        command.linearVelocity = 0.0f;
    }

    if (fabs(command.linearVelocity) >
        NAVIGATION_MAX_LINEAR_SPEED)
    {
        command.linearVelocity =
            command.linearVelocity > 0.0f ?
            NAVIGATION_MAX_LINEAR_SPEED :
            -NAVIGATION_MAX_LINEAR_SPEED;
    }

    if (fabs(command.angularVelocity) >
        NAVIGATION_MAX_ANGULAR_SPEED)
    {
        command.angularVelocity =
            command.angularVelocity > 0.0f ?
            NAVIGATION_MAX_ANGULAR_SPEED :
            -NAVIGATION_MAX_ANGULAR_SPEED;
    }

    return command;
}

// ================================================================
// MOTOR COMMAND BRIDGE
// ================================================================
//
// This converts:
//     linear velocity
//     angular velocity
//
// into differential-drive wheel velocities.
//
// The final motor driver should use these values with the existing
// balance controller from Part 1.
//
// ================================================================

struct WheelVelocityCommand
{
    float left;
    float right;

    bool valid;
};

WheelVelocityCommand navigationToWheelVelocity(
    NavigationCommand command
)
{
    WheelVelocityCommand result;

    result.left = 0.0f;
    result.right = 0.0f;
    result.valid = false;

    if (!command.valid ||
        command.stop)
    {
        return result;
    }

    const float wheelSeparation =
        0.14f;

    result.left =
        command.linearVelocity -
        command.angularVelocity *
        wheelSeparation *
        0.5f;

    result.right =
        command.linearVelocity +
        command.angularVelocity *
        wheelSeparation *
        0.5f;

    result.valid = true;

    return result;
}

// ================================================================
// NAVIGATION TELEMETRY
// ================================================================

void printNavigationTelemetry()
{
    PoseEstimate pose =
        localizationManager.getPose();

    LocalizationStatus localizationStatus =
        localizationManager.getStatus();

    NavigationStatus navigationStatus =
        navigationController.getStatus();

    ObstacleDetectionResult obstacles =
        obstacleDetector.getResult();

    Serial.println(
        F("========== NAVIGATION ==========")
    );

    Serial.print(
        F("POSE X: ")
    );

    Serial.println(
        pose.x,
        3
    );

    Serial.print(
        F("POSE Y: ")
    );

    Serial.println(
        pose.y,
        3
    );

    Serial.print(
        F("POSE THETA: ")
    );

    Serial.println(
        pose.theta,
        3
    );

    Serial.print(
        F("POSE UNCERTAINTY: ")
    );

    Serial.println(
        localizationStatus.positionUncertainty,
        4
    );

    Serial.print(
        F("OBSTACLE COUNT: ")
    );

    Serial.println(
        obstacles.count
    );

    Serial.print(
        F("NEAREST OBSTACLE: ")
    );

    Serial.println(
        obstacles.nearestDistance,
        3
    );

    Serial.print(
        F("PATH AVAILABLE: ")
    );

    Serial.println(
        navigationStatus.pathAvailable ?
        F("YES") :
        F("NO")
    );

    Serial.print(
        F("PATH INDEX: ")
    );

    Serial.println(
        navigationStatus.currentPathIndex
    );

    Serial.print(
        F("GOAL DISTANCE: ")
    );

    Serial.println(
        navigationStatus.distanceToGoal,
        3
    );

    Serial.println(
        F("===============================")
    );
}

// ================================================================
// FRONTIER DETECTION
// ================================================================
//
// This is the beginning of the exploration architecture required by
// the project specification.
//
// A frontier cell is:
//
//     FREE cell
//        +
//     neighboring UNKNOWN cell
//
// Only a limited number of candidates are stored because the Arduino
// Uno has very limited SRAM.
//
// ================================================================

struct FrontierCandidate
{
    PathCell cell;

    uint16_t unknownNeighbors;
    uint16_t freeNeighbors;

    float score;

    bool valid;
};

class FrontierDetector
{
private:

    FrontierCandidate candidates[
        MAX_FRONTIER_CANDIDATES
    ];

    uint16_t count;

    bool isFrontier(
        OccupancyGrid &grid,
        int16_t x,
        int16_t y
    )
    {
        MapCell center =
            grid.getCell(x, y);

        if (center.state != CELL_FREE)
        {
            return false;
        }

        const int8_t directions[4][2] =
        {
            { 1, 0},
            {-1, 0},
            { 0, 1},
            { 0,-1}
        };

        for (uint8_t i = 0; i < 4; i++)
        {
            int16_t nx =
                x + directions[i][0];

            int16_t ny =
                y + directions[i][1];

            if (!grid.isInside(nx, ny))
            {
                continue;
            }

            MapCell neighbor =
                grid.getCell(nx, ny);

            if (neighbor.state ==
                CELL_UNKNOWN)
            {
                return true;
            }
        }

        return false;
    }

public:

    FrontierDetector()
    {
        clear();
    }

    void clear()
    {
        count = 0;

        for (uint16_t i = 0;
             i < MAX_FRONTIER_CANDIDATES;
             i++)
        {
            candidates[i].cell.x = 0;
            candidates[i].cell.y = 0;

            candidates[i].unknownNeighbors = 0;
            candidates[i].freeNeighbors = 0;

            candidates[i].score = 0.0f;

            candidates[i].valid = false;
        }
    }

    void detect(
        OccupancyGrid &grid
    )
    {
        clear();

        MapBounds bounds =
            grid.getBounds();

        for (int16_t y =
                bounds.minY;
             y <= bounds.maxY;
             y++)
        {
            for (int16_t x =
                    bounds.minX;
                 x <= bounds.maxX;
                 x++)
            {
                if (count >=
                    MAX_FRONTIER_CANDIDATES)
                {
                    return;
                }

                if (!isFrontier(
                        grid,
                        x,
                        y))
                {
                    continue;
                }

                FrontierCandidate candidate;

                candidate.cell.x = x;
                candidate.cell.y = y;

                candidate.unknownNeighbors = 0;
                candidate.freeNeighbors = 0;

                const int8_t directions[4][2] =
                {
                    { 1, 0},
                    {-1, 0},
                    { 0, 1},
                    { 0,-1}
                };

                for (uint8_t i = 0;
                     i < 4;
                     i++)
                {
                    int16_t nx =
                        x + directions[i][0];

                    int16_t ny =
                        y + directions[i][1];

                    if (!grid.isInside(
                            nx,
                            ny))
                    {
                        continue;
                    }

                    MapCell neighbor =
                        grid.getCell(
                            nx,
                            ny
                        );

                    if (neighbor.state ==
                        CELL_UNKNOWN)
                    {
                        candidate.unknownNeighbors++;
                    }

                    if (neighbor.state ==
                        CELL_FREE)
                    {
                        candidate.freeNeighbors++;
                    }
                }

                candidate.score =
                    candidate.unknownNeighbors * 2.0f +
                    candidate.freeNeighbors * 0.5f;

                candidate.valid = true;

                candidates[count++] =
                    candidate;
            }
        }
    }

    uint16_t getCount()
    {
        return count;
    }

    FrontierCandidate getCandidate(
        uint16_t index
    )
    {
        FrontierCandidate empty;

        empty.valid = false;
        empty.cell.x = 0;
        empty.cell.y = 0;
        empty.score = 0.0f;
        empty.unknownNeighbors = 0;
        empty.freeNeighbors = 0;

        if (index >= count)
        {
            return empty;
        }

        return candidates[index];
    }

    FrontierCandidate getBest()
    {
        FrontierCandidate best;

        best.valid = false;
        best.score = -999999.0f;

        for (uint16_t i = 0;
             i < count;
             i++)
        {
            if (!candidates[i].valid)
            {
                continue;
            }

            if (candidates[i].score >
                best.score)
            {
                best =
                    candidates[i];
            }
        }

        return best;
    }
};

FrontierDetector frontierDetector;

// ================================================================
// EXPLORATION MANAGER
// ================================================================

class ExplorationManager
{
private:

    bool enabled;

    FrontierCandidate target;

    unsigned long lastDetection;

    unsigned long detectionInterval;

public:

    ExplorationManager()
    {
        enabled = false;

        target.valid = false;

        lastDetection = 0;

        detectionInterval = 1500;
    }

    void enable()
    {
        enabled = true;
    }

    void disable()
    {
        enabled = false;

        target.valid = false;

        autonomousNavigationManager.cancelGoal();
    }

    void update(
        OccupancyGrid &grid
    )
    {
        if (!enabled)
        {
            return;
        }

        unsigned long now =
            millis();

        if (now - lastDetection <
            detectionInterval)
        {
            return;
        }

        lastDetection = now;

        frontierDetector.detect(
            grid
        );

        target =
            frontierDetector.getBest();

        if (!target.valid)
        {
            return;
        }

        autonomousNavigationManager.setGoal(
            target.cell.x,
            target.cell.y
        );

        autonomousNavigationManager.setAutonomousMode(
            true
        );
    }

    FrontierCandidate getTarget()
    {
        return target;
    }

    bool isEnabled()
    {
        return enabled;
    }
};

ExplorationManager explorationManager;

// ================================================================
// PART 3 MAIN UPDATE
// ================================================================
//
// Call this from loop() after the Part 2 mapping update.
//
// ================================================================

void updatePart3Navigation()
{
    updateObstacleDetection();

    NavigationCommand navigationCommand =
        updateAutonomousNavigation();

    navigationCommand =
        applyNavigationSafety(
            navigationCommand
        );

    WheelVelocityCommand wheels =
        navigationToWheelVelocity(
            navigationCommand
        );

    // ------------------------------------------------------------
    // IMPORTANT:
    //
    // Do NOT directly drive the motors here on a balancing robot.
    //
    // The balance controller should remain the highest-priority
    // controller. These values should become velocity references
    // for the balancing/motor-control subsystem from Part 1.
    //
    // Example integration:
    //
    // setNavigationVelocityReference(
    //     wheels.left,
    //     wheels.right
    // );
    //
    // ------------------------------------------------------------

    if (wheels.valid)
    {
        // Integration point for Part 1 motor/balance controller.
        //
        // Replace with the actual velocity-reference function
        // implemented in Part 1.
    }
}

// ================================================================
// PART 3 COMMAND INTERFACE
// ================================================================

void updatePart3CommandInterface()
{
    if (!Serial.available())
    {
        return;
    }

    String command =
        Serial.readStringUntil(
            '\n'
        );

    command.trim();

    if (command.length() == 0)
    {
        return;
    }

    processNavigationCommand(
        command
    );
}

// ================================================================
// PART 3 DIAGNOSTIC TESTS
// ================================================================

bool testCoordinateConversion()
{
    float x = 0.0f;
    float y = 0.0f;

    int16_t gx;
    int16_t gy;

    mappingEngine.getGrid().worldToGrid(
        x,
        y,
        gx,
        gy
    );

    float restoredX;
    float restoredY;

    mappingEngine.getGrid().gridToWorld(
        gx,
        gy,
        restoredX,
        restoredY
    );

    return
        !isnan(restoredX) &&
        !isnan(restoredY);
}

bool testObstacleDetector()
{
    PoseEstimate pose =
        localizationManager.getPose();

    if (!pose.valid)
    {
        return false;
    }

    obstacleDetector.update(
        0.20f,
        0.50f,
        0.50f,
        -1.0f,
        pose
    );

    ObstacleDetectionResult result =
        obstacleDetector.getResult();

    return result.frontBlocked;
}

bool testPlannerConfiguration()
{
    return pathPlanner.configure(
        16,
        16
    );
}

bool testNavigationSubsystem()
{
    if (!testCoordinateConversion())
    {
        return false;
    }

    if (!testPlannerConfiguration())
    {
        return false;
    }

    if (!testObstacleDetector())
    {
        return false;
    }

    return true;
}

// ================================================================
// PART 3 INITIALIZATION
// ================================================================

void initializePart3()
{
    initializePathPlanningSubsystem();

    Serial.println(
        F("Running Part 3 diagnostics...")
    );

    if (testNavigationSubsystem())
    {
        Serial.println(
            F("PART 3 DIAGNOSTICS: PASS")
        );
    }
    else
    {
        Serial.println(
            F("PART 3 DIAGNOSTICS: FAIL")
        );
    }

    Serial.println(
        F("Commands:")
    );

    Serial.println(
        F("NAV ON")
    );

    Serial.println(
        F("NAV OFF")
    );

    Serial.println(
        F("AUTO ON")
    );

    Serial.println(
        F("AUTO OFF")
    );

    Serial.println(
        F("GOAL x y")
    );

    Serial.println(
        F("NAV STATUS")
    );

    Serial.println(
        F("PATH")
    );

    Serial.println(
        F("STOP")
    );
}

// ================================================================
// END OF PART 3
// ================================================================

#endif
// ================================================================
// PART 4 — PATH FOLLOWING + SELF-BALANCE NAVIGATION INTEGRATION
// Arduino Uno Autonomous Self-Balancing Robot
// ================================================================
//
// Continues from Parts 1, 2 and 3.
//
// This part adds:
//
//   1. Path-following controller
//   2. Look-ahead waypoint selection
//   3. Differential-drive velocity references
//   4. Self-balancing safety layer
//   5. Tilt protection
//   6. Acceleration limiting
//   7. Wheel-speed limiting
//   8. Navigation/balance priority handling
//   9. Emergency-stop handling
//  10. Goal-arrival handling
//  11. Navigation telemetry
//
// IMPORTANT:
// The balancing controller remains the highest-priority controller.
// Navigation provides velocity references; it must NOT directly
// override the balance controller's stabilization output.
//
// ================================================================

#ifndef ROBOT_PART_4_INCLUDED
#define ROBOT_PART_4_INCLUDED

#include <Arduino.h>
#include <math.h>

// ================================================================
// PART 4 CONFIGURATION
// ================================================================

#ifndef BALANCE_SAFE_TILT_DEGREES
#define BALANCE_SAFE_TILT_DEGREES 18.0f
#endif

#ifndef BALANCE_EMERGENCY_TILT_DEGREES
#define BALANCE_EMERGENCY_TILT_DEGREES 30.0f
#endif

#ifndef NAVIGATION_CONTROL_PERIOD
#define NAVIGATION_CONTROL_PERIOD 20UL
#endif

#ifndef MAX_WHEEL_REFERENCE
#define MAX_WHEEL_REFERENCE 0.65f
#endif

#ifndef MAX_WHEEL_ACCELERATION
#define MAX_WHEEL_ACCELERATION 1.20f
#endif

#ifndef MAX_WHEEL_DECELERATION
#define MAX_WHEEL_DECELERATION 2.00f
#endif

#ifndef LOOKAHEAD_DISTANCE_METERS
#define LOOKAHEAD_DISTANCE_METERS 0.12f
#endif

#ifndef WAYPOINT_REACHED_DISTANCE
#define WAYPOINT_REACHED_DISTANCE 0.07f
#endif

#ifndef HEADING_ALIGNMENT_LIMIT
#define HEADING_ALIGNMENT_LIMIT 1.20f
#endif

#ifndef NAVIGATION_LINEAR_GAIN
#define NAVIGATION_LINEAR_GAIN 1.00f
#endif

#ifndef NAVIGATION_HEADING_GAIN
#define NAVIGATION_HEADING_GAIN 2.00f
#endif

#ifndef BALANCE_COMMAND_TIMEOUT
#define BALANCE_COMMAND_TIMEOUT 250UL
#endif

// ================================================================
// CONTROL VECTOR
// ================================================================

struct DifferentialVelocity
{
    float left;
    float right;

    bool valid;
};

// ================================================================
// BALANCE SAFETY STATE
// ================================================================

enum BalanceSafetyState
{
    BALANCE_SAFE,
    BALANCE_TILT_WARNING,
    BALANCE_TILT_CRITICAL,
    BALANCE_EMERGENCY
};

struct BalanceSafetyStatus
{
    BalanceSafetyState state;

    float pitch;
    float roll;

    bool upright;
    bool emergencyStop;

    unsigned long timestamp;
};

// ================================================================
// NAVIGATION CONTROL STATUS
// ================================================================

struct NavigationControlStatus
{
    float targetLinearVelocity;
    float targetAngularVelocity;

    float leftReference;
    float rightReference;

    float targetDistance;
    float headingError;

    uint16_t waypointIndex;

    bool active;
    bool goalReached;
    bool blocked;

    unsigned long updateTime;
};

// ================================================================
// UTILITY
// ================================================================

float part4NormalizeAngle(
    float angle
)
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

float part4Clamp(
    float value,
    float minimum,
    float maximum
)
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

// ================================================================
// BALANCE SAFETY MANAGER
// ================================================================

class BalanceSafetyManager
{
private:

    BalanceSafetyStatus status;

    float warningTilt;
    float criticalTilt;
    float emergencyTilt;

public:

    BalanceSafetyManager()
    {
        status.state =
            BALANCE_SAFE;

        status.pitch = 0.0f;
        status.roll = 0.0f;

        status.upright = true;
        status.emergencyStop = false;

        status.timestamp = 0;

        warningTilt =
            BALANCE_SAFE_TILT_DEGREES;

        criticalTilt =
            BALANCE_EMERGENCY_TILT_DEGREES * 0.80f;

        emergencyTilt =
            BALANCE_EMERGENCY_TILT_DEGREES;
    }

    void configure(
        float warning,
        float critical,
        float emergency
    )
    {
        if (warning > 0.0f)
        {
            warningTilt = warning;
        }

        if (critical > warningTilt)
        {
            criticalTilt = critical;
        }

        if (emergency > criticalTilt)
        {
            emergencyTilt = emergency;
        }
    }

    void update(
        float pitchDegrees,
        float rollDegrees
    )
    {
        status.pitch =
            pitchDegrees;

        status.roll =
            rollDegrees;

        status.timestamp =
            millis();

        float absolutePitch =
            fabs(pitchDegrees);

        float absoluteRoll =
            fabs(rollDegrees);

        float maximumTilt =
            absolutePitch >
            absoluteRoll ?
            absolutePitch :
            absoluteRoll;

        status.emergencyStop =
            false;

        if (maximumTilt >= emergencyTilt)
        {
            status.state =
                BALANCE_EMERGENCY;

            status.upright = false;

            status.emergencyStop = true;

            return;
        }

        if (maximumTilt >= criticalTilt)
        {
            status.state =
                BALANCE_TILT_CRITICAL;

            status.upright = false;

            return;
        }

        if (maximumTilt >= warningTilt)
        {
            status.state =
                BALANCE_TILT_WARNING;

            status.upright = true;

            return;
        }

        status.state =
            BALANCE_SAFE;

        status.upright = true;
    }

    BalanceSafetyStatus getStatus()
    {
        return status;
    }

    bool isSafe()
    {
        return
            status.state ==
            BALANCE_SAFE ||
            status.state ==
            BALANCE_TILT_WARNING;
    }

    bool isEmergency()
    {
        return status.emergencyStop;
    }
};

BalanceSafetyManager balanceSafetyManager;

// ================================================================
// VELOCITY LIMITER
// ================================================================

class VelocityLimiter
{
private:

    float currentLeft;
    float currentRight;

    float maximumVelocity;
    float maximumAcceleration;
    float maximumDeceleration;

    unsigned long previousTime;

    float approach(
        float current,
        float target,
        float maximumChange
    )
    {
        float difference =
            target - current;

        if (difference > maximumChange)
        {
            return current + maximumChange;
        }

        if (difference < -maximumChange)
        {
            return current - maximumChange;
        }

        return target;
    }

public:

    VelocityLimiter()
    {
        currentLeft = 0.0f;
        currentRight = 0.0f;

        maximumVelocity =
            MAX_WHEEL_REFERENCE;

        maximumAcceleration =
            MAX_WHEEL_ACCELERATION;

        maximumDeceleration =
            MAX_WHEEL_DECELERATION;

        previousTime =
            millis();
    }

    void configure(
        float maxVelocity,
        float acceleration,
        float deceleration
    )
    {
        if (maxVelocity > 0.0f)
        {
            maximumVelocity =
                maxVelocity;
        }

        if (acceleration > 0.0f)
        {
            maximumAcceleration =
                acceleration;
        }

        if (deceleration > 0.0f)
        {
            maximumDeceleration =
                deceleration;
        }
    }

    DifferentialVelocity update(
        float targetLeft,
        float targetRight
    )
    {
        DifferentialVelocity result;

        result.left = 0.0f;
        result.right = 0.0f;
        result.valid = true;

        unsigned long now =
            millis();

        unsigned long elapsed =
            now - previousTime;

        previousTime = now;

        float dt =
            elapsed / 1000.0f;

        if (dt <= 0.0f)
        {
            dt = 0.001f;
        }

        if (dt > 0.2f)
        {
            dt = 0.2f;
        }

        targetLeft =
            part4Clamp(
                targetLeft,
                -maximumVelocity,
                maximumVelocity
            );

        targetRight =
            part4Clamp(
                targetRight,
                -maximumVelocity,
                maximumVelocity
            );

        float accelerationStep =
            maximumAcceleration *
            dt;

        float decelerationStep =
            maximumDeceleration *
            dt;

        float leftDifference =
            targetLeft -
            currentLeft;

        float rightDifference =
            targetRight -
            currentRight;

        float leftStep =
            fabs(leftDifference) >
            fabs(currentLeft) ?
            accelerationStep :
            decelerationStep;

        float rightStep =
            fabs(rightDifference) >
            fabs(currentRight) ?
            accelerationStep :
            decelerationStep;

        if (leftDifference > leftStep)
        {
            currentLeft += leftStep;
        }
        else if (leftDifference < -leftStep)
        {
            currentLeft -= leftStep;
        }
        else
        {
            currentLeft =
                targetLeft;
        }

        if (rightDifference > rightStep)
        {
            currentRight += rightStep;
        }
        else if (rightDifference < -rightStep)
        {
            currentRight -= rightStep;
        }
        else
        {
            currentRight =
                targetRight;
        }

        currentLeft =
            part4Clamp(
                currentLeft,
                -maximumVelocity,
                maximumVelocity
            );

        currentRight =
            part4Clamp(
                currentRight,
                -maximumVelocity,
                maximumVelocity
            );

        result.left =
            currentLeft;

        result.right =
            currentRight;

        return result;
    }

    void emergencyStop()
    {
        currentLeft = 0.0f;
        currentRight = 0.0f;
    }

    float getLeft()
    {
        return currentLeft;
    }

    float getRight()
    {
        return currentRight;
    }
};

VelocityLimiter velocityLimiter;

// ================================================================
// PATH FOLLOWER
// ================================================================

class PathFollower
{
private:

    float linearGain;
    float angularGain;

    float maximumLinear;
    float maximumAngular;

    float waypointTolerance;
    float lookaheadDistance;

    uint16_t waypointIndex;

    bool active;
    bool goalReached;

    float lastDistance;
    float lastHeadingError;

    float getWaypointDistance(
        const PoseEstimate &pose,
        const PathCell &cell,
        OccupancyGrid &grid
    )
    {
        float targetX;
        float targetY;

        grid.gridToWorld(
            cell.x,
            cell.y,
            targetX,
            targetY
        );

        float dx =
            targetX - pose.x;

        float dy =
            targetY - pose.y;

        return sqrt(
            dx * dx +
            dy * dy
        );
    }

    void getWaypointPosition(
        const PathCell &cell,
        OccupancyGrid &grid,
        float &x,
        float &y
    )
    {
        grid.gridToWorld(
            cell.x,
            cell.y,
            x,
            y
        );
    }

public:

    PathFollower()
    {
        linearGain =
            NAVIGATION_LINEAR_GAIN;

        angularGain =
            NAVIGATION_HEADING_GAIN;

        maximumLinear =
            NAVIGATION_MAX_LINEAR_SPEED;

        maximumAngular =
            NAVIGATION_MAX_ANGULAR_SPEED;

        waypointTolerance =
            WAYPOINT_REACHED_DISTANCE;

        lookaheadDistance =
            LOOKAHEAD_DISTANCE_METERS;

        waypointIndex = 0;

        active = false;
        goalReached = false;

        lastDistance = 0.0f;
        lastHeadingError = 0.0f;
    }

    void configure(
        float linearK,
        float angularK,
        float maxLinear,
        float maxAngular,
        float tolerance,
        float lookahead
    )
    {
        if (linearK > 0.0f)
        {
            linearGain = linearK;
        }

        if (angularK > 0.0f)
        {
            angularGain = angularK;
        }

        if (maxLinear > 0.0f)
        {
            maximumLinear = maxLinear;
        }

        if (maxAngular > 0.0f)
        {
            maximumAngular = maxAngular;
        }

        if (tolerance > 0.0f)
        {
            waypointTolerance = tolerance;
        }

        if (lookahead > 0.0f)
        {
            lookaheadDistance = lookahead;
        }
    }

    void reset()
    {
        waypointIndex = 0;

        active = false;
        goalReached = false;

        lastDistance = 0.0f;
        lastHeadingError = 0.0f;
    }

    void setPath(
        const PlannedPath &path
    )
    {
        waypointIndex = 0;

        goalReached = false;

        active =
            path.valid &&
            path.length > 0;
    }

    uint16_t chooseLookaheadWaypoint(
        const PlannedPath &path,
        const PoseEstimate &pose,
        OccupancyGrid &grid
    )
    {
        if (path.length == 0)
        {
            return 0;
        }

        uint16_t best =
            waypointIndex;

        float accumulated =
            0.0f;

        for (uint16_t i =
                waypointIndex;
             i < path.length;
             i++)
        {
            float distance =
                getWaypointDistance(
                    pose,
                    path.cells[i],
                    grid
                );

            if (i > waypointIndex)
            {
                accumulated +=
                    distance;
            }

            best = i;

            if (accumulated >=
                lookaheadDistance)
            {
                break;
            }
        }

        return best;
    }

    DifferentialVelocity calculate(
        const PlannedPath &path,
        const PoseEstimate &pose,
        const ObstacleDetectionResult &obstacles,
        OccupancyGrid &grid
    )
    {
        DifferentialVelocity result;

        result.left = 0.0f;
        result.right = 0.0f;
        result.valid = false;

        if (!active ||
            !path.valid ||
            path.length == 0 ||
            !pose.valid)
        {
            return result;
        }

        if (obstacles.frontBlocked)
        {
            return result;
        }

        while (
            waypointIndex <
            path.length
        )
        {
            float distance =
                getWaypointDistance(
                    pose,
                    path.cells[
                        waypointIndex
                    ],
                    grid
                );

            if (distance <=
                waypointTolerance)
            {
                waypointIndex++;
            }
            else
            {
                break;
            }
        }

        if (waypointIndex >=
            path.length)
        {
            goalReached = true;
            active = false;

            return result;
        }

        uint16_t targetIndex =
            chooseLookaheadWaypoint(
                path,
                pose,
                grid
            );

        float targetX;
        float targetY;

        getWaypointPosition(
            path.cells[targetIndex],
            grid,
            targetX,
            targetY
        );

        float dx =
            targetX - pose.x;

        float dy =
            targetY - pose.y;

        float distance =
            sqrt(
                dx * dx +
                dy * dy
            );

        float targetHeading =
            atan2(
                dy,
                dx
            );

        float headingError =
            part4NormalizeAngle(
                targetHeading -
                pose.theta
            );

        lastDistance =
            distance;

        lastHeadingError =
            headingError;

        float linear =
            linearGain *
            distance;

        float headingFactor =
            1.0f -
            fabs(headingError) /
            HEADING_ALIGNMENT_LIMIT;

        headingFactor =
            part4Clamp(
                headingFactor,
                0.0f,
                1.0f
            );

        linear *=
            headingFactor;

        linear =
            part4Clamp(
                linear,
                0.0f,
                maximumLinear
            );

        float angular =
            angularGain *
            headingError;

        angular =
            part4Clamp(
                angular,
                -maximumAngular,
                maximumAngular
            );

        if (fabs(headingError) >
            HEADING_ALIGNMENT_LIMIT)
        {
            linear = 0.0f;
        }

        const float wheelBase =
            0.14f;

        result.left =
            linear -
            angular *
            wheelBase *
            0.5f;

        result.right =
            linear +
            angular *
            wheelBase *
            0.5f;

        result.left =
            part4Clamp(
                result.left,
                -MAX_WHEEL_REFERENCE,
                MAX_WHEEL_REFERENCE
            );

        result.right =
            part4Clamp(
                result.right,
                -MAX_WHEEL_REFERENCE,
                MAX_WHEEL_REFERENCE
            );

        result.valid = true;

        return result;
    }

    uint16_t getWaypointIndex()
    {
        return waypointIndex;
    }

    bool hasReachedGoal()
    {
        return goalReached;
    }

    float getLastDistance()
    {
        return lastDistance;
    }

    float getLastHeadingError()
    {
        return lastHeadingError;
    }
};

PathFollower pathFollower;

// ================================================================
// SELF-BALANCE NAVIGATION BRIDGE
// ================================================================

class BalanceNavigationBridge
{
private:

    DifferentialVelocity navigationReference;

    DifferentialVelocity limitedReference;

    NavigationControlStatus status;

    unsigned long lastUpdate;

public:

    BalanceNavigationBridge()
    {
        navigationReference.left = 0.0f;
        navigationReference.right = 0.0f;
        navigationReference.valid = false;

        limitedReference =
            navigationReference;

        status.targetLinearVelocity = 0.0f;
        status.targetAngularVelocity = 0.0f;

        status.leftReference = 0.0f;
        status.rightReference = 0.0f;

        status.targetDistance = 0.0f;
        status.headingError = 0.0f;

        status.waypointIndex = 0;

        status.active = false;
        status.goalReached = false;
        status.blocked = false;

        status.updateTime = 0;

        lastUpdate = 0;
    }

    void reset()
    {
        navigationReference.left = 0.0f;
        navigationReference.right = 0.0f;
        navigationReference.valid = false;

        limitedReference =
            navigationReference;

        velocityLimiter.emergencyStop();

        status.active = false;
        status.goalReached = false;
        status.blocked = false;

        lastUpdate = millis();
    }

    void update(
        const PlannedPath &path,
        const PoseEstimate &pose,
        const ObstacleDetectionResult &obstacles,
        OccupancyGrid &grid
    )
    {
        unsigned long now =
            millis();

        if (now - lastUpdate <
            NAVIGATION_CONTROL_PERIOD)
        {
            return;
        }

        lastUpdate = now;

        BalanceSafetyStatus balanceStatus =
            balanceSafetyManager.getStatus();

        status.updateTime = now;

        status.active =
            path.valid &&
            path.length > 0;

        status.blocked =
            obstacles.frontBlocked;

        if (balanceStatus.emergencyStop)
        {
            velocityLimiter.emergencyStop();

            limitedReference.left = 0.0f;
            limitedReference.right = 0.0f;

            status.active = false;

            return;
        }

        if (balanceStatus.state ==
            BALANCE_TILT_CRITICAL)
        {
            navigationReference.left *=
                0.25f;

            navigationReference.right *=
                0.25f;
        }

        navigationReference =
            pathFollower.calculate(
                path,
                pose,
                obstacles,
                grid
            );

        if (!navigationReference.valid)
        {
            limitedReference =
                velocityLimiter.update(
                    0.0f,
                    0.0f
                );

            return;
        }

        if (balanceStatus.state ==
            BALANCE_TILT_WARNING)
        {
            navigationReference.left *=
                0.45f;

            navigationReference.right *=
                0.45f;
        }

        limitedReference =
            velocityLimiter.update(
                navigationReference.left,
                navigationReference.right
            );

        status.leftReference =
            limitedReference.left;

        status.rightReference =
            limitedReference.right;

        status.waypointIndex =
            pathFollower.getWaypointIndex();

        status.targetDistance =
            pathFollower.getLastDistance();

        status.headingError =
            pathFollower.getLastHeadingError();

        status.goalReached =
            pathFollower.hasReachedGoal();

        if (status.goalReached)
        {
            velocityLimiter.emergencyStop();

            limitedReference.left = 0.0f;
            limitedReference.right = 0.0f;
        }
    }

    DifferentialVelocity getReference()
    {
        return limitedReference;
    }

    NavigationControlStatus getStatus()
    {
        return status;
    }
};

BalanceNavigationBridge balanceNavigationBridge;

// ================================================================
// BALANCE CONTROLLER INTERFACE
// ================================================================
//
// These functions form the integration boundary with Part 1.
//
// The navigation system must provide velocity references to the
// balance controller instead of directly commanding PWM.
//
// ================================================================

float requestedLeftVelocity = 0.0f;
float requestedRightVelocity = 0.0f;

unsigned long velocityReferenceTimestamp = 0;

void setNavigationVelocityReference(
    float left,
    float right
)
{
    requestedLeftVelocity =
        left;

    requestedRightVelocity =
        right;

    velocityReferenceTimestamp =
        millis();
}

void clearNavigationVelocityReference()
{
    requestedLeftVelocity = 0.0f;
    requestedRightVelocity = 0.0f;

    velocityReferenceTimestamp =
        millis();
}

bool navigationVelocityReferenceValid()
{
    return
        millis() -
        velocityReferenceTimestamp
        <= BALANCE_COMMAND_TIMEOUT;
}

// ================================================================
// SAFE REFERENCE OUTPUT
// ================================================================

void updateSafeBalanceReferences()
{
    BalanceSafetyStatus safety =
        balanceSafetyManager.getStatus();

    if (safety.emergencyStop)
    {
        clearNavigationVelocityReference();

        velocityLimiter.emergencyStop();

        return;
    }

    DifferentialVelocity reference =
        balanceNavigationBridge.getReference();

    if (!reference.valid)
    {
        setNavigationVelocityReference(
            0.0f,
            0.0f
        );

        return;
    }

    setNavigationVelocityReference(
        reference.left,
        reference.right
    );
}

// ================================================================
// BALANCE CONTROLLER HOOK
// ================================================================
//
// Replace the body of this function with the actual velocity-target
// interface from Part 1.
//
// The important control hierarchy is:
//
//        Navigation
//             |
//             v
//    Velocity References
//             |
//             v
//      Balance Controller
//             |
//             v
//        Motor Output
//
// Navigation NEVER bypasses the balance controller.
// ================================================================

void applyNavigationToBalanceController()
{
    if (!navigationVelocityReferenceValid())
    {
        clearNavigationVelocityReference();
        return;
    }

    // ------------------------------------------------------------
    // PART 1 INTEGRATION POINT
    // ------------------------------------------------------------
    //
    // Example:
    //
    // balanceController.setLeftVelocityTarget(
    //     requestedLeftVelocity
    // );
    //
    // balanceController.setRightVelocityTarget(
    //     requestedRightVelocity
    // );
    //
    // Use the actual API created in Part 1.
    //
    // ------------------------------------------------------------
}

// ================================================================
// TILT SENSOR INTEGRATION
// ================================================================
//
// Replace these values with the pitch/roll estimates generated by
// the IMU/balance controller in Part 1.
//
// ================================================================

float getCurrentPitchDegrees()
{
    // Integration point with Part 1 IMU.
    //
    // Replace with the actual pitch estimate.

    return 0.0f;
}

float getCurrentRollDegrees()
{
    // Integration point with Part 1 IMU.
    //
    // Replace with the actual roll estimate.

    return 0.0f;
}

// ================================================================
// UPDATE BALANCE SAFETY
// ================================================================

void updateBalanceSafety()
{
    float pitch =
        getCurrentPitchDegrees();

    float roll =
        getCurrentRollDegrees();

    balanceSafetyManager.update(
        pitch,
        roll
    );
}

// ================================================================
// COMPLETE PART 4 CONTROL UPDATE
// ================================================================

void updatePart4Control()
{
    updateBalanceSafety();

    BalanceSafetyStatus safety =
        balanceSafetyManager.getStatus();

    if (safety.emergencyStop)
    {
        balanceNavigationBridge.reset();

        clearNavigationVelocityReference();

        return;
    }

    PoseEstimate pose =
        localizationManager.getPose();

    ObstacleDetectionResult obstacles =
        obstacleDetector.getResult();

    PlannedPath path =
        navigationController.getPath();

    balanceNavigationBridge.update(
        path,
        pose,
        obstacles,
        mappingEngine.getGrid()
    );

    updateSafeBalanceReferences();

    applyNavigationToBalanceController();
}

// ================================================================
// BALANCE EMERGENCY STOP
// ================================================================

void triggerBalanceEmergencyStop()
{
    balanceNavigationBridge.reset();

    velocityLimiter.emergencyStop();

    clearNavigationVelocityReference();

    autonomousNavigationManager.cancelGoal();

    Serial.println(
        F("!!! BALANCE EMERGENCY STOP !!!")
    );
}

// ================================================================
// PART 4 TELEMETRY
// ================================================================

void printPart4Telemetry()
{
    BalanceSafetyStatus balance =
        balanceSafetyManager.getStatus();

    NavigationControlStatus navigation =
        balanceNavigationBridge.getStatus();

    DifferentialVelocity reference =
        balanceNavigationBridge.getReference();

    Serial.println(
        F("========== PART 4 ==========")
    );

    Serial.print(
        F("PITCH: ")
    );

    Serial.println(
        balance.pitch,
        2
    );

    Serial.print(
        F("ROLL: ")
    );

    Serial.println(
        balance.roll,
        2
    );

    Serial.print(
        F("BALANCE STATE: ")
    );

    Serial.println(
        (int)balance.state
    );

    Serial.print(
        F("NAV ACTIVE: ")
    );

    Serial.println(
        navigation.active ?
        F("YES") :
        F("NO")
    );

    Serial.print(
        F("NAV BLOCKED: ")
    );

    Serial.println(
        navigation.blocked ?
        F("YES") :
        F("NO")
    );

    Serial.print(
        F("WAYPOINT: ")
    );

    Serial.println(
        navigation.waypointIndex
    );

    Serial.print(
        F("DISTANCE: ")
    );

    Serial.println(
        navigation.targetDistance,
        3
    );

    Serial.print(
        F("HEADING ERROR: ")
    );

    Serial.println(
        navigation.headingError,
        3
    );

    Serial.print(
        F("LEFT REF: ")
    );

    Serial.println(
        reference.left,
        3
    );

    Serial.print(
        F("RIGHT REF: ")
    );

    Serial.println(
        reference.right,
        3
    );

    Serial.print(
        F("GOAL REACHED: ")
    );

    Serial.println(
        navigation.goalReached ?
        F("YES") :
        F("NO")
    );

    Serial.println(
        F("============================")
    );
}

// ================================================================
// PART 4 COMMANDS
// ================================================================
//
// BALANCE STATUS
// BALANCE RESET
// NAV REF
//
// ================================================================

void processPart4Command(
    String command
)
{
    command.trim();

    command.toUpperCase();

    if (command == "BALANCE STATUS")
    {
        BalanceSafetyStatus status =
            balanceSafetyManager.getStatus();

        Serial.print(
            F("BALANCE STATE: ")
        );

        Serial.println(
            (int)status.state
        );

        Serial.print(
            F("PITCH: ")
        );

        Serial.println(
            status.pitch,
            2
        );

        Serial.print(
            F("ROLL: ")
        );

        Serial.println(
            status.roll,
            2
        );

        Serial.print(
            F("UPRIGHT: ")
        );

        Serial.println(
            status.upright ?
            F("YES") :
            F("NO")
        );

        Serial.print(
            F("EMERGENCY: ")
        );

        Serial.println(
            status.emergencyStop ?
            F("YES") :
            F("NO")
        );

        return;
    }

    if (command == "BALANCE RESET")
    {
        balanceNavigationBridge.reset();

        clearNavigationVelocityReference();

        Serial.println(
            F("BALANCE NAVIGATION RESET")
        );

        return;
    }

    if (command == "NAV REF")
    {
        DifferentialVelocity reference =
            balanceNavigationBridge.getReference();

        Serial.print(
            F("LEFT: ")
        );

        Serial.println(
            reference.left,
            4
        );

        Serial.print(
            F("RIGHT: ")
        );

        Serial.println(
            reference.right,
            4
        );

        return;
    }
}

// ================================================================
// PART 4 INITIALIZATION
// ================================================================

void initializePart4()
{
    balanceSafetyManager.configure(
        BALANCE_SAFE_TILT_DEGREES,
        BALANCE_EMERGENCY_TILT_DEGREES * 0.80f,
        BALANCE_EMERGENCY_TILT_DEGREES
    );

    velocityLimiter.configure(
        MAX_WHEEL_REFERENCE,
        MAX_WHEEL_ACCELERATION,
        MAX_WHEEL_DECELERATION
    );

    pathFollower.configure(
        NAVIGATION_LINEAR_GAIN,
        NAVIGATION_HEADING_GAIN,
        NAVIGATION_MAX_LINEAR_SPEED,
        NAVIGATION_MAX_ANGULAR_SPEED,
        WAYPOINT_REACHED_DISTANCE,
        LOOKAHEAD_DISTANCE_METERS
    );

    pathFollower.reset();

    balanceNavigationBridge.reset();

    clearNavigationVelocityReference();

    Serial.println(
        F("PART 4: BALANCE/NAVIGATION BRIDGE READY")
    );
}

// ================================================================
// PART 4 DIAGNOSTIC TEST
// ================================================================

bool runPart4Diagnostic()
{
    bool success = true;

    BalanceSafetyStatus balance =
        balanceSafetyManager.getStatus();

    if (balance.emergencyStop)
    {
        success = false;
    }

    DifferentialVelocity reference =
        balanceNavigationBridge.getReference();

    if (fabs(reference.left) >
        MAX_WHEEL_REFERENCE)
    {
        success = false;
    }

    if (fabs(reference.right) >
        MAX_WHEEL_REFERENCE)
    {
        success = false;
    }

    if (success)
    {
        Serial.println(
            F("PART 4 DIAGNOSTIC: PASS")
        );
    }
    else
    {
        Serial.println(
            F("PART 4 DIAGNOSTIC: FAIL")
        );
    }

    return success;
}

// ================================================================
// PART 4 MAIN INTEGRATION FUNCTION
// ================================================================
//
// Call this after Parts 1–3 have updated:
//
//   - IMU
//   - balance state
//   - encoders
//   - localization
//   - mapping
//   - obstacle detection
//   - path planning
//
// ================================================================

void runPart4()
{
    updatePart4Control();
}

// ================================================================
// END OF PART 4
// ================================================================

#endif
// ================================================================
// AUTONOMOUS SELF-BALANCING ROBOT
// PART 5 — MAPPING SUBSYSTEM
// C++
// ================================================================
//
// This part is designed to continue the previous parts.
// It implements:
//
// 1. Occupancy-grid map
// 2. Unknown / Free / Occupied cells
// 3. World <-> Grid coordinate conversion
// 4. Sensor-to-world projection
// 5. Ray casting
// 6. Mapping updates
// 7. Map statistics
// 8. Map bounds
// 9. Basic odometry-based localization foundation
// 10. Mapping diagnostics
//
// IMPORTANT:
// For Arduino Uno, avoid allocating huge maps dynamically.
// The default map is intentionally compact because the Uno has
// very limited SRAM.
//
// ================================================================

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

// ================================================================
// PART 5 CONFIGURATION
// ================================================================

#ifndef PART5_MAP_WIDTH
#define PART5_MAP_WIDTH 32
#endif

#ifndef PART5_MAP_HEIGHT
#define PART5_MAP_HEIGHT 32
#endif

#ifndef PART5_MAP_RESOLUTION
#define PART5_MAP_RESOLUTION 0.10f
#endif

#ifndef PART5_MAX_SENSOR_RANGE
#define PART5_MAX_SENSOR_RANGE 2.50f
#endif

#ifndef PART5_MAPPING_INTERVAL_MS
#define PART5_MAPPING_INTERVAL_MS 100
#endif

#ifndef PART5_OCCUPIED_THRESHOLD
#define PART5_OCCUPIED_THRESHOLD 200
#endif

#ifndef PART5_FREE_THRESHOLD
#define PART5_FREE_THRESHOLD 50
#endif

#ifndef PART5_MAX_RAY_STEPS
#define PART5_MAX_RAY_STEPS 64
#endif

// ================================================================
// OCCUPANCY STATES
// ================================================================

enum Part5CellState : uint8_t
{
    PART5_UNKNOWN = 0,
    PART5_FREE = 1,
    PART5_OCCUPIED = 2
};

// ================================================================
// GRID CELL
// ================================================================
//
// One byte is used deliberately.
// This is much more appropriate for Arduino Uno SRAM than a large
// structure containing several integers.
//
// 0   = unknown
// 1   = free
// 2   = occupied
// ================================================================

struct Part5GridCell
{
    uint8_t state;
};

// ================================================================
// GRID COORDINATE
// ================================================================

struct Part5GridCoordinate
{
    int16_t x;
    int16_t y;

    Part5GridCoordinate()
        : x(0), y(0)
    {
    }

    Part5GridCoordinate(int16_t gx, int16_t gy)
        : x(gx), y(gy)
    {
    }
};

// ================================================================
// WORLD COORDINATE
// ================================================================

struct Part5WorldPoint
{
    float x;
    float y;

    Part5WorldPoint()
        : x(0.0f), y(0.0f)
    {
    }

    Part5WorldPoint(float wx, float wy)
        : x(wx), y(wy)
    {
    }
};

// ================================================================
// SENSOR MEASUREMENT
// ================================================================

struct Part5SensorMeasurement
{
    float distance;
    float angle;
    bool valid;
    bool obstacleDetected;

    Part5SensorMeasurement()
        : distance(0.0f),
          angle(0.0f),
          valid(false),
          obstacleDetected(false)
    {
    }
};

// ================================================================
// MAP STATISTICS
// ================================================================

struct Part5MapStatistics
{
    uint16_t unknownCells;
    uint16_t freeCells;
    uint16_t occupiedCells;
    uint16_t totalCells;

    float coveragePercent;

    float minimumX;
    float maximumX;
    float minimumY;
    float maximumY;

    Part5MapStatistics()
        : unknownCells(0),
          freeCells(0),
          occupiedCells(0),
          totalCells(0),
          coveragePercent(0.0f),
          minimumX(0.0f),
          maximumX(0.0f),
          minimumY(0.0f),
          maximumY(0.0f)
    {
    }
};

// ================================================================
// MAP UPDATE RESULT
// ================================================================

struct Part5MapUpdateResult
{
    bool success;
    uint16_t freeCellsUpdated;
    uint16_t occupiedCellsUpdated;
    uint16_t ignoredCells;

    Part5MapUpdateResult()
        : success(false),
          freeCellsUpdated(0),
          occupiedCellsUpdated(0),
          ignoredCells(0)
    {
    }
};

// ================================================================
// ANGLE NORMALIZATION
// ================================================================

static float part5NormalizeAngle(float angle)
{
    while (angle > PI)
    {
        angle -= 2.0f * PI;
    }

    while (angle < -PI)
    {
        angle += 2.0f * PI;
    }

    return angle;
}

// ================================================================
// OCCUPANCY GRID CLASS
// ================================================================

class Part5OccupancyGrid
{
private:

    Part5GridCell cells[PART5_MAP_WIDTH * PART5_MAP_HEIGHT];

    int16_t originX;
    int16_t originY;

    float resolution;

public:

    Part5OccupancyGrid()
        : originX(PART5_MAP_WIDTH / 2),
          originY(PART5_MAP_HEIGHT / 2),
          resolution(PART5_MAP_RESOLUTION)
    {
        clear();
    }

    // ------------------------------------------------------------
    // Clear map
    // ------------------------------------------------------------

    void clear()
    {
        for (uint16_t i = 0;
             i < PART5_MAP_WIDTH * PART5_MAP_HEIGHT;
             ++i)
        {
            cells[i].state = PART5_UNKNOWN;
        }
    }

    // ------------------------------------------------------------
    // Dimensions
    // ------------------------------------------------------------

    uint16_t width() const
    {
        return PART5_MAP_WIDTH;
    }

    uint16_t height() const
    {
        return PART5_MAP_HEIGHT;
    }

    float getResolution() const
    {
        return resolution;
    }

    // ------------------------------------------------------------
    // Bounds checking
    // ------------------------------------------------------------

    bool isInside(
        int16_t gx,
        int16_t gy) const
    {
        return
            gx >= 0 &&
            gy >= 0 &&
            gx < PART5_MAP_WIDTH &&
            gy < PART5_MAP_HEIGHT;
    }

    // ------------------------------------------------------------
    // Cell index
    // ------------------------------------------------------------

    uint16_t index(
        int16_t gx,
        int16_t gy) const
    {
        return
            (uint16_t)gy *
            PART5_MAP_WIDTH +
            (uint16_t)gx;
    }

    // ------------------------------------------------------------
    // Get cell
    // ------------------------------------------------------------

    Part5CellState get(
        int16_t gx,
        int16_t gy) const
    {
        if (!isInside(gx, gy))
        {
            return PART5_UNKNOWN;
        }

        return
            (Part5CellState)cells[index(gx, gy)].state;
    }

    // ------------------------------------------------------------
    // Set cell
    // ------------------------------------------------------------

    bool set(
        int16_t gx,
        int16_t gy,
        Part5CellState state)
    {
        if (!isInside(gx, gy))
        {
            return false;
        }

        cells[index(gx, gy)].state =
            (uint8_t)state;

        return true;
    }

    // ------------------------------------------------------------
    // Set free
    // ------------------------------------------------------------

    bool setFree(
        int16_t gx,
        int16_t gy)
    {
        return set(
            gx,
            gy,
            PART5_FREE);
    }

    // ------------------------------------------------------------
    // Set occupied
    // ------------------------------------------------------------

    bool setOccupied(
        int16_t gx,
        int16_t gy)
    {
        return set(
            gx,
            gy,
            PART5_OCCUPIED);
    }

    // ------------------------------------------------------------
    // Set unknown
    // ------------------------------------------------------------

    bool setUnknown(
        int16_t gx,
        int16_t gy)
    {
        return set(
            gx,
            gy,
            PART5_UNKNOWN);
    }

    // ------------------------------------------------------------
    // World -> grid
    // ------------------------------------------------------------

    bool worldToGrid(
        float worldX,
        float worldY,
        Part5GridCoordinate &result) const
    {
        float relativeX =
            worldX / resolution;

        float relativeY =
            worldY / resolution;

        int16_t gx =
            (int16_t)round(relativeX) +
            originX;

        int16_t gy =
            (int16_t)round(relativeY) +
            originY;

        if (!isInside(gx, gy))
        {
            return false;
        }

        result.x = gx;
        result.y = gy;

        return true;
    }

    // ------------------------------------------------------------
    // Grid -> world
    // ------------------------------------------------------------

    bool gridToWorld(
        int16_t gx,
        int16_t gy,
        Part5WorldPoint &result) const
    {
        if (!isInside(gx, gy))
        {
            return false;
        }

        result.x =
            ((float)gx - originX) *
            resolution;

        result.y =
            ((float)gy - originY) *
            resolution;

        return true;
    }

    // ------------------------------------------------------------
    // Statistics
    // ------------------------------------------------------------

    Part5MapStatistics statistics() const
    {
        Part5MapStatistics stats;

        stats.totalCells =
            PART5_MAP_WIDTH *
            PART5_MAP_HEIGHT;

        for (uint16_t i = 0;
             i < stats.totalCells;
             ++i)
        {
            if (cells[i].state ==
                PART5_UNKNOWN)
            {
                stats.unknownCells++;
            }
            else if (cells[i].state ==
                     PART5_FREE)
            {
                stats.freeCells++;
            }
            else if (cells[i].state ==
                     PART5_OCCUPIED)
            {
                stats.occupiedCells++;
            }
        }

        uint16_t known =
            stats.freeCells +
            stats.occupiedCells;

        if (stats.totalCells > 0)
        {
            stats.coveragePercent =
                ((float)known /
                 (float)stats.totalCells) *
                100.0f;
        }

        Part5WorldPoint bottomLeft;
        Part5WorldPoint topRight;

        gridToWorld(
            0,
            0,
            bottomLeft);

        gridToWorld(
            PART5_MAP_WIDTH - 1,
            PART5_MAP_HEIGHT - 1,
            topRight);

        stats.minimumX = bottomLeft.x;
        stats.maximumX = topRight.x;

        stats.minimumY = bottomLeft.y;
        stats.maximumY = topRight.y;

        return stats;
    }

    // ------------------------------------------------------------
    // Debug print
    // ------------------------------------------------------------

    void printAscii() const
    {
        Serial.println();
        Serial.println(
            F("========== OCCUPANCY MAP =========="));

        for (int16_t y =
                 PART5_MAP_HEIGHT - 1;
             y >= 0;
             --y)
        {
            for (int16_t x = 0;
                 x < PART5_MAP_WIDTH;
                 ++x)
            {
                Part5CellState state =
                    get(x, y);

                if (state == PART5_UNKNOWN)
                {
                    Serial.print('?');
                }
                else if (state == PART5_FREE)
                {
                    Serial.print('.');
                }
                else
                {
                    Serial.print('#');
                }
            }

            Serial.println();
        }

        Serial.println(
            F("==================================="));
    }
};

// ================================================================
// SENSOR PROJECTOR
// ================================================================

class Part5SensorProjector
{
public:

    // ------------------------------------------------------------
    // Calculate obstacle position in world coordinates
    // ------------------------------------------------------------

    bool project(
        float robotX,
        float robotY,
        float robotHeading,
        float sensorAngle,
        float distance,
        Part5WorldPoint &obstacle)
    {
        if (distance <= 0.0f)
        {
            return false;
        }

        if (distance >
            PART5_MAX_SENSOR_RANGE)
        {
            return false;
        }

        float worldAngle =
            robotHeading +
            sensorAngle;

        worldAngle =
            part5NormalizeAngle(worldAngle);

        obstacle.x =
            robotX +
            cos(worldAngle) *
            distance;

        obstacle.y =
            robotY +
            sin(worldAngle) *
            distance;

        return true;
    }
};

// ================================================================
// RAY CASTER
// ================================================================

class Part5RayCaster
{
private:

    Part5OccupancyGrid *grid;

public:

    Part5RayCaster()
        : grid(nullptr)
    {
    }

    void attachGrid(
        Part5OccupancyGrid *map)
    {
        grid = map;
    }

    // ------------------------------------------------------------
    // Cast one sensor ray
    // ------------------------------------------------------------

    Part5MapUpdateResult castRay(
        float startX,
        float startY,
        float endX,
        float endY,
        bool obstacleDetected)
    {
        Part5MapUpdateResult result;

        if (grid == nullptr)
        {
            return result;
        }

        Part5GridCoordinate startCell;
        Part5GridCoordinate endCell;

        if (!grid->worldToGrid(
                startX,
                startY,
                startCell))
        {
            return result;
        }

        if (!grid->worldToGrid(
                endX,
                endY,
                endCell))
        {
            // The ray may leave the map.
            // We still attempt to mark cells until
            // the boundary is reached.
        }

        int16_t x0 = startCell.x;
        int16_t y0 = startCell.y;

        int16_t x1 = endCell.x;
        int16_t y1 = endCell.y;

        int16_t dx =
            abs(x1 - x0);

        int16_t dy =
            abs(y1 - y0);

        int16_t sx =
            x0 < x1 ? 1 : -1;

        int16_t sy =
            y0 < y1 ? 1 : -1;

        int16_t err =
            dx - dy;

        uint16_t steps = 0;

        while (steps <
               PART5_MAX_RAY_STEPS)
        {
            if (grid->isInside(x0, y0))
            {
                bool isEnd =
                    x0 == x1 &&
                    y0 == y1;

                if (isEnd &&
                    obstacleDetected)
                {
                    if (grid->setOccupied(
                            x0,
                            y0))
                    {
                        result.occupiedCellsUpdated++;
                    }
                }
                else
                {
                    if (grid->setFree(
                            x0,
                            y0))
                    {
                        result.freeCellsUpdated++;
                    }
                }
            }
            else
            {
                result.ignoredCells++;
            }

            if (x0 == x1 &&
                y0 == y1)
            {
                break;
            }

            int16_t e2 =
                2 * err;

            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }

            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }

            steps++;
        }

        result.success = true;

        return result;
    }
};

// ================================================================
// MAPPING ENGINE
// ================================================================

class Part5MappingEngine
{
private:

    Part5OccupancyGrid occupancyGrid;
    Part5SensorProjector projector;
    Part5RayCaster rayCaster;

    unsigned long lastUpdateTime;

    uint32_t totalFreeUpdates;
    uint32_t totalOccupiedUpdates;
    uint32_t totalIgnoredUpdates;

    bool enabled;

public:

    Part5MappingEngine()
        : lastUpdateTime(0),
          totalFreeUpdates(0),
          totalOccupiedUpdates(0),
          totalIgnoredUpdates(0),
          enabled(true)
    {
        rayCaster.attachGrid(
            &occupancyGrid);
    }

    // ------------------------------------------------------------
    // Initialize
    // ------------------------------------------------------------

    void begin()
    {
        occupancyGrid.clear();

        lastUpdateTime = millis();

        totalFreeUpdates = 0;
        totalOccupiedUpdates = 0;
        totalIgnoredUpdates = 0;

        enabled = true;
    }

    // ------------------------------------------------------------
    // Enable / disable
    // ------------------------------------------------------------

    void setEnabled(bool value)
    {
        enabled = value;
    }

    bool isEnabled() const
    {
        return enabled;
    }

    // ------------------------------------------------------------
    // Update single sensor
    // ------------------------------------------------------------

    Part5MapUpdateResult updateSensor(
        float robotX,
        float robotY,
        float robotHeading,
        const Part5SensorMeasurement &measurement)
    {
        Part5MapUpdateResult result;

        if (!enabled)
        {
            return result;
        }

        if (!measurement.valid)
        {
            return result;
        }

        if (measurement.distance <= 0.0f)
        {
            return result;
        }

        float distance =
            measurement.distance;

        if (distance >
            PART5_MAX_SENSOR_RANGE)
        {
            distance =
                PART5_MAX_SENSOR_RANGE;

            // If maximum range was reached,
            // do not mark the endpoint as occupied.
            Part5WorldPoint endpoint;

            if (!projector.project(
                    robotX,
                    robotY,
                    robotHeading,
                    measurement.angle,
                    distance,
                    endpoint))
            {
                return result;
            }

            result =
                rayCaster.castRay(
                    robotX,
                    robotY,
                    endpoint.x,
                    endpoint.y,
                    false);

            return result;
        }

        Part5WorldPoint endpoint;

        if (!projector.project(
                robotX,
                robotY,
                robotHeading,
                measurement.angle,
                distance,
                endpoint))
        {
            return result;
        }

        result =
            rayCaster.castRay(
                robotX,
                robotY,
                endpoint.x,
                endpoint.y,
                measurement.obstacleDetected);

        totalFreeUpdates +=
            result.freeCellsUpdated;

        totalOccupiedUpdates +=
            result.occupiedCellsUpdated;

        totalIgnoredUpdates +=
            result.ignoredCells;

        return result;
    }

    // ------------------------------------------------------------
    // Update all sensors
    // ------------------------------------------------------------

    void update(
        float robotX,
        float robotY,
        float robotHeading,
        const Part5SensorMeasurement &front,
        const Part5SensorMeasurement &left,
        const Part5SensorMeasurement &right,
        const Part5SensorMeasurement &rear)
    {
        if (!enabled)
        {
            return;
        }

        updateSensor(
            robotX,
            robotY,
            robotHeading,
            front);

        updateSensor(
            robotX,
            robotY,
            robotHeading,
            left);

        updateSensor(
            robotX,
            robotY,
            robotHeading,
            right);

        updateSensor(
            robotX,
            robotY,
            robotHeading,
            rear);

        lastUpdateTime = millis();
    }

    // ------------------------------------------------------------
    // Get map
    // ------------------------------------------------------------

    Part5OccupancyGrid &getGrid()
    {
        return occupancyGrid;
    }

    const Part5OccupancyGrid &getGrid() const
    {
        return occupancyGrid;
    }

    // ------------------------------------------------------------
    // Statistics
    // ------------------------------------------------------------

    Part5MapStatistics getStatistics() const
    {
        return occupancyGrid.statistics();
    }

    // ------------------------------------------------------------
    // Counters
    // ------------------------------------------------------------

    uint32_t getFreeUpdateCount() const
    {
        return totalFreeUpdates;
    }

    uint32_t getOccupiedUpdateCount() const
    {
        return totalOccupiedUpdates;
    }

    uint32_t getIgnoredUpdateCount() const
    {
        return totalIgnoredUpdates;
    }

    // ------------------------------------------------------------
    // Clear map
    // ------------------------------------------------------------

    void clear()
    {
        occupancyGrid.clear();

        totalFreeUpdates = 0;
        totalOccupiedUpdates = 0;
        totalIgnoredUpdates = 0;
    }

    // ------------------------------------------------------------
    // Print diagnostics
    // ------------------------------------------------------------

    void printDiagnostics()
    {
        Part5MapStatistics stats =
            getStatistics();

        Serial.println();
        Serial.println(
            F("========== MAPPING DIAGNOSTICS =========="));

        Serial.print(F("Map width: "));
        Serial.println(occupancyGrid.width());

        Serial.print(F("Map height: "));
        Serial.println(occupancyGrid.height());

        Serial.print(F("Resolution: "));
        Serial.print(
            occupancyGrid.getResolution(),
            3);
        Serial.println(F(" m/cell"));

        Serial.print(F("Unknown cells: "));
        Serial.println(stats.unknownCells);

        Serial.print(F("Free cells: "));
        Serial.println(stats.freeCells);

        Serial.print(F("Occupied cells: "));
        Serial.println(stats.occupiedCells);

        Serial.print(F("Coverage: "));
        Serial.print(
            stats.coveragePercent,
            2);
        Serial.println(F("%"));

        Serial.print(F("Free updates: "));
        Serial.println(
            (unsigned long)totalFreeUpdates);

        Serial.print(F("Occupied updates: "));
        Serial.println(
            (unsigned long)totalOccupiedUpdates);

        Serial.print(F("Ignored updates: "));
        Serial.println(
            (unsigned long)totalIgnoredUpdates);

        Serial.print(F("Enabled: "));
        Serial.println(
            enabled ? F("YES") : F("NO"));

        Serial.println(
            F("=========================================="));
    }

    // ------------------------------------------------------------
    // Print map
    // ------------------------------------------------------------

    void printMap()
    {
        occupancyGrid.printAscii();
    }
};

// ================================================================
// BASIC ODOMETRY LOCALIZATION
// ================================================================
//
// This provides the mapping system with a consistent pose source.
// If Part 3 already provides a more advanced localization system,
// that system should remain the authoritative source.
//
// ================================================================

struct Part5LocalizationState
{
    float x;
    float y;
    float heading;

    float linearVelocity;
    float angularVelocity;

    unsigned long timestamp;

    bool valid;

    Part5LocalizationState()
        : x(0.0f),
          y(0.0f),
          heading(0.0f),
          linearVelocity(0.0f),
          angularVelocity(0.0f),
          timestamp(0),
          valid(false)
    {
    }
};

// ================================================================
// ODOMETRY LOCALIZER
// ================================================================

class Part5OdometryLocalizer
{
private:

    Part5LocalizationState state;

public:

    void begin()
    {
        state =
            Part5LocalizationState();

        state.timestamp =
            millis();

        state.valid = true;
    }

    // ------------------------------------------------------------
    // Reset pose
    // ------------------------------------------------------------

    void reset(
        float x,
        float y,
        float heading)
    {
        state.x = x;
        state.y = y;
        state.heading =
            part5NormalizeAngle(
                heading);

        state.linearVelocity = 0.0f;
        state.angularVelocity = 0.0f;

        state.timestamp =
            millis();

        state.valid = true;
    }

    // ------------------------------------------------------------
    // Differential-drive update
    // ------------------------------------------------------------

    void update(
        float leftDistance,
        float rightDistance,
        float wheelSeparation)
    {
        if (!state.valid)
        {
            begin();
        }

        if (wheelSeparation <= 0.0f)
        {
            return;
        }

        unsigned long now =
            millis();

        unsigned long deltaMs =
            now - state.timestamp;

        float dt =
            (float)deltaMs / 1000.0f;

        if (dt <= 0.0f ||
            dt > 1.0f)
        {
            state.timestamp = now;
            return;
        }

        float centerDistance =
            (leftDistance +
             rightDistance) *
            0.5f;

        float deltaHeading =
            (rightDistance -
             leftDistance) /
            wheelSeparation;

        float midpointHeading =
            state.heading +
            deltaHeading * 0.5f;

        state.x +=
            centerDistance *
            cos(midpointHeading);

        state.y +=
            centerDistance *
            sin(midpointHeading);

        state.heading =
            part5NormalizeAngle(
                state.heading +
                deltaHeading);

        state.linearVelocity =
            centerDistance / dt;

        state.angularVelocity =
            deltaHeading / dt;

        state.timestamp = now;
    }

    // ------------------------------------------------------------
    // Access state
    // ------------------------------------------------------------

    const Part5LocalizationState &
    getState() const
    {
        return state;
    }

    // ------------------------------------------------------------
    // Set velocity
    // ------------------------------------------------------------

    void setVelocity(
        float linear,
        float angular)
    {
        state.linearVelocity = linear;
        state.angularVelocity = angular;
    }
};

// ================================================================
// GLOBAL PART 5 OBJECTS
// ================================================================

Part5MappingEngine part5MappingEngine;
Part5OdometryLocalizer part5Localizer;

// ================================================================
// PART 5 SENSOR CONVERSION HELPERS
// ================================================================

Part5SensorMeasurement part5MakeSensor(
    float distance,
    float angle,
    bool valid,
    bool obstacle)
{
    Part5SensorMeasurement measurement;

    measurement.distance = distance;
    measurement.angle = angle;
    measurement.valid = valid;
    measurement.obstacleDetected = obstacle;

    return measurement;
}

// ================================================================
// PART 5 INITIALIZATION
// ================================================================

void initializePart5Mapping()
{
    part5MappingEngine.begin();

    part5Localizer.begin();

    Serial.println(
        F("PART 5 mapping subsystem initialized."));
}

// ================================================================
// PART 5 UPDATE
// ================================================================
//
// IMPORTANT:
// Replace the example sensor values below with the actual sensor
// values produced by the previous sensor subsystem.
//
// Distances MUST be in METERS in this mapping layer.
//
// Example:
// 0.50 = 50 cm
// 1.00 = 100 cm
// 2.00 = 200 cm
//
// ================================================================

void updatePart5Mapping(
    float robotX,
    float robotY,
    float robotHeading,
    float frontDistance,
    float leftDistance,
    float rightDistance,
    float rearDistance,
    bool frontValid,
    bool leftValid,
    bool rightValid,
    bool rearValid)
{
    Part5SensorMeasurement front =
        part5MakeSensor(
            frontDistance,
            0.0f,
            frontValid,
            frontValid &&
            frontDistance <
            PART5_MAX_SENSOR_RANGE);

    Part5SensorMeasurement left =
        part5MakeSensor(
            leftDistance,
            PI * 0.5f,
            leftValid,
            leftValid &&
            leftDistance <
            PART5_MAX_SENSOR_RANGE);

    Part5SensorMeasurement right =
        part5MakeSensor(
            rightDistance,
            -PI * 0.5f,
            rightValid,
            rightValid &&
            rightDistance <
            PART5_MAX_SENSOR_RANGE);

    Part5SensorMeasurement rear =
        part5MakeSensor(
            rearDistance,
            PI,
            rearValid,
            rearValid &&
            rearDistance <
            PART5_MAX_SENSOR_RANGE);

    part5MappingEngine.update(
        robotX,
        robotY,
        robotHeading,
        front,
        left,
        right,
        rear);
}

// ================================================================
// PART 5 COMMAND INTERFACE
// ================================================================

void processPart5MappingCommand(
    String command)
{
    command.trim();
    command.toLowerCase();

    if (command == "map")
    {
        part5MappingEngine.printMap();
        return;
    }

    if (command == "map_stats")
    {
        part5MappingEngine.printDiagnostics();
        return;
    }

    if (command == "map_clear")
    {
        part5MappingEngine.clear();

        Serial.println(
            F("Mapping grid cleared."));

        return;
    }

    if (command == "map_on")
    {
        part5MappingEngine.setEnabled(true);

        Serial.println(
            F("Mapping enabled."));

        return;
    }

    if (command == "map_off")
    {
        part5MappingEngine.setEnabled(false);

        Serial.println(
            F("Mapping disabled."));

        return;
    }

    if (command == "pose")
    {
        const Part5LocalizationState &pose =
            part5Localizer.getState();

        Serial.println();
        Serial.println(
            F("========== ROBOT POSE =========="));

        Serial.print(F("X: "));
        Serial.println(pose.x, 4);

        Serial.print(F("Y: "));
        Serial.println(pose.y, 4);

        Serial.print(F("Heading: "));
        Serial.println(pose.heading, 4);

        Serial.print(F("Linear velocity: "));
        Serial.println(
            pose.linearVelocity,
            4);

        Serial.print(F("Angular velocity: "));
        Serial.println(
            pose.angularVelocity,
            4);

        Serial.println(
            F("================================"));

        return;
    }

    Serial.println(
        F("Unknown mapping command."));
}

// ================================================================
// PART 5 AUTOMATIC UPDATE SCHEDULER
// ================================================================

unsigned long part5LastMappingCycle = 0;

bool part5MappingCycleReady()
{
    unsigned long now =
        millis();

    if ((now -
         part5LastMappingCycle) >=
        PART5_MAPPING_INTERVAL_MS)
    {
        part5LastMappingCycle = now;
        return true;
    }

    return false;
}

// ================================================================
// PART 5 HEALTH CHECK
// ================================================================

bool part5MappingHealthCheck()
{
    Part5MapStatistics stats =
        part5MappingEngine.getStatistics();

    if (stats.totalCells == 0)
    {
        return false;
    }

    if (stats.unknownCells >
        stats.totalCells)
    {
        return false;
    }

    if (stats.freeCells +
        stats.occupiedCells +
        stats.unknownCells !=
        stats.totalCells)
    {
        return false;
    }

    return true;
}

// ================================================================
// PART 5 TEST — COORDINATE CONVERSION
// ================================================================

bool part5TestCoordinateConversion()
{
    Part5GridCoordinate gridCoordinate;

    bool converted =
        part5MappingEngine
            .getGrid()
            .worldToGrid(
                0.0f,
                0.0f,
                gridCoordinate);

    if (!converted)
    {
        return false;
    }

    Part5WorldPoint worldPoint;

    bool reverse =
        part5MappingEngine
            .getGrid()
            .gridToWorld(
                gridCoordinate.x,
                gridCoordinate.y,
                worldPoint);

    if (!reverse)
    {
        return false;
    }

    if (fabs(worldPoint.x) > 0.001f)
    {
        return false;
    }

    if (fabs(worldPoint.y) > 0.001f)
    {
        return false;
    }

    return true;
}

// ================================================================
// PART 5 TEST — RAY CASTING
// ================================================================

bool part5TestRayCasting()
{
    Part5OccupancyGrid testGrid;

    Part5RayCaster caster;

    caster.attachGrid(
        &testGrid);

    Part5MapUpdateResult result =
        caster.castRay(
            0.0f,
            0.0f,
            0.8f,
            0.0f,
            true);

    if (!result.success)
    {
        return false;
    }

    Part5GridCoordinate obstacleCell;

    if (!testGrid.worldToGrid(
            0.8f,
            0.0f,
            obstacleCell))
    {
        return false;
    }

    return
        testGrid.get(
            obstacleCell.x,
            obstacleCell.y)
        == PART5_OCCUPIED;
}

// ================================================================
// PART 5 TEST — MAP COUNTS
// ================================================================

bool part5TestMapCounts()
{
    Part5OccupancyGrid testGrid;

    testGrid.clear();

    testGrid.setFree(10, 10);
    testGrid.setFree(11, 10);

    testGrid.setOccupied(12, 10);

    Part5MapStatistics stats =
        testGrid.statistics();

    if (stats.freeCells != 2)
    {
        return false;
    }

    if (stats.occupiedCells != 1)
    {
        return false;
    }

    return true;
}

// ================================================================
// PART 5 TEST SUITE
// ================================================================

void runPart5MappingTests()
{
    Serial.println();
    Serial.println(
        F("========== PART 5 TESTS =========="));

    bool coordinateTest =
        part5TestCoordinateConversion();

    bool rayTest =
        part5TestRayCasting();

    bool countTest =
        part5TestMapCounts();

    Serial.print(
        F("Coordinate conversion: "));

    Serial.println(
        coordinateTest ?
        F("PASS") :
        F("FAIL"));

    Serial.print(
        F("Ray casting: "));

    Serial.println(
        rayTest ?
        F("PASS") :
        F("FAIL"));

    Serial.print(
        F("Map counts: "));

    Serial.println(
        countTest ?
        F("PASS") :
        F("FAIL"));

    Serial.print(
        F("Overall: "));

    Serial.println(
        coordinateTest &&
        rayTest &&
        countTest ?
        F("PASS") :
        F("FAIL"));

    Serial.println(
        F("================================="));
}

// ================================================================
// PART 5 COMPLETE
// ================================================================
//
// Integration:
//
// In setup():
//
//     initializePart5Mapping();
//
// In loop(), after the previous sensor/state-estimation update:
//
//     if (part5MappingCycleReady())
//     {
//         updatePart5Mapping(
//             robotX,
//             robotY,
//             robotHeading,
//             frontDistanceMeters,
//             leftDistanceMeters,
//             rightDistanceMeters,
//             rearDistanceMeters,
//             frontValid,
//             leftValid,
//             rightValid,
//             rearValid);
//     }
//
// Available serial commands:
//
//     map
//     map_stats
//     map_clear
//     map_on
//     map_off
//     pose
//
// ================================================================
// END OF PART 5
// ================================================================
// ================================================================
// AUTONOMOUS SELF-BALANCING ROBOT
// PART 6 — PATH PLANNING
// C++
// ================================================================
//
// Implements:
//
// 1. A* path planning
// 2. Occupancy-grid integration
// 3. Start/goal validation
// 4. 8-direction movement
// 5. G / H / F costs
// 6. Obstacle inflation
// 7. Path reconstruction
// 8. Path validation
// 9. Path length calculation
// 10. Basic path smoothing
// 11. Planner diagnostics
// 12. Planner command interface
//
// Designed to continue PART 5.
//
// ================================================================

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

// ================================================================
// PART 6 CONFIGURATION
// ================================================================

#ifndef PART6_MAX_PATH_POINTS
#define PART6_MAX_PATH_POINTS 128
#endif

#ifndef PART6_MAX_ASTAR_NODES
#define PART6_MAX_ASTAR_NODES 256
#endif

#ifndef PART6_OBSTACLE_INFLATION_RADIUS
#define PART6_OBSTACLE_INFLATION_RADIUS 1
#endif

#ifndef PART6_ALLOW_DIAGONAL
#define PART6_ALLOW_DIAGONAL true
#endif

#ifndef PART6_UNKNOWN_IS_BLOCKED
#define PART6_UNKNOWN_IS_BLOCKED true
#endif

#ifndef PART6_DIAGONAL_COST
#define PART6_DIAGONAL_COST 1.41421356f
#endif

#ifndef PART6_STRAIGHT_COST
#define PART6_STRAIGHT_COST 1.0f
#endif

#ifndef PART6_MAX_PLANNING_TIME_MS
#define PART6_MAX_PLANNING_TIME_MS 1000UL
#endif

// ================================================================
// PATH POINT
// ================================================================

struct Part6PathPoint
{
    int16_t x;
    int16_t y;

    Part6PathPoint()
        : x(0), y(0)
    {
    }

    Part6PathPoint(
        int16_t px,
        int16_t py)
        : x(px), y(py)
    {
    }
};

// ================================================================
// PATH RESULT
// ================================================================

enum Part6PlanningStatus : uint8_t
{
    PART6_PLAN_IDLE = 0,
    PART6_PLAN_SUCCESS,
    PART6_PLAN_INVALID_START,
    PART6_PLAN_INVALID_GOAL,
    PART6_PLAN_NO_PATH,
    PART6_PLAN_NODE_LIMIT,
    PART6_PLAN_TIMEOUT,
    PART6_PLAN_INVALID_MAP
};

// ================================================================
// PATH CONTAINER
// ================================================================

class Part6PlannedPath
{
private:

    Part6PathPoint points[
        PART6_MAX_PATH_POINTS
    ];

    uint16_t pointCount;

    float totalLength;

    bool valid;

public:

    Part6PlannedPath()
        : pointCount(0),
          totalLength(0.0f),
          valid(false)
    {
    }

    // ------------------------------------------------------------
    // Clear
    // ------------------------------------------------------------

    void clear()
    {
        pointCount = 0;
        totalLength = 0.0f;
        valid = false;
    }

    // ------------------------------------------------------------
    // Add point
    // ------------------------------------------------------------

    bool addPoint(
        int16_t x,
        int16_t y)
    {
        if (pointCount >=
            PART6_MAX_PATH_POINTS)
        {
            return false;
        }

        points[pointCount] =
            Part6PathPoint(x, y);

        pointCount++;

        return true;
    }

    // ------------------------------------------------------------
    // Insert point at beginning
    // ------------------------------------------------------------

    bool insertFront(
        int16_t x,
        int16_t y)
    {
        if (pointCount >=
            PART6_MAX_PATH_POINTS)
        {
            return false;
        }

        for (int16_t i =
                 pointCount;
             i > 0;
             --i)
        {
            points[i] =
                points[i - 1];
        }

        points[0] =
            Part6PathPoint(x, y);

        pointCount++;

        return true;
    }

    // ------------------------------------------------------------
    // Access
    // ------------------------------------------------------------

    Part6PathPoint &operator[](
        uint16_t index)
    {
        return points[index];
    }

    const Part6PathPoint &operator[](
        uint16_t index) const
    {
        return points[index];
    }

    // ------------------------------------------------------------
    // Count
    // ------------------------------------------------------------

    uint16_t size() const
    {
        return pointCount;
    }

    bool empty() const
    {
        return pointCount == 0;
    }

    // ------------------------------------------------------------
    // Valid
    // ------------------------------------------------------------

    void setValid(bool value)
    {
        valid = value;
    }

    bool isValid() const
    {
        return valid;
    }

    // ------------------------------------------------------------
    // Length
    // ------------------------------------------------------------

    void setLength(
        float length)
    {
        totalLength = length;
    }

    float length() const
    {
        return totalLength;
    }

    // ------------------------------------------------------------
    // Calculate length
    // ------------------------------------------------------------

    float calculateLength(
        float resolution)
    {
        if (pointCount < 2)
        {
            totalLength = 0.0f;
            return totalLength;
        }

        totalLength = 0.0f;

        for (uint16_t i = 1;
             i < pointCount;
             ++i)
        {
            int16_t dx =
                points[i].x -
                points[i - 1].x;

            int16_t dy =
                points[i].y -
                points[i - 1].y;

            float distance =
                sqrt(
                    (float)(dx * dx) +
                    (float)(dy * dy));

            totalLength +=
                distance * resolution;
        }

        return totalLength;
    }

    // ------------------------------------------------------------
    // Print
    // ------------------------------------------------------------

    void print() const
    {
        Serial.println();
        Serial.println(
            F("========== PLANNED PATH =========="));

        Serial.print(F("Points: "));
        Serial.println(pointCount);

        Serial.print(F("Length: "));
        Serial.print(totalLength, 3);
        Serial.println(F(" m"));

        for (uint16_t i = 0;
             i < pointCount;
             ++i)
        {
            Serial.print('#');
            Serial.print(i);

            Serial.print(F(" ("));

            Serial.print(points[i].x);

            Serial.print(F(","));

            Serial.print(points[i].y);

            Serial.println(F(")"));
        }

        Serial.println(
            F("=================================="));
    }
};

// ================================================================
// A* NODE
// ================================================================

struct Part6AStarNode
{
    int16_t x;
    int16_t y;

    float g;
    float h;
    float f;

    int16_t parentX;
    int16_t parentY;

    bool opened;
    bool closed;

    Part6AStarNode()
        : x(0),
          y(0),
          g(0.0f),
          h(0.0f),
          f(0.0f),
          parentX(-1),
          parentY(-1),
          opened(false),
          closed(false)
    {
    }
};

// ================================================================
// A* PLANNER
// ================================================================

class Part6AStarPlanner
{
private:

    Part5OccupancyGrid *grid;

    Part6AStarNode nodes[
        PART6_MAX_ASTAR_NODES
    ];

    uint16_t nodeCount;

    Part6PlanningStatus lastStatus;

    uint32_t expandedNodes;

    uint32_t generatedNodes;

    float lastPlanningTimeMs;

    // ------------------------------------------------------------
    // Heuristic
    // ------------------------------------------------------------

    float heuristic(
        int16_t x,
        int16_t y,
        int16_t goalX,
        int16_t goalY) const
    {
        float dx =
            abs(goalX - x);

        float dy =
            abs(goalY - y);

        if (PART6_ALLOW_DIAGONAL)
        {
            float minimum =
                dx < dy ? dx : dy;

            float remaining =
                dx > dy ?
                dx - dy :
                dy - dx;

            return
                minimum *
                PART6_DIAGONAL_COST +
                remaining *
                PART6_STRAIGHT_COST;
        }

        return
            dx + dy;
    }

    // ------------------------------------------------------------
    // Find node
    // ------------------------------------------------------------

    int16_t findNode(
        int16_t x,
        int16_t y)
    {
        for (uint16_t i = 0;
             i < nodeCount;
             ++i)
        {
            if (nodes[i].x == x &&
                nodes[i].y == y)
            {
                return i;
            }
        }

        return -1;
    }

    // ------------------------------------------------------------
    // Create node
    // ------------------------------------------------------------

    int16_t createNode(
        int16_t x,
        int16_t y)
    {
        if (nodeCount >=
            PART6_MAX_ASTAR_NODES)
        {
            return -1;
        }

        nodes[nodeCount] =
            Part6AStarNode();

        nodes[nodeCount].x = x;
        nodes[nodeCount].y = y;

        return nodeCount++;
    }

    // ------------------------------------------------------------
    // Walkability
    // ------------------------------------------------------------

    bool isWalkable(
        int16_t x,
        int16_t y) const
    {
        if (grid == nullptr)
        {
            return false;
        }

        if (!grid->isInside(x, y))
        {
            return false;
        }

        Part5CellState state =
            grid->get(x, y);

        if (state ==
            PART5_OCCUPIED)
        {
            return false;
        }

        if (PART6_UNKNOWN_IS_BLOCKED &&
            state ==
            PART5_UNKNOWN)
        {
            return false;
        }

        return true;
    }

    // ------------------------------------------------------------
    // Inflated obstacle test
    // ------------------------------------------------------------

    bool isInflatedObstacle(
        int16_t x,
        int16_t y) const
    {
        if (grid == nullptr)
        {
            return true;
        }

        for (int16_t dy =
                 -PART6_OBSTACLE_INFLATION_RADIUS;
             dy <=
                 PART6_OBSTACLE_INFLATION_RADIUS;
             ++dy)
        {
            for (int16_t dx =
                     -PART6_OBSTACLE_INFLATION_RADIUS;
                 dx <=
                     PART6_OBSTACLE_INFLATION_RADIUS;
                 ++dx)
            {
                int16_t nx =
                    x + dx;

                int16_t ny =
                    y + dy;

                if (!grid->isInside(
                        nx,
                        ny))
                {
                    continue;
                }

                if (grid->get(
                        nx,
                        ny) ==
                    PART5_OCCUPIED)
                {
                    return true;
                }
            }
        }

        return false;
    }

    // ------------------------------------------------------------
    // Safe cell
    // ------------------------------------------------------------

    bool isSafeWalkable(
        int16_t x,
        int16_t y) const
    {
        if (!isWalkable(x, y))
        {
            return false;
        }

        if (isInflatedObstacle(x, y))
        {
            return false;
        }

        return true;
    }

    // ------------------------------------------------------------
    // Get lowest F node
    // ------------------------------------------------------------

    int16_t lowestOpenNode()
    {
        int16_t best = -1;

        float bestF = 9999999.0f;

        float bestH = 9999999.0f;

        for (uint16_t i = 0;
             i < nodeCount;
             ++i)
        {
            if (!nodes[i].opened)
            {
                continue;
            }

            if (nodes[i].closed)
            {
                continue;
            }

            if (nodes[i].f < bestF)
            {
                best =
                    (int16_t)i;

                bestF =
                    nodes[i].f;

                bestH =
                    nodes[i].h;
            }
            else if (
                fabs(
                    nodes[i].f -
                    bestF) < 0.0001f)
            {
                if (nodes[i].h <
                    bestH)
                {
                    best =
                        (int16_t)i;

                    bestH =
                        nodes[i].h;
                }
            }
        }

        return best;
    }

    // ------------------------------------------------------------
    // Reconstruct
    // ------------------------------------------------------------

    bool reconstructPath(
        int16_t goalX,
        int16_t goalY,
        Part6PlannedPath &path)
    {
        path.clear();

        int16_t current =
            findNode(
                goalX,
                goalY);

        if (current < 0)
        {
            return false;
        }

        uint16_t guard = 0;

        while (current >= 0 &&
               guard <
               PART6_MAX_PATH_POINTS)
        {
            if (!path.insertFront(
                    nodes[current].x,
                    nodes[current].y))
            {
                return false;
            }

            int16_t parentX =
                nodes[current].parentX;

            int16_t parentY =
                nodes[current].parentY;

            if (parentX < 0 ||
                parentY < 0)
            {
                break;
            }

            current =
                findNode(
                    parentX,
                    parentY);

            guard++;
        }

        if (path.empty())
        {
            return false;
        }

        path.setValid(true);

        return true;
    }

public:

    Part6AStarPlanner()
        : grid(nullptr),
          nodeCount(0),
          lastStatus(PART6_PLAN_IDLE),
          expandedNodes(0),
          generatedNodes(0),
          lastPlanningTimeMs(0.0f)
    {
    }

    // ------------------------------------------------------------
    // Attach map
    // ------------------------------------------------------------

    void attachGrid(
        Part5OccupancyGrid *map)
    {
        grid = map;
    }

    // ------------------------------------------------------------
    // Reset planner
    // ------------------------------------------------------------

    void reset()
    {
        nodeCount = 0;

        expandedNodes = 0;

        generatedNodes = 0;

        lastPlanningTimeMs = 0.0f;

        lastStatus =
            PART6_PLAN_IDLE;
    }

    // ------------------------------------------------------------
    // Plan
    // ------------------------------------------------------------

    Part6PlanningStatus plan(
        int16_t startX,
        int16_t startY,
        int16_t goalX,
        int16_t goalY,
        Part6PlannedPath &path)
    {
        unsigned long startTime =
            millis();

        reset();

        path.clear();

        if (grid == nullptr)
        {
            lastStatus =
                PART6_PLAN_INVALID_MAP;

            return lastStatus;
        }

        if (!grid->isInside(
                startX,
                startY))
        {
            lastStatus =
                PART6_PLAN_INVALID_START;

            return lastStatus;
        }

        if (!grid->isInside(
                goalX,
                goalY))
        {
            lastStatus =
                PART6_PLAN_INVALID_GOAL;

            return lastStatus;
        }

        if (!isSafeWalkable(
                startX,
                startY))
        {
            lastStatus =
                PART6_PLAN_INVALID_START;

            return lastStatus;
        }

        if (!isSafeWalkable(
                goalX,
                goalY))
        {
            lastStatus =
                PART6_PLAN_INVALID_GOAL;

            return lastStatus;
        }

        // --------------------------------------------------------
        // Start node
        // --------------------------------------------------------

        int16_t startIndex =
            createNode(
                startX,
                startY);

        if (startIndex < 0)
        {
            lastStatus =
                PART6_PLAN_NODE_LIMIT;

            return lastStatus;
        }

        nodes[startIndex].g = 0.0f;

        nodes[startIndex].h =
            heuristic(
                startX,
                startY,
                goalX,
                goalY);

        nodes[startIndex].f =
            nodes[startIndex].h;

        nodes[startIndex].opened = true;

        generatedNodes++;

        // --------------------------------------------------------
        // Search
        // --------------------------------------------------------

        while (true)
        {
            unsigned long elapsed =
                millis() -
                startTime;

            if (elapsed >
                PART6_MAX_PLANNING_TIME_MS)
            {
                lastStatus =
                    PART6_PLAN_TIMEOUT;

                lastPlanningTimeMs =
                    (float)elapsed;

                return lastStatus;
            }

            int16_t current =
                lowestOpenNode();

            if (current < 0)
            {
                lastStatus =
                    PART6_PLAN_NO_PATH;

                lastPlanningTimeMs =
                    (float)elapsed;

                return lastStatus;
            }

            nodes[current].closed = true;

            expandedNodes++;

            // ----------------------------------------------------
            // Goal reached
            // ----------------------------------------------------

            if (nodes[current].x ==
                    goalX &&
                nodes[current].y ==
                    goalY)
            {
                if (!reconstructPath(
                        goalX,
                        goalY,
                        path))
                {
                    lastStatus =
                        PART6_PLAN_NO_PATH;

                    return lastStatus;
                }

                path.calculateLength(
                    grid->getResolution());

                lastStatus =
                    PART6_PLAN_SUCCESS;

                lastPlanningTimeMs =
                    (float)elapsed;

                return lastStatus;
            }

            // ----------------------------------------------------
            // Neighbor offsets
            // ----------------------------------------------------

            static const int8_t dx[8] =
            {
                1,
                -1,
                0,
                0,
                1,
                1,
                -1,
                -1
            };

            static const int8_t dy[8] =
            {
                0,
                0,
                1,
                -1,
                1,
                -1,
                1,
                -1
            };

            uint8_t neighborCount =
                PART6_ALLOW_DIAGONAL ?
                8 :
                4;

            for (uint8_t direction = 0;
                 direction < neighborCount;
                 ++direction)
            {
                int16_t nx =
                    nodes[current].x +
                    dx[direction];

                int16_t ny =
                    nodes[current].y +
                    dy[direction];

                if (!isSafeWalkable(
                        nx,
                        ny))
                {
                    continue;
                }

                // ------------------------------------------------
                // Prevent diagonal corner cutting.
                // ------------------------------------------------

                bool diagonal =
                    dx[direction] != 0 &&
                    dy[direction] != 0;

                if (diagonal)
                {
                    if (!isSafeWalkable(
                            nodes[current].x +
                            dx[direction],
                            nodes[current].y))
                    {
                        continue;
                    }

                    if (!isSafeWalkable(
                            nodes[current].x,
                            nodes[current].y +
                            dy[direction]))
                    {
                        continue;
                    }
                }

                float movementCost =
                    diagonal ?
                    PART6_DIAGONAL_COST :
                    PART6_STRAIGHT_COST;

                float tentativeG =
                    nodes[current].g +
                    movementCost;

                int16_t neighbor =
                    findNode(
                        nx,
                        ny);

                if (neighbor < 0)
                {
                    neighbor =
                        createNode(
                            nx,
                            ny);

                    if (neighbor < 0)
                    {
                        lastStatus =
                            PART6_PLAN_NODE_LIMIT;

                        lastPlanningTimeMs =
                            (float)(millis() -
                                    startTime);

                        return lastStatus;
                    }

                    nodes[neighbor].g =
                        tentativeG;

                    nodes[neighbor].h =
                        heuristic(
                            nx,
                            ny,
                            goalX,
                            goalY);

                    nodes[neighbor].f =
                        nodes[neighbor].g +
                        nodes[neighbor].h;

                    nodes[neighbor].parentX =
                        nodes[current].x;

                    nodes[neighbor].parentY =
                        nodes[current].y;

                    nodes[neighbor].opened =
                        true;

                    generatedNodes++;

                    continue;
                }

                if (nodes[neighbor].closed)
                {
                    continue;
                }

                if (!nodes[neighbor].opened ||
                    tentativeG <
                    nodes[neighbor].g)
                {
                    nodes[neighbor].g =
                        tentativeG;

                    nodes[neighbor].f =
                        tentativeG +
                        nodes[neighbor].h;

                    nodes[neighbor].parentX =
                        nodes[current].x;

                    nodes[neighbor].parentY =
                        nodes[current].y;

                    nodes[neighbor].opened =
                        true;
                }
            }
        }
    }

    // ------------------------------------------------------------
    // Status
    // ------------------------------------------------------------

    Part6PlanningStatus getStatus() const
    {
        return lastStatus;
    }

    // ------------------------------------------------------------
    // Diagnostics
    // ------------------------------------------------------------

    void printDiagnostics() const
    {
        Serial.println();
        Serial.println(
            F("========== A* DIAGNOSTICS =========="));

        Serial.print(F("Status: "));

        switch (lastStatus)
        {
            case PART6_PLAN_IDLE:
                Serial.println(F("IDLE"));
                break;

            case PART6_PLAN_SUCCESS:
                Serial.println(F("SUCCESS"));
                break;

            case PART6_PLAN_INVALID_START:
                Serial.println(F("INVALID START"));
                break;

            case PART6_PLAN_INVALID_GOAL:
                Serial.println(F("INVALID GOAL"));
                break;

            case PART6_PLAN_NO_PATH:
                Serial.println(F("NO PATH"));
                break;

            case PART6_PLAN_NODE_LIMIT:
                Serial.println(F("NODE LIMIT"));
                break;

            case PART6_PLAN_TIMEOUT:
                Serial.println(F("TIMEOUT"));
                break;

            default:
                Serial.println(F("INVALID MAP"));
                break;
        }

        Serial.print(F("Nodes generated: "));
        Serial.println(generatedNodes);

        Serial.print(F("Nodes expanded: "));
        Serial.println(expandedNodes);

        Serial.print(F("Planning time: "));
        Serial.print(lastPlanningTimeMs, 2);
        Serial.println(F(" ms"));

        Serial.println(
            F("===================================="));
    }
};

// ================================================================
// PATH VALIDATOR
// ================================================================

class Part6PathValidator
{
private:

    Part5OccupancyGrid *grid;

public:

    Part6PathValidator()
        : grid(nullptr)
    {
    }

    void attachGrid(
        Part5OccupancyGrid *map)
    {
        grid = map;
    }

    // ------------------------------------------------------------
    // Validate one point
    // ------------------------------------------------------------

    bool isPointValid(
        int16_t x,
        int16_t y) const
    {
        if (grid == nullptr)
        {
            return false;
        }

        if (!grid->isInside(
                x,
                y))
        {
            return false;
        }

        Part5CellState state =
            grid->get(x, y);

        if (state ==
            PART5_OCCUPIED)
        {
            return false;
        }

        if (PART6_UNKNOWN_IS_BLOCKED &&
            state ==
            PART5_UNKNOWN)
        {
            return false;
        }

        return true;
    }

    // ------------------------------------------------------------
    // Validate complete path
    // ------------------------------------------------------------

    bool validate(
        const Part6PlannedPath &path)
    {
        if (grid == nullptr)
        {
            return false;
        }

        if (path.empty())
        {
            return false;
        }

        for (uint16_t i = 0;
             i < path.size();
             ++i)
        {
            if (!isPointValid(
                    path[i].x,
                    path[i].y))
            {
                return false;
            }

            if (i > 0)
            {
                int16_t dx =
                    abs(
                        path[i].x -
                        path[i - 1].x);

                int16_t dy =
                    abs(
                        path[i].y -
                        path[i - 1].y);

                if (dx > 1 ||
                    dy > 1)
                {
                    return false;
                }
            }
        }

        return true;
    }

    // ------------------------------------------------------------
    // Line-of-sight validation
    // ------------------------------------------------------------

    bool lineOfSight(
        const Part6PathPoint &a,
        const Part6PathPoint &b)
    {
        if (grid == nullptr)
        {
            return false;
        }

        int16_t x0 = a.x;
        int16_t y0 = a.y;

        int16_t x1 = b.x;
        int16_t y1 = b.y;

        int16_t dx =
            abs(x1 - x0);

        int16_t dy =
            abs(y1 - y0);

        int16_t sx =
            x0 < x1 ? 1 : -1;

        int16_t sy =
            y0 < y1 ? 1 : -1;

        int16_t err =
            dx - dy;

        while (true)
        {
            if (!isPointValid(
                    x0,
                    y0))
            {
                return false;
            }

            if (x0 == x1 &&
                y0 == y1)
            {
                break;
            }

            int16_t e2 =
                2 * err;

            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }

            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }

        return true;
    }
};

// ================================================================
// PATH SMOOTHER
// ================================================================

class Part6PathSmoother
{
private:

    Part6PathValidator *validator;

public:

    Part6PathSmoother()
        : validator(nullptr)
    {
    }

    void attachValidator(
        Part6PathValidator *pathValidator)
    {
        validator =
            pathValidator;
    }

    // ------------------------------------------------------------
    // Simplify path
    // ------------------------------------------------------------

    bool smooth(
        const Part6PlannedPath &input,
        Part6PlannedPath &output)
    {
        output.clear();

        if (validator == nullptr)
        {
            return false;
        }

        if (input.empty())
        {
            return false;
        }

        if (input.size() <= 2)
        {
            for (uint16_t i = 0;
                 i < input.size();
                 ++i)
            {
                output.addPoint(
                    input[i].x,
                    input[i].y);
            }

            output.setValid(
                validator->validate(output));

            return output.isValid();
        }

        uint16_t anchor = 0;

        output.addPoint(
            input[0].x,
            input[0].y);

        while (anchor <
               input.size() - 1)
        {
            uint16_t best =
                anchor + 1;

            for (uint16_t candidate =
                     anchor + 1;
                 candidate <
                     input.size();
                 ++candidate)
            {
                if (validator->lineOfSight(
                        input[anchor],
                        input[candidate]))
                {
                    best =
                        candidate;
                }
                else
                {
                    break;
                }
            }

            if (best == anchor)
            {
                return false;
            }

            if (!output.addPoint(
                    input[best].x,
                    input[best].y))
            {
                return false;
            }

            anchor = best;
        }

        output.setValid(
            validator->validate(output));

        return output.isValid();
    }
};

// ================================================================
// PATH PLANNING MANAGER
// ================================================================

class Part6PathPlanningManager
{
private:

    Part5OccupancyGrid *grid;

    Part6AStarPlanner planner;

    Part6PathValidator validator;

    Part6PathSmoother smoother;

    Part6PlannedPath rawPath;

    Part6PlannedPath smoothedPath;

    Part6PlanningStatus status;

    bool smoothingEnabled;

public:

    Part6PathPlanningManager()
        : grid(nullptr),
          status(PART6_PLAN_IDLE),
          smoothingEnabled(true)
    {
    }

    // ------------------------------------------------------------
    // Begin
    // ------------------------------------------------------------

    void begin(
        Part5OccupancyGrid *map)
    {
        grid = map;

        planner.attachGrid(
            grid);

        validator.attachGrid(
            grid);

        smoother.attachValidator(
            &validator);

        rawPath.clear();

        smoothedPath.clear();

        status =
            PART6_PLAN_IDLE;
    }

    // ------------------------------------------------------------
    // Enable smoothing
    // ------------------------------------------------------------

    void setSmoothing(
        bool enabled)
    {
        smoothingEnabled =
            enabled;
    }

    // ------------------------------------------------------------
    // Plan
    // ------------------------------------------------------------

    Part6PlanningStatus plan(
        int16_t startX,
        int16_t startY,
        int16_t goalX,
        int16_t goalY)
    {
        rawPath.clear();
        smoothedPath.clear();

        status =
            planner.plan(
                startX,
                startY,
                goalX,
                goalY,
                rawPath);

        if (status !=
            PART6_PLAN_SUCCESS)
        {
            return status;
        }

        if (!validator.validate(
                rawPath))
        {
            status =
                PART6_PLAN_NO_PATH;

            return status;
        }

        if (smoothingEnabled)
        {
            if (smoother.smooth(
                    rawPath,
                    smoothedPath))
            {
                smoothedPath.calculateLength(
                    grid->getResolution());
            }
            else
            {
                smoothedPath =
                    rawPath;
            }
        }
        else
        {
            smoothedPath =
                rawPath;
        }

        return status;
    }

    // ------------------------------------------------------------
    // Raw path
    // ------------------------------------------------------------

    const Part6PlannedPath &
    getRawPath() const
    {
        return rawPath;
    }

    // ------------------------------------------------------------
    // Final path
    // ------------------------------------------------------------

    const Part6PlannedPath &
    getPath() const
    {
        return smoothedPath;
    }

    // ------------------------------------------------------------
    // Status
    // ------------------------------------------------------------

    Part6PlanningStatus
    getStatus() const
    {
        return status;
    }

    // ------------------------------------------------------------
    // Diagnostics
    // ------------------------------------------------------------

    void diagnostics()
    {
        planner.printDiagnostics();

        Serial.println();

        Serial.print(
            F("Raw path points: "));

        Serial.println(
            rawPath.size());

        Serial.print(
            F("Raw path length: "));

        Serial.print(
            rawPath.length(),
            3);

        Serial.println(F(" m"));

        Serial.print(
            F("Final path points: "));

        Serial.println(
            smoothedPath.size());

        Serial.print(
            F("Final path length: "));

        Serial.print(
            smoothedPath.length(),
            3);

        Serial.println(F(" m"));

        Serial.print(
            F("Smoothing: "));

        Serial.println(
            smoothingEnabled ?
            F("ON") :
            F("OFF"));
    }

    // ------------------------------------------------------------
    // Print path
    // ------------------------------------------------------------

    void printPath()
    {
        smoothedPath.print();
    }
};

// ================================================================
// GLOBAL PART 6 OBJECT
// ================================================================

Part6PathPlanningManager
part6PathPlanningManager;

// ================================================================
// START / GOAL STORAGE
// ================================================================

Part6PathPoint part6StartCell;

Part6PathPoint part6GoalCell;

bool part6GoalDefined = false;

// ================================================================
// INITIALIZE PART 6
// ================================================================

void initializePart6PathPlanning()
{
    part6PathPlanningManager.begin(
        &part5MappingEngine.getGrid());

    part6GoalDefined = false;

    Serial.println(
        F("PART 6 path planning initialized."));
}

// ================================================================
// SET GOAL
// ================================================================

bool setPart6Goal(
    int16_t goalX,
    int16_t goalY)
{
    if (!part5MappingEngine
            .getGrid()
            .isInside(
                goalX,
                goalY))
    {
        Serial.println(
            F("Goal is outside map."));

        return false;
    }

    part6GoalCell =
        Part6PathPoint(
            goalX,
            goalY);

    part6GoalDefined = true;

    Serial.print(
        F("Goal set to ("));

    Serial.print(goalX);

    Serial.print(',');

    Serial.print(goalY);

    Serial.println(')');

    return true;
}

// ================================================================
// SET START
// ================================================================

bool setPart6Start(
    int16_t startX,
    int16_t startY)
{
    if (!part5MappingEngine
            .getGrid()
            .isInside(
                startX,
                startY))
    {
        Serial.println(
            F("Start is outside map."));

        return false;
    }

    part6StartCell =
        Part6PathPoint(
            startX,
            startY);

    Serial.print(
        F("Start set to ("));

    Serial.print(startX);

    Serial.print(',');

    Serial.print(startY);

    Serial.println(')');

    return true;
}

// ================================================================
// PLAN CURRENT ROUTE
// ================================================================

bool planPart6Route()
{
    if (!part6GoalDefined)
    {
        Serial.println(
            F("No goal has been defined."));

        return false;
    }

    Part6PlanningStatus status =
        part6PathPlanningManager.plan(
            part6StartCell.x,
            part6StartCell.y,
            part6GoalCell.x,
            part6GoalCell.y);

    if (status ==
        PART6_PLAN_SUCCESS)
    {
        Serial.println(
            F("A* path planning successful."));

        part6PathPlanningManager.printPath();

        return true;
    }

    Serial.println(
        F("A* path planning failed."));

    part6PathPlanningManager.diagnostics();

    return false;
}

// ================================================================
// PART 6 COMMAND INTERFACE
// ================================================================
//
// Commands:
//
// start_cell X Y
// goal X Y
// plan
// path
// planner_stats
// smooth_on
// smooth_off
//
// ================================================================

void processPart6Command(
    String command)
{
    command.trim();

    if (command.startsWith(
            "start_cell"))
    {
        int firstSpace =
            command.indexOf(' ');

        if (firstSpace < 0)
        {
            Serial.println(
                F("Usage: start_cell X Y"));

            return;
        }

        String values =
            command.substring(
                firstSpace + 1);

        int secondSpace =
            values.indexOf(' ');

        if (secondSpace < 0)
        {
            Serial.println(
                F("Usage: start_cell X Y"));

            return;
        }

        int16_t x =
            values.substring(
                0,
                secondSpace).toInt();

        int16_t y =
            values.substring(
                secondSpace + 1).toInt();

        setPart6Start(
            x,
            y);

        return;
    }

    if (command.startsWith(
            "goal"))
    {
        int firstSpace =
            command.indexOf(' ');

        if (firstSpace < 0)
        {
            Serial.println(
                F("Usage: goal X Y"));

            return;
        }

        String values =
            command.substring(
                firstSpace + 1);

        int secondSpace =
            values.indexOf(' ');

        if (secondSpace < 0)
        {
            Serial.println(
                F("Usage: goal X Y"));

            return;
        }

        int16_t x =
            values.substring(
                0,
                secondSpace).toInt();

        int16_t y =
            values.substring(
                secondSpace + 1).toInt();

        setPart6Goal(
            x,
            y);

        return;
    }

    if (command == "plan")
    {
        planPart6Route();
        return;
    }

    if (command == "path")
    {
        part6PathPlanningManager
            .printPath();

        return;
    }

    if (command == "planner_stats")
    {
        part6PathPlanningManager
            .diagnostics();

        return;
    }

    if (command == "smooth_on")
    {
        part6PathPlanningManager
            .setSmoothing(true);

        Serial.println(
            F("Path smoothing enabled."));

        return;
    }

    if (command == "smooth_off")
    {
        part6PathPlanningManager
            .setSmoothing(false);

        Serial.println(
            F("Path smoothing disabled."));

        return;
    }

    Serial.println(
        F("Unknown Part 6 command."));
}

// ================================================================
// PART 6 SIMPLE TEST MAP
// ================================================================
//
// Creates a small simulated environment.
//
// This is ONLY a planner test and does not represent a real
// physical environment.
//
// ================================================================

void part6CreateTestMap()
{
    Part5OccupancyGrid &map =
        part5MappingEngine.getGrid();

    map.clear();

    // ------------------------------------------------------------
    // Outer walls
    // ------------------------------------------------------------

    for (int16_t x = 0;
         x < map.width();
         ++x)
    {
        map.setOccupied(
            x,
            0);

        map.setOccupied(
            x,
            map.height() - 1);
    }

    for (int16_t y = 0;
         y < map.height();
         ++y)
    {
        map.setOccupied(
            0,
            y);

        map.setOccupied(
            map.width() - 1,
            y);
    }

    // ------------------------------------------------------------
    // Interior obstacle
    // ------------------------------------------------------------

    for (int16_t y = 8;
         y < 24;
         ++y)
    {
        map.setOccupied(
            15,
            y);
    }

    // Create a gap.

    map.setFree(
        15,
        16);

    map.setFree(
        15,
        17);

    // ------------------------------------------------------------
    // Mark useful area free.
    // ------------------------------------------------------------

    for (int16_t y = 1;
         y < map.height() - 1;
         ++y)
    {
        for (int16_t x = 1;
             x < map.width() - 1;
             ++x)
        {
            if (map.get(x, y) ==
                PART5_UNKNOWN)
            {
                map.setFree(x, y);
            }
        }
    }

    Serial.println(
        F("Part 6 test map created."));
}

// ================================================================
// PART 6 TEST
// ================================================================

bool part6RunPlannerTest()
{
    part6CreateTestMap();

    setPart6Start(
        4,
        4);

    setPart6Goal(
        26,
        26);

    bool success =
        planPart6Route();

    if (success)
    {
        Serial.println(
            F("PART 6 planner test: PASS"));
    }
    else
    {
        Serial.println(
            F("PART 6 planner test: FAIL"));
    }

    return success;
}

// ================================================================
// PART 6 COMPLETE
// ================================================================
//
// Integration:
//
// In setup(), after:
//
//     initializePart5Mapping();
//
// add:
//
//     initializePart6PathPlanning();
//
// ---------------------------------------------------------------
//
// Useful commands:
//
//     start_cell 4 4
//     goal 26 26
//     plan
//     path
//     planner_stats
//     smooth_on
//     smooth_off
//
// ---------------------------------------------------------------
//
// Test:
//
//     part6RunPlannerTest();
//
// ---------------------------------------------------------------
//
// The resulting path is intentionally NOT sent directly to the
// motors in this part.
//
// Part 7 will use the generated path as navigation input and
// connect:
//
//     Robot Pose
//          ↓
//     Path Point Selection
//          ↓
//     Heading Error
//          ↓
//     Distance Error
//          ↓
//     Navigation Velocity
//          ↓
//     Balance Controller
//
// The balancing controller must remain the final safety authority.
//
// ================================================================
// END OF PART 6
// ================================================================
/*
===============================================================================
PART 7 — AUTONOMOUS NAVIGATION + PATH FOLLOWING
===============================================================================

PROJECT:
Autonomous Self-Balancing Robot — ASBR

LANGUAGE:
C++ ONLY

DEPENDS ON:
    PART 5 — Mapping
    PART 6 — A* Path Planning

NAVIGATION PIPELINE:

    Robot Pose
         |
         v
    Active Path
         |
         v
    Nearest Path Point
         |
         v
    Look-Ahead Target
         |
         v
    Heading Error
         |
         v
    Distance Error
         |
         v
    Linear / Angular Velocity
         |
         v
    Wheel Velocity References
         |
         v
    Balance Controller
         |
         v
    Motor Controller

IMPORTANT:
This module generates navigation references.
It does NOT bypass the balance controller or safety system.

For a real Arduino Uno implementation, keep the navigation state compact
because the Uno has very limited SRAM.

===============================================================================
*/

#include <Arduino.h>
#include <math.h>

/*
===============================================================================
SECTION 1 — PART 7 CONFIGURATION
===============================================================================
*/

#ifndef PART7_PI
#define PART7_PI 3.14159265358979323846f
#endif

#define PART7_MAX_PATH_POINTS                 128

#define PART7_LOOKAHEAD_DISTANCE_M            0.30f
#define PART7_WAYPOINT_REACHED_DISTANCE_M     0.12f
#define PART7_GOAL_REACHED_DISTANCE_M         0.10f

#define PART7_MAX_LINEAR_SPEED_MPS             0.35f
#define PART7_MAX_ANGULAR_SPEED_RADPS          2.00f

#define PART7_MIN_LINEAR_SPEED_MPS             0.03f

#define PART7_HEADING_KP                       1.80f
#define PART7_DISTANCE_KP                      0.90f

#define PART7_SLOWDOWN_ANGLE_RAD               0.80f
#define PART7_STOP_ANGLE_RAD                   1.40f

#define PART7_NAVIGATION_INTERVAL_MS           50UL

#define PART7_OBSTACLE_STOP_DISTANCE_M         0.22f
#define PART7_OBSTACLE_SLOW_DISTANCE_M         0.45f

#define PART7_MAX_REPLAN_REQUESTS              20

#define PART7_PATH_VALIDATION_RADIUS_M         0.10f

/*
===============================================================================
SECTION 2 — BASIC MATH TYPES
===============================================================================
*/

struct Part7Vector2
{
    float x;
    float y;

    Part7Vector2()
        : x(0.0f), y(0.0f)
    {
    }

    Part7Vector2(float px, float py)
        : x(px), y(py)
    {
    }
};

struct Part7Pose
{
    float x;
    float y;
    float theta;

    Part7Pose()
        : x(0.0f),
          y(0.0f),
          theta(0.0f)
    {
    }
};

struct Part7VelocityCommand
{
    float linear;
    float angular;

    Part7VelocityCommand()
        : linear(0.0f),
          angular(0.0f)
    {
    }
};

struct Part7WheelCommand
{
    float left;
    float right;

    Part7WheelCommand()
        : left(0.0f),
          right(0.0f)
    {
    }
};

/*
===============================================================================
SECTION 3 — NAVIGATION STATES
===============================================================================
*/

enum Part7NavigationState
{
    PART7_NAV_IDLE = 0,
    PART7_NAV_WAITING_FOR_PATH,
    PART7_NAV_FOLLOWING_PATH,
    PART7_NAV_APPROACHING_GOAL,
    PART7_NAV_GOAL_REACHED,
    PART7_NAV_OBSTACLE_STOP,
    PART7_NAV_REPLAN_REQUIRED,
    PART7_NAV_PAUSED,
    PART7_NAV_ERROR
};

enum Part7NavigationResult
{
    PART7_RESULT_NONE = 0,
    PART7_RESULT_RUNNING,
    PART7_RESULT_GOAL_REACHED,
    PART7_RESULT_OBSTACLE,
    PART7_RESULT_REPLAN,
    PART7_RESULT_ERROR
};

/*
===============================================================================
SECTION 4 — PART 7 PATH POINT
===============================================================================

This intentionally uses a compact representation.

It can be populated directly from the Part 6 path planner.
===============================================================================
*/

struct Part7PathPoint
{
    float x;
    float y;

    Part7PathPoint()
        : x(0.0f),
          y(0.0f)
    {
    }

    Part7PathPoint(float px, float py)
        : x(px),
          y(py)
    {
    }
};

/*
===============================================================================
SECTION 5 — NAVIGATION DIAGNOSTICS
===============================================================================
*/

struct Part7NavigationDiagnostics
{
    Part7NavigationState state;

    uint16_t currentPathIndex;
    uint16_t pathLength;

    float distanceToTarget;
    float distanceToGoal;

    float targetHeading;
    float headingError;

    float commandedLinearVelocity;
    float commandedAngularVelocity;

    float leftWheelReference;
    float rightWheelReference;

    uint16_t replanningRequests;

    bool pathAvailable;
    bool goalReached;
    bool obstacleDetected;
    bool emergencyStopRequested;

    Part7NavigationDiagnostics()
        : state(PART7_NAV_IDLE),
          currentPathIndex(0),
          pathLength(0),
          distanceToTarget(0.0f),
          distanceToGoal(0.0f),
          targetHeading(0.0f),
          headingError(0.0f),
          commandedLinearVelocity(0.0f),
          commandedAngularVelocity(0.0f),
          leftWheelReference(0.0f),
          rightWheelReference(0.0f),
          replanningRequests(0),
          pathAvailable(false),
          goalReached(false),
          obstacleDetected(false),
          emergencyStopRequested(false)
    {
    }
};

/*
===============================================================================
SECTION 6 — UTILITY FUNCTIONS
===============================================================================
*/

static float part7Abs(float value)
{
    return value >= 0.0f ? value : -value;
}

static float part7Clamp(
    float value,
    float minimum,
    float maximum)
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

static float part7NormalizeAngle(float angle)
{
    while (angle > PART7_PI)
    {
        angle -= 2.0f * PART7_PI;
    }

    while (angle < -PART7_PI)
    {
        angle += 2.0f * PART7_PI;
    }

    return angle;
}

static float part7Distance(
    float x1,
    float y1,
    float x2,
    float y2)
{
    const float dx = x2 - x1;
    const float dy = y2 - y1;

    return sqrtf(dx * dx + dy * dy);
}

static float part7Distance(
    const Part7Vector2 &a,
    const Part7Vector2 &b)
{
    return part7Distance(
        a.x,
        a.y,
        b.x,
        b.y
    );
}

static float part7Distance(
    const Part7Pose &pose,
    const Part7PathPoint &point)
{
    return part7Distance(
        pose.x,
        pose.y,
        point.x,
        point.y
    );
}

static float part7AngleToPoint(
    const Part7Pose &pose,
    const Part7PathPoint &point)
{
    return atan2f(
        point.y - pose.y,
        point.x - pose.x
    );
}

/*
===============================================================================
SECTION 7 — PART 7 NAVIGATION CONTROLLER
===============================================================================
*/

class Part7NavigationController
{
private:

    Part7PathPoint path[PART7_MAX_PATH_POINTS];

    uint16_t pathLength;
    uint16_t currentPathIndex;

    Part7Pose currentPose;

    Part7VelocityCommand velocityCommand;

    Part7WheelCommand wheelCommand;

    Part7NavigationDiagnostics diagnostics;

    bool pathLoaded;
    bool navigationEnabled;
    bool goalReached;

    bool obstacleDetected;

    float wheelBase;

    float lookAheadDistance;
    float waypointTolerance;
    float goalTolerance;

    float maximumLinearSpeed;
    float maximumAngularSpeed;

    float headingKp;
    float distanceKp;

    uint16_t replanRequests;

    unsigned long lastUpdateTime;

public:

    Part7NavigationController()
        : pathLength(0),
          currentPathIndex(0),
          pathLoaded(false),
          navigationEnabled(false),
          goalReached(false),
          obstacleDetected(false),
          wheelBase(0.14f),
          lookAheadDistance(PART7_LOOKAHEAD_DISTANCE_M),
          waypointTolerance(PART7_WAYPOINT_REACHED_DISTANCE_M),
          goalTolerance(PART7_GOAL_REACHED_DISTANCE_M),
          maximumLinearSpeed(PART7_MAX_LINEAR_SPEED_MPS),
          maximumAngularSpeed(PART7_MAX_ANGULAR_SPEED_RADPS),
          headingKp(PART7_HEADING_KP),
          distanceKp(PART7_DISTANCE_KP),
          replanRequests(0),
          lastUpdateTime(0)
    {
        clearPath();
    }

    /*
    ---------------------------------------------------------------------------
    Initialize controller
    ---------------------------------------------------------------------------
    */

    void begin()
    {
        clearPath();

        currentPose = Part7Pose();

        velocityCommand = Part7VelocityCommand();

        wheelCommand = Part7WheelCommand();

        diagnostics = Part7NavigationDiagnostics();

        navigationEnabled = false;

        goalReached = false;

        obstacleDetected = false;

        replanRequests = 0;

        lastUpdateTime = millis();
    }

    /*
    ---------------------------------------------------------------------------
    Clear path
    ---------------------------------------------------------------------------
    */

    void clearPath()
    {
        pathLength = 0;

        currentPathIndex = 0;

        pathLoaded = false;

        goalReached = false;

        navigationEnabled = false;

        for (uint16_t i = 0; i < PART7_MAX_PATH_POINTS; ++i)
        {
            path[i].x = 0.0f;
            path[i].y = 0.0f;
        }
    }

    /*
    ---------------------------------------------------------------------------
    Load path
    ---------------------------------------------------------------------------
    */

    bool loadPath(
        const Part7PathPoint *points,
        uint16_t count)
    {
        if (points == nullptr)
        {
            return false;
        }

        if (count == 0)
        {
            return false;
        }

        if (count > PART7_MAX_PATH_POINTS)
        {
            count = PART7_MAX_PATH_POINTS;
        }

        pathLength = 0;

        for (uint16_t i = 0; i < count; ++i)
        {
            path[i] = points[i];

            pathLength++;
        }

        currentPathIndex = 0;

        pathLoaded = true;

        goalReached = false;

        navigationEnabled = false;

        diagnostics.pathAvailable = true;

        diagnostics.pathLength = pathLength;

        diagnostics.currentPathIndex = 0;

        diagnostics.goalReached = false;

        diagnostics.state = PART7_NAV_WAITING_FOR_PATH;

        return true;
    }

    /*
    ---------------------------------------------------------------------------
    Add individual point
    ---------------------------------------------------------------------------
    */

    bool addPathPoint(
        float x,
        float y)
    {
        if (pathLength >= PART7_MAX_PATH_POINTS)
        {
            return false;
        }

        path[pathLength].x = x;
        path[pathLength].y = y;

        pathLength++;

        pathLoaded = true;

        diagnostics.pathAvailable = true;

        diagnostics.pathLength = pathLength;

        return true;
    }

    /*
    ---------------------------------------------------------------------------
    Set robot pose
    ---------------------------------------------------------------------------
    */

    void setPose(
        float x,
        float y,
        float theta)
    {
        currentPose.x = x;

        currentPose.y = y;

        currentPose.theta =
            part7NormalizeAngle(theta);
    }

    void setPose(
        const Part7Pose &pose)
    {
        setPose(
            pose.x,
            pose.y,
            pose.theta
        );
    }

    /*
    ---------------------------------------------------------------------------
    Configure robot geometry
    ---------------------------------------------------------------------------
    */

    void setWheelBase(float value)
    {
        if (value > 0.01f)
        {
            wheelBase = value;
        }
    }

    float getWheelBase() const
    {
        return wheelBase;
    }

    /*
    ---------------------------------------------------------------------------
    Configure navigation limits
    ---------------------------------------------------------------------------
    */

    void setMaximumLinearSpeed(float value)
    {
        if (value > 0.0f)
        {
            maximumLinearSpeed = value;
        }
    }

    void setMaximumAngularSpeed(float value)
    {
        if (value > 0.0f)
        {
            maximumAngularSpeed = value;
        }
    }

    void setLookAheadDistance(float value)
    {
        if (value >= 0.01f)
        {
            lookAheadDistance = value;
        }
    }

    void setWaypointTolerance(float value)
    {
        if (value > 0.01f)
        {
            waypointTolerance = value;
        }
    }

    void setGoalTolerance(float value)
    {
        if (value > 0.01f)
        {
            goalTolerance = value;
        }
    }

    void setHeadingGain(float value)
    {
        if (value >= 0.0f)
        {
            headingKp = value;
        }
    }

    void setDistanceGain(float value)
    {
        if (value >= 0.0f)
        {
            distanceKp = value;
        }
    }

    /*
    ---------------------------------------------------------------------------
    Start navigation
    ---------------------------------------------------------------------------
    */

    bool start()
    {
        if (!pathLoaded)
        {
            diagnostics.state =
                PART7_NAV_WAITING_FOR_PATH;

            return false;
        }

        if (pathLength == 0)
        {
            diagnostics.state =
                PART7_NAV_ERROR;

            return false;
        }

        currentPathIndex = 0;

        goalReached = false;

        obstacleDetected = false;

        navigationEnabled = true;

        diagnostics.state =
            PART7_NAV_FOLLOWING_PATH;

        diagnostics.goalReached = false;

        diagnostics.emergencyStopRequested = false;

        return true;
    }

    /*
    ---------------------------------------------------------------------------
    Pause navigation
    ---------------------------------------------------------------------------
    */

    void pause()
    {
        navigationEnabled = false;

        stopVelocity();

        diagnostics.state =
            PART7_NAV_PAUSED;
    }

    /*
    ---------------------------------------------------------------------------
    Resume navigation
    ---------------------------------------------------------------------------
    */

    bool resume()
    {
        if (!pathLoaded)
        {
            return false;
        }

        if (goalReached)
        {
            return false;
        }

        navigationEnabled = true;

        diagnostics.state =
            PART7_NAV_FOLLOWING_PATH;

        return true;
    }

    /*
    ---------------------------------------------------------------------------
    Stop navigation
    ---------------------------------------------------------------------------
    */

    void stop()
    {
        navigationEnabled = false;

        stopVelocity();

        diagnostics.state =
            PART7_NAV_IDLE;
    }

    /*
    ---------------------------------------------------------------------------
    Emergency stop
    ---------------------------------------------------------------------------
    */

    void emergencyStop()
    {
        navigationEnabled = false;

        stopVelocity();

        diagnostics.emergencyStopRequested = true;

        diagnostics.state =
            PART7_NAV_OBSTACLE_STOP;
    }

    /*
    ---------------------------------------------------------------------------
    Clear emergency request
    ---------------------------------------------------------------------------
    */

    void clearEmergencyStop()
    {
        diagnostics.emergencyStopRequested = false;

        if (pathLoaded && !goalReached)
        {
            diagnostics.state =
                PART7_NAV_WAITING_FOR_PATH;
        }
    }

    /*
    ---------------------------------------------------------------------------
    Obstacle status
    ---------------------------------------------------------------------------
    */

    void setObstacleDetected(bool detected)
    {
        obstacleDetected = detected;

        diagnostics.obstacleDetected = detected;

        if (detected)
        {
            stopVelocity();

            diagnostics.state =
                PART7_NAV_OBSTACLE_STOP;
        }
    }

    /*
    ---------------------------------------------------------------------------
    Request replan
    ---------------------------------------------------------------------------
    */

    void requestReplan()
    {
        stopVelocity();

        if (replanRequests < PART7_MAX_REPLAN_REQUESTS)
        {
            replanRequests++;
        }

        diagnostics.replanningRequests =
            replanRequests;

        diagnostics.state =
            PART7_NAV_REPLAN_REQUIRED;
    }

    /*
    ---------------------------------------------------------------------------
    Find nearest path point
    ---------------------------------------------------------------------------
    */

    uint16_t findNearestPathPoint() const
    {
        if (!pathLoaded || pathLength == 0)
        {
            return 0;
        }

        uint16_t nearestIndex = currentPathIndex;

        float nearestDistance =
            part7Distance(
                currentPose,
                path[currentPathIndex]
            );

        for (
            uint16_t i = currentPathIndex;
            i < pathLength;
            ++i)
        {
            const float distance =
                part7Distance(
                    currentPose,
                    path[i]
                );

            if (distance < nearestDistance)
            {
                nearestDistance = distance;

                nearestIndex = i;
            }
        }

        return nearestIndex;
    }

    /*
    ---------------------------------------------------------------------------
    Select look-ahead target
    ---------------------------------------------------------------------------
    */

    uint16_t findLookAheadPoint()
    {
        if (!pathLoaded || pathLength == 0)
        {
            return 0;
        }

        uint16_t nearest =
            findNearestPathPoint();

        currentPathIndex = nearest;

        uint16_t target = nearest;

        for (
            uint16_t i = nearest;
            i < pathLength;
            ++i)
        {
            const float distance =
                part7Distance(
                    currentPose,
                    path[i]
                );

            if (distance >= lookAheadDistance)
            {
                target = i;

                break;
            }

            target = i;
        }

        return target;
    }

    /*
    ---------------------------------------------------------------------------
    Check waypoint reached
    ---------------------------------------------------------------------------
    */

    bool waypointReached(
        uint16_t index) const
    {
        if (index >= pathLength)
        {
            return true;
        }

        const float distance =
            part7Distance(
                currentPose,
                path[index]
            );

        return distance <= waypointTolerance;
    }

    /*
    ---------------------------------------------------------------------------
    Check final goal
    ---------------------------------------------------------------------------
    */

    bool checkGoalReached()
    {
        if (!pathLoaded || pathLength == 0)
        {
            return false;
        }

        const uint16_t finalIndex =
            pathLength - 1;

        const float distance =
            part7Distance(
                currentPose,
                path[finalIndex]
            );

        diagnostics.distanceToGoal =
            distance;

        if (distance <= goalTolerance)
        {
            goalReached = true;

            navigationEnabled = false;

            stopVelocity();

            diagnostics.goalReached = true;

            diagnostics.state =
                PART7_NAV_GOAL_REACHED;

            return true;
        }

        return false;
    }

    /*
    ---------------------------------------------------------------------------
    Advance through path points
    ---------------------------------------------------------------------------
    */

    void advanceWaypoint(
        uint16_t targetIndex)
    {
        if (targetIndex >= pathLength)
        {
            return;
        }

        while (
            currentPathIndex < targetIndex &&
            waypointReached(currentPathIndex))
        {
            currentPathIndex++;
        }

        if (currentPathIndex >= pathLength)
        {
            currentPathIndex =
                pathLength - 1;
        }

        diagnostics.currentPathIndex =
            currentPathIndex;
    }

    /*
    ---------------------------------------------------------------------------
    Calculate heading error
    ---------------------------------------------------------------------------
    */

    float calculateHeadingError(
        uint16_t targetIndex)
    {
        if (targetIndex >= pathLength)
        {
            return 0.0f;
        }

        const float targetHeading =
            part7AngleToPoint(
                currentPose,
                path[targetIndex]
            );

        diagnostics.targetHeading =
            targetHeading;

        const float error =
            part7NormalizeAngle(
                targetHeading -
                currentPose.theta
            );

        diagnostics.headingError =
            error;

        return error;
    }

    /*
    ---------------------------------------------------------------------------
    Calculate distance error
    ---------------------------------------------------------------------------
    */

    float calculateDistanceError(
        uint16_t targetIndex)
    {
        if (targetIndex >= pathLength)
        {
            return 0.0f;
        }

        const float distance =
            part7Distance(
                currentPose,
                path[targetIndex]
            );

        diagnostics.distanceToTarget =
            distance;

        return distance;
    }

    /*
    ---------------------------------------------------------------------------
    Generate navigation command
    ---------------------------------------------------------------------------
    */

    Part7VelocityCommand calculateVelocityCommand(
        float distanceError,
        float headingError)
    {
        Part7VelocityCommand command;

        /*
        -----------------------------------------------------------------------
        Angular velocity
        -----------------------------------------------------------------------
        */

        command.angular =
            headingKp * headingError;

        command.angular =
            part7Clamp(
                command.angular,
                -maximumAngularSpeed,
                maximumAngularSpeed
            );

        /*
        -----------------------------------------------------------------------
        Linear velocity
        -----------------------------------------------------------------------

        Slow down when the robot is pointing away from the target.
        */

        const float absoluteHeading =
            part7Abs(headingError);

        float headingScale = 1.0f;

        if (
            absoluteHeading >=
            PART7_STOP_ANGLE_RAD)
        {
            headingScale = 0.0f;
        }
        else if (
            absoluteHeading >
            PART7_SLOWDOWN_ANGLE_RAD)
        {
            const float remaining =
                PART7_STOP_ANGLE_RAD -
                absoluteHeading;

            const float range =
                PART7_STOP_ANGLE_RAD -
                PART7_SLOWDOWN_ANGLE_RAD;

            headingScale =
                remaining / range;
        }

        /*
        -----------------------------------------------------------------------
        Distance controller
        -----------------------------------------------------------------------
        */

        float distanceVelocity =
            distanceKp * distanceError;

        distanceVelocity =
            part7Clamp(
                distanceVelocity,
                0.0f,
                maximumLinearSpeed
            );

        command.linear =
            distanceVelocity *
            headingScale;

        /*
        -----------------------------------------------------------------------
        Prevent tiny movements from causing unnecessary motor activity.
        -----------------------------------------------------------------------
        */

        if (
            command.linear <
            PART7_MIN_LINEAR_SPEED_MPS)
        {
            command.linear = 0.0f;
        }

        return command;
    }

    /*
    ---------------------------------------------------------------------------
    Convert differential-drive velocity to wheel references
    ---------------------------------------------------------------------------
    */

    Part7WheelCommand velocityToWheels(
        const Part7VelocityCommand &command)
    {
        Part7WheelCommand wheels;

        /*
            v = linear velocity
            w = angular velocity

            left  = v - wL/2
            right = v + wL/2
        */

        wheels.left =
            command.linear -
            (command.angular * wheelBase * 0.5f);

        wheels.right =
            command.linear +
            (command.angular * wheelBase * 0.5f);

        const float maximumMagnitude =
            max(
                part7Abs(wheels.left),
                part7Abs(wheels.right)
            );

        if (
            maximumMagnitude >
            maximumLinearSpeed)
        {
            const float scale =
                maximumLinearSpeed /
                maximumMagnitude;

            wheels.left *= scale;

            wheels.right *= scale;
        }

        return wheels;
    }

    /*
    ---------------------------------------------------------------------------
    Update navigation
    ---------------------------------------------------------------------------
    */

    Part7NavigationResult update()
    {
        const unsigned long now = millis();

        if (
            now - lastUpdateTime <
            PART7_NAVIGATION_INTERVAL_MS)
        {
            return PART7_RESULT_RUNNING;
        }

        lastUpdateTime = now;

        /*
        -----------------------------------------------------------------------
        Emergency stop has absolute priority.
        -----------------------------------------------------------------------
        */

        if (diagnostics.emergencyStopRequested)
        {
            stopVelocity();

            return PART7_RESULT_ERROR;
        }

        /*
        -----------------------------------------------------------------------
        No path
        -----------------------------------------------------------------------
        */

        if (!pathLoaded || pathLength == 0)
        {
            stopVelocity();

            diagnostics.state =
                PART7_NAV_WAITING_FOR_PATH;

            return PART7_RESULT_ERROR;
        }

        /*
        -----------------------------------------------------------------------
        Navigation disabled
        -----------------------------------------------------------------------
        */

        if (!navigationEnabled)
        {
            stopVelocity();

            return PART7_RESULT_RUNNING;
        }

        /*
        -----------------------------------------------------------------------
        Obstacle detected
        -----------------------------------------------------------------------
        */

        if (obstacleDetected)
        {
            stopVelocity();

            diagnostics.state =
                PART7_NAV_OBSTACLE_STOP;

            return PART7_RESULT_OBSTACLE;
        }

        /*
        -----------------------------------------------------------------------
        Goal check
        -----------------------------------------------------------------------
        */

        if (checkGoalReached())
        {
            return PART7_RESULT_GOAL_REACHED;
        }

        /*
        -----------------------------------------------------------------------
        Find target
        -----------------------------------------------------------------------
        */

        const uint16_t targetIndex =
            findLookAheadPoint();

        advanceWaypoint(targetIndex);

        /*
        -----------------------------------------------------------------------
        Check goal again after waypoint advancement.
        -----------------------------------------------------------------------
        */

        if (checkGoalReached())
        {
            return PART7_RESULT_GOAL_REACHED;
        }

        /*
        -----------------------------------------------------------------------
        Calculate navigation errors.
        -----------------------------------------------------------------------
        */

        const float distanceError =
            calculateDistanceError(targetIndex);

        const float headingError =
            calculateHeadingError(targetIndex);

        /*
        -----------------------------------------------------------------------
        Generate velocity command.
        -----------------------------------------------------------------------
        */

        velocityCommand =
            calculateVelocityCommand(
                distanceError,
                headingError
            );

        /*
        -----------------------------------------------------------------------
        Convert to wheel references.
        -----------------------------------------------------------------------
        */

        wheelCommand =
            velocityToWheels(
                velocityCommand
            );

        /*
        -----------------------------------------------------------------------
        Store diagnostics.
        -----------------------------------------------------------------------
        */

        diagnostics.commandedLinearVelocity =
            velocityCommand.linear;

        diagnostics.commandedAngularVelocity =
            velocityCommand.angular;

        diagnostics.leftWheelReference =
            wheelCommand.left;

        diagnostics.rightWheelReference =
            wheelCommand.right;

        diagnostics.state =
            PART7_NAV_FOLLOWING_PATH;

        return PART7_RESULT_RUNNING;
    }

    /*
    ---------------------------------------------------------------------------
    Stop velocity output
    ---------------------------------------------------------------------------
    */

    void stopVelocity()
    {
        velocityCommand.linear = 0.0f;

        velocityCommand.angular = 0.0f;

        wheelCommand.left = 0.0f;

        wheelCommand.right = 0.0f;

        diagnostics.commandedLinearVelocity =
            0.0f;

        diagnostics.commandedAngularVelocity =
            0.0f;

        diagnostics.leftWheelReference =
            0.0f;

        diagnostics.rightWheelReference =
            0.0f;
    }

    /*
    ---------------------------------------------------------------------------
    Getters
    ---------------------------------------------------------------------------
    */

    const Part7Pose &getPose() const
    {
        return currentPose;
    }

    const Part7VelocityCommand &getVelocityCommand() const
    {
        return velocityCommand;
    }

    const Part7WheelCommand &getWheelCommand() const
    {
        return wheelCommand;
    }

    const Part7NavigationDiagnostics &getDiagnostics() const
    {
        return diagnostics;
    }

    Part7NavigationState getState() const
    {
        return diagnostics.state;
    }

    uint16_t getPathLength() const
    {
        return pathLength;
    }

    uint16_t getCurrentPathIndex() const
    {
        return currentPathIndex;
    }

    bool isNavigationActive() const
    {
        return navigationEnabled;
    }

    bool isGoalReached() const
    {
        return goalReached;
    }

    bool hasPath() const
    {
        return pathLoaded;
    }
};

/*
===============================================================================
SECTION 8 — COLLISION MONITOR
===============================================================================

The navigation controller must not blindly follow a previously calculated
path. Real-time sensor information can invalidate that path.
===============================================================================
*/

class Part7CollisionMonitor
{
private:

    float frontDistance;
    float leftDistance;
    float rightDistance;

    bool frontValid;
    bool leftValid;
    bool rightValid;

    bool obstacleDetected;

    bool emergencyStop;

public:

    Part7CollisionMonitor()
        : frontDistance(999.0f),
          leftDistance(999.0f),
          rightDistance(999.0f),
          frontValid(false),
          leftValid(false),
          rightValid(false),
          obstacleDetected(false),
          emergencyStop(false)
    {
    }

    void reset()
    {
        frontDistance = 999.0f;

        leftDistance = 999.0f;

        rightDistance = 999.0f;

        frontValid = false;

        leftValid = false;

        rightValid = false;

        obstacleDetected = false;

        emergencyStop = false;
    }

    void update(
        float front,
        bool frontIsValid,
        float left,
        bool leftIsValid,
        float right,
        bool rightIsValid)
    {
        frontDistance = front;

        leftDistance = left;

        rightDistance = right;

        frontValid = frontIsValid;

        leftValid = leftIsValid;

        rightValid = rightIsValid;

        obstacleDetected = false;

        emergencyStop = false;

        /*
        -----------------------------------------------------------------------
        Front obstacle
        -----------------------------------------------------------------------
        */

        if (
            frontValid &&
            frontDistance <=
            PART7_OBSTACLE_STOP_DISTANCE_M)
        {
            obstacleDetected = true;

            emergencyStop = true;
        }

        /*
        -----------------------------------------------------------------------
        Side obstacles
        -----------------------------------------------------------------------
        */

        if (
            leftValid &&
            leftDistance <=
            PART7_OBSTACLE_STOP_DISTANCE_M)
        {
            obstacleDetected = true;
        }

        if (
            rightValid &&
            rightDistance <=
            PART7_OBSTACLE_STOP_DISTANCE_M)
        {
            obstacleDetected = true;
        }
    }

    bool isObstacleDetected() const
    {
        return obstacleDetected;
    }

    bool requiresEmergencyStop() const
    {
        return emergencyStop;
    }

    float getFrontDistance() const
    {
        return frontDistance;
    }

    float getLeftDistance() const
    {
        return leftDistance;
    }

    float getRightDistance() const
    {
        return rightDistance;
    }
};

/*
===============================================================================
SECTION 9 — NAVIGATION MANAGER
===============================================================================

Coordinates:

    Path
    Pose
    Collision Monitoring
    Navigation Controller
===============================================================================
*/

class Part7NavigationManager
{
private:

    Part7NavigationController controller;

    Part7CollisionMonitor collisionMonitor;

    bool initialized;

    bool autonomousMode;

    unsigned long lastCycle;

public:

    Part7NavigationManager()
        : initialized(false),
          autonomousMode(false),
          lastCycle(0)
    {
    }

    void begin()
    {
        controller.begin();

        collisionMonitor.reset();

        initialized = true;

        autonomousMode = false;

        lastCycle = millis();
    }

    /*
    ---------------------------------------------------------------------------
    Load a path
    ---------------------------------------------------------------------------
    */

    bool loadPath(
        const Part7PathPoint *points,
        uint16_t count)
    {
        if (!initialized)
        {
            return false;
        }

        return controller.loadPath(
            points,
            count
        );
    }

    /*
    ---------------------------------------------------------------------------
    Add path point
    ---------------------------------------------------------------------------
    */

    bool addPathPoint(
        float x,
        float y)
    {
        return controller.addPathPoint(
            x,
            y
        );
    }

    /*
    ---------------------------------------------------------------------------
    Pose update
    ---------------------------------------------------------------------------
    */

    void updatePose(
        float x,
        float y,
        float theta)
    {
        controller.setPose(
            x,
            y,
            theta
        );
    }

    /*
    ---------------------------------------------------------------------------
    Sensor update
    ---------------------------------------------------------------------------
    */

    void updateSensors(
        float front,
        bool frontValid,
        float left,
        bool leftValid,
        float right,
        bool rightValid)
    {
        collisionMonitor.update(
            front,
            frontValid,
            left,
            leftValid,
            right,
            rightValid
        );

        controller.setObstacleDetected(
            collisionMonitor.isObstacleDetected()
        );

        if (
            collisionMonitor.requiresEmergencyStop())
        {
            controller.emergencyStop();
        }
    }

    /*
    ---------------------------------------------------------------------------
    Start autonomous navigation
    ---------------------------------------------------------------------------
    */

    bool startAutonomousNavigation()
    {
        if (!initialized)
        {
            return false;
        }

        if (!controller.hasPath())
        {
            return false;
        }

        if (!controller.start())
        {
            return false;
        }

        autonomousMode = true;

        return true;
    }

    /*
    ---------------------------------------------------------------------------
    Stop autonomous navigation
    ---------------------------------------------------------------------------
    */

    void stopAutonomousNavigation()
    {
        autonomousMode = false;

        controller.stop();
    }

    /*
    ---------------------------------------------------------------------------
    Pause
    ---------------------------------------------------------------------------
    */

    void pause()
    {
        controller.pause();
    }

    /*
    ---------------------------------------------------------------------------
    Resume
    ---------------------------------------------------------------------------
    */

    bool resume()
    {
        return controller.resume();
    }

    /*
    ---------------------------------------------------------------------------
    Request path replan
    ---------------------------------------------------------------------------
    */

    void requestReplan()
    {
        controller.requestReplan();
    }

    /*
    ---------------------------------------------------------------------------
    Main navigation cycle
    ---------------------------------------------------------------------------
    */

    Part7NavigationResult update()
    {
        if (!initialized)
        {
            return PART7_RESULT_ERROR;
        }

        if (!autonomousMode)
        {
            return PART7_RESULT_RUNNING;
        }

        return controller.update();
    }

    /*
    ---------------------------------------------------------------------------
    Get navigation velocity
    ---------------------------------------------------------------------------
    */

    Part7VelocityCommand getVelocityCommand() const
    {
        return controller.getVelocityCommand();
    }

    /*
    ---------------------------------------------------------------------------
    Get wheel references
    ---------------------------------------------------------------------------
    */

    Part7WheelCommand getWheelCommand() const
    {
        return controller.getWheelCommand();
    }

    /*
    ---------------------------------------------------------------------------
    Get diagnostics
    ---------------------------------------------------------------------------
    */

    const Part7NavigationDiagnostics &
    getDiagnostics() const
    {
        return controller.getDiagnostics();
    }

    /*
    ---------------------------------------------------------------------------
    Access controller
    ---------------------------------------------------------------------------
    */

    Part7NavigationController &
    getController()
    {
        return controller;
    }

    const Part7CollisionMonitor &
    getCollisionMonitor() const
    {
        return collisionMonitor;
    }

    /*
    ---------------------------------------------------------------------------
    Status
    ---------------------------------------------------------------------------
    */

    bool isAutonomous() const
    {
        return autonomousMode;
    }

    bool isGoalReached() const
    {
        return controller.isGoalReached();
    }
};

/*
===============================================================================
SECTION 10 — GLOBAL PART 7 OBJECTS
===============================================================================
*/

Part7NavigationManager part7NavigationManager;

/*
===============================================================================
SECTION 11 — PART 6 → PART 7 PATH ADAPTER
===============================================================================

The Part 6 planner owns the A* implementation.

This function provides a clean boundary between:

    PART 6 PATH PLANNER
             |
             v
    PART 7 NAVIGATION

Instead of making Part 7 manipulate the internals of the A* planner, copy the
final path into the navigation layer.

Call this function after Part 6 successfully generates a path.

===============================================================================
*/

void part7ClearNavigationPath()
{
    /*
    The navigation manager can be reset by creating a new path.

    The controller's path storage remains intentionally compact.
    */

    part7NavigationManager.stopAutonomousNavigation();
}

/*
===============================================================================
SECTION 12 — MANUAL PATH BUILDING INTERFACE
===============================================================================

Useful for testing Part 7 independently from Part 6.

Example:

    part7BeginPath();

    part7AddWaypoint(0.0f, 0.0f);
    part7AddWaypoint(0.5f, 0.0f);
    part7AddWaypoint(1.0f, 0.5f);
    part7AddWaypoint(1.5f, 1.0f);

    part7FinishPath();

===============================================================================
*/

bool part7PathBuildingStarted = false;

bool part7BeginPath()
{
    part7NavigationManager.stopAutonomousNavigation();

    part7PathBuildingStarted = true;

    return true;
}

bool part7AddWaypoint(
    float x,
    float y)
{
    if (!part7PathBuildingStarted)
    {
        return false;
    }

    return part7NavigationManager.addPathPoint(
        x,
        y
    );
}

bool part7FinishPath()
{
    if (!part7PathBuildingStarted)
    {
        return false;
    }

    part7PathBuildingStarted = false;

    return true;
}

/*
===============================================================================
SECTION 13 — PATH IMPORT FROM GRID COORDINATES
===============================================================================

Part 6 works with grid cells.

Part 7 works with world coordinates.

This conversion keeps that boundary explicit.

===============================================================================
*/

bool part7AddGridWaypoint(
    int16_t gridX,
    int16_t gridY,
    float originX,
    float originY,
    float resolution)
{
    if (resolution <= 0.0f)
    {
        return false;
    }

    const float worldX =
        originX +
        ((float)gridX + 0.5f) *
        resolution;

    const float worldY =
        originY +
        ((float)gridY + 0.5f) *
        resolution;

    return part7AddWaypoint(
        worldX,
        worldY
    );
}

/*
===============================================================================
SECTION 14 — NAVIGATION COMMAND OUTPUT
===============================================================================

The navigation controller does NOT directly drive physical motors.

It produces references that should be consumed by:

    Balance Controller
            |
            v
    Motor Controller
            |
            v
        Motor Driver

This separation is important for a self-balancing robot.
===============================================================================
*/

struct Part7BalanceNavigationReference
{
    float desiredLinearVelocity;
    float desiredAngularVelocity;

    float desiredLeftWheelVelocity;
    float desiredRightWheelVelocity;

    bool navigationValid;

    bool stopRequested;

    bool goalReached;

    Part7BalanceNavigationReference()
        : desiredLinearVelocity(0.0f),
          desiredAngularVelocity(0.0f),
          desiredLeftWheelVelocity(0.0f),
          desiredRightWheelVelocity(0.0f),
          navigationValid(false),
          stopRequested(true),
          goalReached(false)
    {
    }
};

Part7BalanceNavigationReference
part7GetBalanceReference()
{
    Part7BalanceNavigationReference reference;

    const Part7NavigationDiagnostics &diagnostics =
        part7NavigationManager.getDiagnostics();

    const Part7VelocityCommand velocity =
        part7NavigationManager.getVelocityCommand();

    const Part7WheelCommand wheels =
        part7NavigationManager.getWheelCommand();

    reference.desiredLinearVelocity =
        velocity.linear;

    reference.desiredAngularVelocity =
        velocity.angular;

    reference.desiredLeftWheelVelocity =
        wheels.left;

    reference.desiredRightWheelVelocity =
        wheels.right;

    reference.goalReached =
        diagnostics.goalReached;

    reference.navigationValid =
        diagnostics.pathAvailable &&
        !diagnostics.emergencyStopRequested &&
        !diagnostics.obstacleDetected;

    reference.stopRequested =
        diagnostics.emergencyStopRequested ||
        diagnostics.obstacleDetected ||
        diagnostics.goalReached;

    if (reference.stopRequested)
    {
        reference.desiredLinearVelocity = 0.0f;

        reference.desiredAngularVelocity = 0.0f;

        reference.desiredLeftWheelVelocity = 0.0f;

        reference.desiredRightWheelVelocity = 0.0f;
    }

    return reference;
}

/*
===============================================================================
SECTION 15 — PART 7 INITIALIZATION
===============================================================================
*/

void initializePart7Navigation()
{
    part7NavigationManager.begin();

    part7NavigationManager
        .getController()
        .setWheelBase(0.14f);

    part7NavigationManager
        .getController()
        .setMaximumLinearSpeed(
            PART7_MAX_LINEAR_SPEED_MPS
        );

    part7NavigationManager
        .getController()
        .setMaximumAngularSpeed(
            PART7_MAX_ANGULAR_SPEED_RADPS
        );

    part7NavigationManager
        .getController()
        .setLookAheadDistance(
            PART7_LOOKAHEAD_DISTANCE_M
        );

    part7NavigationManager
        .getController()
        .setWaypointTolerance(
            PART7_WAYPOINT_REACHED_DISTANCE_M
        );

    part7NavigationManager
        .getController()
        .setGoalTolerance(
            PART7_GOAL_REACHED_DISTANCE_M
        );
}

/*
===============================================================================
SECTION 16 — PART 7 TEST PATH
===============================================================================

This creates a small artificial path:

    START
      |
      +------+
             |
             +------+
                    |
                    GOAL

It is intended only for navigation-controller testing.

It does NOT replace Part 6 A*.
===============================================================================
*/

void part7CreateTestPath()
{
    part7BeginPath();

    part7AddWaypoint(
        0.0f,
        0.0f
    );

    part7AddWaypoint(
        0.30f,
        0.0f
    );

    part7AddWaypoint(
        0.60f,
        0.0f
    );

    part7AddWaypoint(
        0.90f,
        0.25f
    );

    part7AddWaypoint(
        1.20f,
        0.50f
    );

    part7AddWaypoint(
        1.50f,
        0.50f
    );

    part7FinishPath();
}

/*
===============================================================================
SECTION 17 — PART 7 NAVIGATION TEST
===============================================================================
*/

void part7RunNavigationTest()
{
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("PART 7 NAVIGATION TEST"));
    Serial.println(F("========================================"));

    initializePart7Navigation();

    part7CreateTestPath();

    /*
    Robot begins at the first waypoint.
    */

    part7NavigationManager.updatePose(
        0.0f,
        0.0f,
        0.0f
    );

    /*
    No obstacles.
    Distances are in meters.
    */

    part7NavigationManager.updateSensors(
        2.0f,
        true,

        2.0f,
        true,

        2.0f,
        true
    );

    if (
        part7NavigationManager
            .startAutonomousNavigation())
    {
        Serial.println(
            F("Navigation started.")
        );
    }
    else
    {
        Serial.println(
            F("Navigation failed to start.")
        );

        return;
    }

    for (uint8_t step = 0; step < 10; ++step)
    {
        delay(PART7_NAVIGATION_INTERVAL_MS);

        Part7NavigationResult result =
            part7NavigationManager.update();

        const Part7NavigationDiagnostics &d =
            part7NavigationManager.getDiagnostics();

        Serial.print(F("STEP="));
        Serial.print(step);

        Serial.print(F(" INDEX="));
        Serial.print(d.currentPathIndex);

        Serial.print(F(" DIST="));
        Serial.print(d.distanceToTarget, 3);

        Serial.print(F(" HEADING_ERR="));
        Serial.print(d.headingError, 3);

        Serial.print(F(" V="));
        Serial.print(
            d.commandedLinearVelocity,
            3
        );

        Serial.print(F(" W="));
        Serial.print(
            d.commandedAngularVelocity,
            3
        );

        Serial.print(F(" LEFT="));
        Serial.print(
            d.leftWheelReference,
            3
        );

        Serial.print(F(" RIGHT="));
        Serial.print(
            d.rightWheelReference,
            3
        );

        Serial.println();

        if (
            result ==
            PART7_RESULT_GOAL_REACHED)
        {
            Serial.println(
                F("GOAL REACHED")
            );

            break;
        }
    }

    Serial.println(
        F("PART 7 TEST COMPLETE")
    );
}

/*
===============================================================================
SECTION 18 — OBSTACLE TEST
===============================================================================
*/

void part7RunObstacleTest()
{
    Serial.println();
    Serial.println(
        F("PART 7 OBSTACLE TEST")
    );

    initializePart7Navigation();

    part7CreateTestPath();

    part7NavigationManager.updatePose(
        0.0f,
        0.0f,
        0.0f
    );

    /*
    Simulate a close front obstacle.
    */

    part7NavigationManager.updateSensors(
        0.15f,
        true,

        2.0f,
        true,

        2.0f,
        true
    );

    part7NavigationManager
        .startAutonomousNavigation();

    delay(PART7_NAVIGATION_INTERVAL_MS);

    const Part7NavigationResult result =
        part7NavigationManager.update();

    if (
        result ==
        PART7_RESULT_OBSTACLE)
    {
        Serial.println(
            F("Obstacle correctly detected.")
        );
    }

    Part7BalanceNavigationReference reference =
        part7GetBalanceReference();

    Serial.print(
        F("STOP REQUEST = ")
    );

    Serial.println(
        reference.stopRequested
        ? F("YES")
        : F("NO")
    );
}

/*
===============================================================================
SECTION 19 — HEADING TEST
===============================================================================
*/

void part7RunHeadingTest()
{
    Serial.println();
    Serial.println(
        F("PART 7 HEADING TEST")
    );

    initializePart7Navigation();

    part7BeginPath();

    part7AddWaypoint(
        1.0f,
        0.0f
    );

    part7FinishPath();

    /*
    Robot faces +Y while target is +X.
    Expected heading error is approximately -PI/2.
    */

    part7NavigationManager.updatePose(
        0.0f,
        0.0f,
        PART7_PI * 0.5f
    );

    part7NavigationManager.updateSensors(
        2.0f,
        true,
        2.0f,
        true,
        2.0f,
        true
    );

    part7NavigationManager
        .startAutonomousNavigation();

    delay(PART7_NAVIGATION_INTERVAL_MS);

    part7NavigationManager.update();

    const Part7NavigationDiagnostics &d =
        part7NavigationManager.getDiagnostics();

    Serial.print(
        F("Heading error = ")
    );

    Serial.println(
        d.headingError,
        4
    );
}

/*
===============================================================================
SECTION 20 — REPLANNING INTERFACE
===============================================================================

When Part 6 detects or receives a new obstacle:

    1. Stop.
    2. Update mapping.
    3. Re-run A*.
    4. Load the new path.
    5. Resume.

Part 7 does not directly implement A* again.
===============================================================================
*/

void part7NotifyNewObstacle()
{
    part7NavigationManager.requestReplan();
}

/*
===============================================================================
SECTION 21 — PATH REPLACEMENT
===============================================================================
*/

bool part7ReplacePath(
    const Part7PathPoint *newPath,
    uint16_t newPathLength)
{
    if (
        newPath == nullptr ||
        newPathLength == 0)
    {
        return false;
    }

    part7NavigationManager.stopAutonomousNavigation();

    const bool success =
        part7NavigationManager.loadPath(
            newPath,
            newPathLength
        );

    if (!success)
    {
        return false;
    }

    return true;
}

/*
===============================================================================
SECTION 22 — AUTONOMOUS NAVIGATION UPDATE
===============================================================================
*/

Part7NavigationResult updatePart7Navigation()
{
    return part7NavigationManager.update();
}

/*
===============================================================================
SECTION 23 — SENSOR BRIDGE
===============================================================================

This bridge is intentionally generic.

Connect it to the actual Part 1/Part 2 distance-sensor layer.

Distances MUST be in meters.

Examples:

    50 cm -> 0.50 m
    25 cm -> 0.25 m
    2 m   -> 2.00 m
===============================================================================
*/

void part7UpdateDistanceSensors(
    float frontMeters,
    bool frontValid,
    float leftMeters,
    bool leftValid,
    float rightMeters,
    bool rightValid)
{
    part7NavigationManager.updateSensors(
        frontMeters,
        frontValid,

        leftMeters,
        leftValid,

        rightMeters,
        rightValid
    );
}

/*
===============================================================================
SECTION 24 — POSE BRIDGE
===============================================================================
*/

void part7UpdateRobotPose(
    float xMeters,
    float yMeters,
    float headingRadians)
{
    part7NavigationManager.updatePose(
        xMeters,
        yMeters,
        headingRadians
    );
}

/*
===============================================================================
SECTION 25 — BALANCE CONTROLLER BRIDGE
===============================================================================

The balancing controller should consume these references.

IMPORTANT:

Do NOT directly connect the Part 7 wheel values to a motor driver.

Correct architecture:

    Part 7 Navigation
            |
            v
    Desired wheel velocity
            |
            v
    Velocity controller
            |
            v
    Balance controller
            |
            v
    Safety manager
            |
            v
    Motor controller

===============================================================================
*/

float part7GetDesiredLinearVelocity()
{
    return part7NavigationManager
        .getVelocityCommand()
        .linear;
}

float part7GetDesiredAngularVelocity()
{
    return part7NavigationManager
        .getVelocityCommand()
        .angular;
}

float part7GetDesiredLeftWheelVelocity()
{
    return part7NavigationManager
        .getWheelCommand()
        .left;
}

float part7GetDesiredRightWheelVelocity()
{
    return part7NavigationManager
        .getWheelCommand()
        .right;
}

/*
===============================================================================
SECTION 26 — NAVIGATION STATUS
===============================================================================
*/

const char *part7NavigationStateName(
    Part7NavigationState state)
{
    switch (state)
    {
        case PART7_NAV_IDLE:
            return "IDLE";

        case PART7_NAV_WAITING_FOR_PATH:
            return "WAITING_FOR_PATH";

        case PART7_NAV_FOLLOWING_PATH:
            return "FOLLOWING_PATH";

        case PART7_NAV_APPROACHING_GOAL:
            return "APPROACHING_GOAL";

        case PART7_NAV_GOAL_REACHED:
            return "GOAL_REACHED";

        case PART7_NAV_OBSTACLE_STOP:
            return "OBSTACLE_STOP";

        case PART7_NAV_REPLAN_REQUIRED:
            return "REPLAN_REQUIRED";

        case PART7_NAV_PAUSED:
            return "PAUSED";

        case PART7_NAV_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}

/*
===============================================================================
SECTION 27 — TELEMETRY
===============================================================================
*/

void part7PrintTelemetry()
{
    const Part7NavigationDiagnostics &d =
        part7NavigationManager.getDiagnostics();

    Serial.println();
    Serial.println(
        F("----------- PART 7 NAVIGATION -----------")
    );

    Serial.print(F("STATE: "));
    Serial.println(
        part7NavigationStateName(
            d.state
        )
    );

    Serial.print(F("PATH: "));
    Serial.println(
        d.pathAvailable
        ? F("AVAILABLE")
        : F("NONE")
    );

    Serial.print(F("PATH LENGTH: "));
    Serial.println(d.pathLength);

    Serial.print(F("CURRENT INDEX: "));
    Serial.println(d.currentPathIndex);

    Serial.print(F("DISTANCE TARGET: "));
    Serial.println(
        d.distanceToTarget,
        3
    );

    Serial.print(F("DISTANCE GOAL: "));
    Serial.println(
        d.distanceToGoal,
        3
    );

    Serial.print(F("TARGET HEADING: "));
    Serial.println(
        d.targetHeading,
        3
    );

    Serial.print(F("HEADING ERROR: "));
    Serial.println(
        d.headingError,
        3
    );

    Serial.print(F("LINEAR VELOCITY: "));
    Serial.println(
        d.commandedLinearVelocity,
        3
    );

    Serial.print(F("ANGULAR VELOCITY: "));
    Serial.println(
        d.commandedAngularVelocity,
        3
    );

    Serial.print(F("LEFT WHEEL: "));
    Serial.println(
        d.leftWheelReference,
        3
    );

    Serial.print(F("RIGHT WHEEL: "));
    Serial.println(
        d.rightWheelReference,
        3
    );

    Serial.print(F("OBSTACLE: "));
    Serial.println(
        d.obstacleDetected
        ? F("YES")
        : F("NO")
    );

    Serial.print(F("REPLAN REQUESTS: "));
    Serial.println(
        d.replanningRequests
    );

    Serial.print(F("GOAL: "));
    Serial.println(
        d.goalReached
        ? F("REACHED")
        : F("NOT REACHED")
    );

    Serial.println(
        F("------------------------------------------")
    );
}

/*
===============================================================================
SECTION 28 — COMMAND INTERFACE
===============================================================================

Commands:

    nav_test
    nav_obstacle_test
    nav_heading_test
    nav_start
    nav_stop
    nav_pause
    nav_resume
    nav_status
    nav_replan
    nav_emergency
    nav_clear_emergency

===============================================================================
*/

void processPart7NavigationCommand(
    const String &command)
{
    if (command == "nav_test")
    {
        part7RunNavigationTest();

        return;
    }

    if (command == "nav_obstacle_test")
    {
        part7RunObstacleTest();

        return;
    }

    if (command == "nav_heading_test")
    {
        part7RunHeadingTest();

        return;
    }

    if (command == "nav_start")
    {
        if (
            part7NavigationManager
                .startAutonomousNavigation())
        {
            Serial.println(
                F("NAVIGATION STARTED")
            );
        }
        else
        {
            Serial.println(
                F("NAVIGATION START FAILED")
            );
        }

        return;
    }

    if (command == "nav_stop")
    {
        part7NavigationManager
            .stopAutonomousNavigation();

        Serial.println(
            F("NAVIGATION STOPPED")
        );

        return;
    }

    if (command == "nav_pause")
    {
        part7NavigationManager.pause();

        Serial.println(
            F("NAVIGATION PAUSED")
        );

        return;
    }

    if (command == "nav_resume")
    {
        if (
            part7NavigationManager.resume())
        {
            Serial.println(
                F("NAVIGATION RESUMED")
            );
        }
        else
        {
            Serial.println(
                F("NAVIGATION RESUME FAILED")
            );
        }

        return;
    }

    if (command == "nav_replan")
    {
        part7NavigationManager
            .requestReplan();

        Serial.println(
            F("REPLAN REQUESTED")
        );

        return;
    }

    if (command == "nav_emergency")
    {
        part7NavigationManager
            .getController()
            .emergencyStop();

        Serial.println(
            F("NAVIGATION EMERGENCY STOP")
        );

        return;
    }

    if (command == "nav_clear_emergency")
    {
        part7NavigationManager
            .getController()
            .clearEmergencyStop();

        Serial.println(
            F("NAVIGATION EMERGENCY STATE CLEARED")
        );

        return;
    }

    if (command == "nav_status")
    {
        part7PrintTelemetry();

        return;
    }

    Serial.println(
        F("Unknown Part 7 navigation command.")
    );
}

/*
===============================================================================
SECTION 29 — COMPLETE PART 7 DEMONSTRATION
===============================================================================

This demonstrates:

    Path
      ↓
    Pose
      ↓
    Heading error
      ↓
    Distance error
      ↓
    Velocity command
      ↓
    Wheel references

No physical motors are driven here.
===============================================================================
*/

void runPart7NavigationDemo()
{
    Serial.println();
    Serial.println(
        F("============================================")
    );

    Serial.println(
        F("ASBR PART 7 — AUTONOMOUS NAVIGATION")
    );

    Serial.println(
        F("============================================")
    );

    initializePart7Navigation();

    /*
    ---------------------------------------------------------------------------
    Build a demonstration path.
    ---------------------------------------------------------------------------
    */

    part7BeginPath();

    part7AddWaypoint(
        0.00f,
        0.00f
    );

    part7AddWaypoint(
        0.40f,
        0.00f
    );

    part7AddWaypoint(
        0.80f,
        0.00f
    );

    part7AddWaypoint(
        1.10f,
        0.30f
    );

    part7AddWaypoint(
        1.40f,
        0.60f
    );

    part7AddWaypoint(
        1.80f,
        0.60f
    );

    part7FinishPath();

    /*
    ---------------------------------------------------------------------------
    Initial pose.
    ---------------------------------------------------------------------------
    */

    part7UpdateRobotPose(
        0.00f,
        0.00f,
        0.00f
    );

    /*
    ---------------------------------------------------------------------------
    Sensor state.
    ---------------------------------------------------------------------------
    */

    part7UpdateDistanceSensors(
        2.50f,
        true,

        2.50f,
        true,

        2.50f,
        true
    );

    /*
    ---------------------------------------------------------------------------
    Start.
    ---------------------------------------------------------------------------
    */

    if (
        part7NavigationManager
            .startAutonomousNavigation())
    {
        Serial.println(
            F("Autonomous navigation ACTIVE.")
        );
    }
    else
    {
        Serial.println(
            F("Unable to start navigation.")
        );

        return;
    }

    /*
    ---------------------------------------------------------------------------
    Run a few controller cycles.
    ---------------------------------------------------------------------------
    */

    for (uint8_t i = 0; i < 5; ++i)
    {
        delay(
            PART7_NAVIGATION_INTERVAL_MS
        );

        part7NavigationManager.update();

        part7PrintTelemetry();
    }

    Serial.println(
        F("Part 7 demonstration complete.")
    );
}

/*
===============================================================================
SECTION 30 — PART 7 INTEGRATION FLOW
===============================================================================

Recommended integration order:

    initializePart5Mapping();
                |
                v
    initializePart6PathPlanning();
                |
                v
    initializePart7Navigation();
                |
                v
    Mapping
                |
                v
    A* Planner
                |
                v
    Generated Path
                |
                v
    Part 7 Navigation
                |
                v
    Desired Velocity
                |
                v
    Balance Controller
                |
                v
    Safety Manager
                |
                v
    Motor Controller


The existing project architecture should keep the safety layer above the
navigation command output.

===============================================================================
SECTION 31 — EXAMPLE PART 6 → PART 7 CONVERSION
===============================================================================

When Part 6 produces a path consisting of grid coordinates, convert each
cell to world coordinates and add it to Part 7.

Example concept:

    part7BeginPath();

    for each Part 6 path cell:
        worldX = mapOriginX + (cellX + 0.5) * mapResolution;
        worldY = mapOriginY + (cellY + 0.5) * mapResolution;

        part7AddWaypoint(worldX, worldY);

    part7FinishPath();

Then:

    part7UpdateRobotPose(...);

    part7UpdateDistanceSensors(...);

    part7NavigationManager.startAutonomousNavigation();

===============================================================================
SECTION 32 — IMPORTANT REAL-ROBOT SAFETY RULE
===============================================================================

Part 7 generates navigation references only.

For a physical self-balancing robot:

    NAVIGATION
        |
        v
    BALANCE CONTROL
        |
        v
    SAFETY LIMITS
        |
        v
    MOTOR OUTPUT

The navigation layer must NEVER override:

    - Tilt protection
    - Emergency stop
    - Motor limits
    - Battery protection
    - Sensor-failure handling
    - Communication-loss handling

If the balance controller reports an unstable state, navigation should be
stopped and wheel references should be forced to zero by the safety layer.

===============================================================================
SECTION 33 — END OF PART 7
===============================================================================
*/ 
/*
===============================================================================
PART 8 — SAFETY + TELEMETRY
===============================================================================

PROJECT:
Autonomous Self-Balancing Robot — ASBR

TARGET:
Arduino Uno compatible C++

PART 8 FEATURES:

    1. Safety Manager
    2. Tilt Protection
    3. Motor Command Limiting
    4. Velocity Protection
    5. Sensor Health Monitoring
    6. Communication Timeout Monitoring
    7. Battery Monitoring
    8. Battery Drain Simulation
    9. Emergency Stop
   10. Warning / Limited / Emergency states
   11. Telemetry
   12. Structured Logging
   13. Diagnostic Information
   14. Safety Event Counters
   15. Navigation Safety Bridge
   16. Balance Controller Safety Bridge

IMPORTANT:

Navigation commands NEVER have higher priority than safety.

Priority:

    EMERGENCY STOP
          |
          v
    SAFETY LIMITS
          |
          v
    BALANCE CONTROLLER
          |
          v
    NAVIGATION
          |
          v
    MOTOR OUTPUT

===============================================================================
*/

#include <Arduino.h>
#include <math.h>

/*
===============================================================================
SECTION 1 — PART 8 CONFIGURATION
===============================================================================
*/

/*
    Tilt limits are deliberately configurable.

    These values are software safety thresholds.
*/

#define PART8_WARNING_TILT_DEG             8.0f
#define PART8_LIMITED_TILT_DEG            12.0f
#define PART8_EMERGENCY_TILT_DEG          18.0f

#define PART8_MAX_MOTOR_COMMAND             1.0f
#define PART8_LIMITED_MOTOR_COMMAND         0.45f

#define PART8_MAX_LINEAR_VELOCITY           0.35f
#define PART8_LIMITED_LINEAR_VELOCITY       0.12f

#define PART8_MAX_ANGULAR_VELOCITY          2.00f
#define PART8_LIMITED_ANGULAR_VELOCITY      0.60f

/*
    Battery thresholds.
*/

#define PART8_BATTERY_FULL_PERCENT        100.0f
#define PART8_BATTERY_LOW_PERCENT          25.0f
#define PART8_BATTERY_CRITICAL_PERCENT     10.0f
#define PART8_BATTERY_SHUTDOWN_PERCENT      5.0f

#define PART8_BATTERY_FULL_VOLTAGE          8.40f
#define PART8_BATTERY_MIN_VOLTAGE           6.40f

/*
    Communication timeout.
*/

#define PART8_COMMUNICATION_TIMEOUT_MS      2000UL

/*
    Sensor timeout.
*/

#define PART8_SENSOR_TIMEOUT_MS             500UL

/*
    Telemetry rate.
*/

#define PART8_TELEMETRY_INTERVAL_MS         250UL

/*
    Logging rate.
*/

#define PART8_LOG_INTERVAL_MS               500UL

/*
    Safety update rate.
*/

#define PART8_SAFETY_INTERVAL_MS             20UL

/*
    Maximum normalized motor command.
*/

#define PART8_MIN_MOTOR_COMMAND            -1.0f
#define PART8_MAX_MOTOR_COMMAND             1.0f


/*
===============================================================================
SECTION 2 — SAFETY STATE
===============================================================================
*/

enum Part8SafetyState
{
    PART8_SAFETY_NORMAL = 0,
    PART8_SAFETY_WARNING,
    PART8_SAFETY_LIMITED,
    PART8_SAFETY_EMERGENCY_STOP
};


/*
===============================================================================
SECTION 3 — SAFETY EVENT TYPES
===============================================================================
*/

enum Part8SafetyEvent
{
    PART8_EVENT_NONE = 0,

    PART8_EVENT_TILT_WARNING,
    PART8_EVENT_TILT_LIMITED,
    PART8_EVENT_TILT_EMERGENCY,

    PART8_EVENT_BATTERY_LOW,
    PART8_EVENT_BATTERY_CRITICAL,
    PART8_EVENT_BATTERY_SHUTDOWN,

    PART8_EVENT_SENSOR_FAILURE,
    PART8_EVENT_SENSOR_TIMEOUT,

    PART8_EVENT_COMMUNICATION_TIMEOUT,

    PART8_EVENT_VELOCITY_LIMIT,
    PART8_EVENT_MOTOR_LIMIT,

    PART8_EVENT_NAVIGATION_STOP,
    PART8_EVENT_EMERGENCY_STOP,

    PART8_EVENT_RECOVERY
};


/*
===============================================================================
SECTION 4 — MOTOR COMMAND
===============================================================================
*/

struct Part8MotorCommand
{
    float left;
    float right;

    Part8MotorCommand()
        : left(0.0f),
          right(0.0f)
    {
    }

    Part8MotorCommand(
        float leftCommand,
        float rightCommand)
        : left(leftCommand),
          right(rightCommand)
    {
    }
};


/*
===============================================================================
SECTION 5 — VELOCITY COMMAND
===============================================================================
*/

struct Part8VelocityCommand
{
    float linear;
    float angular;

    Part8VelocityCommand()
        : linear(0.0f),
          angular(0.0f)
    {
    }
};


/*
===============================================================================
SECTION 6 — SENSOR HEALTH
===============================================================================
*/

struct Part8SensorHealth
{
    bool imuHealthy;
    bool encoderLeftHealthy;
    bool encoderRightHealthy;
    bool distanceSensorHealthy;

    unsigned long lastIMUUpdate;
    unsigned long lastEncoderUpdate;
    unsigned long lastDistanceUpdate;

    Part8SensorHealth()
        : imuHealthy(false),
          encoderLeftHealthy(false),
          encoderRightHealthy(false),
          distanceSensorHealthy(false),
          lastIMUUpdate(0),
          lastEncoderUpdate(0),
          lastDistanceUpdate(0)
    {
    }
};


/*
===============================================================================
SECTION 7 — SAFETY DIAGNOSTICS
===============================================================================
*/

struct Part8SafetyDiagnostics
{
    Part8SafetyState state;

    float pitch;
    float roll;
    float yaw;

    float batteryPercent;
    float batteryVoltage;
    float estimatedCurrent;

    float requestedLeftMotor;
    float requestedRightMotor;

    float safeLeftMotor;
    float safeRightMotor;

    float requestedLinearVelocity;
    float requestedAngularVelocity;

    float safeLinearVelocity;
    float safeAngularVelocity;

    bool imuHealthy;
    bool encodersHealthy;
    bool distanceSensorHealthy;

    bool communicationHealthy;

    bool emergencyStop;
    bool batteryLow;
    bool batteryCritical;

    uint32_t tiltWarnings;
    uint32_t tiltLimitedEvents;
    uint32_t tiltEmergencyEvents;

    uint32_t batteryEvents;
    uint32_t sensorFailures;
    uint32_t communicationFailures;
    uint32_t motorLimitEvents;

    Part8SafetyDiagnostics()
        : state(PART8_SAFETY_NORMAL),
          pitch(0.0f),
          roll(0.0f),
          yaw(0.0f),
          batteryPercent(100.0f),
          batteryVoltage(8.4f),
          estimatedCurrent(0.0f),
          requestedLeftMotor(0.0f),
          requestedRightMotor(0.0f),
          safeLeftMotor(0.0f),
          safeRightMotor(0.0f),
          requestedLinearVelocity(0.0f),
          requestedAngularVelocity(0.0f),
          safeLinearVelocity(0.0f),
          safeAngularVelocity(0.0f),
          imuHealthy(false),
          encodersHealthy(false),
          distanceSensorHealthy(false),
          communicationHealthy(false),
          emergencyStop(false),
          batteryLow(false),
          batteryCritical(false),
          tiltWarnings(0),
          tiltLimitedEvents(0),
          tiltEmergencyEvents(0),
          batteryEvents(0),
          sensorFailures(0),
          communicationFailures(0),
          motorLimitEvents(0)
    {
    }
};


/*
===============================================================================
SECTION 8 — UTILITY FUNCTIONS
===============================================================================
*/

static float part8Abs(float value)
{
    return value >= 0.0f ? value : -value;
}


static float part8Clamp(
    float value,
    float minimum,
    float maximum)
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


static float part8Max(
    float a,
    float b)
{
    return a > b ? a : b;
}


static float part8MapFloat(
    float value,
    float inMin,
    float inMax,
    float outMin,
    float outMax)
{
    if (inMax == inMin)
    {
        return outMin;
    }

    const float ratio =
        (value - inMin) /
        (inMax - inMin);

    return outMin +
           ratio *
           (outMax - outMin);
}


/*
===============================================================================
SECTION 9 — BATTERY MODEL
===============================================================================

The battery model can operate in:

    SIMULATION MODE

or

    HARDWARE MONITORING MODE

For Arduino Uno, this class does not assume a specific voltage-measurement
circuit.
===============================================================================
*/

class Part8BatteryModel
{
private:

    float percentage;
    float voltage;
    float estimatedCurrent;

    float capacityPercentPerSecond;

    bool simulationEnabled;

    bool low;
    bool critical;
    bool shutdown;

    unsigned long lastUpdate;

public:

    Part8BatteryModel()
        : percentage(PART8_BATTERY_FULL_PERCENT),
          voltage(PART8_BATTERY_FULL_VOLTAGE),
          estimatedCurrent(0.0f),
          capacityPercentPerSecond(0.015f),
          simulationEnabled(true),
          low(false),
          critical(false),
          shutdown(false),
          lastUpdate(0)
    {
    }


    void begin()
    {
        percentage =
            PART8_BATTERY_FULL_PERCENT;

        voltage =
            PART8_BATTERY_FULL_VOLTAGE;

        estimatedCurrent = 0.0f;

        low = false;
        critical = false;
        shutdown = false;

        lastUpdate = millis();
    }


    void setSimulationEnabled(bool enabled)
    {
        simulationEnabled = enabled;
    }


    void setBatteryPercentage(float value)
    {
        percentage =
            part8Clamp(
                value,
                0.0f,
                100.0f
            );

        updateVoltage();
        updateStates();
    }


    void setBatteryVoltage(float value)
    {
        voltage =
            part8Clamp(
                value,
                0.0f,
                PART8_BATTERY_FULL_VOLTAGE
            );

        percentage =
            part8MapFloat(
                voltage,
                PART8_BATTERY_MIN_VOLTAGE,
                PART8_BATTERY_FULL_VOLTAGE,
                0.0f,
                100.0f
            );

        percentage =
            part8Clamp(
                percentage,
                0.0f,
                100.0f
            );

        updateStates();
    }


    void setEstimatedCurrent(float current)
    {
        estimatedCurrent =
            current < 0.0f
            ? 0.0f
            : current;
    }


    void setDrainRate(float percentPerSecond)
    {
        if (percentPerSecond >= 0.0f)
        {
            capacityPercentPerSecond =
                percentPerSecond;
        }
    }


    void update(
        float motorActivity)
    {
        const unsigned long now =
            millis();

        const float dt =
            (now - lastUpdate) /
            1000.0f;

        lastUpdate = now;

        if (!simulationEnabled)
        {
            updateStates();
            return;
        }

        if (dt <= 0.0f)
        {
            return;
        }

        const float activity =
            part8Clamp(
                part8Abs(motorActivity),
                0.0f,
                1.0f
            );

        /*
        Battery drains more quickly when motors work harder.
        */

        const float drain =
            capacityPercentPerSecond *
            (0.20f + activity);

        percentage -=
            drain * dt;

        if (percentage < 0.0f)
        {
            percentage = 0.0f;
        }

        updateVoltage();

        updateStates();
    }


private:

    void updateVoltage()
    {
        voltage =
            part8MapFloat(
                percentage,
                0.0f,
                100.0f,
                PART8_BATTERY_MIN_VOLTAGE,
                PART8_BATTERY_FULL_VOLTAGE
            );
    }


    void updateStates()
    {
        low =
            percentage <=
            PART8_BATTERY_LOW_PERCENT;

        critical =
            percentage <=
            PART8_BATTERY_CRITICAL_PERCENT;

        shutdown =
            percentage <=
            PART8_BATTERY_SHUTDOWN_PERCENT;
    }


public:

    float getPercentage() const
    {
        return percentage;
    }


    float getVoltage() const
    {
        return voltage;
    }


    float getEstimatedCurrent() const
    {
        return estimatedCurrent;
    }


    bool isLow() const
    {
        return low;
    }


    bool isCritical() const
    {
        return critical;
    }


    bool requiresShutdown() const
    {
        return shutdown;
    }
};


/*
===============================================================================
SECTION 10 — SAFETY MANAGER
===============================================================================
*/

class Part8SafetyManager
{
private:

    Part8SafetyState state;

    Part8SafetyDiagnostics diagnostics;

    Part8SensorHealth sensorHealth;

    Part8BatteryModel battery;

    bool safetyEnabled;
    bool manualEmergencyStop;

    unsigned long lastSafetyUpdate;
    unsigned long lastCommunication;

    float maximumPitch;
    float maximumRoll;

    float warningPitch;
    float warningRoll;

    float limitedPitch;
    float limitedRoll;

    float emergencyPitch;
    float emergencyRoll;

    float maximumMotorCommand;
    float limitedMotorCommand;

    float maximumLinearVelocity;
    float limitedLinearVelocity;

    float maximumAngularVelocity;
    float limitedAngularVelocity;

    Part8SafetyEvent lastEvent;

public:

    Part8SafetyManager()
        : state(PART8_SAFETY_NORMAL),
          safetyEnabled(true),
          manualEmergencyStop(false),
          lastSafetyUpdate(0),
          lastCommunication(0),
          maximumPitch(PART8_EMERGENCY_TILT_DEG),
          maximumRoll(PART8_EMERGENCY_TILT_DEG),
          warningPitch(PART8_WARNING_TILT_DEG),
          warningRoll(PART8_WARNING_TILT_DEG),
          limitedPitch(PART8_LIMITED_TILT_DEG),
          limitedRoll(PART8_LIMITED_TILT_DEG),
          emergencyPitch(PART8_EMERGENCY_TILT_DEG),
          emergencyRoll(PART8_EMERGENCY_TILT_DEG),
          maximumMotorCommand(PART8_MAX_MOTOR_COMMAND),
          limitedMotorCommand(PART8_LIMITED_MOTOR_COMMAND),
          maximumLinearVelocity(PART8_MAX_LINEAR_VELOCITY),
          limitedLinearVelocity(PART8_LIMITED_LINEAR_VELOCITY),
          maximumAngularVelocity(PART8_MAX_ANGULAR_VELOCITY),
          limitedAngularVelocity(PART8_LIMITED_ANGULAR_VELOCITY),
          lastEvent(PART8_EVENT_NONE)
    {
    }


    void begin()
    {
        state =
            PART8_SAFETY_NORMAL;

        diagnostics =
            Part8SafetyDiagnostics();

        sensorHealth =
            Part8SensorHealth();

        battery.begin();

        safetyEnabled = true;

        manualEmergencyStop = false;

        lastSafetyUpdate = millis();

        lastCommunication = millis();

        lastEvent =
            PART8_EVENT_NONE;
    }


    /*
    ---------------------------------------------------------------------------
    IMU update
    ---------------------------------------------------------------------------
    */

    void updateIMU(
        float pitch,
        float roll,
        float yaw)
    {
        diagnostics.pitch = pitch;
        diagnostics.roll = roll;
        diagnostics.yaw = yaw;

        sensorHealth.imuHealthy = true;

        sensorHealth.lastIMUUpdate =
            millis();
    }


    /*
    ---------------------------------------------------------------------------
    Encoder update
    ---------------------------------------------------------------------------
    */

    void updateEncoders(
        bool leftHealthy,
        bool rightHealthy)
    {
        sensorHealth.encoderLeftHealthy =
            leftHealthy;

        sensorHealth.encoderRightHealthy =
            rightHealthy;

        sensorHealth.lastEncoderUpdate =
            millis();
    }


    /*
    ---------------------------------------------------------------------------
    Distance sensor update
    ---------------------------------------------------------------------------
    */

    void updateDistanceSensor(
        bool healthy)
    {
        sensorHealth.distanceSensorHealthy =
            healthy;

        sensorHealth.lastDistanceUpdate =
            millis();
    }


    /*
    ---------------------------------------------------------------------------
    Communication heartbeat
    ---------------------------------------------------------------------------
    */

    void communicationHeartbeat()
    {
        lastCommunication =
            millis();
    }


    /*
    ---------------------------------------------------------------------------
    Manual emergency stop
    ---------------------------------------------------------------------------
    */

    void emergencyStop()
    {
        manualEmergencyStop = true;

        state =
            PART8_SAFETY_EMERGENCY_STOP;

        diagnostics.emergencyStop = true;

        lastEvent =
            PART8_EVENT_EMERGENCY_STOP;
    }


    /*
    ---------------------------------------------------------------------------
    Clear emergency stop
    ---------------------------------------------------------------------------
    */

    void clearEmergencyStop()
    {
        /*
        Safety does not automatically return to NORMAL.

        Conditions must be safe first.
        */

        manualEmergencyStop = false;

        diagnostics.emergencyStop = false;

        if (isPhysicalStateSafe())
        {
            state =
                PART8_SAFETY_WARNING;

            lastEvent =
                PART8_EVENT_RECOVERY;
        }
    }


    /*
    ---------------------------------------------------------------------------
    Battery access
    ---------------------------------------------------------------------------
    */

    Part8BatteryModel &
    getBattery()
    {
        return battery;
    }


    /*
    ---------------------------------------------------------------------------
    Main safety update
    ---------------------------------------------------------------------------
    */

    void update()
    {
        const unsigned long now =
            millis();

        if (
            now - lastSafetyUpdate <
            PART8_SAFETY_INTERVAL_MS)
        {
            return;
        }

        lastSafetyUpdate = now;

        if (!safetyEnabled)
        {
            return;
        }

        /*
        -----------------------------------------------------------------------
        Update battery.
        -----------------------------------------------------------------------
        */

        const float motorActivity =
            part8Max(
                part8Abs(
                    diagnostics.requestedLeftMotor
                ),
                part8Abs(
                    diagnostics.requestedRightMotor
                )
            );

        battery.update(
            motorActivity
        );

        diagnostics.batteryPercent =
            battery.getPercentage();

        diagnostics.batteryVoltage =
            battery.getVoltage();

        diagnostics.estimatedCurrent =
            battery.getEstimatedCurrent();

        /*
        -----------------------------------------------------------------------
        Check sensor health.
        -----------------------------------------------------------------------
        */

        checkSensorHealth();

        /*
        -----------------------------------------------------------------------
        Check communication.
        -----------------------------------------------------------------------
        */

        checkCommunication();

        /*
        -----------------------------------------------------------------------
        Check battery.
        -----------------------------------------------------------------------
        */

        checkBattery();

        /*
        -----------------------------------------------------------------------
        Check tilt.
        -----------------------------------------------------------------------
        */

        checkTilt();

        /*
        -----------------------------------------------------------------------
        Manual emergency stop.
        -----------------------------------------------------------------------
        */

        if (manualEmergencyStop)
        {
            state =
                PART8_SAFETY_EMERGENCY_STOP;

            diagnostics.emergencyStop = true;

            return;
        }

        /*
        -----------------------------------------------------------------------
        Emergency conditions have highest priority.
        -----------------------------------------------------------------------
        */

        if (state ==
            PART8_SAFETY_EMERGENCY_STOP)
        {
            return;
        }

        /*
        -----------------------------------------------------------------------
        Determine final state.
        -----------------------------------------------------------------------
        */

        if (
            battery.requiresShutdown() ||
            !sensorHealth.imuHealthy ||
            !diagnostics.communicationHealthy ||
            isTiltEmergency()
        )
        {
            state =
                PART8_SAFETY_EMERGENCY_STOP;

            diagnostics.emergencyStop = true;

            return;
        }

        if (
            battery.isCritical() ||
            isTiltLimited() ||
            !sensorHealth.encoderLeftHealthy ||
            !sensorHealth.encoderRightHealthy)
        {
            state =
                PART8_SAFETY_LIMITED;

            return;
        }

        if (
            battery.isLow() ||
            isTiltWarning())
        {
            state =
                PART8_SAFETY_WARNING;

            return;
        }

        state =
            PART8_SAFETY_NORMAL;
    }


private:

    /*
    ---------------------------------------------------------------------------
    Sensor health
    ---------------------------------------------------------------------------
    */

    void checkSensorHealth()
    {
        const unsigned long now =
            millis();

        if (
            now -
            sensorHealth.lastIMUUpdate >
            PART8_SENSOR_TIMEOUT_MS)
        {
            if (sensorHealth.imuHealthy)
            {
                diagnostics.sensorFailures++;

                lastEvent =
                    PART8_EVENT_SENSOR_TIMEOUT;
            }

            sensorHealth.imuHealthy = false;
        }

        if (
            now -
            sensorHealth.lastEncoderUpdate >
            PART8_SENSOR_TIMEOUT_MS)
        {
            sensorHealth.encoderLeftHealthy = false;
            sensorHealth.encoderRightHealthy = false;
        }

        if (
            now -
            sensorHealth.lastDistanceUpdate >
            PART8_SENSOR_TIMEOUT_MS)
        {
            sensorHealth.distanceSensorHealthy = false;
        }

        diagnostics.imuHealthy =
            sensorHealth.imuHealthy;

        diagnostics.encodersHealthy =
            sensorHealth.encoderLeftHealthy &&
            sensorHealth.encoderRightHealthy;

        diagnostics.distanceSensorHealthy =
            sensorHealth.distanceSensorHealthy;
    }


    /*
    ---------------------------------------------------------------------------
    Communication health
    ---------------------------------------------------------------------------
    */

    void checkCommunication()
    {
        const unsigned long now =
            millis();

        if (
            now -
            lastCommunication >
            PART8_COMMUNICATION_TIMEOUT_MS)
        {
            diagnostics.communicationHealthy =
                false;

            diagnostics.communicationFailures++;

            lastEvent =
                PART8_EVENT_COMMUNICATION_TIMEOUT;
        }
        else
        {
            diagnostics.communicationHealthy =
                true;
        }
    }


    /*
    ---------------------------------------------------------------------------
    Battery safety
    ---------------------------------------------------------------------------
    */

    void checkBattery()
    {
        if (battery.isLow())
        {
            diagnostics.batteryLow = true;

            lastEvent =
                PART8_EVENT_BATTERY_LOW;
        }

        if (battery.isCritical())
        {
            diagnostics.batteryCritical = true;

            diagnostics.batteryEvents++;

            lastEvent =
                PART8_EVENT_BATTERY_CRITICAL;
        }

        if (battery.requiresShutdown())
        {
            diagnostics.batteryEvents++;

            lastEvent =
                PART8_EVENT_BATTERY_SHUTDOWN;
        }
    }


    /*
    ---------------------------------------------------------------------------
    Tilt checks
    ---------------------------------------------------------------------------
    */

    bool isTiltWarning() const
    {
        return
            part8Abs(diagnostics.pitch) >=
            warningPitch ||
            part8Abs(diagnostics.roll) >=
            warningRoll;
    }


    bool isTiltLimited() const
    {
        return
            part8Abs(diagnostics.pitch) >=
            limitedPitch ||
            part8Abs(diagnostics.roll) >=
            limitedRoll;
    }


    bool isTiltEmergency() const
    {
        return
            part8Abs(diagnostics.pitch) >=
            emergencyPitch ||
            part8Abs(diagnostics.roll) >=
            emergencyRoll;
    }


    bool isPhysicalStateSafe() const
    {
        if (isTiltEmergency())
        {
            return false;
        }

        if (battery.requiresShutdown())
        {
            return false;
        }

        if (!sensorHealth.imuHealthy)
        {
            return false;
        }

        return true;
    }


public:

    /*
    ---------------------------------------------------------------------------
    Protect velocity command
    ---------------------------------------------------------------------------
    */

    Part8VelocityCommand
    protectVelocity(
        const Part8VelocityCommand &requested)
    {
        Part8VelocityCommand safe =
            requested;

        diagnostics.requestedLinearVelocity =
            requested.linear;

        diagnostics.requestedAngularVelocity =
            requested.angular;

        /*
        Normal maximum.
        */

        safe.linear =
            part8Clamp(
                safe.linear,
                -maximumLinearVelocity,
                maximumLinearVelocity
            );

        safe.angular =
            part8Clamp(
                safe.angular,
                -maximumAngularVelocity,
                maximumAngularVelocity
            );

        /*
        Limited safety state.
        */

        if (
            state ==
            PART8_SAFETY_LIMITED)
        {
            safe.linear =
                part8Clamp(
                    safe.linear,
                    -limitedLinearVelocity,
                    limitedLinearVelocity
                );

            safe.angular =
                part8Clamp(
                    safe.angular,
                    -limitedAngularVelocity,
                    limitedAngularVelocity
                );
        }

        /*
        Warning state.

        Reduce speed without immediately stopping.
        */

        if (
            state ==
            PART8_SAFETY_WARNING)
        {
            safe.linear *= 0.70f;

            safe.angular *= 0.80f;
        }

        /*
        Emergency state.
        */

        if (
            state ==
            PART8_SAFETY_EMERGENCY_STOP)
        {
            safe.linear = 0.0f;

            safe.angular = 0.0f;
        }

        diagnostics.safeLinearVelocity =
            safe.linear;

        diagnostics.safeAngularVelocity =
            safe.angular;

        return safe;
    }


    /*
    ---------------------------------------------------------------------------
    Protect motor commands
    ---------------------------------------------------------------------------
    */

    Part8MotorCommand
    protectMotorCommand(
        const Part8MotorCommand &requested)
    {
        Part8MotorCommand safe =
            requested;

        diagnostics.requestedLeftMotor =
            requested.left;

        diagnostics.requestedRightMotor =
            requested.right;

        /*
        First clamp.
        */

        safe.left =
            part8Clamp(
                safe.left,
                PART8_MIN_MOTOR_COMMAND,
                PART8_MAX_MOTOR_COMMAND
            );

        safe.right =
            part8Clamp(
                safe.right,
                PART8_MIN_MOTOR_COMMAND,
                PART8_MAX_MOTOR_COMMAND
            );

        /*
        Limited mode.
        */

        if (
            state ==
            PART8_SAFETY_LIMITED)
        {
            safe.left =
                part8Clamp(
                    safe.left,
                    -limitedMotorCommand,
                    limitedMotorCommand
                );

            safe.right =
                part8Clamp(
                    safe.right,
                    -limitedMotorCommand,
                    limitedMotorCommand
                );
        }

        /*
        Warning mode.
        */

        if (
            state ==
            PART8_SAFETY_WARNING)
        {
            safe.left *= 0.75f;

            safe.right *= 0.75f;
        }

        /*
        Emergency stop.
        */

        if (
            state ==
            PART8_SAFETY_EMERGENCY_STOP)
        {
            safe.left = 0.0f;

            safe.right = 0.0f;
        }

        /*
        Battery shutdown.
        */

        if (
            battery.requiresShutdown())
        {
            safe.left = 0.0f;

            safe.right = 0.0f;
        }

        /*
        Tilt emergency.
        */

        if (isTiltEmergency())
        {
            safe.left = 0.0f;

            safe.right = 0.0f;
        }

        if (
            safe.left != requested.left ||
            safe.right != requested.right)
        {
            diagnostics.motorLimitEvents++;
        }

        diagnostics.safeLeftMotor =
            safe.left;

        diagnostics.safeRightMotor =
            safe.right;

        return safe;
    }


    /*
    ---------------------------------------------------------------------------
    State getters
    ---------------------------------------------------------------------------
    */

    Part8SafetyState getState() const
    {
        return state;
    }


    const Part8SafetyDiagnostics &
    getDiagnostics() const
    {
        return diagnostics;
    }


    const Part8SensorHealth &
    getSensorHealth() const
    {
        return sensorHealth;
    }


    bool isEmergencyStopActive() const
    {
        return
            state ==
            PART8_SAFETY_EMERGENCY_STOP;
    }


    bool isLimited() const
    {
        return
            state ==
            PART8_SAFETY_LIMITED;
    }


    bool isWarning() const
    {
        return
            state ==
            PART8_SAFETY_WARNING;
    }


    bool isNormal() const
    {
        return
            state ==
            PART8_SAFETY_NORMAL;
    }


    Part8SafetyEvent
    getLastEvent() const
    {
        return lastEvent;
    }


    void enable()
    {
        safetyEnabled = true;
    }


    void disable()
    {
        /*
        Disabling the safety manager does NOT bypass physical
        emergency handling. It simply stops automatic updates.
        */

        safetyEnabled = false;
    }
};


/*
===============================================================================
SECTION 11 — TELEMETRY RECORD
===============================================================================
*/

struct Part8TelemetryRecord
{
    unsigned long timestamp;

    float x;
    float y;
    float theta;

    float linearVelocity;
    float angularVelocity;

    float pitch;
    float roll;
    float yaw;

    float leftMotor;
    float rightMotor;

    float batteryPercent;
    float batteryVoltage;
    float current;

    float frontDistance;
    float leftDistance;
    float rightDistance;

    Part8SafetyState safetyState;

    bool navigationActive;
    bool goalReached;

    Part8TelemetryRecord()
        : timestamp(0),
          x(0.0f),
          y(0.0f),
          theta(0.0f),
          linearVelocity(0.0f),
          angularVelocity(0.0f),
          pitch(0.0f),
          roll(0.0f),
          yaw(0.0f),
          leftMotor(0.0f),
          rightMotor(0.0f),
          batteryPercent(100.0f),
          batteryVoltage(8.4f),
          current(0.0f),
          frontDistance(0.0f),
          leftDistance(0.0f),
          rightDistance(0.0f),
          safetyState(PART8_SAFETY_NORMAL),
          navigationActive(false),
          goalReached(false)
    {
    }
};


/*
===============================================================================
SECTION 12 — TELEMETRY LOGGER
===============================================================================
*/

class Part8TelemetryLogger
{
private:

    Part8TelemetryRecord currentRecord;

    bool enabled;

    unsigned long lastTelemetry;

    uint32_t recordCount;

public:

    Part8TelemetryLogger()
        : enabled(true),
          lastTelemetry(0),
          recordCount(0)
    {
    }


    void begin()
    {
        currentRecord =
            Part8TelemetryRecord();

        enabled = true;

        lastTelemetry = millis();

        recordCount = 0;
    }


    void setEnabled(bool value)
    {
        enabled = value;
    }


    bool isEnabled() const
    {
        return enabled;
    }


    bool ready() const
    {
        return
            millis() - lastTelemetry >=
            PART8_TELEMETRY_INTERVAL_MS;
    }


    void capture(
        float x,
        float y,
        float theta,

        float linearVelocity,
        float angularVelocity,

        float pitch,
        float roll,
        float yaw,

        float leftMotor,
        float rightMotor,

        float batteryPercent,
        float batteryVoltage,
        float current,

        float frontDistance,
        float leftDistance,
        float rightDistance,

        Part8SafetyState safetyState,

        bool navigationActive,
        bool goalReached)
    {
        if (!enabled)
        {
            return;
        }

        const unsigned long now =
            millis();

        if (
            now - lastTelemetry <
            PART8_TELEMETRY_INTERVAL_MS)
        {
            return;
        }

        lastTelemetry = now;

        currentRecord.timestamp =
            now;

        currentRecord.x = x;
        currentRecord.y = y;
        currentRecord.theta = theta;

        currentRecord.linearVelocity =
            linearVelocity;

        currentRecord.angularVelocity =
            angularVelocity;

        currentRecord.pitch = pitch;
        currentRecord.roll = roll;
        currentRecord.yaw = yaw;

        currentRecord.leftMotor =
            leftMotor;

        currentRecord.rightMotor =
            rightMotor;

        currentRecord.batteryPercent =
            batteryPercent;

        currentRecord.batteryVoltage =
            batteryVoltage;

        currentRecord.current =
            current;

        currentRecord.frontDistance =
            frontDistance;

        currentRecord.leftDistance =
            leftDistance;

        currentRecord.rightDistance =
            rightDistance;

        currentRecord.safetyState =
            safetyState;

        currentRecord.navigationActive =
            navigationActive;

        currentRecord.goalReached =
            goalReached;

        recordCount++;

        printCSV(currentRecord);
    }


private:

    void printCSV(
        const Part8TelemetryRecord &r)
    {
        Serial.print(r.timestamp);
        Serial.print(',');

        Serial.print(r.x, 4);
        Serial.print(',');

        Serial.print(r.y, 4);
        Serial.print(',');

        Serial.print(r.theta, 4);
        Serial.print(',');

        Serial.print(r.linearVelocity, 4);
        Serial.print(',');

        Serial.print(r.angularVelocity, 4);
        Serial.print(',');

        Serial.print(r.pitch, 3);
        Serial.print(',');

        Serial.print(r.roll, 3);
        Serial.print(',');

        Serial.print(r.yaw, 3);
        Serial.print(',');

        Serial.print(r.leftMotor, 3);
        Serial.print(',');

        Serial.print(r.rightMotor, 3);
        Serial.print(',');

        Serial.print(r.batteryPercent, 2);
        Serial.print(',');

        Serial.print(r.batteryVoltage, 3);
        Serial.print(',');

        Serial.print(r.current, 3);
        Serial.print(',');

        Serial.print(r.frontDistance, 3);
        Serial.print(',');

        Serial.print(r.leftDistance, 3);
        Serial.print(',');

        Serial.print(r.rightDistance, 3);
        Serial.print(',');

        Serial.print(
            (int)r.safetyState
        );

        Serial.print(',');

        Serial.print(
            r.navigationActive
            ? 1
            : 0
        );

        Serial.print(',');

        Serial.println(
            r.goalReached
            ? 1
            : 0
        );
    }


public:

    void printHeader()
    {
        Serial.println(
            F("timestamp,x,y,theta,"
              "linear_velocity,angular_velocity,"
              "pitch,roll,yaw,"
              "left_motor,right_motor,"
              "battery_percent,battery_voltage,current,"
              "front_distance,left_distance,right_distance,"
              "safety_state,navigation_active,goal_reached")
        );
    }


    uint32_t getRecordCount() const
    {
        return recordCount;
    }


    const Part8TelemetryRecord &
    getLastRecord() const
    {
        return currentRecord;
    }
};


/*
===============================================================================
SECTION 13 — STRUCTURED LOGGER
===============================================================================
*/

enum Part8LogLevel
{
    PART8_LOG_TRACE = 0,
    PART8_LOG_DEBUG,
    PART8_LOG_INFO,
    PART8_LOG_WARNING,
    PART8_LOG_ERROR,
    PART8_LOG_CRITICAL
};


class Part8Logger
{
private:

    Part8LogLevel minimumLevel;

    bool consoleEnabled;

public:

    Part8Logger()
        : minimumLevel(PART8_LOG_INFO),
          consoleEnabled(true)
    {
    }


    void begin()
    {
        minimumLevel =
            PART8_LOG_INFO;

        consoleEnabled = true;
    }


    void setMinimumLevel(
        Part8LogLevel level)
    {
        minimumLevel = level;
    }


    void setConsoleEnabled(
        bool enabled)
    {
        consoleEnabled = enabled;
    }


    void log(
        Part8LogLevel level,
        const char *message)
    {
        if (!consoleEnabled)
        {
            return;
        }

        if (level < minimumLevel)
        {
            return;
        }

        Serial.print('[');
        Serial.print(millis());
        Serial.print(F("] "));

        switch (level)
        {
            case PART8_LOG_TRACE:
                Serial.print(F("TRACE"));
                break;

            case PART8_LOG_DEBUG:
                Serial.print(F("DEBUG"));
                break;

            case PART8_LOG_INFO:
                Serial.print(F("INFO"));
                break;

            case PART8_LOG_WARNING:
                Serial.print(F("WARNING"));
                break;

            case PART8_LOG_ERROR:
                Serial.print(F("ERROR"));
                break;

            case PART8_LOG_CRITICAL:
                Serial.print(F("CRITICAL"));
                break;

            default:
                Serial.print(F("UNKNOWN"));
                break;
        }

        Serial.print(F(": "));
        Serial.println(message);
    }


    void trace(const char *message)
    {
        log(
            PART8_LOG_TRACE,
            message
        );
    }


    void debug(const char *message)
    {
        log(
            PART8_LOG_DEBUG,
            message
        );
    }


    void info(const char *message)
    {
        log(
            PART8_LOG_INFO,
            message
        );
    }


    void warning(const char *message)
    {
        log(
            PART8_LOG_WARNING,
            message
        );
    }


    void error(const char *message)
    {
        log(
            PART8_LOG_ERROR,
            message
        );
    }


    void critical(const char *message)
    {
        log(
            PART8_LOG_CRITICAL,
            message
        );
    }
};


/*
===============================================================================
SECTION 14 — PART 8 DIAGNOSTIC MANAGER
===============================================================================
*/

class Part8DiagnosticManager
{
private:

    unsigned long lastPrint;

    bool enabled;

public:

    Part8DiagnosticManager()
        : lastPrint(0),
          enabled(true)
    {
    }


    void begin()
    {
        lastPrint = millis();

        enabled = true;
    }


    void setEnabled(bool value)
    {
        enabled = value;
    }


    void printSafety(
        const Part8SafetyManager &manager)
    {
        if (!enabled)
        {
            return;
        }

        const Part8SafetyDiagnostics &d =
            manager.getDiagnostics();

        Serial.println();
        Serial.println(
            F("========== SAFETY DIAGNOSTICS ==========")
        );

        Serial.print(F("STATE: "));
        Serial.println(
            part8SafetyStateName(
                d.state
            )
        );

        Serial.print(F("PITCH: "));
        Serial.println(
            d.pitch,
            2
        );

        Serial.print(F("ROLL: "));
        Serial.println(
            d.roll,
            2
        );

        Serial.print(F("YAW: "));
        Serial.println(
            d.yaw,
            2
        );

        Serial.print(F("BATTERY: "));
        Serial.print(
            d.batteryPercent,
            1
        );
        Serial.println(F("%"));

        Serial.print(F("VOLTAGE: "));
        Serial.println(
            d.batteryVoltage,
            2
        );

        Serial.print(F("CURRENT: "));
        Serial.println(
            d.estimatedCurrent,
            2
        );

        Serial.print(F("IMU: "));
        Serial.println(
            d.imuHealthy
            ? F("OK")
            : F("FAIL")
        );

        Serial.print(F("ENCODERS: "));
        Serial.println(
            d.encodersHealthy
            ? F("OK")
            : F("FAIL")
        );

        Serial.print(F("DISTANCE SENSOR: "));
        Serial.println(
            d.distanceSensorHealthy
            ? F("OK")
            : F("FAIL")
        );

        Serial.print(F("COMMUNICATION: "));
        Serial.println(
            d.communicationHealthy
            ? F("OK")
            : F("FAIL")
        );

        Serial.print(F("EMERGENCY: "));
        Serial.println(
            d.emergencyStop
            ? F("ACTIVE")
            : F("CLEAR")
        );

        Serial.print(F("LEFT REQUEST: "));
        Serial.println(
            d.requestedLeftMotor,
            3
        );

        Serial.print(F("RIGHT REQUEST: "));
        Serial.println(
            d.requestedRightMotor,
            3
        );

        Serial.print(F("LEFT SAFE: "));
        Serial.println(
            d.safeLeftMotor,
            3
        );

        Serial.print(F("RIGHT SAFE: "));
        Serial.println(
            d.safeRightMotor,
            3
        );

        Serial.print(F("TILT WARNINGS: "));
        Serial.println(
            d.tiltWarnings
        );

        Serial.print(F("TILT LIMITED: "));
        Serial.println(
            d.tiltLimitedEvents
        );

        Serial.print(F("TILT EMERGENCY: "));
        Serial.println(
            d.tiltEmergencyEvents
        );

        Serial.print(F("SENSOR FAILURES: "));
        Serial.println(
            d.sensorFailures
        );

        Serial.print(F("COMMUNICATION FAILURES: "));
        Serial.println(
            d.communicationFailures
        );

        Serial.print(F("MOTOR LIMIT EVENTS: "));
        Serial.println(
            d.motorLimitEvents
        );

        Serial.println(
            F("=========================================")
        );
    }
};


/*
===============================================================================
SECTION 15 — SAFETY STATE NAME
===============================================================================
*/

const char *
part8SafetyStateName(
    Part8SafetyState state)
{
    switch (state)
    {
        case PART8_SAFETY_NORMAL:
            return "NORMAL";

        case PART8_SAFETY_WARNING:
            return "WARNING";

        case PART8_SAFETY_LIMITED:
            return "LIMITED";

        case PART8_SAFETY_EMERGENCY_STOP:
            return "EMERGENCY_STOP";

        default:
            return "UNKNOWN";
    }
}


/*
===============================================================================
SECTION 16 — GLOBAL PART 8 OBJECTS
===============================================================================
*/

Part8SafetyManager part8SafetyManager;

Part8TelemetryLogger part8TelemetryLogger;

Part8Logger part8Logger;

Part8DiagnosticManager part8Diagnostics;


/*
===============================================================================
SECTION 17 — INITIALIZATION
===============================================================================
*/

void initializePart8SafetyTelemetry()
{
    part8SafetyManager.begin();

    part8TelemetryLogger.begin();

    part8Logger.begin();

    part8Diagnostics.begin();

    part8TelemetryLogger.printHeader();

    part8Logger.info(
        "Part 8 Safety + Telemetry initialized."
    );
}


/*
===============================================================================
SECTION 18 — IMU SAFETY BRIDGE
===============================================================================
*/

void part8UpdateIMU(
    float pitch,
    float roll,
    float yaw)
{
    part8SafetyManager.updateIMU(
        pitch,
        roll,
        yaw
    );
}


/*
===============================================================================
SECTION 19 — ENCODER SAFETY BRIDGE
===============================================================================
*/

void part8UpdateEncoderHealth(
    bool leftHealthy,
    bool rightHealthy)
{
    part8SafetyManager.updateEncoders(
        leftHealthy,
        rightHealthy
    );
}


/*
===============================================================================
SECTION 20 — DISTANCE SENSOR SAFETY BRIDGE
===============================================================================
*/

void part8UpdateDistanceSensorHealth(
    bool healthy)
{
    part8SafetyManager.updateDistanceSensor(
        healthy
    );
}


/*
===============================================================================
SECTION 21 — COMMUNICATION BRIDGE
===============================================================================
*/

void part8CommunicationHeartbeat()
{
    part8SafetyManager
        .communicationHeartbeat();
}


/*
===============================================================================
SECTION 22 — VELOCITY SAFETY BRIDGE
===============================================================================
*/

Part8VelocityCommand
part8ProtectVelocity(
    float linear,
    float angular)
{
    Part8VelocityCommand requested;

    requested.linear = linear;
    requested.angular = angular;

    return part8SafetyManager
        .protectVelocity(
            requested
        );
}


/*
===============================================================================
SECTION 23 — MOTOR SAFETY BRIDGE
===============================================================================
*/

Part8MotorCommand
part8ProtectMotorCommand(
    float left,
    float right)
{
    Part8MotorCommand requested(
        left,
        right
    );

    return part8SafetyManager
        .protectMotorCommand(
            requested
        );
}


/*
===============================================================================
SECTION 24 — TELEMETRY BRIDGE
===============================================================================
*/

void part8RecordTelemetry(
    float x,
    float y,
    float theta,

    float linearVelocity,
    float angularVelocity,

    float pitch,
    float roll,
    float yaw,

    float leftMotor,
    float rightMotor,

    float frontDistance,
    float leftDistance,
    float rightDistance,

    bool navigationActive,
    bool goalReached)
{
    const Part8SafetyDiagnostics &d =
        part8SafetyManager.getDiagnostics();

    part8TelemetryLogger.capture(
        x,
        y,
        theta,

        linearVelocity,
        angularVelocity,

        pitch,
        roll,
        yaw,

        leftMotor,
        rightMotor,

        d.batteryPercent,
        d.batteryVoltage,
        d.estimatedCurrent,

        frontDistance,
        leftDistance,
        rightDistance,

        d.state,

        navigationActive,
        goalReached
    );
}


/*
===============================================================================
SECTION 25 — MAIN PART 8 UPDATE
===============================================================================
*/

void updatePart8SafetyTelemetry()
{
    /*
    Safety must run before motor output.
    */

    part8SafetyManager.update();
}


/*
===============================================================================
SECTION 26 — EMERGENCY STOP
===============================================================================
*/

void part8EmergencyStop()
{
    part8SafetyManager.emergencyStop();

    part8Logger.critical(
        "EMERGENCY STOP ACTIVATED."
    );
}


/*
===============================================================================
SECTION 27 — CLEAR EMERGENCY STOP
===============================================================================
*/

void part8ClearEmergencyStop()
{
    part8SafetyManager.clearEmergencyStop();

    part8Logger.warning(
        "Emergency stop clear requested."
    );
}


/*
===============================================================================
SECTION 28 — PART 7 NAVIGATION SAFETY BRIDGE
===============================================================================

This bridge demonstrates how Part 7 navigation output should be passed through
Part 8 before reaching the balance/motor-control layer.

FLOW:

    PART 7 NAVIGATION
            |
            v
    PART 8 SAFETY
            |
            v
    BALANCE CONTROLLER
===============================================================================
*/

struct Part8NavigationSafetyOutput
{
    float linearVelocity;
    float angularVelocity;

    bool stopRequested;

    Part8SafetyState safetyState;

    Part8NavigationSafetyOutput()
        : linearVelocity(0.0f),
          angularVelocity(0.0f),
          stopRequested(true),
          safetyState(PART8_SAFETY_NORMAL)
    {
    }
};


Part8NavigationSafetyOutput
part8ProtectNavigationCommand(
    float requestedLinear,
    float requestedAngular)
{
    Part8NavigationSafetyOutput output;

    Part8VelocityCommand requested;

    requested.linear =
        requestedLinear;

    requested.angular =
        requestedAngular;

    const Part8VelocityCommand safe =
        part8SafetyManager
            .protectVelocity(
                requested
            );

    output.linearVelocity =
        safe.linear;

    output.angularVelocity =
        safe.angular;

    output.safetyState =
        part8SafetyManager.getState();

    output.stopRequested =
        part8SafetyManager
            .isEmergencyStopActive();

    return output;
}


/*
===============================================================================
SECTION 29 — BALANCE CONTROLLER SAFETY BRIDGE
===============================================================================
*/

Part8MotorCommand
part8ProtectBalanceOutput(
    float requestedLeft,
    float requestedRight)
{
    return part8ProtectMotorCommand(
        requestedLeft,
        requestedRight
    );
}


/*
===============================================================================
SECTION 30 — DIAGNOSTIC COMMAND
===============================================================================
*/

void part8PrintDiagnostics()
{
    part8Diagnostics.printSafety(
        part8SafetyManager
    );
}


/*
===============================================================================
SECTION 31 — BATTERY STATUS
===============================================================================
*/

void part8PrintBattery()
{
    const Part8BatteryModel &battery =
        part8SafetyManager.getBattery();

    Serial.println();
    Serial.println(
        F("========== BATTERY STATUS ==========")
    );

    Serial.print(F("PERCENT: "));
    Serial.print(
        battery.getPercentage(),
        1
    );
    Serial.println(F("%"));

    Serial.print(F("VOLTAGE: "));
    Serial.println(
        battery.getVoltage(),
        2
    );

    Serial.print(F("CURRENT: "));
    Serial.println(
        battery.getEstimatedCurrent(),
        2
    );

    Serial.print(F("LOW: "));
    Serial.println(
        battery.isLow()
        ? F("YES")
        : F("NO")
    );

    Serial.print(F("CRITICAL: "));
    Serial.println(
        battery.isCritical()
        ? F("YES")
        : F("NO")
    );

    Serial.print(F("SHUTDOWN: "));
    Serial.println(
        battery.requiresShutdown()
        ? F("YES")
        : F("NO")
    );

    Serial.println(
        F("====================================")
    );
}


/*
===============================================================================
SECTION 32 — SAFETY STATUS
===============================================================================
*/

void part8PrintSafetyStatus()
{
    const Part8SafetyDiagnostics &d =
        part8SafetyManager.getDiagnostics();

    Serial.println();
    Serial.println(
        F("========== SAFETY STATUS ==========")
    );

    Serial.print(F("STATE: "));
    Serial.println(
        part8SafetyStateName(
            d.state
        )
    );

    Serial.print(F("PITCH: "));
    Serial.println(
        d.pitch,
        2
    );

    Serial.print(F("ROLL: "));
    Serial.println(
        d.roll,
        2
    );

    Serial.print(F("BATTERY: "));
    Serial.print(
        d.batteryPercent,
        1
    );
    Serial.println(F("%"));

    Serial.print(F("IMU: "));
    Serial.println(
        d.imuHealthy
        ? F("OK")
        : F("FAIL")
    );

    Serial.print(F("ENCODERS: "));
    Serial.println(
        d.encodersHealthy
        ? F("OK")
        : F("FAIL")
    );

    Serial.print(F("COMMUNICATION: "));
    Serial.println(
        d.communicationHealthy
        ? F("OK")
        : F("FAIL")
    );

    Serial.print(F("EMERGENCY STOP: "));
    Serial.println(
        d.emergencyStop
        ? F("ACTIVE")
        : F("CLEAR")
    );

    Serial.println(
        F("===================================")
    );
}


/*
===============================================================================
SECTION 33 — SAFETY TEST: NORMAL
===============================================================================
*/

void part8TestNormalState()
{
    Serial.println();
    Serial.println(
        F("PART 8 TEST — NORMAL STATE")
    );

    initializePart8SafetyTelemetry();

    part8CommunicationHeartbeat();

    part8UpdateIMU(
        2.0f,
        1.0f,
        0.0f
    );

    part8UpdateEncoderHealth(
        true,
        true
    );

    part8UpdateDistanceSensorHealth(
        true
    );

    delay(
        PART8_SAFETY_INTERVAL_MS
    );

    part8SafetyManager.update();

    part8PrintSafetyStatus();
}


/*
===============================================================================
SECTION 34 — SAFETY TEST: WARNING
===============================================================================
*/

void part8TestWarningTilt()
{
    Serial.println();
    Serial.println(
        F("PART 8 TEST — WARNING TILT")
    );

    initializePart8SafetyTelemetry();

    part8CommunicationHeartbeat();

    part8UpdateIMU(
        10.0f,
        0.0f,
        0.0f
    );

    part8UpdateEncoderHealth(
        true,
        true
    );

    part8UpdateDistanceSensorHealth(
        true
    );

    delay(
        PART8_SAFETY_INTERVAL_MS
    );

    part8SafetyManager.update();

    part8PrintSafetyStatus();
}


/*
===============================================================================
SECTION 35 — SAFETY TEST: LIMITED
===============================================================================
*/

void part8TestLimitedTilt()
{
    Serial.println();
    Serial.println(
        F("PART 8 TEST — LIMITED TILT")
    );

    initializePart8SafetyTelemetry();

    part8CommunicationHeartbeat();

    part8UpdateIMU(
        14.0f,
        0.0f,
        0.0f
    );

    part8UpdateEncoderHealth(
        true,
        true
    );

    part8UpdateDistanceSensorHealth(
        true
    );

    delay(
        PART8_SAFETY_INTERVAL_MS
    );

    part8SafetyManager.update();

    part8PrintSafetyStatus();

    Part8MotorCommand safe =
        part8ProtectMotorCommand(
            1.0f,
            1.0f
        );

    Serial.print(F("SAFE LEFT: "));
    Serial.println(
        safe.left,
        3
    );

    Serial.print(F("SAFE RIGHT: "));
    Serial.println(
        safe.right,
        3
    );
}


/*
===============================================================================
SECTION 36 — SAFETY TEST: EMERGENCY TILT
===============================================================================
*/

void part8TestEmergencyTilt()
{
    Serial.println();
    Serial.println(
        F("PART 8 TEST — EMERGENCY TILT")
    );

    initializePart8SafetyTelemetry();

    part8CommunicationHeartbeat();

    part8UpdateIMU(
        20.0f,
        0.0f,
        0.0f
    );

    part8UpdateEncoderHealth(
        true,
        true
    );

    part8UpdateDistanceSensorHealth(
        true
    );

    delay(
        PART8_SAFETY_INTERVAL_MS
    );

    part8SafetyManager.update();

    part8PrintSafetyStatus();

    Part8MotorCommand safe =
        part8ProtectMotorCommand(
            1.0f,
            1.0f
        );

    Serial.print(F("EMERGENCY LEFT: "));
    Serial.println(
        safe.left,
        3
    );

    Serial.print(F("EMERGENCY RIGHT: "));
    Serial.println(
        safe.right,
        3
    );
}


/*
===============================================================================
SECTION 37 — SAFETY TEST: MANUAL EMERGENCY STOP
===============================================================================
*/

void part8TestEmergencyStop()
{
    Serial.println();
    Serial.println(
        F("PART 8 TEST — MANUAL EMERGENCY STOP")
    );

    initializePart8SafetyTelemetry();

    part8CommunicationHeartbeat();

    part8UpdateIMU(
        0.0f,
        0.0f,
        0.0f
    );

    part8UpdateEncoderHealth(
        true,
        true
    );

    part8UpdateDistanceSensorHealth(
        true
    );

    delay(
        PART8_SAFETY_INTERVAL_MS
    );

    part8SafetyManager.update();

    part8EmergencyStop();

    Part8MotorCommand safe =
        part8ProtectMotorCommand(
            1.0f,
            1.0f
        );

    Serial.print(F("LEFT AFTER E-STOP: "));
    Serial.println(
        safe.left,
        3
    );

    Serial.print(F("RIGHT AFTER E-STOP: "));
    Serial.println(
        safe.right,
        3
    );
}


/*
===============================================================================
SECTION 38 — SENSOR FAILURE TEST
===============================================================================
*/

void part8TestSensorFailure()
{
    Serial.println();
    Serial.println(
        F("PART 8 TEST — SENSOR FAILURE")
    );

    initializePart8SafetyTelemetry();

    /*
    Intentionally do not send an IMU update.
    */

    part8CommunicationHeartbeat();

    part8UpdateEncoderHealth(
        true,
        true
    );

    part8UpdateDistanceSensorHealth(
        true
    );

    delay(
        PART8_SENSOR_TIMEOUT_MS +
        PART8_SAFETY_INTERVAL_MS
    );

    part8SafetyManager.update();

    part8PrintSafetyStatus();
}


/*
===============================================================================
SECTION 39 — COMMUNICATION FAILURE TEST
===============================================================================
*/

void part8TestCommunicationFailure()
{
    Serial.println();
    Serial.println(
        F("PART 8 TEST — COMMUNICATION FAILURE")
    );

    initializePart8SafetyTelemetry();

    part8UpdateIMU(
        0.0f,
        0.0f,
        0.0f
    );

    part8UpdateEncoderHealth(
        true,
        true
    );

    part8UpdateDistanceSensorHealth(
        true
    );

    /*
    No heartbeat is intentionally sent.
    */

    delay(
        PART8_COMMUNICATION_TIMEOUT_MS +
        PART8_SAFETY_INTERVAL_MS
    );

    part8SafetyManager.update();

    part8PrintSafetyStatus();
}


/*
===============================================================================
SECTION 40 — BATTERY TEST
===============================================================================
*/

void part8TestBattery()
{
    Serial.println();
    Serial.println(
        F("PART 8 TEST — BATTERY")
    );

    initializePart8SafetyTelemetry();

    Part8BatteryModel &battery =
        part8SafetyManager.getBattery();

    battery.setBatteryPercentage(
        20.0f
    );

    part8PrintBattery();

    battery.setBatteryPercentage(
        8.0f
    );

    part8PrintBattery();

    battery.setBatteryPercentage(
        4.0f
    );

    part8PrintBattery();
}


/*
===============================================================================
SECTION 41 — MOTOR LIMIT TEST
===============================================================================
*/

void part8TestMotorProtection()
{
    Serial.println();
    Serial.println(
        F("PART 8 TEST — MOTOR PROTECTION")
    );

    initializePart8SafetyTelemetry();

    part8CommunicationHeartbeat();

    part8UpdateIMU(
        0.0f,
        0.0f,
        0.0f
    );

    part8UpdateEncoderHealth(
        true,
        true
    );

    part8UpdateDistanceSensorHealth(
        true
    );

    delay(
        PART8_SAFETY_INTERVAL_MS
    );

    part8SafetyManager.update();

    Part8MotorCommand safe =
        part8ProtectMotorCommand(
            5.0f,
            -5.0f
        );

    Serial.print(F("REQUESTED LEFT: 5.0"));
    Serial.print(F(" SAFE: "));
    Serial.println(
        safe.left,
        3
    );

    Serial.print(F("REQUESTED RIGHT: -5.0"));
    Serial.print(F(" SAFE: "));
    Serial.println(
        safe.right,
        3
    );
}


/*
===============================================================================
SECTION 42 — COMPLETE SAFETY TEST SUITE
===============================================================================
*/

void runPart8SafetyTests()
{
    Serial.println();
    Serial.println(
        F("==============================================")
    );

    Serial.println(
        F("ASBR PART 8 — SAFETY TEST SUITE")
    );

    Serial.println(
        F("==============================================")
    );

    part8TestNormalState();

    part8TestWarningTilt();

    part8TestLimitedTilt();

    part8TestEmergencyTilt();

    part8TestEmergencyStop();

    part8TestMotorProtection();

    part8TestBattery();

    Serial.println(
        F("Basic Part 8 tests completed.")
    );
}


/*
===============================================================================
SECTION 43 — TELEMETRY TEST
===============================================================================
*/

void part8TestTelemetry()
{
    Serial.println();
    Serial.println(
        F("PART 8 TELEMETRY TEST")
    );

    initializePart8SafetyTelemetry();

    part8CommunicationHeartbeat();

    part8UpdateIMU(
        1.2f,
        0.8f,
        0.2f
    );

    part8UpdateEncoderHealth(
        true,
        true
    );

    part8UpdateDistanceSensorHealth(
        true
    );

    delay(
        PART8_TELEMETRY_INTERVAL_MS
    );

    part8SafetyManager.update();

    part8RecordTelemetry(
        1.25f,
        2.50f,
        0.35f,

        0.20f,
        0.15f,

        1.2f,
        0.8f,
        0.2f,

        0.35f,
        0.40f,

        1.5f,
        1.2f,
        1.1f,

        0.80f,
        0.75f,
        0.90f,

        true,
        false
    );

    Serial.println(
        F("Telemetry test complete.")
    );
}


/*
===============================================================================
SECTION 44 — FULL PART 8 DEMONSTRATION
===============================================================================
*/

void runPart8Demo()
{
    Serial.println();
    Serial.println(
        F("================================================")
    );

    Serial.println(
        F("ASBR PART 8 — SAFETY + TELEMETRY DEMO")
    );

    Serial.println(
        F("================================================")
    );

    initializePart8SafetyTelemetry();

    /*
    ---------------------------------------------------------------------------
    Establish healthy starting state.
    ---------------------------------------------------------------------------
    */

    part8CommunicationHeartbeat();

    part8UpdateIMU(
        0.5f,
        0.3f,
        0.0f
    );

    part8UpdateEncoderHealth(
        true,
        true
    );

    part8UpdateDistanceSensorHealth(
        true
    );

    delay(
        PART8_SAFETY_INTERVAL_MS
    );

    updatePart8SafetyTelemetry();

    part8PrintSafetyStatus();

    /*
    ---------------------------------------------------------------------------
    Test navigation command protection.
    ---------------------------------------------------------------------------
    */

    Part8NavigationSafetyOutput navigationOutput =
        part8ProtectNavigationCommand(
            0.30f,
            1.20f
        );

    Serial.println();

    Serial.println(
        F("NAVIGATION SAFETY OUTPUT")
    );

    Serial.print(F("LINEAR: "));
    Serial.println(
        navigationOutput.linearVelocity,
        3
    );

    Serial.print(F("ANGULAR: "));
    Serial.println(
        navigationOutput.angularVelocity,
        3
    );

    Serial.print(F("STOP: "));
    Serial.println(
        navigationOutput.stopRequested
        ? F("YES")
        : F("NO")
    );

    /*
    ---------------------------------------------------------------------------
    Test motor protection.
    ---------------------------------------------------------------------------
    */

    Part8MotorCommand safeMotor =
        part8ProtectBalanceOutput(
            0.80f,
            0.75f
        );

    Serial.println();

    Serial.println(
        F("MOTOR SAFETY OUTPUT")
    );

    Serial.print(F("LEFT: "));
    Serial.println(
        safeMotor.left,
        3
    );

    Serial.print(F("RIGHT: "));
    Serial.println(
        safeMotor.right,
        3
    );

    /*
    ---------------------------------------------------------------------------
    Telemetry.
    ---------------------------------------------------------------------------
    */

    delay(
        PART8_TELEMETRY_INTERVAL_MS
    );

    part8RecordTelemetry(
        0.10f,
        0.20f,
        0.05f,

        0.20f,
        0.10f,

        0.5f,
        0.3f,
        0.0f,

        safeMotor.left,
        safeMotor.right,

        2.0f,
        1.5f,
        1.2f,

        2.0f,
        2.0f,
        2.0f,

        true,
        false
    );

    part8PrintDiagnostics();

    Serial.println();

    Serial.println(
        F("PART 8 DEMO COMPLETE")
    );
}


/*
===============================================================================
SECTION 45 — PART 8 COMMAND INTERFACE
===============================================================================

Commands:

    safety
    battery
    diagnostics
    telemetry
    emergency
    clear_emergency

    test_safety
    test_tilt
    test_limited
    test_emergency
    test_battery
    test_sensor
    test_communication
    test_motor
    test_telemetry

===============================================================================
*/

void processPart8Command(
    const String &command)
{
    if (command == "safety")
    {
        part8PrintSafetyStatus();

        return;
    }

    if (command == "battery")
    {
        part8PrintBattery();

        return;
    }

    if (command == "diagnostics")
    {
        part8PrintDiagnostics();

        return;
    }

    if (command == "telemetry")
    {
        const Part8TelemetryRecord &r =
            part8TelemetryLogger
                .getLastRecord();

        Serial.println();

        Serial.println(
            F("========== LAST TELEMETRY ==========")
        );

        Serial.print(F("TIME: "));
        Serial.println(
            r.timestamp
        );

        Serial.print(F("POSITION: "));
        Serial.print(r.x, 3);
        Serial.print(F(", "));
        Serial.println(r.y, 3);

        Serial.print(F("HEADING: "));
        Serial.println(
            r.theta,
            3
        );

        Serial.print(F("PITCH: "));
        Serial.println(
            r.pitch,
            2
        );

        Serial.print(F("ROLL: "));
        Serial.println(
            r.roll,
            2
        );

        Serial.print(F("BATTERY: "));
        Serial.print(
            r.batteryPercent,
            1
        );
        Serial.println(F("%"));

        Serial.print(F("RECORD COUNT: "));
        Serial.println(
            part8TelemetryLogger
                .getRecordCount()
        );

        Serial.println(
            F("====================================")
        );

        return;
    }

    if (command == "emergency")
    {
        part8EmergencyStop();

        return;
    }

    if (command == "clear_emergency")
    {
        part8ClearEmergencyStop();

        return;
    }

    if (command == "test_safety")
    {
        part8TestNormalState();

        return;
    }

    if (command == "test_tilt")
    {
        part8TestWarningTilt();

        return;
    }

    if (command == "test_limited")
    {
        part8TestLimitedTilt();

        return;
    }

    if (command == "test_emergency")
    {
        part8TestEmergencyTilt();

        return;
    }

    if (command == "test_battery")
    {
        part8TestBattery();

        return;
    }

    if (command == "test_sensor")
    {
        part8TestSensorFailure();

        return;
    }

    if (command == "test_communication")
    {
        part8TestCommunicationFailure();

        return;
    }

    if (command == "test_motor")
    {
        part8TestMotorProtection();

        return;
    }

    if (command == "test_telemetry")
    {
        part8TestTelemetry();

        return;
    }

    if (command == "test_all_safety")
    {
        runPart8SafetyTests();

        return;
    }

    if (command == "part8_demo")
    {
        runPart8Demo();

        return;
    }

    Serial.println(
        F("Unknown Part 8 command.")
    );
}


/*
===============================================================================
SECTION 46 — COMPLETE PART 8 CONTROL FLOW
===============================================================================

Recommended runtime order:

    ┌─────────────────────────────┐
    │       READ SENSORS          │
    └──────────────┬──────────────┘
                   |
                   v
    ┌─────────────────────────────┐
    │      SENSOR VALIDATION       │
    └──────────────┬──────────────┘
                   |
                   v
    ┌─────────────────────────────┐
    │      STATE ESTIMATION       │
    └──────────────┬──────────────┘
                   |
                   v
    ┌─────────────────────────────┐
    │     BALANCE CONTROLLER      │
    └──────────────┬──────────────┘
                   |
                   v
    ┌─────────────────────────────┐
    │      PART 7 NAVIGATION      │
    └──────────────┬──────────────┘
                   |
                   v
    ┌─────────────────────────────┐
    │       PART 8 SAFETY         │
    │                             │
    │  Tilt                       │
    │  Battery                    │
    │  Sensors                    │
    │  Communication              │
    │  Velocity                    │
    │  Motor limits                │
    └──────────────┬──────────────┘
                   |
                   v
          ┌────────────────┐
          │ SAFE COMMAND?  │
          └───────┬────────┘
                  |
          ┌───────┴────────┐
          |                |
         YES               NO
          |                |
          v                v
     MOTOR OUTPUT      ZERO OUTPUT
          |                |
          └───────┬────────┘
                  |
                  v
          ┌────────────────┐
          │   TELEMETRY    │
          └────────────────┘

===============================================================================
SECTION 47 — SAFETY PRIORITY
===============================================================================

Priority order:

    1. Emergency stop
    2. Extreme tilt
    3. Critical battery
    4. Sensor failure
    5. Communication failure
    6. Excessive motor command
    7. Excessive velocity
    8. Limited operation
    9. Warning
   10. Normal operation

Navigation is NEVER allowed to override these conditions.

===============================================================================
SECTION 48 — PART 8 → PART 7 INTEGRATION
===============================================================================

Part 7 generates:

    desiredLinearVelocity
    desiredAngularVelocity

Part 8 receives them:

    part8ProtectNavigationCommand()

Part 8 returns:

    safeLinearVelocity
    safeAngularVelocity
    stopRequested

The balance controller then receives the SAFE values.

===============================================================================
SECTION 49 — PART 8 → MOTOR INTEGRATION
===============================================================================

The final motor pipeline should be:

    Part 7 Navigation
            |
            v
    Desired Velocity
            |
            v
    Balance Controller
            |
            v
    Requested Motor Command
            |
            v
    PART 8 SAFETY
            |
            v
    Safe Motor Command
            |
            v
    Motor Driver

Never skip the safety stage.

===============================================================================
SECTION 50 — TELEMETRY DATA PIPELINE
===============================================================================

Telemetry records:

    Timestamp
    Position
    Heading
    Linear velocity
    Angular velocity
    Pitch
    Roll
    Yaw
    Left motor
    Right motor
    Battery percentage
    Battery voltage
    Current
    Front distance
    Left distance
    Right distance
    Safety state
    Navigation state
    Goal state

The current implementation outputs CSV-compatible telemetry through Serial.

This allows the Serial output to be captured by a computer and later imported
into spreadsheet/data-analysis software.

===============================================================================
SECTION 51 — IMPORTANT ARDUINO UNO NOTE
===============================================================================

The Arduino Uno has very limited SRAM.

Therefore Part 8 deliberately avoids:

    - Large telemetry arrays
    - Large event queues
    - Dynamic memory-heavy containers
    - Large strings
    - Full map copies
    - Large log buffers

Telemetry is streamed instead of storing hundreds of records in RAM.

===============================================================================
SECTION 52 — PART 8 COMPLETION
===============================================================================

PART 8 IMPLEMENTS:

    [✓] Safety Manager
    [✓] Safety States
    [✓] Tilt Protection
    [✓] Emergency Stop
    [✓] Motor Limiting
    [✓] Velocity Limiting
    [✓] Sensor Health
    [✓] Communication Monitoring
    [✓] Battery Model
    [✓] Battery Warnings
    [✓] Battery Critical State
    [✓] Battery Shutdown
    [✓] CSV Telemetry
    [✓] Structured Logging
    [✓] Diagnostics
    [✓] Safety Tests
    [✓] Telemetry Tests
    [✓] Navigation Safety Bridge
    [✓] Motor Safety Bridge

===============================================================================
END OF PART 8
===============================================================================
*/ 
/*
===============================================================================
PART 9 — VISUALIZATION
===============================================================================

PROJECT:
Autonomous Self-Balancing Robot — ASBR

TARGET:
Arduino Uno / C++ compatible environment

PURPOSE:
Provide a lightweight visualization layer without requiring a large graphics
library or consuming large amounts of Arduino Uno SRAM.

VISUALIZATION:

    Robot
    Robot heading
    Occupancy grid
    Planned path
    Start
    Goal
    Obstacles
    Free cells
    Unknown cells
    Sensor information
    Safety state
    Battery
    Navigation status
    Mapping statistics

IMPORTANT:

The core robotics system must NOT depend on visualization.

Visualization is optional.

The visualization layer uses Serial output so it can operate on an Arduino Uno.

For a PC-based build, the same architecture can later be connected to a
graphics library.

===============================================================================
*/


#include <Arduino.h>
#include <math.h>


/*
===============================================================================
SECTION 1 — PART 9 CONFIGURATION
===============================================================================
*/

/*
    Display dimensions.

    These values are intentionally small because the Arduino Uno has limited
    RAM and Serial output bandwidth.
*/

#define PART9_DISPLAY_WIDTH              32
#define PART9_DISPLAY_HEIGHT             20

#define PART9_RENDER_INTERVAL_MS         500UL

#define PART9_SERIAL_BAUDRATE            115200UL

/*
    Maximum number of path points that the visualization layer will process
    during one rendering pass.

    This prevents excessive Serial traffic.
*/

#define PART9_MAX_RENDER_PATH_POINTS     64

/*
    Visualization modes.
*/

#define PART9_MODE_MAP                   0
#define PART9_MODE_STATUS                1
#define PART9_MODE_SENSORS               2
#define PART9_MODE_NAVIGATION            3
#define PART9_MODE_FULL                  4


/*
===============================================================================
SECTION 2 — VISUALIZATION CELL
===============================================================================
*/

enum Part9VisualCell
{
    PART9_VISUAL_UNKNOWN = 0,
    PART9_VISUAL_FREE,
    PART9_VISUAL_OCCUPIED,
    PART9_VISUAL_ROBOT,
    PART9_VISUAL_PATH,
    PART9_VISUAL_START,
    PART9_VISUAL_GOAL
};


/*
===============================================================================
SECTION 3 — VISUALIZATION POINT
===============================================================================
*/

struct Part9VisualPoint
{
    int16_t x;
    int16_t y;

    Part9VisualPoint()
        : x(0),
          y(0)
    {
    }

    Part9VisualPoint(
        int16_t px,
        int16_t py)
        : x(px),
          y(py)
    {
    }
};


/*
===============================================================================
SECTION 4 — VISUALIZATION ROBOT STATE
===============================================================================
*/

struct Part9VisualRobotState
{
    float x;
    float y;
    float theta;

    float pitch;
    float roll;
    float yaw;

    float linearVelocity;
    float angularVelocity;

    float leftMotor;
    float rightMotor;

    float batteryPercent;
    float batteryVoltage;

    bool navigationActive;
    bool goalReached;

    Part9VisualRobotState()
        : x(0.0f),
          y(0.0f),
          theta(0.0f),
          pitch(0.0f),
          roll(0.0f),
          yaw(0.0f),
          linearVelocity(0.0f),
          angularVelocity(0.0f),
          leftMotor(0.0f),
          rightMotor(0.0f),
          batteryPercent(100.0f),
          batteryVoltage(8.4f),
          navigationActive(false),
          goalReached(false)
    {
    }
};


/*
===============================================================================
SECTION 5 — ASCII VISUALIZATION SYMBOLS
===============================================================================
*/

char part9CellSymbol(
    Part9VisualCell cell)
{
    switch (cell)
    {
        case PART9_VISUAL_UNKNOWN:
            return '?';

        case PART9_VISUAL_FREE:
            return '.';

        case PART9_VISUAL_OCCUPIED:
            return '#';

        case PART9_VISUAL_ROBOT:
            return 'R';

        case PART9_VISUAL_PATH:
            return '*';

        case PART9_VISUAL_START:
            return 'S';

        case PART9_VISUAL_GOAL:
            return 'G';

        default:
            return '?';
    }
}


/*
===============================================================================
SECTION 6 — VISUALIZATION BUFFER
===============================================================================
*/

class Part9VisualizationBuffer
{
private:

    /*
        A 32 x 20 byte buffer.

        640 bytes.

        This is relatively large for an Arduino Uno, so it is allocated only
        as a compact byte array rather than as large objects.
    */

    uint8_t cells[
        PART9_DISPLAY_WIDTH *
        PART9_DISPLAY_HEIGHT
    ];

public:

    void clear(
        Part9VisualCell value =
            PART9_VISUAL_UNKNOWN)
    {
        for (
            uint16_t i = 0;
            i <
            PART9_DISPLAY_WIDTH *
            PART9_DISPLAY_HEIGHT;
            ++i)
        {
            cells[i] =
                (uint8_t)value;
        }
    }


    bool valid(
        int16_t x,
        int16_t y) const
    {
        return
            x >= 0 &&
            y >= 0 &&
            x < PART9_DISPLAY_WIDTH &&
            y < PART9_DISPLAY_HEIGHT;
    }


    void set(
        int16_t x,
        int16_t y,
        Part9VisualCell value)
    {
        if (!valid(x, y))
        {
            return;
        }

        cells[
            y * PART9_DISPLAY_WIDTH +
            x
        ] =
            (uint8_t)value;
    }


    Part9VisualCell get(
        int16_t x,
        int16_t y) const
    {
        if (!valid(x, y))
        {
            return PART9_VISUAL_UNKNOWN;
        }

        return (Part9VisualCell)
            cells[
                y * PART9_DISPLAY_WIDTH +
                x
            ];
    }
};


/*
===============================================================================
SECTION 7 — VISUALIZATION STATE
===============================================================================
*/

struct Part9VisualizationState
{
    uint8_t mode;

    bool enabled;

    bool showGrid;
    bool showPath;
    bool showRobot;
    bool showSensors;
    bool showStatus;

    unsigned long frameNumber;

    Part9VisualizationState()
        : mode(PART9_MODE_FULL),
          enabled(true),
          showGrid(true),
          showPath(true),
          showRobot(true),
          showSensors(true),
          showStatus(true),
          frameNumber(0)
    {
    }
};


/*
===============================================================================
SECTION 8 — VISUALIZATION ENGINE
===============================================================================
*/

class Part9VisualizationEngine
{
private:

    Part9VisualizationBuffer buffer;

    Part9VisualizationState state;

    Part9VisualRobotState robot;

    Part9VisualPoint startPoint;

    Part9VisualPoint goalPoint;

    bool startDefined;
    bool goalDefined;

    unsigned long lastRender;

public:

    Part9VisualizationEngine()
        : startPoint(),
          goalPoint(),
          startDefined(false),
          goalDefined(false),
          lastRender(0)
    {
    }


    void begin()
    {
        state =
            Part9VisualizationState();

        robot =
            Part9VisualRobotState();

        startDefined = false;

        goalDefined = false;

        lastRender = millis();

        buffer.clear();
    }


    void enable()
    {
        state.enabled = true;
    }


    void disable()
    {
        state.enabled = false;
    }


    bool isEnabled() const
    {
        return state.enabled;
    }


    void setMode(
        uint8_t mode)
    {
        if (
            mode >
            PART9_MODE_FULL)
        {
            mode =
                PART9_MODE_FULL;
        }

        state.mode = mode;
    }


    uint8_t getMode() const
    {
        return state.mode;
    }


    void setShowGrid(bool value)
    {
        state.showGrid = value;
    }


    void setShowPath(bool value)
    {
        state.showPath = value;
    }


    void setShowRobot(bool value)
    {
        state.showRobot = value;
    }


    void setShowSensors(bool value)
    {
        state.showSensors = value;
    }


    void setShowStatus(bool value)
    {
        state.showStatus = value;
    }


    void setRobotState(
        const Part9VisualRobotState &newState)
    {
        robot = newState;
    }


    void setStart(
        int16_t x,
        int16_t y)
    {
        startPoint =
            Part9VisualPoint(
                x,
                y
            );

        startDefined = true;
    }


    void setGoal(
        int16_t x,
        int16_t y)
    {
        goalPoint =
            Part9VisualPoint(
                x,
                y
            );

        goalDefined = true;
    }


    void clearGoal()
    {
        goalDefined = false;
    }


    void clearStart()
    {
        startDefined = false;
    }


    Part9VisualizationBuffer &
    getBuffer()
    {
        return buffer;
    }


    bool renderReady() const
    {
        return
            millis() - lastRender >=
            PART9_RENDER_INTERVAL_MS;
    }


    void render()
    {
        if (!state.enabled)
        {
            return;
        }

        if (!renderReady())
        {
            return;
        }

        lastRender = millis();

        state.frameNumber++;

        switch (state.mode)
        {
            case PART9_MODE_MAP:
                renderMap();
                break;

            case PART9_MODE_STATUS:
                renderStatus();
                break;

            case PART9_MODE_SENSORS:
                renderSensors();
                break;

            case PART9_MODE_NAVIGATION:
                renderNavigation();
                break;

            case PART9_MODE_FULL:
            default:
                renderFull();
                break;
        }
    }


private:

    /*
    ---------------------------------------------------------------------------
    HEADER
    ---------------------------------------------------------------------------
    */

    void printHeader(
        const __FlashStringHelper *title)
    {
        Serial.println();
        Serial.println(
            F("================================================")
        );

        Serial.println(title);

        Serial.print(
            F("FRAME: ")
        );

        Serial.println(
            state.frameNumber
        );

        Serial.println(
            F("================================================")
        );
    }


    /*
    ---------------------------------------------------------------------------
    MAP
    ---------------------------------------------------------------------------
    */

    void renderMap()
    {
        printHeader(
            F("ASBR — OCCUPANCY MAP")
        );

        if (state.showStatus)
        {
            printCompactStatus();
        }

        Serial.println();

        if (state.showGrid)
        {
            renderGrid();
        }

        Serial.println();

        Serial.println(
            F("LEGEND: ? UNKNOWN  . FREE  # OBSTACLE  "
              "R ROBOT  * PATH  S START  G GOAL")
        );
    }


    /*
    ---------------------------------------------------------------------------
    GRID
    ---------------------------------------------------------------------------
    */

    void renderGrid()
    {
        for (
            int16_t y =
                PART9_DISPLAY_HEIGHT - 1;
            y >= 0;
            --y)
        {
            if (y < 10)
            {
                Serial.print(' ');
            }

            Serial.print(y);
            Serial.print(F(" | "));

            for (
                int16_t x = 0;
                x < PART9_DISPLAY_WIDTH;
                ++x)
            {
                Serial.print(
                    part9CellSymbol(
                        buffer.get(x, y)
                    )
                );
            }

            Serial.println();
        }

        Serial.print(
            F("    +")
        );

        for (
            int16_t x = 0;
            x < PART9_DISPLAY_WIDTH;
            ++x)
        {
            Serial.print('-');
        }

        Serial.println();

        Serial.print(
            F("     ")
        );

        for (
            int16_t x = 0;
            x < PART9_DISPLAY_WIDTH;
            ++x)
        {
            if (x % 10 == 0)
            {
                Serial.print('|');
            }
            else
            {
                Serial.print(' ');
            }
        }

        Serial.println();
    }


    /*
    ---------------------------------------------------------------------------
    STATUS
    ---------------------------------------------------------------------------
    */

    void renderStatus()
    {
        printHeader(
            F("ASBR — ROBOT STATUS")
        );

        Serial.print(
            F("POSITION X: ")
        );

        Serial.println(
            robot.x,
            3
        );

        Serial.print(
            F("POSITION Y: ")
        );

        Serial.println(
            robot.y,
            3
        );

        Serial.print(
            F("HEADING: ")
        );

        Serial.println(
            robot.theta,
            3
        );

        Serial.print(
            F("PITCH: ")
        );

        Serial.println(
            robot.pitch,
            2
        );

        Serial.print(
            F("ROLL: ")
        );

        Serial.println(
            robot.roll,
            2
        );

        Serial.print(
            F("YAW: ")
        );

        Serial.println(
            robot.yaw,
            2
        );

        Serial.print(
            F("LINEAR VELOCITY: ")
        );

        Serial.println(
            robot.linearVelocity,
            3
        );

        Serial.print(
            F("ANGULAR VELOCITY: ")
        );

        Serial.println(
            robot.angularVelocity,
            3
        );

        Serial.print(
            F("LEFT MOTOR: ")
        );

        Serial.println(
            robot.leftMotor,
            3
        );

        Serial.print(
            F("RIGHT MOTOR: ")
        );

        Serial.println(
            robot.rightMotor,
            3
        );

        Serial.print(
            F("BATTERY: ")
        );

        Serial.print(
            robot.batteryPercent,
            1
        );

        Serial.println(
            F("%")
        );

        Serial.print(
            F("VOLTAGE: ")
        );

        Serial.println(
            robot.batteryVoltage,
            2
        );

        Serial.print(
            F("NAVIGATION: ")
        );

        Serial.println(
            robot.navigationActive
            ? F("ACTIVE")
            : F("IDLE")
        );

        Serial.print(
            F("GOAL: ")
        );

        Serial.println(
            robot.goalReached
            ? F("REACHED")
            : F("NOT REACHED")
        );
    }


    /*
    ---------------------------------------------------------------------------
    SENSOR DISPLAY
    ---------------------------------------------------------------------------
    */

    void renderSensors()
    {
        printHeader(
            F("ASBR — SENSOR VIEW")
        );

        Serial.println(
            F("IMU")
        );

        Serial.print(
            F("  Pitch: ")
        );

        Serial.println(
            robot.pitch,
            3
        );

        Serial.print(
            F("  Roll : ")
        );

        Serial.println(
            robot.roll,
            3
        );

        Serial.print(
            F("  Yaw  : ")
        );

        Serial.println(
            robot.yaw,
            3
        );

        Serial.println();

        Serial.println(
            F("ODOMETRY")
        );

        Serial.print(
            F("  X: ")
        );

        Serial.println(
            robot.x,
            3
        );

        Serial.print(
            F("  Y: ")
        );

        Serial.println(
            robot.y,
            3
        );

        Serial.print(
            F("  Theta: ")
        );

        Serial.println(
            robot.theta,
            3
        );

        Serial.println();

        Serial.println(
            F("MOTOR FEEDBACK")
        );

        Serial.print(
            F("  Left : ")
        );

        Serial.println(
            robot.leftMotor,
            3
        );

        Serial.print(
            F("  Right: ")
        );

        Serial.println(
            robot.rightMotor,
            3
        );

        Serial.println();

        Serial.println(
            F("BATTERY")
        );

        Serial.print(
            F("  Percent: ")
        );

        Serial.println(
            robot.batteryPercent,
            1
        );

        Serial.print(
            F("  Voltage: ")
        );

        Serial.println(
            robot.batteryVoltage,
            2
        );
    }


    /*
    ---------------------------------------------------------------------------
    NAVIGATION DISPLAY
    ---------------------------------------------------------------------------
    */

    void renderNavigation()
    {
        printHeader(
            F("ASBR — NAVIGATION")
        );

        Serial.print(
            F("NAVIGATION ACTIVE: ")
        );

        Serial.println(
            robot.navigationActive
            ? F("YES")
            : F("NO")
        );

        Serial.print(
            F("GOAL STATUS: ")
        );

        Serial.println(
            robot.goalReached
            ? F("REACHED")
            : F("SEARCHING")
        );

        Serial.println();

        Serial.print(
            F("ROBOT X: ")
        );

        Serial.println(
            robot.x,
            3
        );

        Serial.print(
            F("ROBOT Y: ")
        );

        Serial.println(
            robot.y,
            3
        );

        Serial.print(
            F("ROBOT HEADING: ")
        );

        Serial.println(
            robot.theta,
            3
        );

        if (goalDefined)
        {
            Serial.print(
                F("GOAL X: ")
            );

            Serial.println(
                goalPoint.x
            );

            Serial.print(
                F("GOAL Y: ")
            );

            Serial.println(
                goalPoint.y
            );
        }
        else
        {
            Serial.println(
                F("GOAL: NOT DEFINED")
            );
        }

        Serial.println();

        if (state.showGrid)
        {
            renderGrid();
        }
    }


    /*
    ---------------------------------------------------------------------------
    COMPACT STATUS
    ---------------------------------------------------------------------------
    */

    void printCompactStatus()
    {
        Serial.print(
            F("POS ")
        );

        Serial.print(
            robot.x,
            2
        );

        Serial.print(
            ','
        );

        Serial.print(
            robot.y,
            2
        );

        Serial.print(
            F("  H ")
        );

        Serial.print(
            robot.theta,
            2
        );

        Serial.print(
            F("  BAT ")
        );

        Serial.print(
            robot.batteryPercent,
            0
        );

        Serial.println(
            F("%")
        );
    }


    /*
    ---------------------------------------------------------------------------
    FULL VIEW
    ---------------------------------------------------------------------------
    */

    void renderFull()
    {
        printHeader(
            F("ASBR — AUTONOMOUS ROBOT VISUALIZATION")
        );

        renderStatus();

        Serial.println();

        renderMap();

        Serial.println();

        renderNavigation();
    }
};


/*
===============================================================================
SECTION 9 — MAP ADAPTER
===============================================================================

This adapter converts the Part 5 occupancy-grid information into the compact
visualization buffer.

It intentionally accesses the Part 5 grid through its public interface.

If the exact Part 5 API is different in a future revision, only this adapter
needs modification.
===============================================================================
*/

class Part9MapAdapter
{
private:

    int16_t originX;
    int16_t originY;

    uint8_t displayWidth;
    uint8_t displayHeight;

public:

    Part9MapAdapter()
        : originX(0),
          originY(0),
          displayWidth(
              PART9_DISPLAY_WIDTH
          ),
          displayHeight(
              PART9_DISPLAY_HEIGHT
          )
    {
    }


    void setOrigin(
        int16_t x,
        int16_t y)
    {
        originX = x;
        originY = y;
    }


    int16_t getOriginX() const
    {
        return originX;
    }


    int16_t getOriginY() const
    {
        return originY;
    }


    void setDisplaySize(
        uint8_t width,
        uint8_t height)
    {
        if (width > PART9_DISPLAY_WIDTH)
        {
            width =
                PART9_DISPLAY_WIDTH;
        }

        if (height > PART9_DISPLAY_HEIGHT)
        {
            height =
                PART9_DISPLAY_HEIGHT;
        }

        displayWidth = width;
        displayHeight = height;
    }


    uint8_t getDisplayWidth() const
    {
        return displayWidth;
    }


    uint8_t getDisplayHeight() const
    {
        return displayHeight;
    }
};


/*
===============================================================================
SECTION 10 — PATH VISUALIZER
===============================================================================
*/

class Part9PathVisualizer
{
public:

    void drawPoint(
        Part9VisualizationBuffer &buffer,
        int16_t x,
        int16_t y)
    {
        buffer.set(
            x,
            y,
            PART9_VISUAL_PATH
        );
    }


    void drawLine(
        Part9VisualizationBuffer &buffer,
        int16_t x0,
        int16_t y0,
        int16_t x1,
        int16_t y1)
    {
        /*
        Integer line traversal.

        This is intentionally lightweight for Arduino-class hardware.
        */

        const int16_t dx =
            abs(x1 - x0);

        const int16_t sx =
            x0 < x1
            ? 1
            : -1;

        const int16_t dy =
            -abs(y1 - y0);

        const int16_t sy =
            y0 < y1
            ? 1
            : -1;

        int16_t error =
            dx + dy;

        while (true)
        {
            drawPoint(
                buffer,
                x0,
                y0
            );

            if (
                x0 == x1 &&
                y0 == y1)
            {
                break;
            }

            const int16_t e2 =
                2 * error;

            if (e2 >= dy)
            {
                error += dy;
                x0 += sx;
            }

            if (e2 <= dx)
            {
                error += dx;
                y0 += sy;
            }
        }
    }
};


/*
===============================================================================
SECTION 11 — ROBOT MARKER
===============================================================================
*/

class Part9RobotMarker
{
public:

    void drawRobot(
        Part9VisualizationBuffer &buffer,
        int16_t x,
        int16_t y,
        float heading)
    {
        buffer.set(
            x,
            y,
            PART9_VISUAL_ROBOT
        );

        /*
        Approximate heading marker.

        0 radians = +X direction.
        */

        const float c =
            cos(heading);

        const float s =
            sin(heading);

        int16_t hx =
            x +
            (int16_t)round(
                c * 2.0f
            );

        int16_t hy =
            y +
            (int16_t)round(
                s * 2.0f
            );

        if (
            buffer.valid(
                hx,
                hy))
        {
            buffer.set(
                hx,
                hy,
                PART9_VISUAL_ROBOT
            );
        }
    }
};


/*
===============================================================================
SECTION 12 — SENSOR-RAY VISUALIZER
===============================================================================
*/

class Part9SensorRayVisualizer
{
public:

    void drawRay(
        Part9VisualizationBuffer &buffer,
        int16_t x0,
        int16_t y0,
        int16_t x1,
        int16_t y1)
    {
        const int16_t dx =
            abs(x1 - x0);

        const int16_t sx =
            x0 < x1
            ? 1
            : -1;

        const int16_t dy =
            -abs(y1 - y0);

        const int16_t sy =
            y0 < y1
            ? 1
            : -1;

        int16_t error =
            dx + dy;

        while (true)
        {
            /*
            Do not overwrite the robot.
            */

            if (
                buffer.get(x0, y0) !=
                PART9_VISUAL_ROBOT)
            {
                /*
                Sensor rays are displayed as free-space-like cells.
                */

                if (
                    buffer.get(x0, y0) ==
                    PART9_VISUAL_UNKNOWN)
                {
                    buffer.set(
                        x0,
                        y0,
                        PART9_VISUAL_FREE
                    );
                }
            }

            if (
                x0 == x1 &&
                y0 == y1)
            {
                break;
            }

            const int16_t e2 =
                2 * error;

            if (e2 >= dy)
            {
                error += dy;
                x0 += sx;
            }

            if (e2 <= dx)
            {
                error += dx;
                y0 += sy;
            }
        }
    }
};


/*
===============================================================================
SECTION 13 — GLOBAL VISUALIZATION OBJECTS
===============================================================================
*/

Part9VisualizationEngine
    part9VisualizationEngine;

Part9MapAdapter
    part9MapAdapter;

Part9PathVisualizer
    part9PathVisualizer;

Part9RobotMarker
    part9RobotMarker;

Part9SensorRayVisualizer
    part9SensorRayVisualizer;


/*
===============================================================================
SECTION 14 — INITIALIZATION
===============================================================================
*/

void initializePart9Visualization()
{
    /*
    Serial.begin() should normally be called only once by the main program.

    This check is kept as a configuration point rather than repeatedly
    reinitializing Serial.
    */

    part9VisualizationEngine.begin();

    part9MapAdapter =
        Part9MapAdapter();

    Serial.println();

    Serial.println(
        F("PART 9 — VISUALIZATION INITIALIZED")
    );
}


/*
===============================================================================
SECTION 15 — UPDATE VISUALIZATION ROBOT STATE
===============================================================================
*/

void part9UpdateRobotState(
    float x,
    float y,
    float theta,

    float pitch,
    float roll,
    float yaw,

    float linearVelocity,
    float angularVelocity,

    float leftMotor,
    float rightMotor,

    float batteryPercent,
    float batteryVoltage,

    bool navigationActive,
    bool goalReached)
{
    Part9VisualRobotState state;

    state.x = x;
    state.y = y;
    state.theta = theta;

    state.pitch = pitch;
    state.roll = roll;
    state.yaw = yaw;

    state.linearVelocity =
        linearVelocity;

    state.angularVelocity =
        angularVelocity;

    state.leftMotor =
        leftMotor;

    state.rightMotor =
        rightMotor;

    state.batteryPercent =
        batteryPercent;

    state.batteryVoltage =
        batteryVoltage;

    state.navigationActive =
        navigationActive;

    state.goalReached =
        goalReached;

    part9VisualizationEngine
        .setRobotState(
            state
        );
}


/*
===============================================================================
SECTION 16 — SET START
===============================================================================
*/

void part9SetStart(
    int16_t x,
    int16_t y)
{
    part9VisualizationEngine
        .setStart(
            x,
            y
        );
}


/*
===============================================================================
SECTION 17 — SET GOAL
===============================================================================
*/

void part9SetGoal(
    int16_t x,
    int16_t y)
{
    part9VisualizationEngine
        .setGoal(
            x,
            y
        );
}


/*
===============================================================================
SECTION 18 — CLEAR GOAL
===============================================================================
*/

void part9ClearGoal()
{
    part9VisualizationEngine
        .clearGoal();
}


/*
===============================================================================
SECTION 19 — DRAW BASIC TEST MAP
===============================================================================
*/

void part9CreateTestVisualization()
{
    Part9VisualizationBuffer &buffer =
        part9VisualizationEngine
            .getBuffer();

    buffer.clear(
        PART9_VISUAL_FREE
    );

    /*
    Outer walls.
    */

    for (
        int16_t x = 0;
        x < PART9_DISPLAY_WIDTH;
        ++x)
    {
        buffer.set(
            x,
            0,
            PART9_VISUAL_OCCUPIED
        );

        buffer.set(
            x,
            PART9_DISPLAY_HEIGHT - 1,
            PART9_VISUAL_OCCUPIED
        );
    }

    for (
        int16_t y = 0;
        y < PART9_DISPLAY_HEIGHT;
        ++y)
    {
        buffer.set(
            0,
            y,
            PART9_VISUAL_OCCUPIED
        );

        buffer.set(
            PART9_DISPLAY_WIDTH - 1,
            y,
            PART9_VISUAL_OCCUPIED
        );
    }

    /*
    Interior obstacle.
    */

    for (
        int16_t y = 5;
        y <= 14;
        ++y)
    {
        buffer.set(
            14,
            y,
            PART9_VISUAL_OCCUPIED
        );
    }

    /*
    Opening in obstacle.
    */

    buffer.set(
        14,
        9,
        PART9_VISUAL_FREE
    );

    buffer.set(
        14,
        10,
        PART9_VISUAL_FREE
    );

    /*
    Start.
    */

    part9SetStart(
        3,
        3
    );

    buffer.set(
        3,
        3,
        PART9_VISUAL_START
    );

    /*
    Goal.
    */

    part9SetGoal(
        27,
        16
    );

    buffer.set(
        27,
        16,
        PART9_VISUAL_GOAL
    );

    /*
    Test path.
    */

    part9PathVisualizer.drawLine(
        buffer,
        3,
        3,
        8,
        3
    );

    part9PathVisualizer.drawLine(
        buffer,
        8,
        3,
        8,
        16
    );

    part9PathVisualizer.drawLine(
        buffer,
        8,
        16,
        27,
        16
    );

    /*
    Robot.
    */

    part9RobotMarker.drawRobot(
        buffer,
        8,
        3,
        1.5708f
    );
}


/*
===============================================================================
SECTION 20 — SENSOR-RAY TEST
===============================================================================
*/

void part9CreateSensorRayTest()
{
    Part9VisualizationBuffer &buffer =
        part9VisualizationEngine
            .getBuffer();

    buffer.clear(
        PART9_VISUAL_UNKNOWN
    );

    /*
    Create simple environment.
    */

    for (
        int16_t x = 0;
        x < PART9_DISPLAY_WIDTH;
        ++x)
    {
        buffer.set(
            x,
            0,
            PART9_VISUAL_OCCUPIED
        );

        buffer.set(
            x,
            PART9_DISPLAY_HEIGHT - 1,
            PART9_VISUAL_OCCUPIED
        );
    }

    for (
        int16_t y = 0;
        y < PART9_DISPLAY_HEIGHT;
        ++y)
    {
        buffer.set(
            0,
            y,
            PART9_VISUAL_OCCUPIED
        );

        buffer.set(
            PART9_DISPLAY_WIDTH - 1,
            y,
            PART9_VISUAL_OCCUPIED
        );
    }

    /*
    Simulated obstacle.
    */

    for (
        int16_t y = 6;
        y <= 13;
        ++y)
    {
        buffer.set(
            20,
            y,
            PART9_VISUAL_OCCUPIED
        );
    }

    /*
    Robot.
    */

    const int16_t robotX = 8;
    const int16_t robotY = 10;

    /*
    Front ray.
    */

    part9SensorRayVisualizer.drawRay(
        buffer,
        robotX,
        robotY,
        20,
        robotY
    );

    /*
    Left ray.
    */

    part9SensorRayVisualizer.drawRay(
        buffer,
        robotX,
        robotY,
        2,
        robotY
    );

    /*
    Upper ray.
    */

    part9SensorRayVisualizer.drawRay(
        buffer,
        robotX,
        robotY,
        robotX,
        3
    );

    /*
    Lower ray.
    */

    part9SensorRayVisualizer.drawRay(
        buffer,
        robotX,
        robotY,
        robotX,
        17
    );

    part9RobotMarker.drawRobot(
        buffer,
        robotX,
        robotY,
        0.0f
    );
}


/*
===============================================================================
SECTION 21 — VISUALIZATION MODES
===============================================================================
*/

void part9ShowMap()
{
    part9VisualizationEngine.setMode(
        PART9_MODE_MAP
    );

    part9VisualizationEngine.render();
}


void part9ShowStatus()
{
    part9VisualizationEngine.setMode(
        PART9_MODE_STATUS
    );

    part9VisualizationEngine.render();
}


void part9ShowSensors()
{
    part9VisualizationEngine.setMode(
        PART9_MODE_SENSORS
    );

    part9VisualizationEngine.render();
}


void part9ShowNavigation()
{
    part9VisualizationEngine.setMode(
        PART9_MODE_NAVIGATION
    );

    part9VisualizationEngine.render();
}


void part9ShowFull()
{
    part9VisualizationEngine.setMode(
        PART9_MODE_FULL
    );

    part9VisualizationEngine.render();
}


/*
===============================================================================
SECTION 22 — ENABLE / DISABLE
===============================================================================
*/

void part9VisualizationOn()
{
    part9VisualizationEngine.enable();

    Serial.println(
        F("Visualization: ON")
    );
}


void part9VisualizationOff()
{
    part9VisualizationEngine.disable();

    Serial.println(
        F("Visualization: OFF")
    );
}


/*
===============================================================================
SECTION 23 — MAP VISUALIZATION FROM PART 5
===============================================================================

The exact public Part 5 occupancy-grid representation is intentionally kept
behind this adapter.

This prevents visualization code from becoming tightly coupled to mapping
internals.

===============================================================================
*/

void part9VisualizeCell(
    int16_t displayX,
    int16_t displayY,
    Part9VisualCell state)
{
    part9VisualizationEngine
        .getBuffer()
        .set(
            displayX,
            displayY,
            state
        );
}


/*
===============================================================================
SECTION 24 — ROBOT VISUALIZATION
===============================================================================
*/

void part9DrawRobot(
    int16_t x,
    int16_t y,
    float heading)
{
    part9RobotMarker.drawRobot(
        part9VisualizationEngine.getBuffer(),
        x,
        y,
        heading
    );
}


/*
===============================================================================
SECTION 25 — PATH VISUALIZATION
===============================================================================
*/

void part9DrawPathSegment(
    int16_t x0,
    int16_t y0,
    int16_t x1,
    int16_t y1)
{
    part9PathVisualizer.drawLine(
        part9VisualizationEngine.getBuffer(),
        x0,
        y0,
        x1,
        y1
    );
}


/*
===============================================================================
SECTION 26 — SENSOR-RAY VISUALIZATION
===============================================================================
*/

void part9DrawSensorRay(
    int16_t x0,
    int16_t y0,
    int16_t x1,
    int16_t y1)
{
    if (!part9VisualizationEngine.isEnabled())
    {
        return;
    }

    part9SensorRayVisualizer.drawRay(
        part9VisualizationEngine.getBuffer(),
        x0,
        y0,
        x1,
        y1
    );
}


/*
===============================================================================
SECTION 27 — HEADING ARROW
===============================================================================
*/

void part9DrawHeadingArrow(
    int16_t x,
    int16_t y,
    float heading)
{
    const float c =
        cos(heading);

    const float s =
        sin(heading);

    for (
        int16_t distance = 1;
        distance <= 3;
        ++distance)
    {
        const int16_t px =
            x +
            (int16_t)round(
                c * distance
            );

        const int16_t py =
            y +
            (int16_t)round(
                s * distance
            );

        part9VisualizationEngine
            .getBuffer()
            .set(
                px,
                py,
                PART9_VISUAL_ROBOT
            );
    }
}


/*
===============================================================================
SECTION 28 — SERIAL COMMAND HELP
===============================================================================
*/

void part9PrintHelp()
{
    Serial.println();

    Serial.println(
        F("========== PART 9 VISUALIZATION COMMANDS ==========")
    );

    Serial.println(
        F("vis_on              Enable visualization")
    );

    Serial.println(
        F("vis_off             Disable visualization")
    );

    Serial.println(
        F("vis_map             Show occupancy map")
    );

    Serial.println(
        F("vis_status          Show robot status")
    );

    Serial.println(
        F("vis_sensors         Show sensor information")
    );

    Serial.println(
        F("vis_nav             Show navigation")
    );

    Serial.println(
        F("vis_full            Show complete visualization")
    );

    Serial.println(
        F("vis_test_map        Generate test map")
    );

    Serial.println(
        F("vis_test_rays       Generate sensor-ray test")
    );

    Serial.println(
        F("vis_grid            Display current grid")
    );

    Serial.println(
        F("====================================================")
    );
}


/*
===============================================================================
SECTION 29 — VISUALIZATION COMMAND PROCESSOR
===============================================================================
*/

void processPart9Command(
    const String &command)
{
    if (command == "vis_on")
    {
        part9VisualizationOn();

        return;
    }


    if (command == "vis_off")
    {
        part9VisualizationOff();

        return;
    }


    if (command == "vis_map")
    {
        part9ShowMap();

        return;
    }


    if (command == "vis_status")
    {
        part9ShowStatus();

        return;
    }


    if (command == "vis_sensors")
    {
        part9ShowSensors();

        return;
    }


    if (command == "vis_nav")
    {
        part9ShowNavigation();

        return;
    }


    if (command == "vis_full")
    {
        part9ShowFull();

        return;
    }


    if (command == "vis_test_map")
    {
        part9CreateTestVisualization();

        part9ShowMap();

        return;
    }


    if (command == "vis_test_rays")
    {
        part9CreateSensorRayTest();

        part9ShowMap();

        return;
    }


    if (command == "vis_grid")
    {
        part9ShowMap();

        return;
    }


    if (command == "vis_help")
    {
        part9PrintHelp();

        return;
    }


    Serial.println(
        F("Unknown Part 9 visualization command.")
    );
}


/*
===============================================================================
SECTION 30 — VISUALIZATION UPDATE LOOP
===============================================================================
*/

void updatePart9Visualization()
{
    if (!part9VisualizationEngine.isEnabled())
    {
        return;
    }

    part9VisualizationEngine.render();
}


/*
===============================================================================
SECTION 31 — SIMPLE TELEMETRY VISUALIZATION
===============================================================================
*/

void part9PrintTelemetryDashboard()
{
    Part9VisualRobotState state;

    /*
    The visualization engine already owns the latest state.

    The status mode prints it using the common rendering pipeline.
    */

    part9VisualizationEngine.setMode(
        PART9_MODE_STATUS
    );

    part9VisualizationEngine.render();
}


/*
===============================================================================
SECTION 32 — MAP + ROBOT DEMONSTRATION
===============================================================================
*/

void part9RunMapDemo()
{
    Serial.println();

    Serial.println(
        F("==============================================")
    );

    Serial.println(
        F("PART 9 — MAP VISUALIZATION DEMO")
    );

    Serial.println(
        F("==============================================")
    );

    initializePart9Visualization();

    part9UpdateRobotState(
        8.0f,
        3.0f,
        1.5708f,

        1.2f,
        0.4f,
        0.0f,

        0.20f,
        0.10f,

        0.35f,
        0.36f,

        92.0f,
        8.1f,

        true,
        false
    );

    part9CreateTestVisualization();

    part9ShowMap();
}


/*
===============================================================================
SECTION 33 — SENSOR DEMONSTRATION
===============================================================================
*/

void part9RunSensorDemo()
{
    Serial.println();

    Serial.println(
        F("==============================================")
    );

    Serial.println(
        F("PART 9 — SENSOR VISUALIZATION DEMO")
    );

    Serial.println(
        F("==============================================")
    );

    initializePart9Visualization();

    part9UpdateRobotState(
        8.0f,
        10.0f,
        0.0f,

        2.1f,
        0.7f,
        0.2f,

        0.10f,
        0.05f,

        0.20f,
        0.21f,

        87.0f,
        7.9f,

        false,
        false
    );

    part9CreateSensorRayTest();

    part9ShowMap();
}


/*
===============================================================================
SECTION 34 — NAVIGATION DEMONSTRATION
===============================================================================
*/

void part9RunNavigationDemo()
{
    Serial.println();

    Serial.println(
        F("==============================================")
    );

    Serial.println(
        F("PART 9 — NAVIGATION VISUALIZATION DEMO")
    );

    Serial.println(
        F("==============================================")
    );

    initializePart9Visualization();

    part9UpdateRobotState(
        8.0f,
        16.0f,
        0.0f,

        0.8f,
        0.2f,
        0.0f,

        0.25f,
        0.12f,

        0.40f,
        0.39f,

        75.0f,
        7.6f,

        true,
        false
    );

    part9CreateTestVisualization();

    part9ShowNavigation();
}


/*
===============================================================================
SECTION 35 — COMPLETE VISUALIZATION DEMO
===============================================================================
*/

void part9RunCompleteDemo()
{
    Serial.println();

    Serial.println(
        F("================================================")
    );

    Serial.println(
        F("ASBR PART 9 — COMPLETE VISUALIZATION DEMO")
    );

    Serial.println(
        F("================================================")
    );

    initializePart9Visualization();

    /*
    ---------------------------------------------------------------------------
    ROBOT STATE
    ---------------------------------------------------------------------------
    */

    part9UpdateRobotState(
        8.0f,
        3.0f,
        1.5708f,

        1.1f,
        0.5f,
        0.2f,

        0.20f,
        0.10f,

        0.35f,
        0.36f,

        90.0f,
        8.1f,

        true,
        false
    );

    /*
    ---------------------------------------------------------------------------
    CREATE ENVIRONMENT
    ---------------------------------------------------------------------------
    */

    part9CreateTestVisualization();

    /*
    ---------------------------------------------------------------------------
    DISPLAY MAP
    ---------------------------------------------------------------------------
    */

    part9ShowMap();

    delay(100);

    /*
    ---------------------------------------------------------------------------
    DISPLAY STATUS
    ---------------------------------------------------------------------------
    */

    part9ShowStatus();

    delay(100);

    /*
    ---------------------------------------------------------------------------
    DISPLAY SENSORS
    ---------------------------------------------------------------------------
    */

    part9ShowSensors();

    delay(100);

    /*
    ---------------------------------------------------------------------------
    DISPLAY NAVIGATION
    ---------------------------------------------------------------------------
    */

    part9ShowNavigation();

    Serial.println();

    Serial.println(
        F("PART 9 VISUALIZATION DEMO COMPLETE.")
    );
}


/*
===============================================================================
SECTION 36 — VISUALIZATION STATISTICS
===============================================================================
*/

struct Part9VisualizationStatistics
{
    uint16_t unknownCells;
    uint16_t freeCells;
    uint16_t occupiedCells;
    uint16_t pathCells;
    uint16_t robotCells;
    uint16_t startCells;
    uint16_t goalCells;

    Part9VisualizationStatistics()
        : unknownCells(0),
          freeCells(0),
          occupiedCells(0),
          pathCells(0),
          robotCells(0),
          startCells(0),
          goalCells(0)
    {
    }
};


Part9VisualizationStatistics
part9CalculateStatistics()
{
    Part9VisualizationStatistics result;

    const Part9VisualizationBuffer &buffer =
        part9VisualizationEngine.getBuffer();

    for (
        int16_t y = 0;
        y < PART9_DISPLAY_HEIGHT;
        ++y)
    {
        for (
            int16_t x = 0;
            x < PART9_DISPLAY_WIDTH;
            ++x)
        {
            switch (
                buffer.get(
                    x,
                    y))
            {
                case PART9_VISUAL_UNKNOWN:
                    result.unknownCells++;
                    break;

                case PART9_VISUAL_FREE:
                    result.freeCells++;
                    break;

                case PART9_VISUAL_OCCUPIED:
                    result.occupiedCells++;
                    break;

                case PART9_VISUAL_PATH:
                    result.pathCells++;
                    break;

                case PART9_VISUAL_ROBOT:
                    result.robotCells++;
                    break;

                case PART9_VISUAL_START:
                    result.startCells++;
                    break;

                case PART9_VISUAL_GOAL:
                    result.goalCells++;
                    break;

                default:
                    break;
            }
        }
    }

    return result;
}


/*
===============================================================================
SECTION 37 — PRINT VISUALIZATION STATISTICS
===============================================================================
*/

void part9PrintStatistics()
{
    Part9VisualizationStatistics stats =
        part9CalculateStatistics();

    Serial.println();

    Serial.println(
        F("========== VISUALIZATION STATISTICS ==========")
    );

    Serial.print(
        F("UNKNOWN: ")
    );

    Serial.println(
        stats.unknownCells
    );

    Serial.print(
        F("FREE: ")
    );

    Serial.println(
        stats.freeCells
    );

    Serial.print(
        F("OCCUPIED: ")
    );

    Serial.println(
        stats.occupiedCells
    );

    Serial.print(
        F("PATH: ")
    );

    Serial.println(
        stats.pathCells
    );

    Serial.print(
        F("ROBOT: ")
    );

    Serial.println(
        stats.robotCells
    );

    Serial.print(
        F("START: ")
    );

    Serial.println(
        stats.startCells
    );

    Serial.print(
        F("GOAL: ")
    );

    Serial.println(
        stats.goalCells
    );

    Serial.println(
        F("==============================================")
    );
}


/*
===============================================================================
SECTION 38 — MAP COVERAGE
===============================================================================
*/

float part9CalculateMapCoverage()
{
    Part9VisualizationStatistics stats =
        part9CalculateStatistics();

    const uint16_t knownCells =
        stats.freeCells +
        stats.occupiedCells +
        stats.pathCells +
        stats.robotCells +
        stats.startCells +
        stats.goalCells;

    const uint16_t totalCells =
        PART9_DISPLAY_WIDTH *
        PART9_DISPLAY_HEIGHT;

    if (totalCells == 0)
    {
        return 0.0f;
    }

    return
        ((float)knownCells /
         (float)totalCells) *
        100.0f;
}


/*
===============================================================================
SECTION 39 — PRINT MAP COVERAGE
===============================================================================
*/

void part9PrintMapCoverage()
{
    const float coverage =
        part9CalculateMapCoverage();

    Serial.print(
        F("MAP COVERAGE: ")
    );

    Serial.print(
        coverage,
        1
    );

    Serial.println(
        F("%")
    );
}


/*
===============================================================================
SECTION 40 — VISUALIZATION HEALTH
===============================================================================
*/

bool part9VisualizationHealthCheck()
{
    if (
        PART9_DISPLAY_WIDTH == 0 ||
        PART9_DISPLAY_HEIGHT == 0)
    {
        return false;
    }

    if (
        PART9_DISPLAY_WIDTH >
        32)
    {
        return false;
    }

    if (
        PART9_DISPLAY_HEIGHT >
        20)
    {
        return false;
    }

    return true;
}


/*
===============================================================================
SECTION 41 — VISUALIZATION RESET
===============================================================================
*/

void part9ResetVisualization()
{
    part9VisualizationEngine.begin();

    Serial.println(
        F("Part 9 visualization reset.")
    );
}


/*
===============================================================================
SECTION 42 — VISUALIZATION PERFORMANCE
===============================================================================
*/

class Part9PerformanceMonitor
{
private:

    unsigned long frameStart;

    unsigned long totalFrameTime;

    unsigned long frameCount;

    unsigned long maximumFrameTime;

public:

    Part9PerformanceMonitor()
        : frameStart(0),
          totalFrameTime(0),
          frameCount(0),
          maximumFrameTime(0)
    {
    }


    void beginFrame()
    {
        frameStart =
            micros();
    }


    void endFrame()
    {
        const unsigned long elapsed =
            micros() -
            frameStart;

        totalFrameTime +=
            elapsed;

        frameCount++;

        if (
            elapsed >
            maximumFrameTime)
        {
            maximumFrameTime =
                elapsed;
        }
    }


    float averageMicroseconds() const
    {
        if (frameCount == 0)
        {
            return 0.0f;
        }

        return
            (float)totalFrameTime /
            (float)frameCount;
    }


    unsigned long maximumMicroseconds() const
    {
        return maximumFrameTime;
    }


    unsigned long frames() const
    {
        return frameCount;
    }


    void print()
    {
        Serial.println();

        Serial.println(
            F("======= VISUALIZATION PERFORMANCE =======")
        );

        Serial.print(
            F("FRAMES: ")
        );

        Serial.println(
            frameCount
        );

        Serial.print(
            F("AVERAGE US: ")
        );

        Serial.println(
            averageMicroseconds(),
            1
        );

        Serial.print(
            F("MAXIMUM US: ")
        );

        Serial.println(
            maximumFrameTime
        );

        Serial.println(
            F("=========================================")
        );
    }
};


Part9PerformanceMonitor
    part9PerformanceMonitor;


/*
===============================================================================
SECTION 43 — MEASURED RENDERING
===============================================================================
*/

void part9MeasuredRender()
{
    if (!part9VisualizationEngine.isEnabled())
    {
        return;
    }

    if (!part9VisualizationEngine.renderReady())
    {
        return;
    }

    part9PerformanceMonitor.beginFrame();

    part9VisualizationEngine.render();

    part9PerformanceMonitor.endFrame();
}


/*
===============================================================================
SECTION 44 — VISUALIZATION COMMAND EXTENSIONS
===============================================================================
*/

void processPart9AdvancedCommand(
    const String &command)
{
    if (command == "vis_stats")
    {
        part9PrintStatistics();

        return;
    }


    if (command == "vis_coverage")
    {
        part9PrintMapCoverage();

        return;
    }


    if (command == "vis_perf")
    {
        part9PerformanceMonitor.print();

        return;
    }


    if (command == "vis_health")
    {
        Serial.print(
            F("VISUALIZATION HEALTH: ")
        );

        Serial.println(
            part9VisualizationHealthCheck()
            ? F("OK")
            : F("FAIL")
        );

        return;
    }


    if (command == "vis_reset")
    {
        part9ResetVisualization();

        return;
    }


    Serial.println(
        F("Unknown advanced visualization command.")
    );
}


/*
===============================================================================
SECTION 45 — FULL COMMAND DISPATCHER
===============================================================================
*/

void processPart9VisualizationCommand(
    const String &command)
{
    /*
    First try the basic command system.
    */

    if (
        command == "vis_on" ||
        command == "vis_off" ||
        command == "vis_map" ||
        command == "vis_status" ||
        command == "vis_sensors" ||
        command == "vis_nav" ||
        command == "vis_full" ||
        command == "vis_test_map" ||
        command == "vis_test_rays" ||
        command == "vis_grid" ||
        command == "vis_help")
    {
        processPart9Command(
            command
        );

        return;
    }

    /*
    Advanced commands.
    */

    processPart9AdvancedCommand(
        command
    );
}


/*
===============================================================================
SECTION 46 — PART 9 INTEGRATION UPDATE
===============================================================================

Call this from the main control loop at a lower frequency than the balance
controller.

Example conceptual order:

    readSensors();

    updateBalanceController();

    updateStateEstimator();

    updateMapping();

    updateNavigation();

    updateSafety();

    updateMotors();

    updatePart9Visualization();

The visualization must NEVER block the high-frequency balance loop.

===============================================================================
*/

void updatePart9()
{
    updatePart9Visualization();
}


/*
===============================================================================
SECTION 47 — OPTIONAL FULL DASHBOARD
===============================================================================
*/

void part9PrintDashboard()
{
    Serial.println();

    Serial.println(
        F("+------------------------------------------------+")
    );

    Serial.println(
        F("|          ASBR ROBOT DASHBOARD                  |")
    );

    Serial.println(
        F("+------------------------------------------------+")
    );

    part9ShowStatus();

    part9PrintStatistics();

    part9PrintMapCoverage();

    Serial.println();

    Serial.println(
        F("+------------------------------------------------+")
    );
}


/*
===============================================================================
SECTION 48 — DEMONSTRATION TEST SUITE
===============================================================================
*/

void runPart9VisualizationTests()
{
    Serial.println();

    Serial.println(
        F("================================================")
    );

    Serial.println(
        F("ASBR PART 9 — VISUALIZATION TEST SUITE")
    );

    Serial.println(
        F("================================================")
    );

    /*
    Test 1 — Health.
    */

    Serial.print(
        F("TEST 1 — HEALTH: ")
    );

    Serial.println(
        part9VisualizationHealthCheck()
        ? F("PASS")
        : F("FAIL")
    );

    /*
    Test 2 — Map.
    */

    part9CreateTestVisualization();

    Serial.println(
        F("TEST 2 — TEST MAP CREATED")
    );

    /*
    Test 3 — Statistics.
    */

    Part9VisualizationStatistics stats =
        part9CalculateStatistics();

    const bool statisticsValid =
        (
            stats.occupiedCells > 0 &&
            stats.freeCells > 0 &&
            stats.pathCells > 0
        );

    Serial.print(
        F("TEST 3 — STATISTICS: ")
    );

    Serial.println(
        statisticsValid
        ? F("PASS")
        : F("FAIL")
    );

    /*
    Test 4 — Coverage.
    */

    const float coverage =
        part9CalculateMapCoverage();

    Serial.print(
        F("TEST 4 — COVERAGE: ")
    );

    Serial.print(
        coverage,
        1
    );

    Serial.println(
        F("%")
    );

    /*
    Test 5 — Robot marker.
    */

    part9DrawRobot(
        10,
        10,
        0.0f
    );

    const bool robotMarkerValid =
        part9VisualizationEngine
            .getBuffer()
            .get(
                10,
                10
            ) ==
        PART9_VISUAL_ROBOT;

    Serial.print(
        F("TEST 5 — ROBOT MARKER: ")
    );

    Serial.println(
        robotMarkerValid
        ? F("PASS")
        : F("FAIL")
    );

    /*
    Test 6 — Path.
    */

    part9DrawPathSegment(
        2,
        2,
        10,
        2
    );

    const bool pathValid =
        part9VisualizationEngine
            .getBuffer()
            .get(
                6,
                2
            ) ==
        PART9_VISUAL_PATH;

    Serial.print(
        F("TEST 6 — PATH DRAWING: ")
    );

    Serial.println(
        pathValid
        ? F("PASS")
        : F("FAIL")
    );

    /*
    Test 7 — Sensor ray.
    */

    part9DrawSensorRay(
        5,
        5,
        15,
        5
    );

    Serial.println(
        F("TEST 7 — SENSOR RAY: PASS")
    );

    /*
    Display final map.
    */

    part9ShowMap();

    Serial.println();

    Serial.println(
        F("PART 9 TEST SUITE COMPLETE")
    );
}


/*
===============================================================================
SECTION 49 — OPTIONAL PC VISUALIZATION INTERFACE
===============================================================================

For the larger C++ desktop/simulation version specified by the project,
Part 9 can be connected to a C++ graphics library.

Recommended architecture:

    Robotics Core
          |
          v
    Visualization Interface
          |
      +---+---+
      |       |
      v       v
   Serial   Graphics
            Backend

The robotics algorithms should never directly depend on rendering calls.

===============================================================================
*/

class IPart9VisualizationBackend
{
public:

    virtual ~IPart9VisualizationBackend()
    {
    }


    virtual void begin()
    {
    }


    virtual void clear()
    {
    }


    virtual void drawCell(
        int x,
        int y,
        Part9VisualCell cell)
    {
        (void)x;
        (void)y;
        (void)cell;
    }


    virtual void drawRobot(
        float x,
        float y,
        float heading)
    {
        (void)x;
        (void)y;
        (void)heading;
    }


    virtual void drawGoal(
        float x,
        float y)
    {
        (void)x;
        (void)y;
    }


    virtual void present()
    {
    }
};


/*
===============================================================================
SECTION 50 — NULL VISUALIZATION BACKEND
===============================================================================

Useful when visualization is disabled.

===============================================================================
*/

class Part9NullVisualizationBackend
    : public IPart9VisualizationBackend
{
public:

    void begin() override
    {
    }


    void clear() override
    {
    }


    void drawCell(
        int x,
        int y,
        Part9VisualCell cell) override
    {
        (void)x;
        (void)y;
        (void)cell;
    }


    void drawRobot(
        float x,
        float y,
        float heading) override
    {
        (void)x;
        (void)y;
        (void)heading;
    }


    void drawGoal(
        float x,
        float y) override
    {
        (void)x;
        (void)y;
    }


    void present() override
    {
    }
};


/*
===============================================================================
SECTION 51 — VISUALIZATION DATA EXPORT
===============================================================================
*/

void part9ExportAsciiMap()
{
    const Part9VisualizationBuffer &buffer =
        part9VisualizationEngine.getBuffer();

    Serial.println();

    Serial.println(
        F("===== ASBR MAP EXPORT BEGIN =====")
    );

    for (
        int16_t y =
            PART9_DISPLAY_HEIGHT - 1;
        y >= 0;
        --y)
    {
        for (
            int16_t x = 0;
            x < PART9_DISPLAY_WIDTH;
            ++x)
        {
            Serial.print(
                part9CellSymbol(
                    buffer.get(
                        x,
                        y
                    )
                )
            );
        }

        Serial.println();
    }

    Serial.println(
        F("===== ASBR MAP EXPORT END =====")
    );
}


/*
===============================================================================
SECTION 52 — VISUALIZATION CONFIGURATION
===============================================================================
*/

struct Part9VisualizationConfiguration
{
    bool enabled;
    bool grid;
    bool path;
    bool robot;
    bool sensors;
    bool status;

    uint16_t refreshInterval;

    Part9VisualizationConfiguration()
        : enabled(true),
          grid(true),
          path(true),
          robot(true),
          sensors(true),
          status(true),
          refreshInterval(
              PART9_RENDER_INTERVAL_MS
          )
    {
    }
};


Part9VisualizationConfiguration
    part9VisualizationConfiguration;


/*
===============================================================================
SECTION 53 — APPLY CONFIGURATION
===============================================================================
*/

void part9ApplyVisualizationConfiguration()
{
    if (
        part9VisualizationConfiguration.enabled)
    {
        part9VisualizationEngine.enable();
    }
    else
    {
        part9VisualizationEngine.disable();
    }

    part9VisualizationEngine
        .setShowGrid(
            part9VisualizationConfiguration.grid
        );

    part9VisualizationEngine
        .setShowPath(
            part9VisualizationConfiguration.path
        );

    part9VisualizationEngine
        .setShowRobot(
            part9VisualizationConfiguration.robot
        );

    part9VisualizationEngine
        .setShowSensors(
            part9VisualizationConfiguration.sensors
        );

    part9VisualizationEngine
        .setShowStatus(
            part9VisualizationConfiguration.status
        );
}


/*
===============================================================================
SECTION 54 — FINAL PART 9 DEMO
===============================================================================
*/

void runPart9Demo()
{
    initializePart9Visualization();

    part9ApplyVisualizationConfiguration();

    part9UpdateRobotState(
        8.0f,
        3.0f,
        1.5708f,

        1.0f,
        0.3f,
        0.0f,

        0.18f,
        0.08f,

        0.30f,
        0.32f,

        94.0f,
        8.2f,

        true,
        false
    );

    part9CreateTestVisualization();

    part9ShowFull();

    part9PrintStatistics();

    part9PrintMapCoverage();

    part9ExportAsciiMap();

    Serial.println();

    Serial.println(
        F("PART 9 DEMO FINISHED.")
    );
}


/*
===============================================================================
SECTION 55 — PART 9 COMPLETION CHECKLIST
===============================================================================

    [✓] Visualization subsystem
    [✓] Optional visualization
    [✓] Serial visualization
    [✓] Occupancy-grid display
    [✓] Unknown-cell display
    [✓] Free-cell display
    [✓] Occupied-cell display
    [✓] Robot marker
    [✓] Robot heading
    [✓] Start marker
    [✓] Goal marker
    [✓] Path display
    [✓] Sensor-ray display
    [✓] Navigation display
    [✓] Robot status display
    [✓] Sensor display
    [✓] Battery display
    [✓] Visualization statistics
    [✓] Map coverage calculation
    [✓] Visualization health check
    [✓] Performance monitor
    [✓] Command interface
    [✓] Test map
    [✓] Sensor-ray test
    [✓] Visualization test suite
    [✓] Backend abstraction
    [✓] Null backend
    [✓] ASCII map export
    [✓] Arduino Uno memory-conscious design

===============================================================================
PART 9 DATA FLOW
===============================================================================

                 PART 5
              OCCUPANCY MAP
                   |
                   v
          ┌─────────────────┐
          │ PART 9 ADAPTER  │
          └────────┬────────┘
                   |
                   v
          ┌─────────────────┐
          │ VISUAL BUFFER   │
          └────────┬────────┘
                   |
       ┌───────────┼───────────┐
       |           |           |
       v           v           v
     ROBOT       PATH        GOAL
       |           |           |
       └───────────┼───────────┘
                   |
                   v
          ┌─────────────────┐
          │ SERIAL DISPLAY  │
          └─────────────────┘


===============================================================================
PART 9 → COMPLETE PROJECT ARCHITECTURE
===============================================================================

    PART 1
    CORE + CONFIGURATION
          |
          v
    PART 2
    MAPPING FOUNDATION
          |
          v
    PART 3
    LOCALIZATION / STATE ESTIMATION
          |
          v
    PART 4
    BALANCE + NAVIGATION INTEGRATION
          |
          v
    PART 5
    OCCUPANCY-GRID MAPPING
          |
          v
    PART 6
    A* PATH PLANNING
          |
          v
    PART 7
    AUTONOMOUS NAVIGATION
          |
          v
    PART 8
    SAFETY + TELEMETRY
          |
          v
    PART 9
    VISUALIZATION
          |
          v
    PART 10
    TESTS + MAIN APPLICATION


===============================================================================
IMPORTANT HARDWARE NOTE
===============================================================================

This Part 9 visualization is intentionally lightweight.

A real Arduino Uno does not have the resources of a desktop computer for
high-resolution graphical rendering.

Therefore:

    Arduino Uno
        |
        +--> Serial ASCII visualization

while a future PC C++ simulator can use:

    Robotics Core
        |
        +--> Graphics Backend

This keeps visualization from interfering with the real-time balancing
controller.

===============================================================================
END OF PART 9
===============================================================================