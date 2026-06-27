/*
    LOW LEVEL CONTROLLER V2 — AUTODRIVE

    Board: Arduino Nano / Uno class AVR
    Motor: BTS7960, one traction motor
    Steering: Servo
    Sensor: Quadrature encoder + TF-Luna UART

    PC/Pi command packets:

    Packet V1 legacy, forward only:
        [0xAA, throttle, steer, crc]
        crc = 0xAA ^ throttle ^ steer

    Packet V2, forward/reverse:
        [0xAA, dir, throttle, steer, crc]
        crc = 0xAA ^ dir ^ throttle ^ steer

    dir:
        0 = stop
        1 = forward
        2 = reverse

    throttle: 0..255
    steer:    0..180, constrained mechanically below

    Telemetry text, every 100 ms:
        TEL,state=RUN,rpm=...,target=...,ramp=...,pwm=...,dist=...,tf=...,hb=...,thr=...,steer=...
*/

#include <Servo.h>
#include <SoftwareSerial.h>

// ============================================================
// PIN MAP
// ============================================================

#define ENC_A       2
#define ENC_B       3

#define RPWM        5
#define LPWM        6
#define REN         7
#define LEN         8

#define SERVO_PIN   9

#define TF_TX       10     // Nano TX -> TF-Luna RX, usually unused
#define TF_RX       11     // Nano RX <- TF-Luna TX
#define TF_SAFE     12     // Optional external emergency input

// ============================================================
// CONFIG
// ============================================================

const uint32_t SERIAL_BAUD = 115200;
const uint32_t TF_BAUD     = 9600;

const uint8_t  PACKET_HEADER = 0xAA;
const uint8_t  PACKET_V1_LEN = 4;
const uint8_t  PACKET_V2_LEN = 5;
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;

const uint8_t SERVO_CENTER = 90;
const uint8_t SERVO_MIN    = 50;
const uint8_t SERVO_MAX    = 130;

const float COUNTS_PER_REV = 1320.0f;
const float MAX_RPM        = 250.0f;

const float ACCEL_RPM_PER_SEC = 80.0f;
const float DECEL_RPM_PER_SEC = 180.0f;

// TF-Luna distance unit is cm in serial mode.
const uint16_t TF_WARN_CM  = 50;
const uint16_t TF_SLOW_CM  = 30;
const uint16_t TF_STOP_CM  = 10;
const uint16_t TF_CLEAR_CM = 35;
const uint16_t TF_MIN_AMP  = 100;
const uint32_t TF_TIMEOUT_MS = 300;
const bool USE_TF_SAFE_PIN = false;

const float PID_DT = 0.005f;

float KP = 0.02f;
float KI = 0.002f;
float KD = 0.00f;

const float FF_GAIN = 0.82f;

const float INTEGRAL_LIMIT = 80.0f;
const float PID_CORRECTION_LIMIT = 30.0f;

const int PWM_MIN_EFFECTIVE = 50;
const int PWM_MAX = 255;

// ============================================================
// OBJECTS
// ============================================================

Servo steering;
SoftwareSerial tfSerial(TF_RX, TF_TX);

// ============================================================
// STATE
// ============================================================

enum SystemState
{
    STATE_WAIT_HEARTBEAT,
    STATE_RUNNING,
    STATE_SAFETY_LIMIT,
    STATE_SAFETY_STOP,
    STATE_EMERGENCY_STOP
};

SystemState state = STATE_WAIT_HEARTBEAT;

volatile long encoderCount = 0;

uint8_t rxPacket[PACKET_V2_LEN];
uint8_t directionCommand = 0;   // 0 stop, 1 forward, 2 reverse
uint8_t appliedDirection = 0;   // direction actually applied to motor
uint8_t pendingDirection = 0;
uint8_t throttleCommand = 0;
uint8_t steerCommand = SERVO_CENTER;

float requestedRPM = 0.0f;    // command from Pi/PC
float safetyLimitRPM = MAX_RPM;
float targetRPM = 0.0f;       // min(requestedRPM, safetyLimitRPM)
float rampRPM = 0.0f;
float currentRPM = 0.0f;

int motorPWM = 0;

float pidIntegral = 0.0f;
float pidPreviousError = 0.0f;

uint16_t tfDistance = 0;
uint16_t tfAmp = 0;
bool tfValid = false;
uint32_t lastTFMs = 0;

uint32_t lastHeartbeatMs = 0;
uint32_t lastTelemetryMs = 0;

// ============================================================
// HELPERS
// ============================================================

const char* stateName(SystemState s)
{
    switch (s)
    {
        case STATE_WAIT_HEARTBEAT: return "WAIT_HB";
        case STATE_RUNNING:        return "RUN";
        case STATE_SAFETY_LIMIT:   return "LIMIT";
        case STATE_SAFETY_STOP:    return "SAFE_STOP";
        case STATE_EMERGENCY_STOP: return "ESTOP";
        default:                   return "UNKNOWN";
    }
}

uint8_t crcN(const uint8_t* data, uint8_t lenWithoutCrc)
{
    uint8_t c = 0;
    for (uint8_t i = 0; i < lenWithoutCrc; i++)
    {
        c ^= data[i];
    }
    return c;
}

const char* directionName(uint8_t d)
{
    if (d == 1) return "FWD";
    if (d == 2) return "REV";
    return "STOP";
}

bool heartbeatOK()
{
    return (millis() - lastHeartbeatMs) <= HEARTBEAT_TIMEOUT_MS;
}

bool tfFresh()
{
    return tfValid && ((millis() - lastTFMs) <= TF_TIMEOUT_MS);
}

bool tfUsable()
{
    return tfFresh() && (tfAmp >= TF_MIN_AMP);
}

void resetPID()
{
    pidIntegral = 0.0f;
    pidPreviousError = 0.0f;
}

void hardMotorStop()
{
    motorPWM = 0;
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
}

void setState(SystemState next)
{
    if (state == next) return;

    state = next;

    if (state == STATE_WAIT_HEARTBEAT ||
        state == STATE_SAFETY_STOP ||
        state == STATE_EMERGENCY_STOP)
    {
        requestedRPM = 0.0f;
        targetRPM = 0.0f;
    }

    if (state == STATE_EMERGENCY_STOP)
    {
        rampRPM = 0.0f;
        resetPID();
        hardMotorStop();
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);

    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);

    pinMode(TF_SAFE, INPUT);      // change to INPUT_PULLUP if your safe line is open-drain

    digitalWrite(REN, HIGH);
    digitalWrite(LEN, HIGH);
    hardMotorStop();

    steering.attach(SERVO_PIN);
    steering.write(SERVO_CENTER);

    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR_B, CHANGE);

    Serial.begin(SERIAL_BAUD);
    tfSerial.begin(TF_BAUD);

    lastHeartbeatMs = 0;
    setState(STATE_WAIT_HEARTBEAT);
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    readCommandPacket();
    readTFLuna();

    static uint32_t last5 = 0;
    static uint32_t last20 = 0;
    static uint32_t last100 = 0;

    uint32_t now = millis();

    if (now - last5 >= 5)
    {
        last5 = now;
        updateSafetyAndState();
        updateTargetRPM();
        updateRamp();
        updatePID();
        driveMotor();
        updateServo();
    }

    if (now - last20 >= 20)
    {
        last20 = now;
        // reserved for faster telemetry/control later
    }

    if (now - last100 >= 100)
    {
        last100 = now;
        updateRPM();
        sendTelemetry();
    }
}

// ============================================================
// UART COMMAND
// ============================================================

void applyCommandV1()
{
    // Legacy packet:
    // [0xAA, throttle, steer, crc]
    directionCommand = (rxPacket[1] > 0) ? 1 : 0;
    throttleCommand = rxPacket[1];
    steerCommand = constrain(rxPacket[2], SERVO_MIN, SERVO_MAX);
    requestedRPM = ((float)throttleCommand * MAX_RPM) / 255.0f;
    lastHeartbeatMs = millis();

    if (state == STATE_WAIT_HEARTBEAT)
    {
        setState(STATE_RUNNING);
    }
}

void applyCommandV2()
{
    // New packet:
    // [0xAA, dir, throttle, steer, crc]
    directionCommand = rxPacket[1];
    throttleCommand = rxPacket[2];
    steerCommand = constrain(rxPacket[3], SERVO_MIN, SERVO_MAX);

    if (directionCommand > 2)
    {
        directionCommand = 0;
    }

    if (directionCommand == 0)
    {
        throttleCommand = 0;
    }

    requestedRPM = ((float)throttleCommand * MAX_RPM) / 255.0f;
    lastHeartbeatMs = millis();

    if (state == STATE_WAIT_HEARTBEAT)
    {
        setState(STATE_RUNNING);
    }
}

void readCommandPacket()
{
    static uint8_t index = 0;

    while (Serial.available())
    {
        uint8_t b = Serial.read();

        if (index == 0 && b != PACKET_HEADER)
        {
            continue;
        }

        rxPacket[index++] = b;

        // Try V1 at byte 4. This keeps old c.py working.
        // Important: V2 can also reach index 4 here, but its crc will not match,
        // so we keep waiting for byte 5.
        if (index == PACKET_V1_LEN)
        {
            if (crcN(rxPacket, 3) == rxPacket[3])
            {
                applyCommandV1();
                index = 0;
                return;
            }
        }

        // Try V2 at byte 5.
        if (index >= PACKET_V2_LEN)
        {
            if (crcN(rxPacket, 4) == rxPacket[4])
            {
                applyCommandV2();
            }

            // Whether valid or invalid, resync from next header.
            index = 0;
            return;
        }
    }
}

// ============================================================
// TF-LUNA
// ============================================================

void readTFLuna()
{
    static uint8_t frame[9];

    while (tfSerial.available() >= 9)
    {
        if (tfSerial.read() != 0x59) continue;
        if (tfSerial.peek() != 0x59) continue;

        frame[0] = 0x59;
        frame[1] = tfSerial.read();

        for (uint8_t i = 2; i < 9; i++)
        {
            frame[i] = tfSerial.read();
        }

        uint8_t sum = 0;
        for (uint8_t i = 0; i < 8; i++) sum += frame[i];

        if (sum != frame[8])
        {
            continue;
        }

        tfDistance = frame[2] | (frame[3] << 8);
        tfAmp = frame[4] | (frame[5] << 8);
        tfValid = true;
        lastTFMs = millis();
    }
}

// ============================================================
// SAFETY / STATE
// ============================================================

bool externalEmergencyActive()
{
    if (!USE_TF_SAFE_PIN) return false;
    return digitalRead(TF_SAFE) == HIGH;
}

void updateSafetyAndState()
{
    if (externalEmergencyActive())
    {
        setState(STATE_EMERGENCY_STOP);
        return;
    }

    if (!heartbeatOK())
    {
        setState(STATE_WAIT_HEARTBEAT);
        return;
    }

    if (tfUsable() && tfDistance <= TF_STOP_CM)
    {
        setState(STATE_SAFETY_STOP);
        return;
    }

    if (state == STATE_SAFETY_STOP)
    {
        if (!tfUsable() || tfDistance >= TF_CLEAR_CM)
        {
            setState(STATE_RUNNING);
        }
        return;
    }

    if (tfUsable() && tfDistance < TF_WARN_CM)
    {
        setState(STATE_SAFETY_LIMIT);
        return;
    }

    if (state == STATE_SAFETY_LIMIT)
    {
        setState(STATE_RUNNING);
        return;
    }

    if (state == STATE_EMERGENCY_STOP)
    {
        setState(STATE_WAIT_HEARTBEAT);
    }
}

void updateTargetRPM()
{
    safetyLimitRPM = MAX_RPM;

    if (state == STATE_WAIT_HEARTBEAT ||
        state == STATE_SAFETY_STOP ||
        state == STATE_EMERGENCY_STOP)
    {
        safetyLimitRPM = 0.0f;
    }
    else if (tfUsable())
    {
        if (tfDistance < TF_SLOW_CM)
        {
            safetyLimitRPM = 40.0f;
        }
        else if (tfDistance < TF_WARN_CM)
        {
            safetyLimitRPM = 80.0f;
        }
    }

    targetRPM = requestedRPM;

    // Direction interlock: before reversing, force ramp down to zero.
    if (appliedDirection != 0 &&
        directionCommand != 0 &&
        directionCommand != appliedDirection)
    {
        targetRPM = 0.0f;
    }

    if (targetRPM > safetyLimitRPM) targetRPM = safetyLimitRPM;
    if (targetRPM < 0.0f) targetRPM = 0.0f;
}

// ============================================================
// RAMP / PID / MOTOR
// ============================================================

void updateRamp()
{
    float accelStep = ACCEL_RPM_PER_SEC * PID_DT;
    float decelStep = DECEL_RPM_PER_SEC * PID_DT;

    if (rampRPM < targetRPM)
    {
        rampRPM += accelStep;
        if (rampRPM > targetRPM) rampRPM = targetRPM;
    }
    else if (rampRPM > targetRPM)
    {
        rampRPM -= decelStep;
        if (rampRPM < targetRPM) rampRPM = targetRPM;
    }
}

void updatePID()
{
    if (rampRPM <= 1.0f)
    {
        motorPWM = 0;
        pidIntegral = 0.0f;
        pidPreviousError = 0.0f;
        motorPWM = 0;
        resetPID();
        return;
    }

    float error = rampRPM - currentRPM;

    pidIntegral += error * PID_DT;
    pidIntegral = constrain(
        pidIntegral,
        -INTEGRAL_LIMIT,
        INTEGRAL_LIMIT
    );

    float correction =
        KP * error +
        KI * pidIntegral;

    correction = constrain(
        correction,
        -PID_CORRECTION_LIMIT,
        PID_CORRECTION_LIMIT
    );

    int basePWM =
        (int)(rampRPM * FF_GAIN);

    motorPWM =
        basePWM +
        (int)correction;

    motorPWM = constrain(
        motorPWM,
        PWM_MIN_EFFECTIVE,
        255
    );

    pidPreviousError = error;
}

void updateDirectionInterlock()
{
    if (targetRPM <= 1.0f || throttleCommand == 0 || directionCommand == 0)
    {
        pendingDirection = directionCommand;

        if (rampRPM <= 1.0f && motorPWM <= 5)
        {
            appliedDirection = directionCommand;
        }

        return;
    }

    if (appliedDirection == 0)
    {
        appliedDirection = directionCommand;
        pendingDirection = directionCommand;
        return;
    }

    if (directionCommand != appliedDirection)
    {
        pendingDirection = directionCommand;
        targetRPM = 0.0f;

        if (rampRPM <= 1.0f && motorPWM <= 5)
        {
            appliedDirection = pendingDirection;
        }
    }
}

void driveMotor()
{
    if (state == STATE_EMERGENCY_STOP)
    {
        hardMotorStop();
        return;
    }

    updateDirectionInterlock();

    motorPWM = constrain(motorPWM, 0, 255);

    if (appliedDirection == 1)
    {
        analogWrite(RPWM, motorPWM);
        analogWrite(LPWM, 0);
    }
    else if (appliedDirection == 2)
    {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, motorPWM);
    }
    else
    {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, 0);
    }
}

void updateServo()
{
    steering.write(constrain(steerCommand, SERVO_MIN, SERVO_MAX));
}

// ============================================================
// ENCODER / RPM
// ============================================================

void encoderISR_A()
{
    bool a = digitalRead(ENC_A);
    bool b = digitalRead(ENC_B);

    if (a == b) encoderCount++;
    else encoderCount--;
}

void encoderISR_B()
{
    bool a = digitalRead(ENC_A);
    bool b = digitalRead(ENC_B);

    if (a != b) encoderCount++;
    else encoderCount--;
}

long getEncoderCountAtomic()
{
    noInterrupts();
    long c = encoderCount;
    interrupts();
    return c;
}

void updateRPM()
{
    static long lastCount = 0;
    static uint32_t lastMs = 0;

    uint32_t now = millis();

    if (lastMs == 0)
    {
        lastMs = now;
        lastCount = getEncoderCountAtomic();
        return;
    }

    uint32_t dt = now - lastMs;
    if (dt == 0) return;

    long count = getEncoderCountAtomic();
    long delta = count - lastCount;

    currentRPM = ((float)delta * 60000.0f) / (COUNTS_PER_REV * (float)dt);
    if (currentRPM < 0.0f) currentRPM = -currentRPM;

    lastCount = count;
    lastMs = now;
}

// ============================================================
// TELEMETRY
// ============================================================

void sendTelemetry()
{
    Serial.print("TEL,state=");
    Serial.print(stateName(state));

    Serial.print(",rpm=");
    Serial.print(currentRPM, 1);

    Serial.print(",target=");
    Serial.print(targetRPM, 1);

    Serial.print(",ramp=");
    Serial.print(rampRPM, 1);

    Serial.print(",pwm=");
    Serial.print(motorPWM);

    Serial.print(",dist=");
    Serial.print(tfDistance);

    Serial.print(",tf=");
    Serial.print(tfUsable() ? 1 : 0);

    Serial.print(",hb=");
    Serial.print(heartbeatOK() ? 1 : 0);

    Serial.print(",dir=");
    Serial.print(directionName(directionCommand));

    Serial.print(",applied=");
    Serial.print(directionName(appliedDirection));

    Serial.print(",thr=");
    Serial.print(throttleCommand);

    Serial.print(",steer=");
    Serial.println(steerCommand);
}
