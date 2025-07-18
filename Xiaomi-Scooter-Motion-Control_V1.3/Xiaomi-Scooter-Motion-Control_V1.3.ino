#include <Arduino.h>
#include <arduino-timer.h>
#include <SoftwareSerial.h> 

//SETTINGS

//===========================================================================
//============================= braking behaviour ===========================
//===========================================================================
// 0 = Disables engine braking (if available)
// 1 = Enables engine braking
#define BREAKBEHAVIOUR 1

// value when breaking is detected
#define BRAKE_THRESHOLD_VAL 47

//===========================================================================
//============================= throttle behaviour ===========================
//===========================================================================
// for every kmh of speed, the scooter will give more percentage of throttle.
#define THROTTLE_PCT_PER_KMH 5

// Boost timer, how many seconds until the throttle starts to wind down for a new kick
#define THROTTLE_TIME 5

// these settings are probably fine for everyone, some throttle buttons have more play than others, assuming the controler knows how to deal with this.
#define THROTTLE_MIN 50
#define THROTTLE_MAX 240
#define THROTTLE_RANGE THROTTLE_MAX - THROTTLE_MIN

//===========================================================================
//=============================Motion behavior  =============================
//===========================================================================

// Smooth readings of the speedometer. The higher the number, the more the readings 
// will be smoothed, but the slower the step will respond to the kicks.
#define SPEED_READINGS_AVERAGE 20

//===========================================================================
//=============================   Pin Settings  =============================
//===========================================================================
#define THROTTLE_PIN 10
#define FAKEKICK_PIN 4
#define LED_PIN 13

//TX & RX pin
SoftwareSerial SoftSerial(2, 3); // RX, TX

//END OF SETTINGS

auto windDownTimer = timer_create_default();
auto motionThinkTimer = timer_create_default();

int BrakeHandle;                // brake lever percent
int Speed;                      // current speed
int Throttle;

int readings[SPEED_READINGS_AVERAGE];    // the readings from the speedometer
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int AverageSpeed = 0;           // the average speed over last X readings
int OldAverageSpeed = 0;   		// the average speed over last X readings in the last loop

// modes of operation
enum class ThrottleState : uint8_t {
    IDLE,
    WIND_DOWN,
    ACCELERATE,
};
ThrottleState throttleState = ThrottleState::IDLE;
uint8_t throttlePowerPercent = 0;

uint8_t readBlocking() {
    while (!SoftSerial.available())
        delay(1);

    return SoftSerial.read();
}

void setup()
{
    pinMode(FAKEKICK_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    // initialize all the readings to 0:
    for (int thisReading = 0; thisReading < SPEED_READINGS_AVERAGE; thisReading++) {
        readings[thisReading] = 0;
    }

    Serial.begin(115200);
    SoftSerial.begin(115200);

    Serial.println("Starting up");

    TCCR1B = TCCR1B & 0b11111001;  //Set PWM of PIN 9 & 10 to 32 khz

    ThrottleWrite(0);

    motionThinkTimer.every(100, motionThink);
}

void loop() {
    motionThinkTimer.tick();

    if (!SoftSerial.available())
        return;

    // messages should start with 0x55 followed by 0xAA
    while (readBlocking() != 0x55)
        if (!SoftSerial.available())
            return;

    if (readBlocking() != 0xAA)
        return;

    uint8_t readBuff[256];
    uint8_t messageLen = readBlocking();
    readBuff[0] = messageLen;
    if (messageLen > 255)
        return;

    uint8_t addr = readBlocking();
    readBuff[1] = addr;

    uint16_t sum = messageLen + addr;
    for (int i = 0; i < messageLen; i++)
    {
        uint8_t curr = readBlocking();
        readBuff[i + 2] = curr;
        sum += curr;
    }

    uint16_t checksum = (uint16_t)readBlocking() | ((uint16_t)readBlocking() << 8);
    if (checksum != (sum ^ 0xFFFF)) // checksum check
        return;

    uint8_t messageOrigin = readBuff[1];
    uint8_t messageType = readBuff[2];

    switch (messageOrigin) {
        case 0x20: { // motor control
            switch (messageType) {
                case 0x64: { // throttle control reading
                    Throttle = readBuff[5];
                    break;
                }
                case 0x65: { // brake lever reading
                    BrakeHandle = readBuff[6];
                    break;
                }
            }
            break;
        }
        case 0x21: { // speed sensor
            switch (messageType) {
                case 0x64: { // speed reading
                    Speed = readBuff[8];
                    break;
                }
            }
            break;
        }
    }

    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the speedometer:
    readings[readIndex] = Speed;
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= SPEED_READINGS_AVERAGE) {
        // ...wrap around to the beginning:
        readIndex = 0;
    }

    // calculate the average:
    AverageSpeed = total / SPEED_READINGS_AVERAGE;
}

bool startWindDown(void *) {
    Serial.println("winding down throttle");
    throttleState = ThrottleState::WIND_DOWN;
    return false;
}

void motionThink() {
    // tick the winddown timer
    windDownTimer.tick();

    if (BrakeHandle > BRAKE_THRESHOLD_VAL) {
        // brake is pressed, stop throttle
        ThrottleWrite(0);
        Serial.println("Braking detected");
        digitalWrite(LED_PIN, HIGH);
        throttleState = ThrottleState::IDLE;
        windDownTimer.cancel();
        return;
    }
        
    // brake is unpressed, turn off led
    digitalWrite(LED_PIN, LOW);

    // check for a kick
    if (Speed > 0 && AverageSpeed > 5) { // we are moving above 5kmh average
        switch (throttleState) {
            case ThrottleState::IDLE:
            case ThrottleState::WIND_DOWN: { // we are in a idle or winddown state
                // calculate our speedTrend
                int speedTrend = AverageSpeed - OldAverageSpeed;
                if (speedTrend > 0) // speedTrend is increasing
                    kickDetected();
                break;
            }
        }

        OldAverageSpeed = AverageSpeed;
    }

    updateThrottle();
}

void kickDetected() {
    
    Serial.println("Kick detected");

    // set initial throttle
    throttlePowerPercent = THROTTLE_PCT_PER_KMH * AverageSpeed;
    if (throttlePowerPercent > 100)
        throttlePowerPercent = 100;

    throttleState = ThrottleState::ACCELERATE;

    windDownTimer.cancel();
    windDownTimer.in(THROTTLE_TIME * 1000, startWindDown); // Set timer to wind down
}

void updateThrottle()
{
    // check if the fake kick button is pressed
    if (!digitalRead(FAKEKICK_PIN))
        kickDetected();

    uint8_t throttle;
    switch (throttleState) {
        case ThrottleState::IDLE: {
            throttle = 0;
            break;
        }
        case ThrottleState::WIND_DOWN: {
            throttle = --throttlePowerPercent;

            // check if we wound down to idle
            if (!throttle)
                throttleState = ThrottleState::IDLE;

            break;
        }
        case ThrottleState::ACCELERATE: {
            if (throttlePowerPercent < 100)
                ++throttlePowerPercent;

            throttle = throttlePowerPercent;
            break;
        }
    }

    ThrottleWrite(throttle);
}

int ThrottleWrite(uint8_t percent)
{
    // clamp input
    if (percent < 0)
        percent = 0;
    else if (percent > 100)
        percent = 100;

    // update global throttle var
    throttlePowerPercent = percent;

    // Linear interpolation from THROTTLE_MIN to THROTTLE_MAX
    int throttleVal = THROTTLE_MIN + (percent * (THROTTLE_RANGE)) / 100;

    Serial.print("ThrottleWrite: ");
    Serial.print(percent);
    Serial.println(" ");

    analogWrite(THROTTLE_PIN, throttleVal);
}
