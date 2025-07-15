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

//===========================================================================
//============================= throttle behaviour ===========================
//===========================================================================
// for every kmh of speed, the scooter will give more percentage of throttle.
#define THROTTLE_PCT_PER_KMH 5

// Boost timer, how many seconds the motor will be powered after a kick
#define THROTTLE_TIME 8

// these settings are probably fine for everyone, some throttle buttons have more play than others, assuming the controler knows how to deal with this.
#define THROTTLE_MIN 50
#define THROTTLE_MAX 200
#define THROTTLE_RANGE THROTTLE_MAX - THROTTLE_MIN

//===========================================================================
//=============================Motion behavior  =============================
//===========================================================================

int kickdelay = 1000; //time before you can do an new kick after boost timer is expired.

// Smooth readings of the speedometer. The higher the number, the more the readings 
// will be smoothed, but the slower the step will respond to the kicks.
const int speedReadings = 20;

//===========================================================================
//=============================   Pin Settings  =============================
//===========================================================================

// Arduino pin where throttle is connected to (only pin 9 & 10 is ok to use)
const int THROTTLE_PIN = 10;
int LED_PCB = 13;

//TX & RX pin
SoftwareSerial SoftSerial(2, 3); // RX, TX

//END OF SETTINGS

auto timer_m = timer_create_default();

int BrakeHandle;                // brake lever percent
int Speed;                      // current speed
int Throttle;

int oldspeed;                   // speed during last loop
int trend = 0;                  // speed trend
int readings[speedReadings];    // the readings from the speedometer
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int AverageSpeed = 0;           // the average speed over last X readings
int OldAverageSpeed = 0;   		// the average speed over last X readings in the last loop

//motionmodes
uint8_t motionstate = 0;
#define motionready 0
#define motionbusy 1
#define motionbreaking 2

void logByteInHex(uint8_t val) {
    if(val < 16)
        Serial.print('0');
        
    Serial.print(val, 16);
    Serial.print(' ');
}

uint8_t readBlocking() {
    while (!SoftSerial.available())
        delay(1);

    return SoftSerial.read();
}

void setup()
{
    pinMode(LED_PCB, OUTPUT);
    // initialize all the readings to 0:
    for (int thisReading = 0; thisReading < speedReadings; thisReading++) {
        readings[thisReading] = 0;
    }

    Serial.begin(115200);
    SoftSerial.begin(115200);

    Serial.println("Starting up");

    TCCR1B = TCCR1B & 0b11111001;  //Set PWM of PIN 9 & 10 to 32 khz

    ThrottleWrite(0);
}

void loop() {
    while (readBlocking() != 0x55);
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
    if (readIndex >= speedReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
    }

    // calculate the average:
    AverageSpeed = total / speedReadings;

    // Serial.print("Speed: ");
    // Serial.print(Speed);
    // Serial.print(" Throttle: ");
    // Serial.print(Throttle);
    // Serial.print(" Brake: ");
    // Serial.print(BrakeHandle);
    // Serial.print(" State: ");
    // Serial.print(motionstate);
    // Serial.println(" ");

    motion_control();
    timer_m.tick();
}

bool release_throttle(void *) {
    Serial.println("Releasing throttle");

#if (BREAKBEHAVIOUR == 0)
    ThrottleWrite(10); // Keep throttle open for 10% to disable KERS. best for essential.
#else
    ThrottleWrite(0); // Fully close throttle, may enable motor regeneration
#endif

    timer_m.in(kickdelay, motion_wait);
    return false; // false to stop
}

bool motion_wait(void *) {
    motionstate = motionready;
    return false;
}

void motion_control() {
    if ((Speed != 0) && (Speed < 5)) {
        // If speed is under 5 km/h, stop throttle
        ThrottleWrite(0); //  0% throttle
    }

    if (BrakeHandle > 47) {
        ThrottleWrite(0); //close throttle directly when break is touched. 0% throttle
        Serial.println("Braking detected");
        digitalWrite(LED_PCB, HIGH);
        motionstate = motionready;
        timer_m.cancel();
    } else {
        digitalWrite(LED_PCB, LOW);
    }

    if (Speed != 0) {
        // Check if auto throttle is off and speed is increasing
        if (motionstate == motionready) {
            trend = AverageSpeed - OldAverageSpeed;
            if (trend > 0) {
                // speed is increasing
                // Check if speed is at least 5 km/h
                if (AverageSpeed > 5) {
                    // Open throttle for 5 seconds
                    kickDetected();
                    motionstate = motionbusy;

                }

            } else if (trend < 0) {
                // speed is decreasing

            } else {
                // no change in speed
            }
        }

        oldspeed = Speed;
        OldAverageSpeed = AverageSpeed;
    }
}

void kickDetected() {
    Serial.println("Kick detected");

    int throttle = THROTTLE_PCT_PER_KMH * AverageSpeed;

    ThrottleWrite(throttle);

    timer_m.in(THROTTLE_TIME * 1000, release_throttle); //Set timer to release throttle
}

int ThrottleWrite(int percent)
{
    // clamp input
    if (percent < 0)
        percent = 0;
    else if (percent > 100)
        percent = 100;

    // Linear interpolation from THROTTLE_MIN to THROTTLE_MAX
    int throttleVal = THROTTLE_MIN + (percent * (THROTTLE_RANGE)) / 100;

    // Serial.print("percent: ");
    // Serial.print(percent);
    // Serial.print(" throttleVal: ");
    // Serial.print(throttleVal);
    // Serial.println(" ");

    analogWrite(THROTTLE_PIN, throttleVal);
}
