#include <Arduino.h>
#include <TM1637Display.h>
#include "pwm_lib.h"

#define MIN_RPM 5.0
#define MAX_RPM 50.0
#define STEPS_PER_REV 25600.0
#define MIN_FREQ (MIN_RPM * STEPS_PER_REV / 60.0)
#define MAX_FREQ (MAX_RPM * STEPS_PER_REV / 60.0)

#define CLK_FREQ 84000000.0 // 84 MHz

const int analogPin = A0;                                                           // Analog input pin for RPM control via potentiometer
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML7_PC24> motor_stp_pin; // PWM output pin (D6 = PC24)
const int motor_en_pin = 69;                                                        // Motor enable pin (D69)
const int motor_dir_pin = 68;                                                       // Motor direction pin (D68)
const int ms_in1 = 2;
const int ms_in2 = 4;

const int CLK = 58; // TM1637 Display CLK pin
const int DIO = 57; // TM1637 Display DIO pin

unsigned long previousMillis = 0;
const long interval = 50; // Interval to read the analog input (50 ms)
float acctual_freq = MIN_FREQ;

TM1637Display display(CLK, DIO);
void update();
void setup_pwm();
void update_pwm(float freq);
float map_float(long x, long in_min, long in_max, float out_min, float out_max);
float map_float_float(float x, float in_min, float in_max, float out_min, float out_max);

void setup()
{
    display.setBrightness(0x0f, true);
    analogReadResolution(10); // Set analog read resolution to 10 bits (0-1023)
    setup_pwm();              // Start PWM on PA2 (D2)

    pinMode(motor_en_pin, OUTPUT);
    pinMode(motor_dir_pin, OUTPUT);

    pinMode(ms_in1, INPUT_PULLUP);
    pinMode(ms_in2, INPUT_PULLUP);
}

void loop()
{

    unsigned long currentMillis = millis();

    // Periodically read potentiometer and update PWM and display
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;
        update();
    }
}

void update()
{
    bool in1 = digitalRead(ms_in1);
    bool in2 = digitalRead(ms_in2);

    bool off = !in1 && in2;
    bool reverse = !in1 && !in2;

    // int d = off * 10 + reverse;
    // display.showNumberDec(d);

    // Read potentiometer value
    int analogValue = analogRead(analogPin);
    analogValue = analogValue & 0xFFFE;             // Mask the least significant bit - denoising
    analogValue = constrain(analogValue, 10, 1020); // work around physical limits of the potentiometer
    float frequency = map_float(analogValue, 10, 1020, MAX_FREQ, MIN_FREQ);

    if (off)
    {
        digitalWrite(motor_en_pin, HIGH);
        frequency = 0;
    }
    else
    {
        digitalWrite(motor_en_pin, LOW);
    }

    if (reverse)
    {
        digitalWrite(motor_dir_pin, HIGH);
        frequency = -frequency;
    }
    else
    {
        digitalWrite(motor_dir_pin, LOW);
    }

    update_pwm(frequency); // Update PWM frequency based on potentiometer

    // Display the current RPM on the 7-segment display
    float rpm = map_float_float(frequency, MIN_FREQ, MAX_FREQ, MIN_RPM, MAX_RPM);
    display.showNumberDecEx(rpm * 10.0, 0b00100000);
}

void setup_pwm()
{
    uint32_t period = CLK_FREQ / acctual_freq;
    motor_stp_pin.start(period, period >> 1);
}

void update_pwm(float freq)
{

    if ((freq > 0 && acctual_freq < 0) || (freq == 0))
    {
        freq = MIN_FREQ;
        acctual_freq = MIN_FREQ;
    }
    if (freq < 0 && acctual_freq > 0)
    {
        freq = -MIN_FREQ;
        acctual_freq = -MIN_FREQ;
    }

    // Calculate new period based on frequency (base clock = 84 MHz)
    acctual_freq = acctual_freq * 0.975 + 0.025 * freq;
    uint32_t period = CLK_FREQ / abs(acctual_freq);
    motor_stp_pin.set_period_and_duty(period, period >> 1); // Update duty cycle to 50%
}

float map_float(long x, long in_min, long in_max, float out_min, float out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
float map_float_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}