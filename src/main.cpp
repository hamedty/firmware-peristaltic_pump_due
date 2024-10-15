#include <Arduino.h>
#include <TM1637Display.h>
#include "pwm_lib.h"

const int analogPin = A0;                                                         // Analog input pin for RPM control via potentiometer
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML7_PC24> stepper_pin; // PWM output pin (D6 = PC24)

const int CLK = 58; // TM1637 Display CLK pin
const int DIO = 57; // TM1637 Display DIO pin

unsigned long previousMillis = 0;
const long interval = 50; // Interval to read the analog input (50 ms)

TM1637Display display(CLK, DIO);
void setup_pwm();
void update_pwm(int freq);
void setup()
{
    display.setBrightness(0x0f, true);
    analogReadResolution(10); // Set analog read resolution to 10 bits (0-1023)
    setup_pwm();              // Start PWM on PA2 (D2)
}

void loop()
{

    unsigned long currentMillis = millis();

    // Periodically read potentiometer and update PWM and display
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;

        // Read potentiometer value
        int analogValue = analogRead(analogPin);

        // Map analog value (0-1023) to frequency and RPM ranges
        int frequency = map(analogValue, 0, 1023, 1000, 10000); // 1kHz to 10kHz frequency
        int rpm = map(analogValue, 0, 1023, 50, 400);           // 50 to 400 RPM

        display.showNumberDec(rpm); // Display the current RPM on the 7-segment display
        // update_pwm(frequency);      // Update PWM frequency based on potentiometer
    }
}

using namespace arduino_due::pwm_lib;

void setup_pwm()
{

#define CAPTURE_TIME_WINDOW 15000000 // usecs
#define DUTY_KEEPING_TIME 30000      // msecs

#define PWM_PERIOD_PIN_35 80000000 // hundredth of usecs (1e-8 secs)
#define PWM_DUTY_PIN_35 40000000   // 100 msecs in hundredth of usecs (1e-8 secs)

    stepper_pin.start(PWM_PERIOD_PIN_35, PWM_DUTY_PIN_35);
}

void update_pwm(int freq)
{
    // Calculate new period based on frequency (base clock = 84 MHz)
    uint32_t period = 84000000 / freq;
    stepper_pin.set_period_and_duty(period, period / 2); // Update duty cycle to 50%
}
