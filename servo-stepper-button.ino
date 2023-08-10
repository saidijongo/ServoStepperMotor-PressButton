#include <AccelStepper.h>
#include <Servo.h>

const int motorInterfaceType = 1;
AccelStepper stepper(motorInterfaceType, 5, 6);
Servo servoMotor;

const int buttonPin = 2;
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int buttonPressCount = 0;

void setup() {
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);

  servoMotor.attach(9); 
  servoMotor.write(90);

  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        buttonPressCount++;

        if (buttonPressCount == 1) {
          rotateStepperAndServo(10); // Rotate clockwise
        } else if (buttonPressCount == 2) {
          rotateStepperAndServo(-10); // Rotate counterclockwise
        }
      }
    }
  }

  if (buttonState == HIGH) {
    buttonPressCount = 0;
    stepper.stop();
    stepper.setCurrentPosition(0);
    servoMotor.write(90); 
  }

  lastButtonState = reading;
}

void rotateStepperAndServo(int seconds) {
  unsigned long startTime = millis();

  while (millis() - startTime < abs(seconds) * 1000) {
    stepper.moveTo(720); 
    stepper.run();

    if (seconds > 0) {
      servoMotor.write(90 - map(millis() - startTime, 0, abs(seconds) * 1000, 0, 180));
    } else {
      servoMotor.write(90 + map(millis() - startTime, 0, abs(seconds) * 1000, 0, 180));
    }
  }

  stepper.stop();
  stepper.setCurrentPosition(0);

  servoMotor.write(90); 
}
