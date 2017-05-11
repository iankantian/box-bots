/*
    Code for 2017 Season Box Bots robot control board
    Arduino Nano clone w/ CH340 USB connected to TB6612 motor driver board

    CH1 - Steering
    CH2 - Throttle
    CH3 - Weopon
*/

#define   lpwm    3     // pulse width modulation for left motor is pin 3
#define   lpin1   4     // left control pin one is pin 4
#define   lpin2   5     // left control pin two is pin 5
#define   standby 7     // standby pin is 6 - LOW=motor driver off, HIGH=motor driver on for new board
#define   rpin1   8     // right control pin one is pin 7
#define   rpin2   9     // right control pin two is pin 8
#define   rpwm    6     // pulse width modulation for right motor is pin 9 for new board 

#define   forward 0
#define   reverse 1
#define   coast   2
#define   brake   3
#define   rightMotor  0
#define   leftMotor 1

#define   in_ch1  10    // input channel one is on pin 10
#define   in_ch2  11    // input channel two is on pin 11
#define   in_ch3  12    // input channel three is on pin 12

#define batterySensePin A5  // through 5:1 voltage divider
#define outputLED 13

/**
   these were measured directly with a variable voltage supply and multimeter,
   and some reference to https://www.helifreak.com/showthread.php?t=333661
   by reading the analog reading on the voltage divider and using Serial.println
   I could compare the analog measurements with the actual battery voltage.
   I want a warning at 20% battery copacity and a more urgent one at 10% capacity.
   the tricky part is doing this AND giving a 'Weapon Active' alert with just
   one LED.
*/
#define twoCellThreshold 102  // minimum reading to detect 2 cell lipo operation
#define twoCellTenPercent 130
#define twoCellTwentyPercent 136
#define threeCellThreshold 157 // minimum reading to detect 3 cell lipo operation
#define threeCellTenPercent 200
#define threeCellTwentyPercent 206
#define tenPercent 10 // representations of battery status
#define twentyPercent 20
#define batteryFull 100

int batteryTotal = 0;
int batteryVoltage = 0;
const int batteryNumSamples = 10;
int statusIndicatorIndex = 0; // where we are in the task of displaying the battery state
const int statusIndicatorSize = 20;
int statusIndicator[statusIndicatorSize]; // how many blinks we are going to give in the indication
int loopsPerTick = 4; // how many loops between updating the display
int loopsTickIndex = 0; // how many loops between updating the display
int batteryReadings[batteryNumSamples]; // we'll do a rolling average of 10 samples of the sense pin
int batterySenseIndex = 0; // the place in the array of readings
int batteryCellCount = 0; // representation of the battery lipo cell count
bool displayingBatteryState = true; // whether or not we are displying the volts of the battery
bool captureDisplayState = true; // whether or not to capture the state of the display
int batteryIndicatorBlinks = 0;

int ch1; // Steering - Joystick x-axis
int ch2; // Thottle - Joystick y-axis
int ch3; // Weapon Switch
int throttle = 0;
int spin = 0;
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;
byte  oldDirection = 0; //for troubleshooting stuttering left motor problem
byte  newDirection = 0;

void motorFunction(byte function, byte motor) {
  switch (motor) {
    case leftMotor:
      switch (function) {
        case forward:
          digitalWrite(lpin1, HIGH);
          digitalWrite(lpin2, LOW);
          break;
        case reverse:
          digitalWrite(lpin1, LOW);
          digitalWrite(lpin2, HIGH);
          break;

        case brake:
          digitalWrite(lpin1, HIGH);
          digitalWrite(lpin2, HIGH);
          break;

        default:  // coast condition
          digitalWrite(lpin1, LOW);
          digitalWrite(lpin2, LOW);
          break;
      }
      break;
    case rightMotor:
      switch (function) {
        case forward:
          digitalWrite(rpin1, HIGH);
          digitalWrite(rpin2, LOW);
          break;

        case reverse:
          digitalWrite(rpin1, LOW);
          digitalWrite(rpin2, HIGH);
          break;

        case brake:
          digitalWrite(rpin1, HIGH);
          digitalWrite(rpin2, HIGH);
          break;

        default:  // coast condition
          digitalWrite(rpin1, LOW);
          digitalWrite(rpin2, LOW);
      }
      break;
    default:
      break;
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(lpwm, OUTPUT);
  pinMode(lpin1, OUTPUT);
  pinMode(lpin2, OUTPUT);
  pinMode(rpwm, OUTPUT);
  pinMode(rpin1, OUTPUT);
  pinMode(rpin2, OUTPUT);
  pinMode(standby, OUTPUT);
  pinMode(outputLED, OUTPUT);

  pinMode(in_ch1, INPUT);       // channel one of RC receiver, x-axis steering
  pinMode(in_ch2, INPUT);       // channel two of RC receiver, y-axis throttle
  pinMode(in_ch3, INPUT);       // channel three of RC receiver, switch

  digitalWrite(lpin1, LOW);
  digitalWrite(lpin2, LOW);
  digitalWrite(rpin1, LOW);
  digitalWrite(rpin2, LOW);
  digitalWrite(standby, HIGH);  // turn on the things
  digitalWrite(outputLED, HIGH); // turn off the indicator light.

  // get initial battery level to decide what kind of battery we have installed!
  for (batterySenseIndex = 0; batterySenseIndex < batteryNumSamples; batterySenseIndex++) {
    batteryReadings[batterySenseIndex] = analogRead(batterySensePin);
    batteryTotal += batteryReadings[batterySenseIndex];
  }
  batteryVoltage = batteryTotal / 10;

  batteryVoltage = threeCellTwentyPercent + 1; // remove me !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


  batterySenseIndex = 0;
  Serial.begin(9600);

  if (batteryVoltage < twoCellThreshold) {
    Serial.println("1 cell battery");
    batteryCellCount = 1;
  } else if (batteryVoltage < threeCellThreshold) {
    Serial.println("2 cell battery");
    batteryCellCount = 2;
  } else {
    Serial.println("3 cell battery");
    batteryCellCount = 3;
  }
}

int batteryBlinkCount(int voltage) {
  int blinks = 0;
  if (batteryCellCount == 2) {
    if (voltage > twoCellTwentyPercent) {
      blinks = 1;
    } else if (voltage > twoCellTenPercent) {
      blinks = 3;
    } else {
      blinks = 6;
    }
  } else if (batteryCellCount == 3) {
    if (voltage > threeCellTwentyPercent) {
      blinks = 1;
    } else if (voltage > threeCellTenPercent) {
      blinks = 3;
    } else {
      blinks = 6;
    }
  }
  return blinks;
}

void loop() {
  // pulsein returning value of 1000 to 2000 (1500 default neutral position)
  // All Numbers are with transmitter channels in Normal position
  ch1 = pulseIn(in_ch1, HIGH, 25000); // Steering : 1000 Left, 2000 Right
  ch2 = pulseIn(in_ch2, HIGH, 25000); // Throttle : 1000 Reverse, 2000 Forward
  ch3 = pulseIn(in_ch3, HIGH, 25000); // Weapon : 1000 OFF, 2000 full ON

  // handle the case in which the signals time
  if (ch1 < 800) {
    ch1 = 1500;
  }
  if (ch2 < 800) {
    ch2 = 1500;
  }
  if (ch3 < 800) {
    ch1 = 1000;
  }

  ch1 = map(ch1, 1000, 2000, -255, 255); //center over 500
  ch2 = map(ch2, 1000, 2000, -255, 255); //center over 500
  ch3 = map(ch3, 1000, 2000, 0, 255);

  if (abs(ch1) < 15) {
    ch1 = 0;
  }
  if (abs(ch2) < 15) {
    ch2 = 0;
  }
  spin = -1 * ch1;
  throttle = -1 * ch2;

  rightMotorSpeed = constrain( throttle + spin, -255, 255);
  leftMotorSpeed = constrain( throttle - spin, -255, 255 );

  if (rightMotorSpeed < 0) {  // outside deadband, in reverse
    //   Serial.print(" Right Back ");
    motorFunction(reverse, rightMotor);
  }
  else {
    //   Serial.print(" Right Fwd ");
    motorFunction(forward, rightMotor);
  }
  if (leftMotorSpeed < 0) {
    //   Serial.print(" Left Back ");
    motorFunction(reverse, leftMotor);
    newDirection = reverse;
  }
  else {
    newDirection = forward;
    //   Serial.print(" Left Fwd ");
    motorFunction(forward, leftMotor);
  }
  if (oldDirection != newDirection) {
    Serial.print("@");
  }
  oldDirection = newDirection;

  analogWrite(lpwm, abs(leftMotorSpeed));
  analogWrite(rpwm, abs(rightMotorSpeed));


  batterySenseIndex++;
  if (batterySenseIndex > batteryNumSamples) { // avoid overflow on battery sense array
    batterySenseIndex = 0;
  }
  batteryReadings[batterySenseIndex] = analogRead(batterySensePin);

  loopsTickIndex++;
  if (loopsTickIndex > loopsPerTick) {
    loopsTickIndex = 0;
    statusIndicatorIndex++;
    if (statusIndicatorIndex > statusIndicatorSize) { // avoid overflow on status indicator array
      statusIndicatorIndex = 0;
      captureDisplayState = true; // after going over the end of the indicator array, restart display capture
    }
  }
  if (captureDisplayState) { // if we are displaying battery status, we shouldn't try to change its state
    for (int i = 0; i < batteryNumSamples; i++) { // calculate the voltage average for the last 10 samples
      batteryTotal += batteryReadings[i];
    }
    batteryVoltage = batteryTotal / 10; // ten sample rolling average

    for (int i = 0; i < statusIndicatorSize; i++) {
      statusIndicator[i] = false;
    }
    for (int i = 0; i < batteryIndicatorBlinks * 2; i += 2) {
      statusIndicator[i] = false;
      statusIndicator[i + 1] = true;
    }
    for (int i = batteryIndicatorBlinks * 2; i < statusIndicatorSize; i++) {
      if (ch3 < 25) {
        statusIndicator[i] = false;
      } else {
        statusIndicator[i] = true;
      }
    }
    captureDisplayState = false;
  }

  // now update the LED with the display data
  if (statusIndicator[statusIndicatorIndex]) {
    digitalWrite(outputLED, LOW);
  } else {
    digitalWrite(outputLED, HIGH);
  }
}
