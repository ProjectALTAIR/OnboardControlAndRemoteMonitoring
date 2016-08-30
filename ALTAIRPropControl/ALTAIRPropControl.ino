// Our Arduino code for propulsion and attitude control
#include <YetAnotherPcInt.h> //library for attaching interrupts to A14, A15 (using library verson 2.1.0)
                             //If nullptr not declared in scope error occurs, update to a more recent version of the avr-gcc compiler

const int ledPin = 13;      // The pin that the LED is attached to.
// Pins 2, 3, and 5 correspond to timer 3 pins.  Pins 6, 7, and 8 correspond to timer 4 pins.
const int pwmPin[6]      = { 2, 3, 5, 6, 7, 8 };
float     setting[7]     = { 0., 0., 0., 0., 0., 7., 0. };
// Pulse timing for measuring the 4 RPMs.
const int rpmTimerPin[4] = { A13, A14, A15, 18 };
const int numRPMPulsesToAverage = 20;
const long    numMicrosBeforeRPMTimeout = 1000000;
const long    numMicrosPerMinute = 60000000;
const int     numMicrosDelayBtwReads = 20;
const int     numPulsesPerRevolution = 4;
int       channelToModify = 0;
const int currentSensorPin[4] = { A0, A1, A2, A3 };
int       currentSensorValue[4];
const int numCurrentValsToAverage = 20;
float     currentInAmps[4][numCurrentValsToAverage];  // average the past 20 values
const int tempSensorPin[8] = { A10, A9, A6, A4, A11, A8, A7, A5 }; //A11,A10 may need to be swapped
int       tempSensorValue;
float     tempInCelsius[8] = { 0., 0., 0., 0., 0., 0., 0., 0. };
const int rotAngSensorPin  = A12;
float     rotAng = 0.0;
float     UM7health = 0.0, UM7temp = 0.0;

const unsigned int rx_read_length = 200;
float yaw     = -999., pitch     = -999., roll     = -999.;
float yawRate = -999., pitchRate = -999., rollRate = -999.;         //In degrees per second
float accel[3] = { -999., -999., -999. };
float accelCorrectionFactor[3] = { 0., 0., 0. };

//Interrupt service routines (ISR) variables
//Volatile variables are used outside the ISRs too.
volatile byte          halfRotations[4]  = {0, 0, 0, 0};            //Keeps track of how much of a rotation is completed
byte                   placeData[4]      = {0, 0, 0, 0};            //Keeps track of where in the pulseDurations array to place the newest piece of data
volatile unsigned long startTimePulse[4] = {-999,-999,-999,-999};
unsigned long          endTimePulse[4];
volatile long          pulseDurations[4][numRPMPulsesToAverage];
//End of ISR variables

struct UM7packet {
  byte  Address;
  byte  PT;
  unsigned int Checksum;
  byte  data_length;
  byte  data[75];
};

typedef union {
  byte  array[4];
  float value;
} ByteToFloat;

float convertBytesToFloat(byte* data) {
  ByteToFloat converter;
  for (byte i = 0; i < 4; i++) {
    converter.array[3 - i] = data[i]; //or converter.array[i] = data[i]; if the endian were reversed
  }
  return converter.value;
}

void getAccelCorrectionFactors() {

}

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial3.begin(115200, SERIAL_8N1);

  // initialize digital pins 2, 3, 5, 6, 7, 8 as output.
  for (int i = 0; i < 6; ++i) pinMode(pwmPin[i], OUTPUT);
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < numCurrentValsToAverage; ++j) currentInAmps[i][j] = 0.;
  }

  initializeRPMsensors();

  // initialize the CHRobotics UM7 yaw-pitch-roll monitor
  checkUM7Health();
  getAccelCorrectionFactors();

  // start up the PWM outputs
  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM32) | _BV(WGM31);
  TCCR3B = _BV(CS32);
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM42) | _BV(WGM41);
  TCCR4B = _BV(CS42);

  OCR3A = 32 + 2 * setting[0]; // pin 5, servo 0
  OCR3C = 32 + 2 * setting[1]; // pin 3, servo 1
  OCR4B = 32 + 2 * setting[2]; // pin 7, servo 2
  OCR4A = 32 + 2 * setting[3]; // pin 6, servo 3
  OCR3B = 32 + 2 * setting[4]; // pin 2,
  OCR4C = 32 + 2 * setting[5]; // pin 8, axle servo
}

void loop() {
  static int loopCount = 0;
  byte inputByte;
  boolean thingsHaveChanged = false;
  double rpm[4] = { 0., 0., 0., 0. };
  float  currentRunningAverage[4] = { 0., 0., 0., 0. };
  int yawInt = -999, pitchInt = -999, rollInt = -999;
  int yawRateInt = -999; int pitchRateInt = -999; int rollRateInt = -999;

  for (int i = 0; i < 4; ++i) {
    currentSensorValue[i] = analogRead(currentSensorPin[i]);
    for (int j = 0; j < numCurrentValsToAverage - 1; ++j) {
      currentRunningAverage[i] += currentInAmps[i][j + 1];
      currentInAmps[i][j] = currentInAmps[i][j + 1];
    }
    currentInAmps[i][numCurrentValsToAverage - 1] = (505 - currentSensorValue[i]) / 3.3;
    currentRunningAverage[i] += currentInAmps[i][numCurrentValsToAverage - 1];
    currentRunningAverage[i] /= numCurrentValsToAverage;
  }
  tempSensorValue = analogRead(tempSensorPin[loopCount % 8]);
  tempInCelsius[loopCount % 8] = 22 + (tempSensorValue - 535) / 2.;
  for (int i = 0; i < 4; ++i) {
    if (loopCount % 10 == 0){ //Only check if the rotors have stopped every 10 cyles.
      isSpinning(i);
    }
    rpm[i] = getRPM(i);
  }
  rotAng = analogRead(rotAngSensorPin);

  // This accepts an input from the usb port and interprets it
  if (Serial.available()) {
    //Serial.flush();
    inputByte = Serial.read();
    if (inputByte == 'm') {
      inputByte = Serial.read();
      if (inputByte == 's') {
        thingsHaveChanged = true;
        inputByte = Serial.read();
        doInstruction(inputByte, &thingsHaveChanged);
      }
    }
  }
  // CHRobotics UM-7 connected to serial port 3, Serial Monitor connected to serial port 0:
  if (Serial3.available()) {
    getDataFromUM7();
    sendALTAIRinfoLine(rpm, currentRunningAverage);

    if (thingsHaveChanged) {
      OCR3A = 32 + 2 * setting[0]; //pin 5, servo 0
      OCR3C = 32 + 2 * setting[1]; //pin 3, servo 1
      OCR4B = 32 + 2 * setting[2]; //pin 7, servo 2
      OCR4A = 32 + 2 * setting[3]; //pin 6, servo 3
      OCR3B = 32 + 2 * setting[4]; //pin 2
      OCR4C = 32 + 2 * setting[5]; //pin 8, axle servo
    }
  }
  loopCount++;
}
