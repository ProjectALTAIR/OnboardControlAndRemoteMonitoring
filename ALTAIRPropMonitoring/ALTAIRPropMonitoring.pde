import controlP5.*;
import processing.serial.*;
import processing.sound.*;

boolean       isArduinoConnected     = true;
final String  fontString             = "SourceCodePro-Regular";
final String  arduinoPortString1     = "tty.usbmodem";   //For Macs
final String  arduinoPortString2     = "COM";            //For Windows
final int     portSpeed              =  9600;
final float   gravAcc                =  9.81;            // m/s^2

PFont         mainFont, smallFont;

Serial        port;
SoundFile     alarm;

float[]       setting      = new float[7];
float[]       rpm          = new float[4];
float[]       current      = new float[4];
float[]       temp         = new float[8];
float[]       accel        = new float[3];
float         yaw, pitch, roll, rotAng;
float         UM7health, UM7temp;

ControlP5[] GUI = new ControlP5[5];
AltairPropulsion module;
boolean inSetup;

void setup(){
  inSetup = true;
  
  size(1200,650,P3D);
  colorMode(RGB , 1);
  stroke(0);
  strokeWeight(.5);
  smooth(5);
  mainFont  = createFont(fontString, 17);
  smallFont = createFont(fontString, 10);
  
  String arduinoPortName = findSubstring(Serial.list(), arduinoPortString1);
  if (arduinoPortName.equals("")) arduinoPortName = findSubstring(Serial.list(), arduinoPortString2);
  if (arduinoPortName.equals("")) {
    println("No Arduino serial port connection found. Here is a list of available serial ports");
    println(Serial.list());
    println("We were looking for a port containing one of the strings: ");
    println(arduinoPortString1); println(arduinoPortString2);
    println("Since no port was found, we will use fake data instead.");
    isArduinoConnected = false;
  }
  if (isArduinoConnected){
    port = new Serial(this, arduinoPortName, portSpeed);
    print("Connectedto ALTAIR via the serial port: "); println(arduinoPortName);
  }
  
  alarm = new SoundFile(this, "Alarm.wav");
  for (int i = 0; i < 5; i++) GUI[i] = new ControlP5(this,mainFont);
  module = new AltairPropulsion(135,400);
  
  inSetup = false;
}

void draw(){
  background(1);
  getAltairArduinoInfoLine();
  module.drawAltair();
  module.soundAlarmIfOn();
}

void getAltairArduinoInfoLine() {
  if (!isArduinoConnected) {
    setFakeAltairValues();
  } else {
    String  infoLine = port.readStringUntil('\n');
    if (infoLine != null) {
      print("Received line: "); println(infoLine);
      if (infoLine.length() > 50) {
        float[] altairValues = float(splitTokens(infoLine));
        println("Line num characters: " + infoLine.length() + "   Line num floats: " + altairValues.length);
        if (altairValues.length == 32) {
          for (int i = 0; i < 7; ++i) setting[i] = altairValues[i];                  // on a floating-point scale from 0. to 10.
          for (int i = 0; i < 4; ++i) rpm[i]     = altairValues[i+7];
          for (int i = 0; i < 4; ++i) current[i] = altairValues[i+11];               // in amps
          for (int i = 0; i < 8; ++i) temp[i]    = altairValues[i+15];               // in degrees C
          for (int i = 0; i < 3; ++i) accel[i]   = altairValues[i+23];               // in "gravities" (converted to m/s^2 below)
          yaw = altairValues[26]; pitch = altairValues[27]; roll = altairValues[28]; // in degrees
          rotAng = altairValues[29];  // propulsion axle rotation angle (wrt a perfect vertical alignment with the gondola body), in degrees
          UM7health = altairValues[30]; UM7temp = altairValues[31];
          accel[2] += cos(radians(pitch))*cos(radians(roll));  accel[2] *= gravAcc;  // Subtract off Earth's gravitational acceleration, so that
          accel[1] += cos(radians(pitch))*sin(radians(roll));  accel[1] *= gravAcc;  // "hovering" should result in IDENTICALLY 0 ACCELERATION,
          accel[0] -= sin(radians(pitch));                     accel[0] *= gravAcc;  // and *= by 9.81, in order to convert "gravities" to m/s^2.        
        }
      }
    }
  }
  module.updateData(setting,rpm,current,temp,accel,yaw,pitch,roll,UM7health,UM7temp,rotAng);
}
void setFakeAltairValues() {
  for (int i = 0; i < 7; ++i) setting[i] = 2.9211;
  for (int i = 0; i < 4; ++i) rpm[i]     = 5100.;
  for (int i = 0; i < 4; ++i) current[i] = 0.;
  for (int i = 0; i < 8; ++i) temp[i]    = 0.;
  for (int i = 0; i < 3; ++i) accel[i]   = 0.;
  temp[5] = 40.;
  accel[2] = -0.5;
  accel[0] = -0.5;
  rotAng = 30.;
  rpm[2] = 0.;
  rpm[1] = 4001;
  yaw = 0.; pitch = 0.; roll = 0.;
  UM7health = 0.; UM7temp = 0.;   
  //current[2] = 60;
  setting[5] = 3;
}
