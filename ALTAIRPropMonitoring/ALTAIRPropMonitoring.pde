import processing.serial.*;
import processing.sound.*;

final boolean testArduinoUnconnected =  true;
final String  fontString             = "SourceCodePro-Regular";
final String  arduinoPortString1     = "tty.usbmodem";
final String  arduinoPortString2     = "COM4";
final int     portSpeed              =  9600;
final int     numAlarms              =    22;
final int     timeBetweenAlarmSounds =    30;    // in approximately 20's of milliseconds (so e.g. a value of 30 ~= 0.6 seconds)
final float   gravAcc                =     9.81; // m/s^2

final float   maxSetting             =    10.0;  // max power setting for each motor: setting ranges from 0 - 10
final float   maxRPM                 =  5000.0;
final float   maxTemp                =    50.0;  // degrees C
final float   minTemp                =   -50.0;  // degrees C
final float   maxCurrent             =    10.0;  // Amperes (per each motor+ESC pair)

final int     buttonSquareSize       =    12;    // pixels
final int     buttonSqCornerRadius   =     6;    // pixels 
final int     buttonSpacing          =    15;    // pixels

float         oldyaw = 0., oldpitch = 0., oldroll = 0.;

PFont         mainFont, smallFont;

Serial        port;
SoundFile     alarm;
boolean       blockButtons = false;
byte[]        alarmOn      = new byte[numAlarms]; // 0 == off, 1 == on but silenced, 2 == on (and not silenced)
int           alarmCounter = 0;

float[]       setting      = new float[7];
float[]       rpm          = new float[4];
float[]       current      = new float[4];
float[]       temp         = new float[8];
float[]       accel        = new float[3];
float[]       propRotAng   = new float[6];
float         yaw, pitch, roll, rotAng;
float         UM7health, UM7temp;
 
void setup()  { 
  size(1200, 600, P3D); 

  colorMode(RGB, 1); 
  stroke(0);
  strokeWeight(0.5);
  smooth(5);

  String arduinoPortName = findSubstring(Serial.list(), arduinoPortString1);
  if (arduinoPortName.equals("")) arduinoPortName = findSubstring(Serial.list(), arduinoPortString2);
  if (arduinoPortName.equals("") && !testArduinoUnconnected) {
    println("ERROR: No ARDUINO serial port connection found!!!  Here is the list of available serial ports:");
    println(Serial.list());
    print("We were looking for a port name containing one of the strings: "); println(arduinoPortString1); println(arduinoPortString2);
    println("Since an ARDUINO port connection does not appear to be among the serial ports, the PROGRAM IS NOW EXITING!!!");
    exit();
  } else {
    if (!testArduinoUnconnected) {
      port     = new Serial(this, arduinoPortName, portSpeed);
      print("Connected to ALTAIR via serial port: "); println(arduinoPortName); 
    }

    alarm      = new SoundFile(this, "Alarm.wav");
    for (int i = 0; i < numAlarms; ++i) alarmOn[i] = 0;

    mainFont   = createFont(fontString, 17);
    smallFont  = createFont(fontString, 10);
    textFont(mainFont);
    textAlign(LEFT);
  }
} 
 
void draw()  { 
  background(1);
  getAltairArduinoInfoLine();
  displayPropulsionSystemInfo();
  displayUM7SystemInfo();
}

String findSubstring(String[] stringList, String substring) {
  for (int i = 0; i < stringList.length; ++i) {
    if (stringList[i].indexOf(substring) != -1) return stringList[i];
  }
  return "";
}

void getAltairArduinoInfoLine() {
  if (testArduinoUnconnected) {
    setFakeAltairValues();
  } else {
    String  infoLine     = port.readStringUntil('\n');
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
}

void displayPropulsionSystemInfo() {
  int upperMotorIsRed = 0, lowerMotorIsRed = 0, upperPropIsGreen = 0, lowerPropIsGreen = 0, upperPropIsRed = 0, lowerPropIsRed = 0;
  
  fill(0.8);
  noStroke();
  rect(135., 400., 900., 5.);
  stroke(0);
  strokeWeight(0.7);
  makeCylinder( 585., 380., 65, 85);

  noFill();
  makeCylinder(1050., 160., 65, 85);
  strokeWeight(0.5);

  for (int i = 0; i < 4; ++i) {
    int    isHigh = 0, isRight = 0;
    if (i == 0 || i == 3) {
           isHigh = 1;
    }
    if (i > 1) isRight = 1;

    float fillHeight = 40.*(setting[i]/(maxSetting*1.2)), fillHeightESC;
    int motorIsRed = 0, escIsRed = 0, propIsRed = 0, propIsGreen = 0;
    if (setting[i] > maxSetting || setting[i] < 0.) if (alarmOn[i] == 0) alarmOn[i] = 2;                                                      
    else                                            alarmOn[i] = 0;
    fill(color(1.,1.,1.));
    ellipse(18.  + 235.*i + 240.*isRight, 465.  - 30.*isHigh,                 4., 40.);
    fill(color(1.*(alarmOn[i] > 0 ? 1 : 0),0.7*(alarmOn[i] > 0 ? 0 : 1),0));
    noStroke();
    ellipse(18.5 + 235.*i + 240.*isRight, 483.5 - 30.*isHigh - fillHeight/2., 3., fillHeight);
    stroke(0);
    drawType("Power",              30. + 235.*i + 240.*isRight, 460. - 30.*isHigh, 1.*(alarmOn[i] > 0 ? 1 : 0),   0.,                             0.);
    drawType("set at:",            83. + 235.*i + 240.*isRight, 460. - 30.*isHigh, 1.*(alarmOn[i] > 0 ? 1 : 0),   0.,                             0.);
    textAlign(RIGHT);
    drawType(nf(setting[i],1,1),  186. + 235.*i + 240.*isRight, 460. - 30.*isHigh, 1.*(alarmOn[i] > 0 ? 1 : 0),   0.6*(alarmOn[i] > 0 ? 0 : 1),   0.);
    textFont(smallFont);
    drawType(nf(int(maxSetting)),  14. + 235.*i + 240.*isRight, 456. - 30.*isHigh, 0.35,                          0.35,                           0.);    
    drawType("0",                  14. + 235.*i + 240.*isRight, 486. - 30.*isHigh, 0.35,                          0.35,                           0.);
    textFont(mainFont);
    textAlign(LEFT);
    drawType("/10",               185. + 235.*i + 240.*isRight, 460. - 30.*isHigh,   0.,                          0.,                             0.);
    makeButtons(                  218. + 235.*i + 240.*isRight, 443. - 30.*isHigh,   i);

    fillHeight = 40.*(rpm[i]/(maxRPM*1.2));
    if (rpm[i] > 0.)                    { propIsGreen = 1;
                                          if (isHigh == 1)   { upperPropIsGreen = 1 - upperPropIsRed; }
                                          else               { lowerPropIsGreen = 1 - lowerPropIsRed; }
                                        }
    if (rpm[i] > maxRPM || rpm[i] < 0.) { motorIsRed = 1; propIsRed = 1; propIsGreen = 0;
                                          if (isHigh == 1)   { upperMotorIsRed = 1; upperPropIsRed = 1; upperPropIsGreen = 0; }
                                          else               { lowerMotorIsRed = 1; lowerPropIsRed = 1; lowerPropIsGreen = 0; }
                                          if (alarmOn[i+4] == 0) alarmOn[i+4] = 2;
                                        } 
    else                                { alarmOn[i+4] = 0; }
    fill(color(1.,1.,1.));
    ellipse(71.  + 235.*i + 240.*isRight, 490.  - 30.*isHigh,                 4., 40.);
    fill(color(1.*(alarmOn[i+4] > 0 ? 1 : 0), 0.7*(alarmOn[i+4] > 0 ? 0 : 1), 0));
    noStroke();
    ellipse(71.5 + 235.*i + 240.*isRight, 508.5 - 30.*isHigh - fillHeight/2., 3., fillHeight);
    stroke(0);
    drawType("RPM:",               83. + 235.*i + 240.*isRight, 500. - 30.*isHigh, 1.*(alarmOn[i+4] > 0 ? 1 : 0), 0.,                             0.); 
    textAlign(RIGHT);
    drawType(nf(int(rpm[i])),     173. + 235.*i + 240.*isRight, 500. - 30.*isHigh, 1.*(alarmOn[i+4] > 0 ? 1 : 0), 0.6*(alarmOn[i+4] > 0 ? 0 : 1), 0.);
    textFont(smallFont);
    drawType(nf(int(maxRPM)),      67. + 235.*i + 240.*isRight, 481. - 30.*isHigh, 0.35,                          0.35,                           0.);    
    drawType("0",                  67. + 235.*i + 240.*isRight, 511. - 30.*isHigh, 0.35,                          0.35,                           0.);
    textFont(mainFont);
    textAlign(LEFT);

    fillHeight    = 40.*((temp[i]   - minTemp)/((maxTemp - minTemp)*1.2));
    fillHeightESC = 40.*((temp[i+4] - minTemp)/((maxTemp - minTemp)*1.2));
    if (temp[i]   > maxTemp || temp[i]   < minTemp) { motorIsRed = 1;
                                                      if (isHigh == 1) { upperMotorIsRed = 1; }
                                                      else             { lowerMotorIsRed = 1; }
                                                      if (alarmOn[i+8] == 0)  alarmOn[i+8]  = 2;
                                                    } 
    else                                            { alarmOn[i+8]  = 0; }
    if (temp[i+4] > maxTemp || temp[i+4] < minTemp) { escIsRed = 1;
                                                      if (alarmOn[i+12] == 0) alarmOn[i+12] = 2;
                                                    } 
    else                                            { alarmOn[i+12] = 0; }
    fill(color(1.,1.,1.));
    ellipse( 71.  + 235.*i + 240.*isRight, 535.  - 30.*isHigh,                 4., 40.);
    ellipse(190.  + 190.*i + 115.*isRight, 340.              ,                 4., 40.);
    noStroke();
    fill(color(1.*(alarmOn[i+8]  > 0 ? 1 : 0), 0.7*(alarmOn[i+8]  > 0 ? 0 : 1), 0));
    ellipse( 71.5 + 235.*i + 240.*isRight, 553.5 - 30.*isHigh - fillHeight/2.,    3., fillHeight);
    fill(color(1.*(alarmOn[i+12] > 0 ? 1 : 0), 0.7*(alarmOn[i+12] > 0 ? 0 : 1), 0));
    ellipse(190.5 + 190.*i + 115.*isRight, 358.5              - fillHeightESC/2., 3., fillHeightESC);    
    stroke(0);
    drawType("Temp:",              83. + 235.*i + 240.*isRight, 540. - 30.*isHigh, 1.*(alarmOn[i+8]  > 0 ? 1 : 0), 0.,                              0.);
    drawType("Temp:",             202. + 190.*i + 115.*isRight, 355.             , 1.*(alarmOn[i+12] > 0 ? 1 : 0), 0.,                              0.);
    textAlign(RIGHT);
    drawType(nf(int(temp[i])),    173. + 235.*i + 240.*isRight, 540. - 30.*isHigh, 1.*(alarmOn[i+8]  > 0 ? 1 : 0), 0.6*(alarmOn[i+8]  > 0 ? 0 : 1), 0.);
    drawType(nf(int(temp[i+4])),  282. + 190.*i + 115.*isRight, 355.             , 1.*(alarmOn[i+12] > 0 ? 1 : 0), 0.6*(alarmOn[i+12] > 0 ? 0 : 1), 0.);
    textFont(smallFont);
    drawType(nf(int(maxTemp)),     67. + 235.*i + 240.*isRight, 526. - 30.*isHigh, 0.35,                           0.35,                            0.);    
    drawType(nf(int(minTemp)),     67. + 235.*i + 240.*isRight, 556. - 30.*isHigh, 0.35,                           0.35,                            0.);
    drawType(nf(int(maxTemp)),    186. + 190.*i + 115.*isRight, 331.             , 0.35,                           0.35,                            0.);    
    drawType(nf(int(minTemp)),    186. + 190.*i + 115.*isRight, 361.             , 0.35,                           0.35,                            0.);
    textFont(mainFont);
    textAlign(LEFT);
    drawType("C",                 178. + 235.*i + 240.*isRight, 540. - 30.*isHigh, 0.,                             0.,                              0.);
    drawType("C",                 287. + 190.*i + 115.*isRight, 355.             , 0.,                             0.,                              0.);

    fillHeight = 40.*(current[i]/(maxCurrent*1.2));
    if (current[i] > maxCurrent || current[i] < 0.) { motorIsRed = 1; escIsRed = 1;
                                                      if (isHigh == 1) { upperMotorIsRed = 1; }
                                                      else             { lowerMotorIsRed = 1; }
                                                      if (alarmOn[i+16] == 0) alarmOn[i+16] = 2;
                                                    } 
    else                                            { alarmOn[i+16] = 0; }
    fill(color(1.,1.,1.));
    ellipse(71.  + 235.*i + 240.*isRight, 580.  - 30.*isHigh,                 4., 40.);
    fill(color(1.*(alarmOn[i+16] > 0 ? 1 : 0), 0.7*(alarmOn[i+16] > 0 ? 0 : 1), 0));
    noStroke();
    ellipse(71.5 + 235.*i + 240.*isRight, 598.5 - 30.*isHigh - fillHeight/2., 3., fillHeight);
    stroke(0);
    drawType("Curr.:",             83. + 235.*i + 240.*isRight, 580. - 30.*isHigh, 1.*(alarmOn[i+16] > 0 ? 1 : 0), 0.,                                0.);
    textAlign(RIGHT);
    drawType(nf(int(current[i])), 173. + 235.*i + 240.*isRight, 580. - 30.*isHigh, 1.*(alarmOn[i+16] > 0 ? 1 : 0), 0.6*(alarmOn[i+16] > 0 ? 0 : 1),   0.);
    textFont(smallFont);
    drawType(nf(int(maxCurrent)),  67. + 235.*i + 240.*isRight, 571. - 30.*isHigh, 0.35,                           0.35,                              0.);    
    drawType("0",                  67. + 235.*i + 240.*isRight, 601. - 30.*isHigh, 0.35,                           0.35,                              0.);
    textFont(mainFont);
    textAlign(LEFT);
    drawType("A",                 178. + 235.*i + 240.*isRight, 580. - 30.*isHigh, 0.,                             0.,                                0.);

    fill(1.0, 0.5*(1.0-motorIsRed), 0.0); 
    makeCylinder(139. + 210.*i + 260.*isRight, 406. - 20.*isHigh, 8., 15.);
    fill(1.0, 0.5*(1.0-escIsRed),   0.0);
    rect(        225. + 195.*i + 105.*isRight, 394.,             18.,  6.);

    pushMatrix();
    translate(139. + 210.*i + 260.*isRight, 425. - 43.*isHigh, 0.);
    if (propIsGreen > 0 || propIsRed > 0) { rotateY(propRotAng[i]); ++propRotAng[i]; }
    fill(1.0*propIsRed, 1.0*propIsGreen, 0.0);
    beginShape(TRIANGLES);
    vertex(   0.,  0.,  0.);
    vertex( 105., -3., -2.);
    vertex( 105.,  3.,  2.);
    vertex(   0.,  0.,  0.);
    vertex(-105., -3.,  2.);
    vertex(-105.,  3., -2.);
    endShape();
    popMatrix();

  }
  pushMatrix();
  translate(1052., 190., 0.);
  rotateZ(radians(rotAng));
  fill(0.8);
  rect(-2., -2., 5., 5.);
  fill(1.0, 0.5*(1.0-upperMotorIsRed), 0.0);
  makeCylinder(0., -17.5, 8., 15.);
  fill(1.0, 0.5*(1.0-lowerMotorIsRed), 0.0);
  makeCylinder(0.,   4.5, 8., 15.);
  pushMatrix();
  translate(0., -22.);
  rotateX(radians(-28.));
  if (upperPropIsGreen > 0 || upperPropIsRed > 0) { rotateY(propRotAng[4]); ++propRotAng[4]; }
  fill(1.0*upperPropIsRed, 1.0*upperPropIsGreen, 0.0);
  beginShape(TRIANGLES);
  vertex(   0.,  0.,  0.);
  vertex(  75., -2., -2.);
  vertex(  75.,  2.,  2.);
  vertex(   0.,  0.,  0.);
  vertex( -75., -2.,  2.);
  vertex( -75.,  2., -2.);
  endShape();
  popMatrix();
  pushMatrix();
  translate(0.,  22.);
  rotateX(radians(-28.));
  if (lowerPropIsGreen > 0 || lowerPropIsRed > 0) { rotateY(propRotAng[5]); ++propRotAng[5]; }
  fill(1.0*lowerPropIsRed, 1.0*lowerPropIsGreen, 0.0);
  beginShape(TRIANGLES);
  vertex(   0.,  0.,  0.);
  vertex(  75., -2., -2.);
  vertex(  75.,  2.,  2.);
  vertex(   0.,  0.,  0.);
  vertex( -75., -2.,  2.);
  vertex( -75.,  2., -2.);
  endShape();
  popMatrix();
  popMatrix();
  
  makePanicButton();
  if (anyAlarmIsOn()) {
    soundAlarm();
    makeSilenceAlarmsButton();
  }
}

void displayUM7SystemInfo() {
  pushMatrix(); 
  translate(width/1.6, height/3.5, -30); 

  rotateX(radians(90.-pitch)); 
  rotateY(radians(-roll));
  rotateZ(radians(-yaw));
  
  fill(0., 1., 0.);

  make3DArrow(90., 10., 1., 0., 0.);

  fill(1, 0.6, 0.6);
  
  make3DArrow(90., 10., 0., 1., 0.);
    
  fill(0.4, 0.6, 1);
  
  make3DArrow(90., 10., 0., 0., 1.);

  float xscreenx = screenX(  0, 150,   0);
  float xscreeny = screenY(  0, 150,   0);
  float yscreenx = screenX(125,   0,   0);
  float yscreeny = screenY(125,   0,   0);
  float zscreenx = screenX(  0,   0, 127);
  float zscreeny = screenY(  0,   0, 127);
  
  popMatrix(); 

  drawType("X", xscreenx, xscreeny, 0.9, 0.5, 0.5);
  drawType("Y", yscreenx, yscreeny, 0.0, 0.7, 0.0);
  drawType("Z", zscreenx, zscreeny, 0.4, 0.6, 1.0);


  pushMatrix(); 
  translate(width/1.8, height/8., -30); 

  rotateX(radians(90.-pitch)); 
  rotateY(radians(-roll));
  rotateZ(radians(-yaw));

  strokeWeight(5.);

  stroke(0., 1., 0.);
  line(0., 0., 0., 200.*accel[1], 0.,          0.);
  
  stroke(1., 0.6, 0.6);
  line(0., 0., 0.,   0.,        200.*accel[0], 0.);
  
  stroke(0.4, 0.6, 1.);
  line(0., 0., 0.,   0.,          0.,        200.*accel[2]);

  strokeWeight(0.5);

/*
  fill(0., 1., 0.);
  if (accel[1] < 0.) flipAccelDirection = -1; else flipAccelDirection = 1;
  make3DArrow(100.*accel[1]*flipAccelDirection, 10., 1.*flipAccelDirection, 0.,                    0.                   );

  fill(1, 0.6, 0.6);
  if (accel[0] < 0.) flipAccelDirection = -1; else flipAccelDirection = 1;
  make3DArrow(100.*accel[0]*flipAccelDirection, 10., 0.,                    1.*flipAccelDirection, 0.                   );
    
  fill(0.4, 0.6, 1);
  if (accel[2] < 0.) flipAccelDirection = -1; else flipAccelDirection = 1;
  make3DArrow(100.*accel[2]*flipAccelDirection, 10., 0.,                    0.,                    1.*flipAccelDirection);
*/

  popMatrix();  
  
/*
  stroke(0.9, 0.5, 0.5);
  if (accel[0] < 0.) flipAccelDirection = -1; else flipAccelDirection = 1;
  line(xscreenx +       zdirx*0.2 + ydirx*0.2 + xdirx* 0.11,                        xscreeny - 20. + zdiry*0.2 + ydiry*0.2 + xdiry* 0.11, 
       xscreenx +       zdirx*0.2 + ydirx*0.2 + xdirx*(0.11 +  accel[0]*50./xdist), xscreeny - 20. + zdiry*0.2 + ydiry*0.2 + xdiry*(0.11 + accel[0]*50./xdist));
  line(xscreenx +       zdirx*0.2 + ydirx*0.2 + xdirx* 0.11,                        xscreeny - 10. + zdiry*0.2 + ydiry*0.2 + xdiry* 0.11, 
       xscreenx +       zdirx*0.2 + ydirx*0.2 + xdirx*(0.11 +  accel[0]*50./xdist), xscreeny - 10. + zdiry*0.2 + ydiry*0.2 + xdiry*(0.11 + accel[0]*50./xdist));
  line(xscreenx +       zdirx*0.2 + ydirx*0.2 + xdirx*(0.11 +  accel[0]*50./xdist), xscreeny - 25. + zdiry*0.2 + ydiry*0.2 + xdiry*(0.11 + accel[0]*50./xdist),
       xscreenx +       zdirx*0.2 + ydirx*0.2 + xdirx*(0.11 + (accel[0]*50.+flipAccelDirection*15.)/xdist), 
       xscreeny - 15. + zdiry*0.2 + ydiry*0.2 + xdiry*(0.11 + (accel[0]*50.+flipAccelDirection*15.)/xdist));
  line(xscreenx +       zdirx*0.2 + ydirx*0.2 + xdirx*(0.11 +  accel[0]*50./xdist), xscreeny -  5. + zdiry*0.2 + ydiry*0.2 + xdiry*(0.11 + accel[0]*50./xdist),
       xscreenx +       zdirx*0.2 + ydirx*0.2 + xdirx*(0.11 + (accel[0]*50.+flipAccelDirection*15.)/xdist), 
       xscreeny - 15. + zdiry*0.2 + ydiry*0.2 + xdiry*(0.11 + (accel[0]*50.+flipAccelDirection*15.)/xdist));
  
  stroke(0.0, 0.7, 0.0);
  if (accel[1] < 0.) flipAccelDirection = -1; else flipAccelDirection = 1;
  line(yscreenx +       zdirx*0.2 + xdirx*0.2 + ydirx* 0.11,                        yscreeny +       zdiry*0.2 + xdiry*0.2 + ydiry* 0.11, 
       yscreenx +       zdirx*0.2 + xdirx*0.2 + ydirx*(0.11 +  accel[1]*50./ydist), yscreeny +       zdiry*0.2 + xdiry*0.2 + ydiry*(0.11 + accel[1]*50./ydist));
  line(yscreenx +       zdirx*0.2 + xdirx*0.2 + ydirx* 0.11,                        yscreeny + 10. + zdiry*0.2 + xdiry*0.2 + ydiry* 0.11, 
       yscreenx +       zdirx*0.2 + xdirx*0.2 + ydirx*(0.11 +  accel[1]*50./ydist), yscreeny + 10. + zdiry*0.2 + xdiry*0.2 + ydiry*(0.11 + accel[1]*50./ydist));
  line(yscreenx +       zdirx*0.2 + xdirx*0.2 + ydirx*(0.11 +  accel[1]*50./ydist), yscreeny -  5. + zdiry*0.2 + xdiry*0.2 + ydiry*(0.11 + accel[1]*50./ydist),
       yscreenx +       zdirx*0.2 + xdirx*0.2 + ydirx*(0.11 + (accel[1]*50.+flipAccelDirection*15.)/ydist), 
       yscreeny +  5. + zdiry*0.2 + xdiry*0.2 + ydiry*(0.11 + (accel[1]*50.+flipAccelDirection*15.)/ydist));
  line(yscreenx +       zdirx*0.2 + xdirx*0.2 + ydirx*(0.11 +  accel[1]*50./ydist), yscreeny + 15. + zdiry*0.2 + xdiry*0.2 + ydiry*(0.11 + accel[1]*50./ydist),
       yscreenx +       zdirx*0.2 + xdirx*0.2 + ydirx*(0.11 + (accel[1]*50.+flipAccelDirection*15.)/ydist), 
       yscreeny +  5. + zdiry*0.2 + xdiry*0.2 + ydiry*(0.11 + (accel[1]*50.+flipAccelDirection*15.)/ydist));

  stroke(0.4, 0.6, 1.0);
  if (accel[2] < 0.) flipAccelDirection = -1; else flipAccelDirection = 1;
  line(zscreenx +       ydirx*0.2 + xdirx*0.2 + zdirx* 0.11,               zscreeny +       ydiry*0.2 + xdiry*0.2 + zdiry* 0.11, 
       zscreenx +       ydirx*0.2 + xdirx*0.2 + zdirx*(0.11 +  accel[2]*50./zdist), zscreeny +       ydiry*0.2 + xdiry*0.2 + zdiry*(0.11 + accel[2]*50./zdist));
  line(zscreenx + 10. + ydirx*0.2 + xdirx*0.2 + zdirx* 0.11,               zscreeny +       ydiry*0.2 + xdiry*0.2 + zdiry* 0.11, 
       zscreenx + 10. + ydirx*0.2 + xdirx*0.2 + zdirx*(0.11 +  accel[2]*50./zdist), zscreeny +       ydiry*0.2 + xdiry*0.2 + zdiry*(0.11 + accel[2]*50./zdist));
  line(zscreenx -  5. + ydirx*0.2 + xdirx*0.2 + zdirx*(0.11 +  accel[2]*50./zdist), zscreeny +       ydiry*0.2 + xdiry*0.2 + zdiry*(0.11 + accel[2]*50./zdist),
       zscreenx +  5. + ydirx*0.2 + xdirx*0.2 + zdirx*(0.11 + (accel[2]*50.+flipAccelDirection*15.)/zdist), 
       zscreeny +       ydiry*0.2 + xdiry*0.2 + zdiry*(0.11 + (accel[2]*50.+flipAccelDirection*15.)/zdist));
  line(zscreenx + 15. + ydirx*0.2 + xdirx*0.2 + zdirx*(0.11 +  accel[2]*50./zdist), zscreeny +       ydiry*0.2 + xdiry*0.2 + zdiry*(0.11 + accel[2]*50./zdist),
       zscreenx +  5. + ydirx*0.2 + xdirx*0.2 + zdirx*(0.11 + (accel[2]*50.+flipAccelDirection*15.)/zdist), 
       zscreeny +       ydiry*0.2 + xdiry*0.2 + zdiry*(0.11 + (accel[2]*50.+flipAccelDirection*15.)/zdist));
*/

  // print out text containing orientation, acceleration, and health info
  drawType("Yaw:  ",             850.,  40., 0.,                        0.,                            0.);
  textAlign(RIGHT);
  drawType(nf(int(yaw)),         945.,  40., 0.,                        0.7,                           0.);
  textFont(smallFont);
  drawType("o",                  952.,  33., 0.,                        0.,                            0.);
  textFont(mainFont);
  textAlign(LEFT);
  drawType("Pitch:",             850.,  70., 0.,                        0.,                            0.);
  textAlign(RIGHT);
  drawType(nf(int(pitch)),       945.,  70., 0.,                        0.7,                           0.);
  textFont(smallFont);
  drawType("o",                  952.,  63., 0.,                        0.,                            0.);
  textFont(mainFont);
  textAlign(LEFT);
  drawType("Roll: ",             850., 100., 0.,                        0.,                            0.);
  textAlign(RIGHT);
  drawType(nf(int(roll)),        945., 100., 0.,                        0.7,                           0.);
  textFont(smallFont);
  drawType("o",                  952.,  93., 0.,                        0.,                            0.);
  textFont(mainFont);
  textAlign(LEFT);

  drawType("Axle Rotation",      990.,  20., 0.,                        0.,                            0.);
  drawType("Servo set at:",      990.,  70., 0.,                        0.,                            0.);
  textAlign(RIGHT);
  drawType(nf(setting[5],1,1),  1155.,  70., 0.,                        0.7,                           0.);
  textAlign(LEFT);
  drawType("/10",               1155.,  70., 0.,                        0.,                            0.);
  makeButtons(                   972.,  52., 5);
  drawType("Meas rot ang:  ",    990., 100., 0.,                        0.,                            0.);
  textAlign(RIGHT);
  drawType(nf(int(rotAng)),     1150., 100., 0.,                        0.7,                           0.);
  textAlign(LEFT);
  textFont(smallFont);
  drawType("o",                 1150.,  93., 0.,                        0.,                            0.);
  textFont(mainFont);
  drawType("Accel  :",           430.,  40., 0.,                        0.,                            0.);
  drawType("z",                  490.,  40., 0.4,                       0.6,                           1.0);
  textAlign(RIGHT);
  drawType(nfp(accel[2],1,1),    555.,  40., 0.,                        0.7,                           0.);
  textAlign(LEFT);
  drawType("m/s",                560.,  40., 0.,                        0.,                            0.);
  textFont(smallFont);
  drawType("2",                  589.,  33., 0.,                        0.,                            0.);
  textFont(mainFont);
  drawType("Accel  :",           430.,  70., 0.,                        0.,                            0.);
  drawType("x",                  490.,  70., 0.9,                       0.5,                           0.5);
  textAlign(RIGHT);
  drawType(nfp(accel[0],1,1),    555.,  70., 0.,                        0.7,                           0.);
  textAlign(LEFT);
  drawType("m/s",                560.,  70., 0.,                        0.,                            0.);
  textFont(smallFont);
  drawType("2",                  589.,  63., 0.,                        0.,                            0.);
  textFont(mainFont);
  drawType("Accel  :",           430., 100., 0.,                        0.,                            0.);
  drawType("y",                  490., 100., 0.,                        0.7,                           0.);
  textAlign(RIGHT);
  drawType(nfp(accel[1],1,1),    555., 100., 0.,                        0.7,                           0.);
  textAlign(LEFT);
  drawType("m/s",                560., 100., 0.,                        0.,                            0.);
  textFont(smallFont);
  drawType("2",                  589.,  93., 0.,                        0.,                            0.);
  textFont(mainFont);
  drawType("Accel",              637.,  20., 0.,                        0.,                            0.);
  drawType("vectors",            627.,  40., 0.,                        0.,                            0.);
  drawType("Orientation",        723.,  20., 0.,                        0.,                            0.);

  drawType("Orientation sensor", 430., 260., UM7health,                 0.,                            0.);
  drawType("health: ",           430., 285., UM7health,                 0.,                            0.);
  if (UM7health == 0) {
    alarmOn[20] = 0;
    drawType("OK",               515., 285., 0.,                        0.7,                           0.);
  } else {
    if (alarmOn[20] == 0) alarmOn[20] = 2;
    drawType("PROBLEM",          515., 285., 1.,                        0.,                            0.);
  }

  if (UM7temp > maxTemp || UM7temp < minTemp) if (alarmOn[21] == 0) alarmOn[21] = 2;
  else                                        alarmOn[21] = 0;
  float fillHeight = 40.*((UM7temp - minTemp)/((maxTemp - minTemp)*1.2));
  fill(color(1.,                           1.,                            1.));
  ellipse(685.,  270.                  , 4., 40.);
  noStroke();
  fill(color(1.*(alarmOn[21] > 0 ? 1 : 0), 0.7*(alarmOn[21] > 0 ? 0 : 1), 0.));
  ellipse(685.5, 288.5  - fillHeight/2., 3., fillHeight);    
  stroke(0);
  drawType("Orientation sensor", 697., 260., (alarmOn[21] > 0 ? 1 : 0), 0.,                            0.);
  drawType("temp: ",             697., 285., (alarmOn[21] > 0 ? 1 : 0), 0.,                            0.);
  textAlign(RIGHT);
  drawType(nf(int(UM7temp)),     777., 285., (alarmOn[21] > 0 ? 1 : 0), 0.7*(alarmOn[21] > 0 ? 0 : 1), 0.);
  textFont(smallFont);
  drawType(nf(int(maxTemp)),     681., 261., 0.35,                      0.35,                          0.);    
  drawType(nf(int(minTemp)),     681., 291., 0.35,                      0.35,                          0.);
  textFont(mainFont);
  textAlign(LEFT);
  drawType("C",                  782., 285., 0.,                        0.,                            0.);

}

void drawType(String theText, float x, float y, float r, float g, float b) {
  fill(r, g, b);
  text(theText, x, y);
}

void makeCylinder(float x, float y, float radius, float tall) {
  arc(x, y+tall, 2.*radius, 0.35*radius, 0, PI);
  line(x-radius, y, x-radius, y+tall);
  line(x+radius, y, x+radius, y+tall);
  noStroke();
  rect(x-radius, y, 2.*radius, tall);
  stroke(0);
  ellipse(x, y, 2.*radius, 0.35*radius);
}

void make3DArrow(float len, float fat, float x, float y, float z) {
  beginShape(QUADS);

  vertex(fat*x,        fat + len*y,  fat + len*z);
  vertex(fat + len*x,  fat + len*y,  fat + len*z);
  vertex(fat + len*x,  fat*y,        fat + len*z);
  vertex(fat*x,        fat*y,        fat + len*z);

  vertex(fat + len*x,  fat + len*y,  fat + len*z);
  vertex(fat + len*x,  fat + len*y,  fat*z      );
  vertex(fat + len*x,  fat*y,        fat*z      );
  vertex(fat + len*x,  fat*y,        fat + len*z);

  vertex(fat + len*x,  fat + len*y,  fat*z      );
  vertex(fat*x,        fat + len*y,  fat*z      );
  vertex(fat*x,        fat*y,        fat*z      );
  vertex(fat + len*x,  fat*y,        fat*z      );

  vertex(fat*x,        fat + len*y,  fat*z      );
  vertex(fat*x,        fat + len*y,  fat + len*z);
  vertex(fat*x,        fat*y,        fat + len*z);
  vertex(fat*x,        fat*y,        fat*z      );

  vertex(fat*x,        fat + len*y,  fat*z      );
  vertex(fat + len*x,  fat + len*y,  fat*z      );
  vertex(fat + len*x,  fat + len*y,  fat + len*z);
  vertex(fat*x,        fat + len*y,  fat + len*z);

  vertex(fat*x,        fat*y,        fat*z      );
  vertex(fat + len*x,  fat*y,        fat*z      );
  vertex(fat + len*x,  fat*y,        fat + len*z);
  vertex(fat*x,        fat*y,        fat + len*z);

  endShape();
  
  beginShape(TRIANGLE_FAN);
  
  vertex( 0.5*fat + len*x + 2.5*fat*x,  0.5*fat + len*y + 2.5*fat*y,             0.5*fat + len*z + 2.5*fat*z);
  vertex( 1.5*fat + len*x - 0.5*fat*x,  1.5*fat + len*y - 0.5*fat*y,             1.5*fat + len*z - 0.5*fat*z);
  vertex(-0.5*fat + len*x + 1.5*fat*x,  1.5*fat + len*y - 0.5*fat*y - 2.*fat*x,  1.5*fat + len*z - 0.5*fat*z);
  vertex(-0.5*fat + len*x + 1.5*fat*x, -0.5*fat + len*y + 1.5*fat*y,            -0.5*fat + len*z + 1.5*fat*z);
  vertex( 1.5*fat + len*x - 0.5*fat*x,  1.5*fat + len*y - 0.5*fat*y - 2.*fat*z, -0.5*fat + len*z + 1.5*fat*z);
  vertex( 1.5*fat + len*x - 0.5*fat*x,  1.5*fat + len*y - 0.5*fat*y,             1.5*fat + len*z - 0.5*fat*z);
  
  endShape();  
}

void makeButtons(float x, float y, int servoNum) {
  if (mouseX > x && mouseX < x+buttonSquareSize && 
      mouseY > y && mouseY < y+buttonSquareSize) {
    if (mousePressed && !blockButtons) {
      fill(0., 0.7, 0.);
      if (!testArduinoUnconnected) {
        port.write('m'); port.write('s');    // 'm'odify 's'ervo (ensure we avoid modifications due to noise)
        port.write('A'+servoNum);            // write 'A' if servo 0, 'B' if servo 1, etc.
      }
      print("Sent command to raise control setting # "); println(servoNum);
      blockButtons = true;
      thread("buttonBlock");
    } else {
      fill(0., 0., 0.);
    }
    stroke(1., 1., 1.);
  } else {
    fill(1., 1., 1.);
    stroke(0., 0., 0.);
  }
  rect(x,                        y,                                      buttonSquareSize,      buttonSquareSize, buttonSqCornerRadius);
  line(x+   buttonSquareSize/4., y+5.*buttonSquareSize/8.,               x+buttonSquareSize/2., y+3.*buttonSquareSize/8.              );
  line(x+3.*buttonSquareSize/4., y+5.*buttonSquareSize/8.,               x+buttonSquareSize/2., y+3.*buttonSquareSize/8.              );
  if (mouseX > x && mouseX < x+buttonSquareSize && 
      mouseY > y+buttonSpacing && mouseY < y+buttonSpacing+buttonSquareSize) {
    if (mousePressed && !blockButtons) {
      fill(0., 0.7, 0.);
      if (!testArduinoUnconnected) {
        port.write('m'); port.write('s');     // 'm'odify 's'ervo (ensure we avoid modifications due to noise)
        port.write('a'+servoNum);             // write 'a' if servo 0, 'b' if servo 1, etc.
      }
      print("Sent command to lower control setting # "); println(servoNum);
      blockButtons = true;
      thread("buttonBlock");
    } else {
      fill(0., 0., 0.);
    }
    stroke(1., 1., 1.);
  } else {
    fill(1., 1., 1.);
    stroke(0., 0., 0.);
  }
  rect(x,                        y+buttonSpacing,                        buttonSquareSize,      buttonSquareSize, buttonSqCornerRadius);
  line(x+   buttonSquareSize/4., y+3.*buttonSquareSize/8.+buttonSpacing, x+buttonSquareSize/2., y+5.*buttonSquareSize/8.+buttonSpacing);
  line(x+3.*buttonSquareSize/4., y+3.*buttonSquareSize/8.+buttonSpacing, x+buttonSquareSize/2., y+5.*buttonSquareSize/8.+buttonSpacing);
  stroke(0., 0., 0.);
}

void makePanicButton() {
  final float x = 545., y = 525.;
  final float xSize = 80., ySize = 40., cornerRadius = 15.;
  if (mouseX > x && mouseX < x+xSize && 
      mouseY > y && mouseY < y+ySize) {
    if (mousePressed && !blockButtons) {
      fill(1.,0.,0.);
      if (!testArduinoUnconnected) {
        port.write('m'); port.write('s');      // 'm'odify 's'ervo (ensure we avoid modifications due to noise)
        port.write('x');                       // 'x' = shut em all down
      }
      println("Pressed PANIC button! -- shutting down all motors.");
      blockButtons = true;
      thread("buttonBlock");
    } else {
      fill(0., 0., 0.);
    }
    stroke(1., 1., 1.);
  } else {
    fill(1., 0.8, 0.8);
    stroke(1., 0., 0.);
  }
  rect(x,                        y,                                      xSize,      ySize, cornerRadius);  
  drawType("PANIC!",             x+13.,  y+25., 1.,      0.,                 0.);
  stroke(0., 0., 0.);
}

void makeSilenceAlarmsButton() {
  final float x = 1020., y = 280.;
  final float xSize = 110., ySize = 65., cornerRadius = 15.;

  if (mouseX > x && mouseX < x+xSize && 
      mouseY > y && mouseY < y+ySize) {
    if (mousePressed && !blockButtons) {
      fill(1.,0.,0.);
      println("Silencing alarms.");
      for (int i = 0; i < numAlarms; ++i) {
        if (alarmOn[i] == 2) alarmOn[i] = 1;
      }
      blockButtons = true;
      thread("buttonBlock");
    } else {
      fill(1., 1., 0.);
    }
    stroke(1., 1., 1.);
  } else {
    fill(1., 1., 0.7);
    stroke(0., 0., 0.);
  }

  rect(x,                        y,                                      xSize,      ySize, cornerRadius);  
  drawType("Silence",            x+20.,  y+20., 0.,      0.,                 0.);
  drawType("present",            x+20.,  y+38., 0.,      0.,                 0.);
  drawType("alarms",             x+26.,  y+56., 0.,      0.,                 0.);
  stroke(0., 0., 0.);  
}

void buttonBlock() {
   delay(300);
   blockButtons = false;
}

boolean anyAlarmIsOn() {
  for (int i = 0; i < numAlarms; ++i) {
    if ( alarmOn[i] == 2 ) return true;
  }
  return false;
}

void soundAlarm() {
  if (alarmCounter % timeBetweenAlarmSounds == 0) alarm.play();
  ++alarmCounter;
}

void setFakeAltairValues() {
  for (int i = 0; i < 7; ++i) setting[i] = 2.9;
  for (int i = 0; i < 4; ++i) rpm[i]     = 5134.;
  for (int i = 0; i < 4; ++i) current[i] = 0.;
  for (int i = 0; i < 8; ++i) temp[i]    = 0.;
  for (int i = 0; i < 3; ++i) accel[i]   = 0.;
  temp[5] = 90.;
  accel[2] = -0.5;
  accel[0] = -0.5;
  rotAng = 30.;
  rpm[2] = 0.;
  yaw = 0.; pitch = 0.; roll = 0.;
  UM7health = 0.; UM7temp = 0.;    
}
