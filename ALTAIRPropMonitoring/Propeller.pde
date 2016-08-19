class Propeller {
  private final float minTemp = -50; //Degrees C
  private final float maxTemp = 50;  //Degrees C
  private final float maxRPM = 5000;
  private final float maxSetting = 10; //Max power setting for each motor: setting ranges from 0 - 10
  private final float maxCurrent = 10; //Amperes
  
  float rpm;
  float temp;
  float current;
  float powerSetting;
  byte alarmOn; // 0 == off, 1 == on but silenced, 2 == on (and not silenced)
  
  private Button powerIncrease;
  private Button powerDecrease;
  
  private int propNum;
  private boolean isRed;
  private boolean isRedRPM;
  private float propRotAng;
  private FillBar PowerSetBar;
  private FillBar RPMbar;
  private FillBar tempBar;
  private FillBar currentBar;
  private int x, y;
  
  public Propeller(int x, int y, int propNum){
    this.propNum = propNum;
    this.x = x;
    this.y = y;
    
    PowerSetBar = new FillBar(x-60, y+50 , 0,       maxSetting,     "Power: ", "/10" , true);
    RPMbar      = new FillBar(x-35, y+100, 0,       maxRPM,         "RPM: "  , ""    , false);
    tempBar     = new FillBar(x-35, y+150, minTemp, maxTemp,        "Temp: " , " C"  , false);
    currentBar  = new FillBar(x-35, y+200, 0,       maxCurrent,     "Curr: " , " A"  , false);
    
    powerIncrease = new Button(GUI[propNum], "increaseSetting");
    powerIncrease.setValue(propNum);
    powerIncrease.setLabel("↑");
    powerIncrease.setSize(12,18);
    powerIncrease.setPosition(x+80,y+30);
    
    powerDecrease = new Button(GUI[propNum], "decreaseSetting");
    powerDecrease.setValue(propNum);
    powerDecrease.setLabel("↓");
    powerDecrease.setSize(12,18);
    powerDecrease.setPosition(x+80,y+52);
  }
  public void updateData(float rpm, float temp, float current, float powerSetting){
    this.rpm = rpm;
    this.temp = temp;
    this.current = current;
    this.powerSetting = powerSetting;
    
    isRedRPM = (rpm > maxRPM);
    isRed = isRedRPM || (temp > maxTemp) || (temp < minTemp) || (powerSetting > maxSetting) || (powerSetting < 0) || (current > maxCurrent) || (current < 0);
    if (isRed) {
      if (alarmOn != 1) alarmOn = 2;
    }
    else alarmOn = 0;
  }
  public void drawProp(){
    fill(1.0, 0.5*(isRed?0:1), 0.0);
    makeCylinder(x, y, 8., 15.);
    fill(0,0,0);
    textFont(mainFont); textAlign(LEFT);
    text(propNum,x-5,y+15);
    
    propAnimation();
    
    PowerSetBar.drawBar(round(100*powerSetting)/100.);
    RPMbar.drawBar(rpm);
    tempBar.drawBar(temp);
    currentBar.drawBar(current);
  }  
  private void propAnimation(){
    pushMatrix();
    translate(x, y - 3 + 20*(propNum == 1 || propNum == 2? 1:0), 0);
    if (rpm > 1) { rotateY(propRotAng); ++propRotAng; }
    fill((isRedRPM?1:0), (!isRedRPM && rpm > 0 ? 1:0), 0);
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
}