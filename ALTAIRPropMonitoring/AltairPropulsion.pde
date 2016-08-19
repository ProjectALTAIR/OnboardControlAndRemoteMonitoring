class AltairPropulsion{
  private final int timeBetweenAlarmSounds = 40;    // in approximately 20's of milliseconds (so e.g. a value of 30 ~= 0.6 seconds)
  
  Propeller[]       props = new Propeller[4];
  SpeedController[] ecss  = new SpeedController[4];
  Axle axle;
  UM7Sensor UM7;
  
  private Button panic;
  private Button silenceAlarms;
  private byte alarmCounter;
  private int x, y;
  
  public AltairPropulsion(int x, int y){
    this.x = x;
    this.y = y;
    for (int i = 0; i < 4; i++){
      props[i] = new Propeller(      x+4  + 210*i + 260*(i>1?1:0),  y+6 - 20*(i==0||i==3?1:0), i);
      ecss[i]  = new SpeedController(x+90 + 195*i + 105*(i>1?1:0),  y-6);
    }
    axle = new Axle(1050, 160);    //x + 915, y - 240 for the same position but relative to the rest of the module;
    UM7  = new UM7Sensor(723, 20); //Recommend not moving these 2 GUI components since the perspective will shift.
    
    panic = new Button(GUI[0], "Panic"); //The word "Panic" is what recognizes the method Panic() for the button
    panic.setColorBackground(color(1,0,0));
    panic.setColorForeground(color(.7,0,0));
    panic.setColorActive    (color(.7,.7,.7));
    panic.setSize(80,70);
    panic.setPosition(x + 410, y + 110);
    
    silenceAlarms = new Button(GUI[0], "SilenceAlarms");
    silenceAlarms.setLabel("Silence Alarms");
    silenceAlarms.setColorBackground  (color(1,  1, 0));
    silenceAlarms.setColorForeground  (color(.7,.7, 0));
    silenceAlarms.setColorActive      (color(.7,.7,.7));
    silenceAlarms.setColorCaptionLabel(color(.5,.5, 0));
    silenceAlarms.setSize(150,40);
    silenceAlarms.setPosition(x + 900 , y - 100);
    silenceAlarms.setVisible(false);
  }
  public void drawAltair(){
    fill(0.8);
    noStroke();
    rect(x, y, 900, 5);
    stroke(0);
    strokeWeight(0.7);
    makeCylinder(x+450, y-20, 65, 85);
    
    for (Propeller prop : props)     prop.drawProp();
    for (SpeedController ecs : ecss) ecs.drawSpeedController();
    axle.drawAxle();
    UM7.drawUM7();
  }
  public void updateData(float[] setting, float[] rpm, float[] current, float[] temp, float[] accel, float yaw, float pitch, float roll, float UM7Health, float UM7temp, float rotAng){
    for(int i = 0; i < 4; i++){
      props[i].updateData(rpm[i],temp[i],current[i],setting[i]);
      ecss[i].updateData(temp[i+4],current[i]);
    }
    axle.updateData(setting[5],rotAng,rpm);
    UM7.updateData(yaw,pitch,roll,UM7Health,UM7temp,accel);
  }
  //Alarm Functions
  private boolean anyAlarmOn(){
    for(Propeller prop : props)     if(prop.alarmOn == 2) return true;
    for(SpeedController ecs : ecss) if(ecs.alarmOn == 2) return true;
    if(UM7.alarmOn == 2) return true;
    else if(axle.alarmOn ==2) return true;
    else return false;
  }
  public void silenceAllAlarms(){
    for(Propeller prop : props)     if(prop.alarmOn == 2) prop.alarmOn = 1;
    for(SpeedController ecs : ecss) if(ecs.alarmOn == 2) ecs.alarmOn = 1;
    if(UM7.alarmOn == 2) UM7.alarmOn = 1;
    if(axle.alarmOn == 2) axle.alarmOn = 1;
  }
  public void soundAlarmIfOn() {
    if (anyAlarmOn()){
      silenceAlarms.show();
      if (alarmCounter % timeBetweenAlarmSounds == 0) alarm.play();
      ++alarmCounter;
    }
    else silenceAlarms.hide();
  }
}