class Axle{
  float servoSetting;
  float measRotAng;
  byte alarmOn;
  
  private float maxSetting = 10;
  private float propRotAng[] = new float[2];
  private boolean isSpinningUpper;
  private boolean isSpinningLower;
  private boolean isRedUpper;
  private boolean isRedLower;
  
  private Button increaseServo;
  private Button decreaseServo;
  private int x, y;
  
  public Axle(int x, int y){
    this.x = x;
    this.y = y;
    
    increaseServo = new Button(GUI[4], "increaseSetting");
    increaseServo.setValue(5);
    increaseServo.setLabel("↑");
    increaseServo.setSize(12,18);
    increaseServo.setPosition(x-75,y-115);
    
    decreaseServo = new Button(GUI[4], "decreaseSetting");
    decreaseServo.setValue(5);
    decreaseServo.setLabel("↓");
    decreaseServo.setSize(12,18);
    decreaseServo.setPosition(x-75,y-95);
  }
  public void updateData(float servoSetting, float measRotAng, float[] rpm){
    this.servoSetting = servoSetting;
    this.measRotAng = measRotAng;
    
    isSpinningUpper = (rpm[0] > 0) || (rpm[3] > 0);
    isSpinningLower = (rpm[1] > 0) || (rpm[2] > 0);
    isRedUpper = (rpm[0] > 5000) || (rpm[3] > 5000);
    isRedLower = (rpm[1] > 5000) || (rpm[2] > 5000);
    
    if(servoSetting > maxSetting || servoSetting < 0){
      if (alarmOn != 1) alarmOn = 2;
    }
    else alarmOn = 0;
  }
  public void drawAxle(){
    noFill();
    makeCylinder(x, y, 65, 85);
    strokeWeight(0.5);
    //Text and Labels
    textFont(mainFont); textAlign(LEFT);
    fill(0,0,0); 
    text("Axle Rotation", x-60,y-140); text("Servo set at: ", x-60, y-90);
    text("Meas Rot Ang: ",x-60,y-60);
    textAlign(RIGHT);
    if(servoSetting > 10 || servoSetting < 0) fill(1,0,0); else fill(0,.7,0);
    text(nf(servoSetting,1,1),x+80,y-90);
    fill(0,0,0);
    text(str(int(measRotAng)),x+80,y-60);
    textAlign(LEFT);
    text("/10",x+80,y-90);
    textFont(smallFont);
    text("o",x+81,y-67);
    
    axleAnimation();
  }
  private void axleAnimation(){
    pushMatrix();
    translate(x + 2, y + 30, 0);
    rotateZ(radians(measRotAng));
    fill(0.8);
    rect(-2, -2, 5, 5);   
    propShape(isRedUpper, isSpinningUpper, true);
    propShape(isRedLower, isSpinningLower, false);
    popMatrix();
  }
  private void propShape(boolean isRed, boolean isSpinning, boolean isUpper){
    fill(1, 0.5*(isRed?0:1), 0);
    makeCylinder(0, (isUpper?-17.5:4.5), 8, 15);
    pushMatrix();
    translate(0, 22.*(isUpper? -1:1));
    rotateX(radians(-28.));
    if (isSpinning) { rotateY(propRotAng[isUpper?0:1]); ++propRotAng[isUpper?0:1]; }
    fill((isRed?1:0), (isSpinning && !isRed?1:0), 0);
    
    beginShape(TRIANGLES);
    vertex(   0.,  0.,  0.);
    vertex(  75., -2., -2.);
    vertex(  75.,  2.,  2.);
    vertex(   0.,  0.,  0.);
    vertex( -75., -2.,  2.);
    vertex( -75.,  2., -2.);
    endShape();
    popMatrix();
  }
}