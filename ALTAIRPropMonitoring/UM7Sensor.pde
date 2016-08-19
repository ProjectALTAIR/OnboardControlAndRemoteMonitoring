class UM7Sensor{
  private final float maxTemp = 50; //Degrees C
  private final float minTemp = -50; //Degrees C
  
  float[] accel = new float[3];
  float yaw, pitch, roll;
  float health, temp;
  byte alarmOn; //0 for off, 1 for on but silenced, 2 for on
  
  private int x, y;
  private FillBar tempBar;
  
  public UM7Sensor(int x, int y){
    this.x = x;
    this.y = y;
    accel[0] = accel[2] = -.5; //Test Value
    
    tempBar = new FillBar(x-293, y+190, minTemp, maxTemp, "UM7 Temp: ", " C", false);
  }
  public void updateData(float yaw, float pitch, float roll, float health, float temp, float[] accel){
    this.yaw = yaw;             this.pitch = pitch;       this.roll = roll;
    this.health = health;       this.temp = temp;
    for (int i = 0; i < 3; i++) this.accel[i] = accel[i];
    
    if ((temp > maxTemp) || (temp < minTemp) || (health > 0)) {
      if(alarmOn != 1) alarmOn = 2;
    }
    else alarmOn = 0;
  }
  public void drawUM7(){
    textAlign(LEFT);
    textFont(mainFont);
    fill(0,0,0);
    text("Orientation",x,    y);
    text("vectors",    x-96, y+20);
    text("Accel",      x-86, y);
    
    rotationLabel("Yaw: ",   yaw,   x+127, y+20);
    rotationLabel("Pitch: ", pitch, x+127, y+50);
    rotationLabel("Roll: ",  roll,  x+127, y+80);
    
    accelLabel("z", accel[2], x-293, y+20,    .4, .6, 1.0);
    accelLabel("x", accel[1], x-293, y+50,    .9, .5,  .5);
    accelLabel("y", accel[0], x-293, y+80,     0, .7,   0);
    
    drawUM7health(health, x-293, y+120);
    tempBar.drawBar(temp);
    drawArrows();
  }
  private void rotationLabel(String label, float angle, int x, int y){
    textFont(mainFont); textAlign(LEFT);
    fill(0,0,0); text(label,x,y);
    
    textFont(smallFont);
    text("o",x+80,y-7);
    
    //value draw
    textFont(mainFont); textAlign(RIGHT);
    fill(0,.7,0); text(int(angle),x+80,y);
  }
  private void accelLabel(String label, float accel, int x, int y, float r, float g, float b){
    textFont(mainFont); textAlign(LEFT);
    fill(0,0,0); text("Accel: ", x, y); 
                 text("m/s", x+140, y);
    fill(r,g,b); text(label, x+textWidth("Accel: "), y);
    
    textFont(smallFont);
    text("2", x+167, y-6);
    
    //value draw
    textFont(mainFont); textAlign(RIGHT);
    fill(0,.7,0);
    text(nfp(accel,1,1), x+138, y);
  }
  private void drawUM7health(float value, int x, int y){
    textFont(mainFont); textAlign(LEFT);
    fill(0,0,0);  text("Orientation Sensor",x,y);
                  text("Health: ", x, y+25);
                  
    if(value > 0) { fill(1,0,0);  text("PROBLEM", x+textWidth("Health: ")+10, y+25); }
    else          { fill(0,.7,0); text("OK",      x+textWidth("Health: ")+10, y+25); }  
  }
  private void drawArrows(){
    pushMatrix(); 
    translate(x+27, y+166, -30); 

    rotateX(radians(90.-pitch)); 
    rotateY(radians(-roll));
    rotateZ(radians(-yaw));
  
    fill(0.0, 1.0, 0.0); make3DArrow(90., 10., 1., 0., 0.);
    fill(1.0, 0.6, 0.6); make3DArrow(90., 10., 0., 1., 0.);
    fill(0.4, 0.6, 1.0); make3DArrow(90., 10., 0., 0., 1.);

    float xscreenx = screenX(  0, 150,   0);
    float xscreeny = screenY(  0, 150,   0);
    float yscreenx = screenX(125,   0,   0);
    float yscreeny = screenY(125,   0,   0);
    float zscreenx = screenX(  0,   0, 127);
    float zscreeny = screenY(  0,   0, 127);
  
    popMatrix(); 

    fill(0.9, 0.5, 0.5); text("X", xscreenx, xscreeny);
    fill(0.0, 0.7, 0.0); text("Y", yscreenx, yscreeny);
    fill(0.4, 0.6, 1.0); text("Z", zscreenx, zscreeny);

    pushMatrix(); 
    translate(x-56, y+61, -30); 

    rotateX(radians(90.-pitch)); 
    rotateY(radians(-roll));
    rotateZ(radians(-yaw));

    strokeWeight(5.);

    stroke(0.0, 1.0, 0.0); line(0, 0, 0, 200*accel[1],            0,            0);
    stroke(1.0, 0.6, 0.6); line(0, 0, 0,            0, 200*accel[0],            0);
    stroke(0.4, 0.6, 1.0); line(0, 0, 0,            0,            0, 200*accel[2]);

    strokeWeight(0.5);
    popMatrix();
  }
  private void make3DArrow(float len, float fat, float x, float y, float z) {
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
}