class SpeedController{ //ESC <=> Electronic Speed Controller
  private final float minTemp = -50;   //Degrees C
  private final float maxTemp = 50;    //Degrees C
  private final float maxCurrent = 10; //Amperes
  
  float temp;    //Degrees C
  float current; //Amperes
  byte alarmOn;  // 0 == off, 1 == on but silenced, 2 == on (and not silenced)
  
  private int x, y;
  private boolean isRed;
  private FillBar tempBar;
  
  public SpeedController(int x, int y){
    this.x = x;
    this.y = y;
    
    tempBar = new FillBar(x - 35, y - 50, minTemp, maxTemp, "Temp: "," C", false);
  }
  public void updateData(float temp, float current){
    this.temp = temp;
    this.current = current;
    
    isRed = (temp > maxTemp) || (temp < minTemp) || (current > maxCurrent) || (current < 0);
    if(isRed) {
      if (alarmOn != 1) alarmOn = 2;
    }
    else alarmOn = 0;
  }
  public void drawSpeedController(){
    fill(1,(isRed?0:.5), 0);
    rect(x, y, 18, 6);
    
    tempBar.drawBar(temp);
  }
}