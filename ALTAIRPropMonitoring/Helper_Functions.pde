//Beginning of button methods
//These get called automatically when their correspponding button is pressed.
void Panic(){
  println("The panic button was pressed. Shutting down all motors...");
  sendArduino('x');
}
void SilenceAlarms(){
  println("The alarms have been silenced.");
  module.silenceAllAlarms();
}
void increaseSetting(int value){
  if (inSetup) return;
  char[] commands = {'A','B','C','D','E','F'};
  sendArduino(commands[value]);
  println("Sent command to raise control setting: " + value + " (" + commands[value] + ")");
}
void decreaseSetting(int value){
  if (inSetup) return;
  char[] commands = {'a','b','c','d','e','f'};
  sendArduino(commands[value]);
  println("Sent command to lower control setting: " + value + " (" + commands[value] + ")");
}
//End of button methods 

boolean sendArduino(char command){
  if (isArduinoConnected) {
    port.write('m'); port.write('s');    // 'm'odify 's'etting (ensure we avoid modifications due to noise)
    port.write(command);                 // command = a charater representing the action for the arduino to do
    return true;                         // command send
  }
  else {
    //println("Sorry, Arduino is not connected");
    return false;                        // command failed to send
  }
}

String findSubstring(String[] stringList, String substring) {
  for (int i = 0; i < stringList.length; ++i) {
    if (stringList[i].indexOf(substring) != -1) return stringList[i];
  }
  return "";
}

//Drawing methods and classes
void makeCylinder(float x, float y, float radius, float tall) {
  arc(x, y+tall, 2.*radius, 0.35*radius, 0, PI);
  line(x-radius, y, x-radius, y+tall);
  line(x+radius, y, x+radius, y+tall);
  noStroke();
  rect(x-radius, y, 2.*radius, tall);
  stroke(0);
  ellipse(x, y, 2.*radius, 0.35*radius);
}

class FillBar{
  int x, y;
  float value;
  private float max, min;
  private boolean isRed;
  private String valueName;
  private String valueUnit;
  private boolean displayDecimals; //Prints a float if true, an int if false
  
  public FillBar(int x, int y, float min, float max, String valueName, String valueUnit, boolean displayDecimals){
    this.x = x;     this.y = y;
    this.min = min; this.max = max;
    this.valueName = valueName;
    this.valueUnit = valueUnit;
    this.displayDecimals = displayDecimals;
  }
  public void drawBar(float value){
    String drawValue = displayDecimals? str(value) : str(int(value));
    isRed = (value > max) || (value < min);
    //draw the Bar
    stroke(0);
    fill(color(1.,1.,1.));
    ellipse(x, y, 4, 40);
    noStroke();
    fill(color(1.*(isRed? 1 : 0), 0.7*(isRed? 0 : 1), 0));
    float fillHeight = constrain(40.*(value - min)/(max - min),-5,40);
    ellipse(x, y + 20 - fillHeight/2, 4, fillHeight);
    stroke(0);
    
    //draws the min and max labels
    textFont(smallFont); textAlign(RIGHT);
    fill(.35,.35,0);
    text(str(int(min)),x-5,y+21);
    text(str(int(max)),x-5,y-15);
    
    //draws the label, value, and unit
    fill((isRed?1:0),0,0);
    textFont(mainFont); textAlign(LEFT);
    text(valueName,x+8,y+6);
    text(valueUnit, x+8+textWidth(valueName)+textWidth(drawValue),y+6);
    fill((isRed?1:0),(isRed?0:.7),0);
    text(drawValue, x+8+textWidth(valueName),                     y+6);
  }
}