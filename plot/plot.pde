import processing.serial.*;

int lf = 10; // Linefeed(\n) in ASCII
String rawRCValues = null;
Serial myPort;

PFont f;

void setup() {
  size(550, 300);
  background(#FFFFFF);
  
  f = createFont("Arial", 10);
  textFont(f);
  
  String portName = Serial.list()[0];
  println("Porta: " + portName);
  myPort = new Serial(this, portName, 57600);
  myPort.clear();
  
  // Throw out the first reading, in case we started reading 
  // in the middle of a string from the sender.
  rawRCValues = myPort.readStringUntil(lf);
  rawRCValues = null;
}

void draw() {
  rawRCValues = readDataFromSerial();
  
  if (rawRCValues != null) {
    println(rawRCValues);
    String[] RCValues = split(rawRCValues, ";");
    
    if (RCValues.length < 5) {
      return;
    }
  
    float rcThrottle = float(RCValues[0]);
    rcThrottle = map(rcThrottle, 1000.0, 2000.0, 0.0, 100.0);
    
    float rcYaw = float(RCValues[1]);
    rcYaw = map(rcYaw, -150.0, 150.0, -50.0, 50.0);
    
    float rcPitch = float(RCValues[2]);
    rcPitch = map(rcPitch, -45.0, 45.0, -50, 50.0);
    
    float rcRoll = float(RCValues[3]);
    rcRoll = map(rcRoll, -45.0, 45.0, -50.0, 50.0);
    
    float rcOnOff = float(RCValues[4]);
    rcOnOff = map(rcOnOff, 0.0, 100.0, 0.0, 100.0);
    
    float rcDimmer = float(RCValues[5]);
    rcDimmer = map(rcDimmer, 0.0, 1000.0, 0.0, 100.0);
    
    float interval = float(RCValues[6]);
    interval = 1000000 / interval;
    
    background(#FFFFFF);
    stroke(#000000);
    fill(#000000);
        
    /* Throttle */
    fill(#FFFFFF);
    rect(10, 200, 20, -100);
    fill(#000000);
    rect(10, 200, 20, -rcThrottle);
    text("Throttle", 10, 220);
    text(int(rcThrottle) + "%", 10, 240);
    
    /* Yaw */
    fill(#FFFFFF);
    rect(40, 200, 100, -20);
    fill(#000000);
    rect(90, 200, rcYaw, -20);    
    text("Yaw", 70, 220);
    text(int(rcYaw) + "%", 70, 240);
    
    /* Pitch */
    fill(#FFFFFF);
    rect(160, 100, 20, 100);
    fill(#000000);
    rect(160, 150, 20, -rcPitch); 
    text("Pitch", 160, 220);
    text(int(rcPitch) + "%", 160, 240);
    
    /* Roll */
    fill(#FFFFFF);
    rect(190, 200, 100, -20);
    fill(#000000);
    rect(240, 200, rcRoll, -20);
    text("Roll", 220, 220);
    text(int(rcRoll) + "%", 220, 240);
    
    /* ("On/Off */
    if (rcOnOff > 50) {
      fill(#00FF00);
    } else {
      fill(#004000);
    }   
    ellipse(320, 180, 40, 40);        
    fill(#000000);
    text("On/Off", 300, 220);
    text(int(rcOnOff) + "%", 300, 240);

    /* Dimmer */
    fill(#FFFFFF);
    ellipse(380, 180, 40, 40);
    fill(#000000);
    arc(380, 180, 40, 40, 0, 2 * PI * rcDimmer / 100, PIE);
    text("Dimmer", 360, 220);
    text(int(rcDimmer) + "%", 360, 240);
    
    text("Frequency", 420, 220);
    text(interval + "Hz", 420, 240);
  }
}

String readDataFromSerial() {
  if (myPort.available() > 0) {   
    String data = myPort.readStringUntil(lf);
    if (data != null) {   
      return data;
    }
  }
  
  return null;
}
