import processing.serial.*;

int lf = 10; // Linefeed(\n) in ASCII
String rawRCValues = null;
Serial myPort;

PFont f;

void setup() {
  size(500, 300);
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
    
    if (RCValues.length < 6) {
      return;
    }
  
    float ch1 = float(RCValues[0]);
    ch1 = map(ch1, 1000.0, 2000.0, 1.0, 100.0);
    
    float ch2 = float(RCValues[1]);
    ch2 = 100 - map(ch2, 1000.0, 2000.0, 1.0, 100.0);
    
    float ch3 = float(RCValues[2]);
    ch3 = map(ch3, 1000.0, 2000.0, 1.0, 100.0);
    
    float ch4 = float(RCValues[3]);
    ch4 = map(ch4, 1000.0, 2000.0, 1.0, 100.0);
    
    float ch5 = float(RCValues[4]);
    ch5 = map(ch5, 1000.0, 2000.0, 1.0, 100.0);
    
    float ch6 = float(RCValues[5]);
    ch6 = map(ch6, 1000.0, 2000.0, 1.0, 100.0);
    
    background(#FFFFFF);
    fill(#000000);
    stroke(#000000);
    
    /* Channel 3 */
    rect(10, 200, 20, -ch3);
    text("Canal 3", 10, 220);
    text(int(ch3) + "%", 10, 240);
    
    /* Channel 4 */
    rect(50, 200-20, ch4, 20);    
    text("Canal 4", 50, 220);
    text(int(ch4) + "%", 50, 240);
    
    /* Channel 2 */
    rect(160, 200, 20, -ch2); 
    text("Canal 2", 160, 220);
    text(int(ch2) + "%", 160, 240);
    
    /* Channel 1 */
    rect(200, 200-20, ch1, 20);
    text("Canal 1", 200, 220);
    text(int(ch1) + "%", 200, 240);
    
    /* Channel 5 */
    if (ch5 > 50) {
      fill(#00FF00);
    } else {
      fill(#004000);
    }   
    ellipse(320, 180, 40, 40);        
    fill(#000000);
    text("Canal 5", 300, 220);
    text(int(ch5) + "%", 300, 240);

    /* Channel 6 */
    arc(380, 180, 40, 40, 0, 2*PI*ch6/100, PIE);
    text("Canal 6", 360, 220);
    text(int(ch6) + "%", 360, 240);
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
