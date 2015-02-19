/**
 * XbeeBlinkMController -- 
 *
 *
 *
 * 2009, Tod E. Kurt, ThingM Corp, http://thingm.com/
 *
 */

import processing.serial.*;

String portname = null;  // set to null to attempt to automatically find port
Serial port;
//int portSpeed = 230400;
int portSpeed = 115200;
//int portSpeed = 19200;

int sketchWidth = 540;
int sketchHeight = 400;

int frameRate = 40;
float blinkmUpdateMillis = 250;  // target: 50 millis = 20fps

int numPixels = 255;
int numChannels = 4;    // number of different blinkms
int numSlices = 20;     // number of different time slices per blinkm

int debug = 2;          // debug level, 0 = off, higher numbers is more debug

int px; // playhead position
// each slice is "timelineWidth/numSlices" wide and must be transited in 
// blinkmUpdateMillis

int timelineX,timelineY;
int timelineWidth,timelineHeight;

int colorPickerX, colorPickerY;
int colorPickerWidth,colorPickerHeight;
int pickX, pickY;

int previewX,previewY;
int previewWidth, previewHeight;

int sliceWidth,sliceHeight;

ColorSlice[][] colorSlices = new ColorSlice[numSlices][numChannels];

//color nullColor = color(226);
color backColor = color(226);
color nullColor = color(0);
color previewColor = color(255,0,255); // start off with a default

PFont font;

long lastmillis;     // the last time we sent a 'frame' of colors
int blinkmSlicePos;  // runs from 0 to numSlices-1

boolean isSending = false;

// the buffer used to send the actual color data to the BlinkMs
// 3 bytes per BlinkM + 4 bytes of header 
// current firmware is set to 8 channels so we'll use that
byte[] sendBuffer = new byte[ 4 + (numPixels*3)];

PGraphics colorPickerImage;

// Processing's setup
void setup() {
  size(sketchWidth, sketchHeight);
  frameRate(frameRate);
  smooth();

  font = loadFont("LucidaSans-12.vlw");

  timelineX = 20;
  timelineY = 20;
  timelineWidth = width-40;
  timelineHeight = height/3;

  sliceWidth = timelineWidth/numSlices;
  sliceHeight= timelineHeight/numChannels;

  colorPickerX = 10;
  colorPickerY = height/3 + 50;
  colorPickerWidth = 255;
  colorPickerHeight = 207;

  pickX = colorPickerX + (colorPickerWidth/8)*7; // hack
  pickY = colorPickerY + colorPickerHeight/2;

  previewX = colorPickerX + colorPickerWidth + 30;
  previewY = colorPickerY - 5 ;
  previewWidth = sliceWidth;
  previewHeight = sliceHeight;

  // create the colorslices
  for( int i=0; i<numSlices; i++ ) { 
    for( int j=0; j<numChannels; j++ ) {
      int x = timelineX + i*sliceWidth;
      int y = timelineY + j*sliceHeight;
      colorSlices[i][j] = new ColorSlice(x,y, sliceWidth,sliceHeight,nullColor);
    }
  }
  
  colorPickerImage = createColorPickerImage();

  openSerialPort();
}

// Processing's draw
void draw() {
  background( backColor );
  drawColorPicker();
  drawTimeline();
  drawPlayhead(timelineX, timelineY, timelineWidth, timelineHeight);

  drawPreview( previewX, previewY, previewWidth, previewHeight);

  fill(10);
  textFont( font, 12 );
  text( (isSending) ? "Transmitting": "Not Transmitting", width-130,height-200);
  text( "updateMillis:"+int(blinkmUpdateMillis), width-130, height-20);
  fill(90);
  text( "spacebar toggles", width-120, height-188);
  text( "arrows adjust",    width-120, height-8);

  long mils = millis();
  if( mils - lastmillis >= blinkmUpdateMillis ) {
    lastmillis = mils;
    blinkmSlicePos++;
    if( blinkmSlicePos == numSlices ) blinkmSlicePos = 0;
    px += timelineWidth/numSlices;  // FIXME: hack hack
    if( isSending ) sendFrame(blinkmSlicePos);
  }
}

// 
void keyPressed() {
  if( key == CODED ) { 
    if( keyCode == UP && blinkmUpdateMillis < 1100 ) 
      blinkmUpdateMillis += 10;
    else if( keyCode == DOWN && blinkmUpdateMillis > 20 )
      blinkmUpdateMillis -= 10;
  } 
  else if( key == RETURN || key == ENTER ) { 
      blinkmUpdateMillis = 100;
  }
  else if( key == ' ' ) {
    isSending = !isSending;
  }

}

//
void mousePressed() {
  // if in colorpicker, we're updating previewColor
  if( mouseX > colorPickerX && mouseX < (colorPickerX+colorPickerWidth) &&
      mouseY > colorPickerY && mouseY < (colorPickerY+colorPickerHeight) ) {
    previewColor = get(mouseX,mouseY);
    pickX = mouseX; pickY = mouseY;
    return;
  }
  // else if in timeline, color a slice
  for( int i=0; i<numSlices; i++ ) {
    for( int j=0; j<numChannels; j++ ) {
      if( colorSlices[i][j].hit(mouseX, mouseY) ) { 
        colorSlices[i][j].c = previewColor;
      }
    }
  }
}
// handle drag the same way as a press, kinda lame, yeah
void mouseDragged() {
  mousePressed();
}

// really just render the pre-computed image, created w/ createColorPickerImage
void drawColorPicker() {
  fill(10);
  textFont(font, 12);
  text("pick a color", colorPickerX, colorPickerY-5);
  image(colorPickerImage, colorPickerX, colorPickerY);
  noFill();
  stroke( 200 );
  rect( pickX-4,pickY-4, 8,8 );
  stroke( 40 );
  rect( pickX-3,pickY-3, 8,8 );
}

// draw timeline slices
void drawTimeline() { 
  fill(10);
  textFont(font, 12);
  text("color timeline", 20,15);
  for( int i= 0; i<numChannels; i++) { 
    text( i, 10, 42 + i*sliceHeight);
  }
  strokeWeight(1);
  stroke(210);
  for( int i=0; i<numSlices; i++ ) 
    for( int j=0; j<numChannels; j++ )
      colorSlices[i][j].draw();
}

// playhead cursor going over slices
void drawPlayhead(int x, int y, int w, int h) {
  pushMatrix();
  translate(x+12,y);           // FIXME: should be derived
  strokeWeight(24);            // FIXME: should be derived
  stroke(100, 100);
  //px = px + (w/frameRate);   // FIXME: 
  if( px>= w ) px = 0;
  line(px,0, px,h);
  popMatrix();
}

//
void drawPreview(int x, int y, int w, int h) {
  color c =  previewColor;
  pushMatrix();
  translate(x,y);
  textFont(font, 12);
  text("color", 0,0);
  text("selected", 0,12);
  strokeWeight(1.0);
  stroke(180);
  fill(c);
  rect( 0,15, w,h);
  fill(10);
  textFont(font,8);
  //text( int(red(c))+","+int(green(c))+","+int(blue(c)), 0,h+30);
  text( int(red(c)),   30,25);
  text( int(green(c)), 30,35);
  text( int(blue(c)),  30,45);
  popMatrix();
}

// just do this plotting once because it's static and 
// sucks up cycles if computed every draw()
PGraphics createColorPickerImage() {
  PGraphics pg;
  pg = createGraphics(255,200+4+4,JAVA2D);
  pg.beginDraw();
  pg.colorMode(HSB,255,100,100);
  pg.stroke(255,0,100);
  pg.fill(255,0,100);
  pg.rect(0,0, 255,4);

  for (int i = 0; i < 255; i++) {
    for (int j = 0; j < 100; j++) {
      pg.stroke( i, j, 100);
      pg.point( i, 4+j);
      pg.stroke( i, 100, 100-j);
      pg.point( i, 4+j+100 );
    }
  }
  pg.stroke(255,0,0);
  pg.fill(255,0,0);
  pg.rect(0,204, 255,4);
  pg.endDraw();
  return pg;
}

//--------------------------------------------------------------------------


// for a given position in the timeline, 
// send a "frame" of all the colors at that time
void sendFrame(int p) {
  debug(1, "send colors["+blinkmSlicePos+"]");
  sendBuffer[0] = (byte)0xFE;
  sendBuffer[1] = (byte)0x01;
  sendBuffer[2] = (byte)0x02;
  sendBuffer[3] = (byte)0x03;
  for( int j=0; j< numPixels; j++ ) {  // go through entire frame at time 'p'
    color c = color(11,22,33);  // default color for all the non-channel pixels
    if( j < numChannels )   
      c = colorSlices[p][j].c;
    sendBuffer[4+0+(j*3)] = byte(red(c));
    sendBuffer[4+1+(j*3)] = byte(green(c));
    sendBuffer[4+2+(j*3)] = byte(blue(c));
  }
  if( debug>2 ) {
    for( int i=0; i<sendBuffer.length; i++ ) 
      print( hex(sendBuffer[i]) + ",");
    println();
  }
  if( port!=null ) 
    port.write(sendBuffer); // actually send the bolus to the blinkms
}

// for a given position in the timeline, 
// send a "frame" of all the colors at that time
void sendFrameOrig(int p) {
  debug(1, "send colors["+blinkmSlicePos+"]");
  sendBuffer[0] = (byte)0xFE;
  sendBuffer[1] = (byte)0x01;
  sendBuffer[2] = (byte)0x02;
  sendBuffer[3] = (byte)0x03;
  for( int j=0; j< numChannels; j++ ) {  // go through entire frame at time 'p'
    color c = colorSlices[p][j].c;
    sendBuffer[4+0+(j*3)] = byte(red(c));
    sendBuffer[4+1+(j*3)] = byte(green(c));
    sendBuffer[4+2+(j*3)] = byte(blue(c));
  }
  if( debug>2 ) {
    for( int i=0; i<sendBuffer.length; i++ ) 
      print( hex(sendBuffer[i]) + ",");
    println();
  }
  if( port!=null ) 
    port.write(sendBuffer); // actually send the bolus to the blinkms
}

//
void openSerialPort() {
  String[] portlist = Serial.list();
  println(portlist);
  if( portname==null || portname.equals("") ) {
    for( int i=0; i<portlist.length; i++) {
      if( portlist[i].startsWith("/dev/tty.usbserial") ) {
        portname = portlist[i];
        break;
      }
    }
  }
  if( portname != null && ! portname.equals("") ) {
    println("opening serial port "+portname);
    port = new Serial(this, portlist[0], portSpeed);
  }
}

// debug!
void serialEvent(Serial p) {
  println( "serial:"+(char)p.read() );
}

//-------------------------------------------------------------------------

void debug(int lvl, String s) {
  if(debug>lvl) println(s);
}
void debug(String s) {
  debug(0,s);
}

//-------------------------------------------------------------------------

// just little utility class to make dealing with the color slices easier
class ColorSlice { 
  int x,y,w,h;
  color c;
  
  ColorSlice( int x, int y, int w, int h, color c ) { 
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
    this.c = c;
  }
  
  void draw() {
    fill(c);
    rect(x,y, w,h);
  }
  
  boolean hit(int x, int y) { 
    return 
      (x >= this.x && x < (this.x+this.w)) &&
      (y >= this.y && y < (this.y+this.h));
  }
}
