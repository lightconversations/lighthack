

// Copyright (c) 2017 Electronic Theatre Controls, Inc., http://www.etcconnect.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


/*******************************************************************************
     Electronic Theatre Controls
     lighthack - Box 1
     (C) 2017-2018 by ETC
     This code implements a Pan/Tilt module using two encoders and three
     buttons. The two encoders function as pan and tilt controllers with one
     button being reserved for controlling fine/coarse mode. The other two
     buttons are assigned to Next and Last which make it easy to switch between
     channels.
 *******************************************************************************
    NOTE: UPDATE VERSION_STRING IN DEFINITIONS BELOW WHEN VERSION NUMBER CHANGES
    Revision History
    yyyy-mm-dd   Vxx      By_Who                 Comment
    2017-07-21   1.0.0.1  Ethan Oswald Massey    Original creation
    2017-10-19   1.0.0.2  Sam Kearney            Fix build errors on some
                                                 Arduino platforms. Change
                                                 OSC subscribe parameters
    2017-10-24   1.0.0.3  Sam Kearney            Add ability to scale encoder
                                                 output
    2017-11-22   1.0.0.4  Hans Hinrichsen        Add splash msg before Eos
                                                 connects
    2017-12-07   1.0.0.5  Hans Hinrichsen        Added timeout to disconnect
                                                 and show splash screen again
    2018-10-25   2.0.0.0  Richard Thompson       Generalised to support other
                                                 ETC consoles
    2018-10-25   2.0.0.1  Richard Thompson       Add basic support for ColorSource
 ******************************************************************************/

/*******************************************************************************
   Includes
 ******************************************************************************/
//#include <OSCBoards.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCMatch.h>
#include <OSCMessage.h>
//#include <OSCTiming.h>
#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial);
#endif
#include <SPI.h>
//#include <string.h>
#include <Adafruit_SSD1306.h>
//#include <splash.h>
//#include <Adafruit_GFX.h>
//#include <wire.h>
//#include <Adafruit_SPITFT.h>
//#include <Adafruit_SPITFT_Macros.h>
//#include <gfxfont.h>

/*******************************************************************************
   Macros and Constants
 ******************************************************************************/
#define DHTPIN 2
#define DHTTYPE
#define OLED_Size_W       128
#define OLED_Size_H       32
#define OLED_RESET        -1
Adafruit_SSD1306 display(OLED_Size_W, OLED_Size_H, &Wire, OLED_RESET);

#define NEXT_BTN            6
#define LAST_BTN            7
#define SHIFT_BTN           8

#define SUBSCRIBE           ((int32_t)1)
#define UNSUBSCRIBE         ((int32_t)0)

#define EDGE_DOWN           ((int32_t)1)
#define EDGE_UP             ((int32_t)0)

#define FORWARD             0
#define REVERSE             1

// Change these values to switch which direction increase/decrease pan/tilt
#define PAN_DIR             FORWARD
#define TILT_DIR            FORWARD

// Use these values to make the encoder more coarse or fine.
// This controls the number of wheel "ticks" the device sends to the console
// for each tick of the encoder. 1 is the default and the most fine setting.
// Must be an integer.
#define PAN_SCALE           1
#define TILT_SCALE          1

/*
   PIN connection:
   pin connection see: https://www.arduino.cc/en/Reference/Wire
   for UNO: SDA to A4, SCL to A5
   for Mega2560: SDA to 20, SCL to 21
   for Leonardo: SDA to 2, SCL to 3
   for Due: SDA to 20, SCL to 21
   VCC to 5V
   GND to GND :-)
*/

#if (OLED_Size_H != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";

#define VERSION_STRING      "2.0.0.1"
#define BOX_NAME_STRING     "Tlaloc_box" //Max 10 characters

// Change these values to alter how long we wait before sending an OSC ping
// to see if Eos is still there, and then finally how long before we
// disconnect and show the splash screen
// Values are in milliseconds
#define PING_AFTER_IDLE_INTERVAL    2500
#define TIMEOUT_AFTER_IDLE_INTERVAL 5000

/*******************************************************************************
   Local Types
 ******************************************************************************/
enum WHEEL_TYPE { TILT, PAN };
enum WHEEL_MODE { COARSE, FINE };

struct Encoder
{
  uint8_t pinA;
  uint8_t pinB;
  int pinAPrevious;
  int pinBPrevious;
  float pos;
  uint8_t direction;
};
struct Encoder panWheel;
struct Encoder tiltWheel;

enum ConsoleType
{
  ConsoleNone,
  ConsoleEos,
  ConsoleCobalt,
  ConsoleColorSource
};

char *msgArray[] = {
"/eos/wheel/fine/pan",
"/eos/wheel/coarse/pan",
"/eos/wheel/fine/tilt",
"/eos/wheel/coarse/tilt",
"/cobalt/param/pan/wheel",
"/cobalt/param/tilt/wheel",
"/cs/param/pan/wheel",
"/cs/param/tilt/wheel"};

/*******************************************************************************
   Global Variables
 ******************************************************************************/

// initialize the library with the numbers of the interface pins


ConsoleType connectedToConsole = ConsoleNone;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;

/*******************************************************************************
   Local Functions
 ******************************************************************************/

// Send debugging message to EOS console
void pingEos(int txt)
{
  OSCMessage ping("/eos/ping");
  ping.add(BOX_NAME_STRING);
  ping.add(txt);
  SLIPSerial.beginPacket();
  ping.send(SLIPSerial);
  SLIPSerial.endPacket();
}

/*******************************************************************************
   Issues all our subscribes to Eos. When subscribed, Eos will keep us updated
   with the latest values for a given parameter.
   Parameters:  none
   Return Value: void
 ******************************************************************************/
void issueEosSubscribes()
{
  // Add a filter so we don't get spammed with unwanted OSC messages from Eos
  OSCMessage filter("/eos/filter/add");
  filter.add("/eos/out/param/*");
  filter.add("/eos/out/ping");
  SLIPSerial.beginPacket();
  filter.send(SLIPSerial);
  SLIPSerial.endPacket();

  OSCMessage subPan("/eos/subscribe/param/pan/tilt");
  subPan.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subPan.send(SLIPSerial);
  SLIPSerial.endPacket();

}

/*******************************************************************************
   Given a valid OSCMessage (relevant to Pan/Tilt), we update our Encoder struct
   with the new position information.
   Parameters:
    msg - The OSC message we will use to update our internal data
    addressOffset - Unused (allows for multiple nested roots)
   Return Value: void
 ******************************************************************************/
void parseFloatPanUpdate(OSCMessage& msg, int addressOffset)
{
  panWheel.pos = msg.getOSCData(0)->getFloat();
}

void parseFloatTiltUpdate(OSCMessage& msg, int addressOffset)
{
  tiltWheel.pos = msg.getOSCData(0)->getFloat();
}

void parseEos(OSCMessage& msg, int addressOffset)
{
  // If we don't think we're connected, reconnect and subscribe
  if (connectedToConsole != ConsoleEos)
  {
    issueEosSubscribes();
    connectedToConsole = ConsoleEos;
  }

  if (!msg.route("/out/param/tilt", parseFloatTiltUpdate, addressOffset))
    msg.route("/out/param/pan", parseFloatPanUpdate, addressOffset);
}

/******************************************************************************/

void parseCobalt(OSCMessage& msg, int addressOffset)
{
  // Cobalt doesn't currently send anything other than ping
  connectedToConsole = ConsoleCobalt;
}

void parseColorSource(OSCMessage& msg, int addressOffset)
{
  // ColorSource doesn't currently send anything other than ping
  connectedToConsole = ConsoleColorSource;
}

/*******************************************************************************
   Given an unknown OSC message we check to see if it's a handshake message.
   If it's a handshake we issue a subscribe, otherwise we begin route the OSC
   message to the appropriate function.
   Parameters:
    msg - The OSC message of unknown importance
   Return Value: void
 ******************************************************************************/
void parseOSCMessage(String& msg)
{
  // check to see if this is the handshake string
  if (msg.indexOf(HANDSHAKE_QUERY) != -1)
  {
    // handshake string found!
    SLIPSerial.beginPacket();
    SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
    SLIPSerial.endPacket();

    // An Eos would do nothing until subscribed
    // Let Eos know we want updates on some things
    issueEosSubscribes();
  }
  else
  {
    // prepare the message for routing by filling an OSCMessage object with our message string
    OSCMessage oscmsg;
    oscmsg.fill((uint8_t*)msg.c_str(), (int)msg.length());
    // route pan/tilt messages to the relevant update function

    // Try the various OSC routes
    if (oscmsg.route("/eos", parseEos))
      return;
    if (oscmsg.route("/cobalt", parseCobalt))
      return;
    if (oscmsg.route("/cs", parseColorSource))
      return;
  }
  switch (connectedToConsole) {
    case ConsoleNone:
      {
        display.clearDisplay();
        display.setCursor(0, 0);     // Start at top-left corner
        display.println(BOX_NAME_STRING);
        display.println("Connecting");
        display.display();
      } break;

    case ConsoleEos:
      {
        display.clearDisplay();
        display.setCursor(0, 0);     // Start at top-left corner
        display.print("Pan:");
        display.println(panWheel.pos);
        display.print("Tlt:");
        display.println(tiltWheel.pos);
        display.display();
      } break;

    case ConsoleCobalt:
      {
        display.setCursor(7, 0);
        display.println("Cobalt");
        display.setCursor(0, 1);
        display.println("Pan");
        display.setCursor(12, 1);
        display.println("Tilt");
        display.display();
      } break;

    case ConsoleColorSource:
      {
        display.setCursor(2, 0);
        display.println("ColorSource");
        display.setCursor(0, 1);
        display.println("Pan");
        display.setCursor(12, 1);
        display.println("Tilt");
        display.display();
      } break;

  }
}

/*******************************************************************************
   Initializes a given encoder struct to the requested parameters.
   Parameters:
    encoder - Pointer to the encoder we will be initializing
    pinA - Where the A pin is connected to the Arduino
    pinB - Where the B pin is connected to the Arduino
    direction - Determines if clockwise or counterclockwise is "forward"
   Return Value: void
 ******************************************************************************/
void initEncoder(struct Encoder* encoder, uint8_t pinA, uint8_t pinB, uint8_t direction)
{
  encoder->pinA = pinA;
  encoder->pinB = pinB;
  encoder->pos = 0;
  encoder->direction = direction;

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  encoder->pinAPrevious = digitalRead(pinA);
  encoder->pinBPrevious = digitalRead(pinB);
}

/*******************************************************************************
   Checks if the encoder has moved by comparing the previous state of the pins
   with the current state. If they are different, we know there is movement.
   In the event of movement we update the current state of our pins.
   Parameters:
    encoder - Pointer to the encoder we will be checking for motion
   Return Value:
    encoderMotion - Returns the 0 if the encoder has not moved
                                1 for forward motion
                               -1 for reverse motion
 ******************************************************************************/
int8_t updateEncoder(struct Encoder* encoder)
{
  int8_t encoderMotion = 0;
  int pinACurrent = digitalRead(encoder->pinA);
  int pinBCurrent = digitalRead(encoder->pinB);

  // has the encoder moved at all?
  if (encoder->pinAPrevious != pinACurrent)
  {
    // Since it has moved, we must determine if the encoder has moved forwards or backwards
    encoderMotion = (encoder->pinAPrevious == encoder->pinBPrevious) ? -1 : 1;

    // If we are in reverse mode, flip the direction of the encoder motion
    if (encoder->direction == REVERSE)
      encoderMotion = -encoderMotion;
  }
  encoder->pinAPrevious = pinACurrent;
  encoder->pinBPrevious = pinBCurrent;

  return encoderMotion;
}

/*******************************************************************************
   Sends a message to Eos informing them of a wheel movement.
   Parameters:
    type - the type of wheel that's moving (i.e. pan or tilt)
    ticks - the direction and intensity of the movement
   Return Value: void
 ******************************************************************************/

void sendOscMessage(int msgIndex, float value)
{
  OSCMessage msg(msgArray[msgIndex]);
  msg.add(value);
  SLIPSerial.beginPacket();
  msg.send(SLIPSerial);
  SLIPSerial.endPacket();
}

void sendEosWheelMove(WHEEL_TYPE type, float ticks)
{
  int msgIndex;
  if (type == PAN)
    if (digitalRead(SHIFT_BTN) == LOW)
      msgIndex = 0;
    else
      msgIndex = 1;
  else if (type == TILT)
    if (digitalRead(SHIFT_BTN) == LOW)
      msgIndex = 2;
    else
      msgIndex = 3;
  else
    // something has gone very wrong
    return;

  sendOscMessage(msgIndex, ticks);
}

void sendCobaltWheelMove(WHEEL_TYPE type, float ticks)
{
  int msgIndex;

  if (type == PAN)
    msgIndex = 4;
  else if (type == TILT)
    msgIndex = 5;
  else
    // something has gone very wrong
    return;

  if (digitalRead(SHIFT_BTN) != LOW)
    ticks = ticks * 16;

  sendOscMessage(msgIndex, ticks);
}

void sendColorSourceWheelMove(WHEEL_TYPE type, float ticks)
{
  int msgIndex;

  if (type == PAN)
    msgIndex = 6;
  else if (type == TILT)
    msgIndex = 7;
  else
    // something has gone very wrong
    return;

  if (digitalRead(SHIFT_BTN) != LOW)
    ticks = ticks * 2;

  sendOscMessage(msgIndex, ticks);
}

/******************************************************************************/

void sendWheelMove(WHEEL_TYPE type, float ticks)
{
  switch (connectedToConsole)
  {
    default:
    case ConsoleEos:
      sendEosWheelMove(type, ticks);
      break;
    case ConsoleCobalt:
      sendCobaltWheelMove(type, ticks);
      break;
    case ConsoleColorSource:
      sendColorSourceWheelMove(type, ticks);
      break;
  }
}

/*******************************************************************************
   Sends a message to the console informing them of a key press.
   Parameters:
    down - whether a key has been pushed down (true) or released (false)
    key - the OSC key name that has moved
   Return Value: void
 ******************************************************************************/
void sendKeyPress(bool down, const String &key)
{
  String keyAddress;
  switch (connectedToConsole)
  {
    default:
    case ConsoleEos:
      keyAddress = "/eos/key/" + key;
      break;
    case ConsoleCobalt:
      keyAddress = "/cobalt/key/" + key;
      break;
    case ConsoleColorSource:
      keyAddress = "/cs/key/" + key;
      break;
  }
  OSCMessage keyMsg(keyAddress.c_str());

  if (down)
    keyMsg.add(EDGE_DOWN);
  else
    keyMsg.add(EDGE_UP);

  SLIPSerial.beginPacket();
  keyMsg.send(SLIPSerial);
  SLIPSerial.endPacket();
}

/*******************************************************************************
   Checks the status of all the relevant buttons (i.e. Next & Last)
   NOTE: This does not check the shift key. The shift key is used in tandem with
   the encoder to determine coarse/fine mode and thus does not report directly.
   Parameters: none
   Return Value: void
 ******************************************************************************/

void checkButtons()
{
  // OSC configuration
  const int keyCount = 2;
  const int keyPins[2] = {NEXT_BTN, LAST_BTN};
  const String keyNames[4] = {
    "NEXT", "LAST",
    "soft6", "soft4"
  };

  static int keyStates[2] = {HIGH, HIGH};

  // Eos and Cobalt buttons are the same
  // ColorSource is different
  int firstKey = (connectedToConsole == ConsoleColorSource) ? 2 : 0;

  // Loop over the buttons
  for (int keyNum = 0; keyNum < keyCount; ++keyNum)
  {
    // Has the button state changed
    if (digitalRead(keyPins[keyNum]) != keyStates[keyNum])
    {
      // Notify console of this key press
      if (keyStates[keyNum] == LOW)
      {
        sendKeyPress(false, keyNames[firstKey + keyNum]);
        keyStates[keyNum] = HIGH;
      }
      else
      {
        sendKeyPress(true, keyNames[firstKey + keyNum]);
        keyStates[keyNum] = LOW;
      }
    }
  }
}

void displaySetup()
{
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
}



/*******************************************************************************
   Here we setup our encoder, lcd, and various input devices. We also prepare
   to communicate OSC with Eos by setting up SLIPSerial. Once we are done with
   setup() we pass control over to loop() and never call setup() again.
   NOTE: This function is the entry function. This is where control over the
   Arduino is passed to us (the end user).
   Parameters: none
   Return Value: void
 ******************************************************************************/
void setup()
{ Wire.begin();

  Serial.begin(9600);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  displaySetup();
  display.clearDisplay();
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(VERSION_STRING);
  display.println("loading...");
  display.display();
        
  SLIPSerial.begin(115200);

  // This is necessary for reconnecting a device because it needs some time
  // for the serial port to open. The handshake message may have been sent
  // from the console before #lighthack was ready

  SLIPSerial.beginPacket();
  SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
  SLIPSerial.endPacket();

  // If it's an Eos, request updates on some things
  issueEosSubscribes();

  initEncoder(&panWheel, 3, 4, PAN_DIR);
  initEncoder(&tiltWheel, A0, A1, TILT_DIR);

  pinMode(NEXT_BTN, INPUT_PULLUP);
  pinMode(LAST_BTN, INPUT_PULLUP);
  pinMode(SHIFT_BTN, INPUT_PULLUP);

}

/*******************************************************************************
   Here we service, monitor, and otherwise control all our peripheral devices.
   First, we retrieve the status of our encoders and buttons and update Eos.
   Next, we check if there are any OSC messages for us.
   Finally, we update our display (if an update is necessary)
   NOTE: This function is our main loop and thus this function will be called
   repeatedly forever
   Parameters: none
   Return Value: void
 ******************************************************************************/
void loop()
{
  static String curMsg;
  int size;
  // get the updated state of each encoder
  int32_t panMotion = updateEncoder(&panWheel);
  int32_t tiltMotion = updateEncoder(&tiltWheel);

  // Scale the result by a scaling factor
  panMotion *= PAN_SCALE;
  tiltMotion *= TILT_SCALE;

  // check for next/last updates
//  checkButtons();

  // now update our wheels
  if (tiltMotion != 0)
    sendWheelMove(TILT, tiltMotion);

  if (panMotion != 0)
    sendWheelMove(PAN, panMotion);

  // Then we check to see if any OSC commands have come from Eos
  // and update the display accordingly.
  size = SLIPSerial.available();
  if (size > 0)
  {
    // Fill the msg with all of the available bytes
    while (size--)
      curMsg += (char)(SLIPSerial.read());
  }
  if (SLIPSerial.endofPacket())
  {
    parseOSCMessage(curMsg);
    lastMessageRxTime = millis();
    // We only care about the ping if we haven't heard recently
    // Clear flag when we get any traffic
    timeoutPingSent = false;
    curMsg = String();
  }

  if (lastMessageRxTime > 0)
  {
    unsigned long diff = millis() - lastMessageRxTime;
    //We first check if it's been too long and we need to time out
    if (diff > TIMEOUT_AFTER_IDLE_INTERVAL)
    {
      connectedToConsole = ConsoleNone;
      lastMessageRxTime = 0;
      timeoutPingSent = false;
    }

    //It could be the console is sitting idle. Send a ping once to
    // double check that it's still there, but only once after 2.5s have passed
    if (!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL)
    {
      OSCMessage ping("/eos/ping");
      ping.add(BOX_NAME_STRING "_is_online"); // This way we know who is sending the ping
      SLIPSerial.beginPacket();
      ping.send(SLIPSerial);
      SLIPSerial.endPacket();
      timeoutPingSent = true;
    }
  }
}
