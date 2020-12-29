/********************************************************************************
  Code for controlling digital crossover system DIGI1:
  
  + dual AK4490 DACs
  + Ian's McFIFO with McDualXO reclocker
  - updated display with BigFont volume
  - control over WIFI (ESP8266 Croduino Nova 2)
  - ADAU1452 DSP (MediaWorks Shenzen)
  
  (+ marks which devices are controlled in this file, other files will have other versions)
 ******************************************************************************/

#include <Wire.h>
#include <ESP8266WiFi.h>
// #include <uMQTTBroker.h>
#include <WebSocketsServer.h>
// #include <Hash.h>
#include <ArduinoJson.h>
#include "FifoMessage.hpp"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // call to lcd has to be before including bigFontChars
#include <bigFontChars.h>           // big font library
#include <SimpleRotary.h>

// McFIFO
CfifoMessage fifoMessage;
package_t package;

SimpleRotary rotary(12, 13, 14); // Pin A, Pin B, Button Pin

byte ak4490 = 0x10;   // device ak4490, 0010000 = 0x10 (if CAD1=L, CAD0=L, see datasheet p. 50) ----- IGOR, moj DAC#2 (novi)
byte ak4490_1 = 0x11; // device ak4490, 0010001 = 0x11 (if CAD1=L, CAD0=H, see datasheet p. 50) ----- IGOR, moj DAC#1
//int ak4490 = 0x12;  // device ak4490, 0010010 = 0x12 (if CAD1=H, CAD0=L, see datasheet p. 50)
//int ak4490 = 0x13;  // device ak4490, 0010011 = 0x13 (if CAD1=CAD0=H, see datasheet p. 50)

byte error;
byte r;
int mode = 0;
int dsdsignal = 0; // Variable to hold whether the incoming signal is PCM or DSD. PCM = 0, DSD = 1.

int mainVolume = 220; // type set to INT for constrain to work properly, if set to BYTE then volume down goes from 0 to 255
int tempVolume;       // variable to save main volume value when muting
bool muted = 0;
int volumeDiff = 0;
bool volMode = false;
String signalMode;
String sampleRate;
String fifoXO;
String fifoXOfreq;
String fifoDelay;

int activeFilter = 0;

const byte PDN = 2;    // Set PDN pin to PB4 (for use with an STM32 uC) or any digital pin (for an Arduino). You should connect this pin to your 4490's PDN pin.
const byte DSDPIN = 0; // Set DSD pi to PB3 (for use with an STM32 uC) or any digital pin (for an Arduino). You should connect this pin to your USB-to-I2S module's "DSD indicator" pin.

// Predefine starting display positions on lcd (20x4)
int pos_Vol[] = {11, 2};
int pos_Mute[] = {16, 1};
int pos_Flt[] = {8, 1}; // filter number only
int pos_SSID[] = {0, 0};
int pos_RSSI[] = {17, 0};
int pos_Mode[] = {0, 2};
int pos_SR[] = {0, 3};

unsigned long previousMillis = 0;

int rssiVal = 0;
int rssiPercent;
int rssiBreakPoints[] = {60, 75, 90};

bool isFromRemote = false;

#define iconEmpty 0xDB;
#define iconFull 0xFF;

// WiFiServer server(80);
// Set your Static IP address
// IPAddress local_IP(192, 168, 1, 66);
// Set your Gateway IP address
// IPAddress gateway(192, 168, 1, 1);
// IPAddress subnet(255, 255, 0, 0);

const char *ssid = "NajOptimalnije";
const char *password = "OcuNaInternet";
// const char * ssid     = "HuaweIgor";
// const char * password = "DajMiNet";

StaticJsonDocument<50> json;

WebSocketsServer webSocket = WebSocketsServer(80);

/************************ MAIN PROGRAM ************************************************************/

void setup()
{

  pinMode(PDN, OUTPUT);   // ak4490 PDN pin.
  digitalWrite(PDN, LOW); // Keep PDN pin low.
  pinMode(DSDPIN, INPUT); // DSD detection pin.

  // ROTARY ENCODER
  rotary.setDebounceDelay(1); // Set the debounce delay in ms  (Default: 2)
  rotary.setErrorDelay(100);  // Set the error correction delay in ms  (Default: 200)

  Serial.begin(9600); // Start the serial port (for troubleshooting)
  Serial.println("Starting...");

  Wire.begin(); // Join The I2C Bus As A Master

  Wire.beginTransmission(ak4490);
  Wire.beginTransmission(ak4490_1);

  lcd.begin();

  //Assign bigFont segments write numbers
  lcd.createChar(1, C_UB);
  lcd.createChar(2, C_RT);
  lcd.createChar(3, C_LL);
  lcd.createChar(4, C_LB);
  lcd.createChar(5, C_LR);
  lcd.createChar(6, C_UMB);
  lcd.createChar(7, C_LMB);
  lcd.createChar(8, C_LT);
  // lcd.createChar(0.1, fulldot);

  lcd.clear();
  lcd.home();
  lcd.print("#Digi1");

  DeviceInit(ak4490);
  DeviceInit(ak4490_1);

  SetVol(mainVolume);
  SetPCM();
  mode = 1;
  SetFilter(2);

  error = Wire.endTransmission();
  if (error == 0)
  {
    Serial.println("ak4490 found!");
  }
  else if (error == 4)
  {
    Serial.print("Unknown error at address ");
    Serial.println(ak4490);
  }
  else
  {
    Serial.println("No response from ak4490!");
    Serial.print("error code: ");
    Serial.println(error);
  }

  displayHelp();

  // WiFi.mode(WIFI_OFF);
  // WiFi.mode(WIFI_STA);
  // WiFi.begin("HuaweIgor", "DajMiNet");
  // WiFi.begin("NajOptimalnije", "OcuNaInternet");

  // Connect to wifi
  WiFi.begin(ssid, password);

  // Wait some time to connect to wifi
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++)
  {
    displayNetworkInfo();
    Serial.print(".");
    delay(1000);
  }
  displayNetworkInfo();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

/*********************** Loop Section *************************************************************/
void loop()
{

  delayMicroseconds(600);
  getMcFIFOdata();

  printRSSI();

  webSocket.loop();

  //dsdsignal = digitalRead(DSDPIN);
  if (dsdsignal == 1 && mode == 1)
  {
    SetDSD();
    EnableDirectDSD(1);
    SetDSDFilter(50);
    mode = 2;
    PrintDebugReg(ak4490);
    PrintDebugReg(ak4490_1);
  }

  if (dsdsignal == 0 && mode == 2)
  {
    EnableDirectDSD(0);
    SetPCM();
    mode = 1;
    PrintDebugReg(ak4490);
    PrintDebugReg(ak4490_1);
  }

  // ROTATE
  byte rot;
  rot = rotary.rotate();

  if (volMode == false)
  {
    if (rot == 1)
    {
      if (muted)
      {
        tempVolume = tempVolume + 5;
        tempVolume = constrain(tempVolume, 0, 255);
        printVol();
      }
      else
      {
        mainVolume = mainVolume + 5;
        mainVolume = constrain(mainVolume, 0, 255);
        SetVol(mainVolume);
      }
    }

    if (rot == 2)
    {
      if (muted)
      {
        tempVolume = tempVolume - 5;
        tempVolume = constrain(tempVolume, 0, 255);
        printVol();
      }
      else
      {
        mainVolume = mainVolume - 5;
        mainVolume = constrain(mainVolume, 0, 255);
        SetVol(mainVolume);
      }
    }
  }

  // PUSH
  byte p;

  p = rotary.pushType(700);

  if (p == 1)
  {
    Serial.println("Pushed");
    toggleMute();
  }

  // LONG PUSH
  if (p == 2)
  {
    Serial.println("Long Pushed");

    if (volMode == false)
    {
      volMode = true;
      Serial.print("volMode: ");
      Serial.println(volMode);
      displayVolDiff();
    }
    else if (volMode == true)
    {
      volMode = false;
      displayMain();
    }

    //    if ( muted ) {
    //      tempVolume = 150;
    //      lcd.setCursor(0, 1); lcd.print("Volume:"); lcd.print("("); lcd.print(tempVolume); lcd.print(")");
    //    } else {
    //      mainVolume = 150;
    //      SetVol(mainVolume);
    //    }
  }

  char command = getCommand();
  switch (command)
  {
    //    case 'a':
    //      EnableDirectDSD(1);
    //      break;
    //    case 'b':
    //      EnableDirectDSD(0);
    //      break;
    //    case 'c':
    //      SetDSDFilter(50);
    //      break;
    //    case 'd':
    //      SetDSDFilter(150);
    //      break;
  case 'e':
    SetFilter(1);
    break;
  case 'f':
    SetFilter(2);
    break;
  case 'g':
    SetFilter(3);
    break;
  case 'h':
    SetFilter(4);
    break;
  case 'i':
    EnableSuperslow(0);
    break;
  case 'j':
    EnableSuperslow(1);
    break;
  case 'k':
    SetSoundMode(1);
    break;
  case 'l':
    SetSoundMode(2);
    break;
  case 'm':
    SetSoundMode(3);
    break;
  case 'p':
    SetPCM();
    break;
    //    case 'q':
    //      SetDSD();
    //      break;
  case 'q':
    SetSpeed(0);
    break;
  case 'w':
    SetSpeed(1);
    break;

  // case '0':
  //   SetBits(0);
  //   break;

  // read registers from ak4490
  case '1':
    PrintDebugReg(ak4490);
    break;
  // read registers from ak4490_1
  case '2':
    PrintDebugReg(ak4490_1);
    break;
  case '3':
    SetBits(0);
    break;
  case '4':
    SetBits(1);
    break;
  // case '5':
  //   SetBits(5);
  //   break;
  // case 'y':
  //   PrintDebugReg();
  //   break;
  case 'z':
    displayHelp();
    break;

    //    case '0':

    //    case '1':
    //      DeviceInit(ak4490);
    //      SetVol(200);
    //      SetPCM();
    //      mode = 1;
    //      displayHelp();
    //      break;
    //    case '2':
    //      DeviceInit(ak4490_1);
    //      SetVol(170);
    //      SetPCM();
    //      mode = 1;
    //      displayHelp();
    //      break;

  // Separate DAC#1 and DAC#2 for testing
  case 'c':
    SetDeviceVol(ak4490, 150);
    break;
  case 'v':
    SetDeviceVol(ak4490_1, 150);
    break;

  default:
    break;
  }
}
// LOOP END

/****************************** Functions **********************************************************/

// McFIFO data procedure
void getMcFIFOdata()
{
  if (Serial.available()) // Using "while" instead of "if" sends ESP8266 into WDT reset loop!
  {
    if (fifoMessage.getNewPackageFlag() == 0) //any possible package was processed
    {
      fifoMessage.processReceiving((char)Serial.read());
    }

    else if (fifoMessage.getNewPackageFlag()) //there is new package received
    {
      package = fifoMessage.getPackage();
      if (!fifoMessage.getFinishInitialFlag()) //it's first time receiving a package -- WORKS
      {
        posCursor(pos_Mode);
        lcd.print("           ");
        posCursor(pos_SR);
        lcd.print("           ");
        fifoMessage.setFinishInitialFlag(); //set the flag
      }
      // Serial.println(package.command);
      // Serial.println(package.valueStr);

      switch (package.command)
      {
      case UNLOCK:
        posCursor(pos_Mode);
        lcd.print("UN-LOCK");
        signalMode = "Unlocked";
        break;
      case NOSIGNAL:
        posCursor(pos_Mode);
        lcd.print("NO SIGNAL");
        signalMode = "No signal";
        break;
      case WRONGSIGNAL:
        posCursor(pos_Mode);
        lcd.print("WRONG SGN");
        signalMode = "Wrong signal";
        break;
      case I2S:
        posCursor(pos_Mode);
        lcd.print("I2S");
        posCursor(pos_SR);
        lcd.print(normalizeStrLen(package.valueStr, 8));
        signalMode = "PCM";
        if (package.valueStr != sampleRate) // set only if new value received
        {
          sampleRate = package.valueStr;
          sendMode();
          sendSR();
        }
        break;
      case DSD:
        posCursor(pos_Mode);
        lcd.print("DSD");
        posCursor(pos_SR);
        lcd.print(normalizeStrLen(package.valueStr, 8));
        signalMode = "DSD";
        break;
      case XONUMBER:
        package.valueStr.remove(2);
        if (package.valueStr != fifoXO) // set only if new value received
        {
          fifoXO = package.valueStr;
          sendXO();
        }
        break;
      case FMCK:
        package.valueStr.remove(10);
        if (package.valueStr != fifoXOfreq) // set only if new value received
        {
          fifoXOfreq = package.valueStr;
          sendXO();
        }
        break;
      case DELAYTIME:
        package.valueStr.remove(4);
        if (package.valueStr != fifoDelay) // set only if new value received
        {
          fifoDelay = package.valueStr;
          sendDelay();
        }
        break;
      // case MASTER:
      //   lcd.setCursor(14, 2);
      //   lcd.print("  ");
      //   break; //'MA' master mode is default, so no need to display
      // case SLAVE:
      //   lcd.setCursor(14, 2);
      //   lcd.print("SL");
      //   break;
      case CLEAR:
        if (package.valueStr.charAt(0) == '0') //clear one line on lcd
        {
          posCursor(pos_Mode);
          lcd.print("          "); //clear screen line1 --- original comment, but not applicable
        }
        else if (package.valueStr.charAt(0) == '1')
        {
          posCursor(pos_SR);
          lcd.print("          "); //clear screen line2 --- original comment, but not applicable
        }
        break;
      default:
        break;
      }
    }
  }
}

// printVol
void printVol()
{
  clearVol();
  if (mainVolume >= 100)
  {
    BFwriteString(String(mainVolume), pos_Vol[0], pos_Vol[1]); // convert mainVolume (int) to string
  }
  else if (mainVolume < 100 && mainVolume > 9)
  {
    BFwriteString(String(mainVolume), pos_Vol[0] + 3, pos_Vol[1]);
  }
  else
  {
    BFwriteString(String(mainVolume), pos_Vol[0] + 6, pos_Vol[1]);
  }
}
// clearVol
void clearVol()
{
  posCursor(pos_Vol);
  lcd.print("         ");
  lcd.setCursor(pos_Vol[0], pos_Vol[1] + 1);
  lcd.print("         ");
}

// printFilter
void printFilter()
{
  lcd.setCursor(pos_Flt[0], pos_Flt[1]);
  // lcd.print();
}

// network checks and lcd prints
void displayNetworkInfo()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    posCursor(pos_SSID);
    lcd.print("/offline");
    posCursor(pos_RSSI);
    lcd.print("   ");
  }
  else if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("Connected to: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal: ");
    Serial.println(WiFi.RSSI());
    printSSID();
    // printRSSI(); RSSI will be printed periodically from the loop
  }
}

// print network SSID
void printSSID()
{
  posCursor(pos_SSID);
  lcd.print("/");
  lcd.print(WiFi.SSID());
}

// print network signal strength
void printRSSI()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 5000)
  {
    previousMillis = currentMillis;

    int tempRssiVal = abs(WiFi.RSSI());
    Serial.printf("rssi: %d \n", tempRssiVal);

    // print and send to remote only if value different from current
    if (tempRssiVal != rssiVal)
    {
      rssiVal = tempRssiVal;
      sendRSSI();

      if (rssiVal <= rssiBreakPoints[0])
      {
        rssiPercent = 1;
      }
      else if (rssiBreakPoints[0] < rssiVal && rssiVal <= rssiBreakPoints[1])
      {
        rssiPercent = 2;
      }
      else if (rssiBreakPoints[1] < rssiVal && rssiVal <= rssiBreakPoints[2])
      {
        rssiPercent = 3;
      }
      else if (rssiVal > rssiBreakPoints[2])
      {
        rssiPercent = 4;
      }

      posCursor(pos_RSSI);
      switch (rssiPercent)
      {
      case 1:
        lcd.print("***");
        break;
      case 2:
        lcd.print("**-");
        break;
      case 3:
        lcd.print("*--");
        break;
      case 4:
        lcd.print("!--");
        break;
      }
    }
  }
}

// Handle Websocket events and messages
void webSocketEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {

  // Connected
  case WStype_CONNECTED:
  {
    IPAddress ip = webSocket.remoteIP(client);
    Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", client, ip[0], ip[1], ip[2], ip[3], payload);
    // client is of int type, starts with 0, so if only one client 0 can be used instead of client

    // send message to client
    webSocket.sendTXT(client, "Connected");

    // Send network info
    sendNetworkInfo();
    sendRSSI();
    sendMode();
    sendSR();
    sendVol();
    sendFilter();
    sendXO();
    sendDelay();
  }
  break;

  // String message received
  case WStype_TEXT:
    Serial.printf("[%u] get Text: %s\n", client, payload);

    // send message to client
    // webSocket.sendTXT(client, "message here");

    // send data to all connected clients
    // webSocket.broadcastTXT("message here");

    // JSON parse incoming message
    deserializeJson(json, payload);
    // Check for different payloads
    if (json["filter"])
    {
      // You can use a String to get an element of a JsonObject
      // No duplication is done.
      int filterVal = json[String("filter")];
      isFromRemote = true;
      Serial.print("filterVal: ");
      Serial.println(filterVal);
      SetFilter(filterVal);
    }
    else if (json["volume"])
    {
      byte volumeVal = json[String("volume")];
      Serial.print("volumeVal: ");
      Serial.println(volumeVal);
      isFromRemote = true;
      SetVol(volumeVal);
    }
    break;

  // Binary message received
  case WStype_BIN:
    Serial.printf("[%u] get binary length: %u\n", client, length);
    hexdump(payload, length);

    // send message to client
    // webSocket.sendBIN(client, payload, length);
    break;

  // Disconnected
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", client);
    break;
  }
}

// Send Network info to remote
void sendNetworkInfo()
{
  json.clear();
  json["ssid"] = WiFi.SSID();
  json["IPaddress"] = WiFi.localIP().toString();
  String package;
  serializeJson(json, package);
  webSocket.sendTXT(0, package);
}

// Send RSSI to remote
void sendRSSI()
{
  json.clear();
  json["rssi"] = WiFi.RSSI();
  String package;
  serializeJson(json, package);
  webSocket.sendTXT(0, package);
}

// Send Volume to remote
void sendVol()
{
  json.clear();
  json["volume"] = mainVolume;
  String package;
  serializeJson(json, package);
  webSocket.sendTXT(0, package);
}

// Send Volume to remote
void sendFilter()
{
  json.clear();
  json["filter"] = activeFilter;
  String package;
  serializeJson(json, package);
  webSocket.sendTXT(0, package);
}

// Send mode to remote
void sendMode()
{
  json.clear();
  json["mode"] = signalMode;
  String package;
  serializeJson(json, package);
  webSocket.sendTXT(0, package);
}

// Send sample rate to remote
void sendSR()
{
  json.clear();
  json["sr"] = sampleRate;
  String package;
  serializeJson(json, package);
  webSocket.sendTXT(0, package);
}

// Send crystal num and frequency to remote
void sendXO()
{
  json.clear();
  json["fifoXO"] = fifoXO;
  json["fifoXOfreq"] = fifoXOfreq;
  String package;
  serializeJson(json, package);
  webSocket.sendTXT(0, package);
}

// Send delay to remote
void sendDelay()
{
  json.clear();
  json["delay"] = fifoDelay;
  String package;
  serializeJson(json, package);
  webSocket.sendTXT(0, package);
}

// Send Volume to remote
// void sendToRemote(keys[], values[])
// {
//   json.clear();
//   json[keys[0]] = values[0];
//   String package;
//   serializeJson(json, package);
//   webSocket.sendTXT(0, package);
// }

// helper function for positioning cursor to preset positions
void posCursor(int pos[2])
{
  lcd.setCursor(pos[0], pos[1]);
};

// Set DAC Volume
void SetVol(byte setVolume)
{
  mainVolume = setVolume;
  WriteRegister(ak4490, 0x03, mainVolume);   // Set Up Volume In DAC-L ATT
  WriteRegister(ak4490, 0x04, mainVolume);   // Set Up Volume In DAC-R ATT
  WriteRegister(ak4490_1, 0x03, mainVolume); // Set Up Volume In DAC-L ATT
  WriteRegister(ak4490_1, 0x04, mainVolume); // Set Up Volume In DAC-R ATT

  Serial.print("Volume set to: ");
  Serial.println(mainVolume);

  printVol();

  if (isFromRemote)
  {
    // reset boolean
    isFromRemote = false;
  }
  else
  {
    sendVol();
  }
}

// Set each DAC volume separately - allows for different volumes or constant volume offset
void SetDeviceVol(int device, byte setVolume)
{

  WriteRegister(device, 0x03, setVolume); // Set Up Volume In DAC-L ATT
  WriteRegister(device, 0x04, setVolume); // Set Up Volume In DAC-R ATT

  Serial.print("Device: ");
  Serial.print(device);
  Serial.print(" Volume set to: ");
  Serial.println(setVolume);

  if (device == ak4490)
  {
    lcd.setCursor(0, 1);
    lcd.print("Vol1: ");
    lcd.print(setVolume);
  }
  else if (device == ak4490_1)
  {
    lcd.setCursor(10, 1);
    lcd.print("Vol2: ");
    lcd.print(setVolume);
  }
}

void SetDSD()
{
  Serial.println("");
  Serial.println("Setting up for DSD.");
  byte temp = 0;
  temp = ReadRegister(ak4490, 0x02);
  bitSet(temp, 7);
  WriteRegister(ak4490, 0x02, temp); // Set To DSD Mode
  temp = 0;
  WriteRegister(ak4490, 0x00, B00000000); // Reset
  WriteRegister(ak4490, 0x00, B10001111); // Set To Master Clock Frequency Auto / 24Bit I2S Mode
  WriteRegister(ak4490, 0x06, B10011001); // Set To DSD Data Mute / DSD Mute Control / DSD Mute Release
  WriteRegister(ak4490, 0x09, B00000001); // Set To DSD Sampling Speed Control
}

void SetPCM()
{
  Serial.println("");
  Serial.println("Setting up for PCM.");

  byte temp = 0;
  byte temp_1 = 0;

  temp = ReadRegister(ak4490, 0x02);
  temp_1 = ReadRegister(ak4490_1, 0x02);
  Serial.print("Register ak4490 0x02: ");
  Serial.println(temp);
  Serial.print("Register ak4490_1 0x02: ");
  Serial.println(temp_1);

  bitClear(temp, 7);
  bitClear(temp_1, 7);

  WriteRegister(ak4490, 0x02, temp); // Set To PCM Mode
  temp = 0;
  WriteRegister(ak4490, 0x00, B00000000); // Reset
  WriteRegister(ak4490, 0x00, B10001111); // Set To Master Clock Frequency Auto / 24Bit I2S Mode

  WriteRegister(ak4490_1, 0x02, temp_1);
  temp_1 = 0;
  WriteRegister(ak4490_1, 0x00, B00000000);
  WriteRegister(ak4490_1, 0x00, B10001111);

  // displayMode("PCM");
}

void SetSpeed(byte x)
{
  if (x == 0)
  {
    WriteRegister(ak4490, 0x01, B00000010);
    WriteRegister(ak4490_1, 0x01, B00000010);
    WriteRegister(ak4490, 0x07, B00000000);
    WriteRegister(ak4490_1, 0x07, B00000000);
    WriteRegister(ak4490, 0x00, B00000000);
    WriteRegister(ak4490, 0x00, B10000111);
    WriteRegister(ak4490_1, 0x00, B00000000);
    WriteRegister(ak4490_1, 0x00, B10000111);
  }
  else if (x == 1)
  {
    WriteRegister(ak4490, 0x01, B00001010);
    WriteRegister(ak4490_1, 0x01, B00001010);
    WriteRegister(ak4490, 0x07, B00000001);
    WriteRegister(ak4490_1, 0x07, B00000001);
    WriteRegister(ak4490, 0x00, B00000000);
    WriteRegister(ak4490, 0x00, B10000111);
    WriteRegister(ak4490_1, 0x00, B00000000);
    WriteRegister(ak4490_1, 0x00, B10000111);
  }
}

void SetBits(byte bitDepth)
{
  if (bitDepth == 0)
  {
    WriteRegister(ak4490, 0x00, B00000000);
    WriteRegister(ak4490_1, 0x00, B00000000);
    WriteRegister(ak4490, 0x00, B10000111);
    WriteRegister(ak4490_1, 0x00, B10000111); // 24bit I2S
  }
  else if (bitDepth == 1)
  {
    WriteRegister(ak4490, 0x00, B00000000);
    WriteRegister(ak4490_1, 0x00, B00000000);
    WriteRegister(ak4490, 0x00, B10001111);
    WriteRegister(ak4490_1, 0x00, B10001111); // 32bit I2S
  }
  else if (bitDepth == 2)
  {
    WriteRegister(ak4490, 0x00, B00000000);
    WriteRegister(ak4490_1, 0x00, B00000000);
    WriteRegister(ak4490, 0x00, B10000101);
    WriteRegister(ak4490_1, 0x00, B10000101); // 24bit MSB
  }
  else if (bitDepth == 3)
  {
    WriteRegister(ak4490, 0x00, B00000000);
    WriteRegister(ak4490_1, 0x00, B00000000);
    WriteRegister(ak4490, 0x00, B10001001);
    WriteRegister(ak4490_1, 0x00, B10001001); // 24bit LSB
  }
  else if (bitDepth == 4)
  {
    WriteRegister(ak4490, 0x00, B00000000);
    WriteRegister(ak4490_1, 0x00, B00000000);
    WriteRegister(ak4490, 0x00, B10000001);
    WriteRegister(ak4490_1, 0x00, B10001011); // 32bit LSB
  }
  else if (bitDepth == 5)
  {
    WriteRegister(ak4490, 0x00, B00000000);
    WriteRegister(ak4490_1, 0x00, B00000000);
    WriteRegister(ak4490, 0x00, B10001101);
    WriteRegister(ak4490_1, 0x00, B10001101); // 32bit MSB
  }
  else if (bitDepth == 6)
  {
    WriteRegister(ak4490, 0x00, B00000000);
    WriteRegister(ak4490_1, 0x00, B00000000);
    WriteRegister(ak4490, 0x00, B10000001);
    WriteRegister(ak4490_1, 0x00, B10000001); // 16bit LSB
  }
}

// Set Filter
void SetFilter(int FiltNum)
{
  activeFilter = FiltNum;

  lcd.setCursor(0, 1);
  lcd.print("Filter ");

  if (isFromRemote)
  {
    isFromRemote = false;
  }
  else
  {
    sendFilter();
  }

  byte filt = 0;
  switch (FiltNum)
  {
  case 1:
    Serial.println("Sharp roll-off filter"); // SD=0, SLOW=0
    filt = ReadRegister(ak4490, 0x01);
    bitClear(filt, 5);
    WriteRegister(ak4490, 0x01, filt);
    WriteRegister(ak4490_1, 0x01, filt);
    filt = 0;
    filt = ReadRegister(ak4490, 0x02);
    bitClear(filt, 0);
    WriteRegister(ak4490, 0x02, filt);
    WriteRegister(ak4490_1, 0x02, filt);
    filt = 0;
    posCursor(pos_Flt);
    lcd.print("1");
    // lcd.print("Filter 1: Sharp RO");
    break;

  case 2:
    Serial.println("Slow roll-off filter"); // SD=0, SLOW=1
    filt = ReadRegister(ak4490, 0x01);
    bitClear(filt, 5);
    WriteRegister(ak4490, 0x01, filt);
    WriteRegister(ak4490_1, 0x01, filt);
    filt = 0;
    filt = ReadRegister(ak4490, 0x02);
    bitSet(filt, 0);
    WriteRegister(ak4490, 0x02, filt);
    WriteRegister(ak4490_1, 0x02, filt);
    filt = 0;
    posCursor(pos_Flt);
    lcd.print("2");
    // lcd.print("Filter 2: Slow RO");
    break;

  case 3:
    Serial.println("Short delay sharp roll off filter"); // SD=1, SLOW=0
    filt = ReadRegister(ak4490, 0x01);
    bitSet(filt, 5);
    WriteRegister(ak4490, 0x01, filt);
    WriteRegister(ak4490_1, 0x01, filt);
    filt = 0;
    filt = ReadRegister(ak4490, 0x02);
    bitClear(filt, 0);
    WriteRegister(ak4490, 0x02, filt);
    WriteRegister(ak4490_1, 0x02, filt);
    filt = 0;
    posCursor(pos_Flt);
    lcd.print("3");
    // lcd.print("Filter 3: ShDel/ShRO");
    break;

  case 4:
    Serial.println("Short delay slow roll off filter"); // SD=1, SLOW=1
    filt = ReadRegister(ak4490, 0x01);
    bitSet(filt, 5);
    WriteRegister(ak4490, 0x01, filt);
    WriteRegister(ak4490_1, 0x01, filt);
    filt = 0;
    filt = ReadRegister(ak4490, 0x02);
    bitSet(filt, 0);
    WriteRegister(ak4490, 0x02, filt);
    WriteRegister(ak4490_1, 0x02, filt);
    filt = 0;
    posCursor(pos_Flt);
    lcd.print("4");
    // lcd.print("Filter 4: ShDel/SlRO");
    break;
  }
  //PrintDebugReg();
}

void EnableSuperslow(bool superslow)
{
  byte filt = 0;
  switch (superslow)
  {
  case 0:
    Serial.println("Super Slow Roll-off Filter Disabled");
    filt = ReadRegister(ak4490, 0x05);
    bitClear(filt, 0);
    WriteRegister(ak4490, 0x05, filt);
    filt = 0;
    break;

  case 1:
    Serial.println("Super Slow Roll-off Filter Enabled");
    filt = ReadRegister(ak4490, 0x05);
    bitSet(filt, 0);
    WriteRegister(ak4490, 0x05, filt);
    filt = 0;
    break;
  }
  //PrintDebugReg();
}

void EnableDirectDSD(bool DirectDSD)
{
  byte dsd = 0;
  switch (DirectDSD)
  {
  case 0:
    Serial.println("DSD Normal Path Set");
    dsd = ReadRegister(ak4490, 0x06);
    bitClear(dsd, 1);
    WriteRegister(ak4490, 0x06, dsd);
    dsd = 0;
    break;

  case 1:
    Serial.println("DSD Direct Path Set");
    dsd = ReadRegister(ak4490, 0x06);
    bitSet(dsd, 1);
    WriteRegister(ak4490, 0x06, dsd);
    dsd = 0;
    break;
  }
  //PrintDebugReg();
}

void SetDSDFilter(int dsdfilter)
{
  byte dsd = 0;
  switch (dsdfilter)
  {

  case 50:
    Serial.println("DSD Cut Off Filter at 50KHz");
    dsd = ReadRegister(ak4490, 0x09);
    bitClear(dsd, 1);
    WriteRegister(ak4490, 0x09, dsd);
    dsd = 0;
    break;

  case 150:
    Serial.println("DSD Cut Off Filter at 150KHz");
    dsd = ReadRegister(ak4490, 0x09);
    bitSet(dsd, 1);
    WriteRegister(ak4490, 0x09, dsd);
    dsd = 0;
    break;
  }
  //PrintDebugReg();
}

void SetSoundMode(int soundcontrol)
{
  switch (soundcontrol)
  {

  case 1:
    Serial.println("Sound Setting 1");
    WriteRegister(ak4490, 0x08, B00000000);
    break;

  case 2:
    Serial.println("Sound Setting 2");
    WriteRegister(ak4490, 0x08, B00000001);
    break;

  case 3:
    Serial.println("Sound Setting 3");
    WriteRegister(ak4490, 0x08, B00000010);
    break;
  }
  //PrintDebugReg();
}

char getCommand() // read a character from the serial port
{
  char c = '\0';
  if (Serial.available())
  {
    c = Serial.read();
  }
  return c;
}

void displayHelp() // displays available commands through the serial port
{
  Serial.println("AK4490 DAC Controller");
  Serial.println();
  Serial.println("Press p to manually select PCM");
  Serial.println("Press q to manually select DSD");
  Serial.println("Press a to enable DirectDSD");
  Serial.println("Press b to disable DirectDSD");
  Serial.println("Press c to set DSD Filter at 50KHz");
  Serial.println("Press d to set DSD Filter at 150KHz");
  Serial.println("Press e to set the Sharp roll-off filter");
  Serial.println("Press f to set the Slow roll-off filter");
  Serial.println("Press g to set the Short delay sharp roll off filter");
  Serial.println("Press h to set the Short delay slow roll off filter");
  Serial.println("Press i to disable the Superslow filter");
  Serial.println("Press j to enable the Superslow filter");
  Serial.println("Press k to set Sound Setting 1");
  Serial.println("Press l to set Sound Setting 2");
  Serial.println("Press m to set Sound Setting 3");
  Serial.println("Press 1 to 9 to set volume from 100 to 255");
  Serial.println("Press y to display all registers");
  Serial.println("Press z to display this menu");
  Serial.println("Press 0 to reinitialize DAC");
  Serial.println();
}

// Device initialization
void DeviceInit(byte devaddr) // resets and initializes the DAC
{
  Serial.println("Initializing AK4490");
  Serial.println(devaddr);
  digitalWrite(PDN, LOW); // Reset the DAC
  delay(1);
  digitalWrite(PDN, HIGH);                 // Power up the DAC
  WriteRegister(devaddr, 0x00, B10000101); // Initialize the DAC. Sets Auto MCLK & SF, I2S 32bit and RSTN=1

  displayDACstatus(devaddr, digitalRead(PDN));
}

// Read DAC registers
byte ReadRegister(int devaddr, byte regaddr) // Read a data register value
{
  Wire.beginTransmission(devaddr);
  Wire.write(regaddr);
  Wire.endTransmission();
  Wire.requestFrom(devaddr, 1); // only one byte
  if (Wire.available())         // Wire.available indicates if data is available
    return Wire.read();         // Wire.read() reads the data on the wire
  else
    return 9999; // In no data in the wire, then return 9999 to indicate error
}

// Write register to DAC
void WriteRegister(int devaddr, byte regaddr, byte dataval) // Write a data register value
{
  Wire.beginTransmission(devaddr); // device
  Wire.write(regaddr);             // register
  Wire.write(dataval);             // data
  Wire.endTransmission();
}

// Print registers to serial monitor
void PrintDebugReg(int devaddr)
{
  Serial.println();
  Serial.println(devaddr);

  byte r = 0;
  Serial.print("Register 00: ");
  r = ReadRegister(devaddr, 0x00);
  Serial.println(r, BIN);
  r = 0;

  Serial.print("Register 01: ");
  r = ReadRegister(devaddr, 0x01);
  Serial.println(r, BIN);
  r = 0;

  Serial.print("Register 02: ");
  r = ReadRegister(devaddr, 0x02);
  Serial.println(r, BIN);
  r = 0;

  Serial.print("Register 03: ");
  r = ReadRegister(devaddr, 0x03);
  Serial.println(r, BIN);
  r = 0;

  Serial.print("Register 04: ");
  r = ReadRegister(devaddr, 0x04);
  Serial.println(r, BIN);
  r = 0;

  Serial.print("Register 05: ");
  r = ReadRegister(devaddr, 0x05);
  Serial.println(r, BIN);
  r = 0;

  Serial.print("Register 06: ");
  r = ReadRegister(devaddr, 0x06);
  Serial.println(r, BIN);
  r = 0;

  Serial.print("Register 07: ");
  r = ReadRegister(devaddr, 0x07);
  Serial.println(r, BIN);
  r = 0;

  Serial.print("Register 08: ");
  r = ReadRegister(devaddr, 0x08);
  Serial.println(r, BIN);
  r = 0;

  Serial.print("Register 09: ");
  r = ReadRegister(devaddr, 0x09);
  Serial.println(r, BIN);
  r = 0;
}

// GET SAMPLE RATE
void readSampleRate()
{
  int devaddr = ak4490;

  // initialize variables for register bits
  int DFS0, DFS1, DFS2;
  byte r;
  String currentSampleRateTitle;

  // read register 01
  r = ReadRegister(devaddr, 0x01);
  // get bits 3 (DFS0) and 4 (DFS1) (from right to left, starting with 0)
  DFS0 = bitRead(r, 3);
  DFS1 = bitRead(r, 4);

  r = 0;

  //read register 05
  r = ReadRegister(devaddr, 0x05);
  // get bit 1 (DFS2)
  DFS2 = bitRead(r, 3);

  // Define sample rates
  // 0	0	0	Normal Speed Mode	30kHz ~ 54kHz
  // 0	0	1	Double Speed Mode	54kHz ~ 108kHz
  // 0	1	0	Quad Speed Mode	  120kHz ~ 216kHz
  // 1	0	0	Oct Speed Mode 	  384kHz
  // 1	0	1	Hex Speed Mode 	  768kHz
  if (DFS2 == 0 && DFS1 == 0 && DFS0 == 0)
  {
    currentSampleRateTitle = "30kHz ~ 54kHz";
  }
  else if (DFS2 == 0 && DFS1 == 0 && DFS0 == 1)
  {
    currentSampleRateTitle = "54kHz ~ 108kHz";
  }
  else if (DFS2 == 0 && DFS1 == 1 && DFS0 == 0)
  {
    currentSampleRateTitle = "120kHz ~ 216kHz";
  }
  else if (DFS2 == 1 && DFS1 == 0 && DFS0 == 0)
  {
    currentSampleRateTitle = "384kHz";
  }
  else if (DFS2 == 1 && DFS1 == 0 && DFS0 == 1)
  {
    currentSampleRateTitle = "768kHz";
  }
}

// TOGGLE MUTE
void toggleMute()
{

  muted = !muted;

  if (muted)
  {
    tempVolume = mainVolume;
    mainVolume = 0;
    SetVol(mainVolume);
    posCursor(pos_Mute);
    lcd.print("MUTE");
  }
  else
  {
    mainVolume = tempVolume;
    SetVol(mainVolume);
    posCursor(pos_Mute);
    lcd.print("    ");
  }
}

void displayDACstatus(int devaddr, bool PDN)
{
  Serial.println(devaddr);
  Serial.println(PDN);
  if (devaddr == 16)
  {
    lcd.setCursor(5, 0);
  }
  else
  {
    lcd.setCursor(17, 0);
  }
  if (PDN)
  {
    lcd.print("On");
  }
  else
  {
    lcd.print("Off");
  }
}

void displayMode(String Mode)
{
  lcd.setCursor(0, 2);
  lcd.print("Mode: ");
  lcd.setCursor(6, 2);
  lcd.print(Mode);
}

void displayVolDiff()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set Volume Diff:");
  lcd.setCursor(0, 1);
  lcd.print(volumeDiff);
}

void displayMain()
{
  lcd.clear();
  SetVol(mainVolume);
}
