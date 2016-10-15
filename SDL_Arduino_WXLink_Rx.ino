// SDL_Arduino_WXLink_Rx
// SwitchDoc Labs May 2016
//

#define SOFTWAREVERSION 001

#include <SoftwareSerial.h>

#include <Wire.h>

#include "Crc16.h"

//Crc 16 library (XModem)
Crc16 crc;

#define LED 3

SoftwareSerial SoftSerial(8, 9); // TX, RX
byte buffer[75];
byte lastGoodMessage[64];
int buflen = 0;

long consecutiveGoodMessages;
long lastGoodMessageID;
long goodMessages;
long badMessages;

int toggle = 0;


void printBuffer(byte *buffer, int buflen)
{
  int i;
  for (i = 0; i < buflen; i++)
  {
    Serial.print("i=");
    Serial.print(i);
    Serial.print(" | ");
    Serial.println(buffer[i], HEX);
  }

}


int convert2BytesToInt(byte *buffer, int bufferStart)
{

  union u_tag {
    byte b[2];
    int fval;
  } u;

  u.b[0] = buffer[bufferStart];
  u.b[1] = buffer[bufferStart + 1];


  return u.fval;

}

long convert4BytesToLong(byte *buffer, int bufferStart)
{

  union u_tag {
    byte b[4];
    long fval;
  } u;

  u.b[0] = buffer[bufferStart];
  u.b[1] = buffer[bufferStart + 1];
  u.b[2] = buffer[bufferStart + 2];
  u.b[3] = buffer[bufferStart + 3];

  return u.fval;

}


float convert4BytesToAM2315Float(byte *buffer, int bufferStart)
{


  union u_tag {
    byte b[4];
    float fval;
  } u;


  u.b[0] = buffer[bufferStart + 3];
  u.b[1] = buffer[bufferStart + 2];
  u.b[2] = buffer[bufferStart + 1];
  u.b[3] = buffer[bufferStart + 0];  
  Serial.print("fval=");
  Serial.println(u.fval);

  return u.fval;


}

float convert4BytesToFloat(byte *buffer, int bufferStart)
{


  union u_tag {
    byte b[4];
    float fval;
  } u;


  u.b[0] = buffer[bufferStart + 0];
  u.b[1] = buffer[bufferStart + 1];
  u.b[2] = buffer[bufferStart + 2];
  u.b[3] = buffer[bufferStart + 3]; 
  

  return u.fval;

 






}



int interpretBuffer(byte *buffer, int buflen)
{
  if (!((buffer[0] == 0xAB) && (buffer[1] == 0x66)))
  {
    // start bytes are not in buffer - reject
    return 1; // no start bytes
  }
  Serial.println("Start Bytes Found");

  if (buflen != 63)
  {
    return 2; // buflen wrong
  }
  unsigned short checksumValue;

  // calculate checksum
  checksumValue = crc.XModemCrc(buffer, 0, 59);
  Serial.print("crc = 0x");
  Serial.println(checksumValue, HEX);

  if ((checksumValue >> 8) != buffer[61])
  {
    // bad checksum
    return 3;  // bad checksum

  }
  if ((checksumValue & 0xFF) != buffer[62])
  {
    // bad checksum
    return 3;  // bad checksum

  }



  //

  Serial.println("Correct Buffer Length");

  Serial.print("Protocol=");
  Serial.println(buffer[2]);

  Serial.print("TimeSinceReboot(msec)=");
  Serial.println(convert4BytesToLong(buffer, 3));

  Serial.print("Wind Direction=");
  Serial.println(convert2BytesToInt(buffer, 7));

  Serial.print("Average Wind Speed (KPH)=");
  Serial.println(convert4BytesToFloat(buffer, 9));

  Serial.print("Wind Clicks=");
  Serial.println(convert4BytesToLong(buffer, 13));

  Serial.print("Total Rain Clicks=");
  Serial.println(convert4BytesToLong(buffer, 17));

  Serial.print("Max Wind Gust=");
  Serial.println(convert4BytesToFloat(buffer, 21));


  
  Serial.print("Outside Temperature=");
  Serial.println(convert4BytesToFloat(buffer, 25));

  Serial.print("OT Hex=");
  Serial.print(buffer[25], HEX);
  Serial.print(buffer[26], HEX);
  Serial.print(buffer[27], HEX);
  Serial.println(buffer[28], HEX);

  Serial.print("Outside Humidity=");
  Serial.println(convert4BytesToFloat(buffer, 29));

  Serial.print("BatteryVoltage=");
  Serial.println(convert4BytesToFloat(buffer, 33));
  Serial.print("BatteryCurrent=");
  Serial.println(convert4BytesToFloat(buffer, 37));
  Serial.print("LoadCurrent=");
  Serial.println(convert4BytesToFloat(buffer, 41));
  Serial.print("SolarPanelVoltage=");
  Serial.println(convert4BytesToFloat(buffer, 45));
  Serial.print("SolarPanelCurrent=");
  Serial.println(convert4BytesToFloat(buffer, 49));

  Serial.print("AuxA=");
  Serial.println(convert4BytesToFloat(buffer, 53));

  Serial.print("Message ID=");
  Serial.println(convert4BytesToLong(buffer, 57));


  Serial.print("Checksum High=0x");
  Serial.println(buffer[61], HEX);
  Serial.print("Checksum Low=0x");
  Serial.println(buffer[62], HEX);



  return 0;

}

// I2C interface - This device communicates with the master via I2C.

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void I2CrequestEvent() {
  // respond with the lastGoodMessage
  // as expected by master
  //Wire.write(lastGoodMessage,64);
  byte command;
  command = Wire.read();
  Serial.print("Command=");
  Serial.println(command);
  byte myBuffer[32];
  int i;
  if (command == 0)
  {
    for (i = 0; i < 32; i++)
    {
      myBuffer[i] = lastGoodMessage[i];
    }
    Wire.write(myBuffer, 32);
    return;
  }

  if (command == 1)
  {

    for (i = 0; i < 32; i++)
    {
      myBuffer[i] = lastGoodMessage[i + 32];
    }
    Wire.write(myBuffer, 32);
    return;
  }

  if (command == 255)
  {

    Serial.print("Toggle=");
    Serial.println(toggle);
    // toggle between 0 and 1
    if (toggle == 1)
    {
      toggle = 0;
      for (i = 0; i < 32; i++)
      {
        myBuffer[i] = lastGoodMessage[i + 32];
      }
      Wire.write(myBuffer, 32);
      return;

    }

    if (toggle == 0)
    {
      toggle = 1;
      for (i = 0; i < 32; i++)
      {
        myBuffer[i] = lastGoodMessage[i];
      }
      Wire.write(myBuffer, 32);
      return;


    }

  }


}

void I2CreceiveEvent(int count)
{
  Serial.print("RE Count =");
  Serial.println(count);

}

void setup()
{

  // set up I2C at address 0x08
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(I2CrequestEvent); // register event
  Wire.onReceive(I2CreceiveEvent);

  SoftSerial.begin(9600);               // the SoftSerial baud rate
  Serial.begin(115200);  // Debugging only
  Serial.println("-------Receive Started---------");

  consecutiveGoodMessages = 0;
  int i;
  for (i = 0; i < 64; i++)
  {
    lastGoodMessage[i] = 0;


  }


  pinMode(LED, OUTPUT);
}

void loop()
{


  if (SoftSerial.available() == 63)              // if date is coming from software serial port
  {

    Serial.println("Message Received");
    while (SoftSerial.available())         // reading data into char array
    {
      buffer[buflen++] = SoftSerial.read();   // writing data into array
      //if (buflen == 64)break;
    }
    for (int i = 0; i < buflen; i++) {
      Serial.print(buffer[i], HEX);           // if no data transmission ends, write buffer to hardware serial port
    }
    Serial.println();
    //printBuffer( buffer, buflen);


    int interpretResult = interpretBuffer(buffer, buflen);

    switch (interpretResult)
    {
      case 0:
        {
          Serial.println("Good Message");
          int previousGoodMessageID = lastGoodMessageID;
          goodMessages++;

          lastGoodMessageID = convert4BytesToLong(buffer, 57);

          if (lastGoodMessageID == previousGoodMessageID + 1)
          {
            consecutiveGoodMessages++;
          }
          Serial.print("Current Message ID=");
          Serial.print(lastGoodMessageID);
          Serial.print(" Consecutive Good Messages =");
          Serial.println(consecutiveGoodMessages);

          // copy bytes to lastGoodMessage array
          // disable interrupts
          noInterrupts();
          int i;
          for (i = 0; i < 63; i++)
          {
            lastGoodMessage[i] = buffer[i];

          }
          lastGoodMessage[63] = 0;

          // enable interrupts
          interrupts();

        }
        break;
      case 1:
        Serial.println("Bad Message - No Start Bytes");
        badMessages++;
        consecutiveGoodMessages = 0;
        resetSoftSerialBuffer();
        break;
      case 2:
        Serial.println("Bad Message - buffer length incorrect");
        badMessages++;
        consecutiveGoodMessages = 0;
        resetSoftSerialBuffer();
        break;
      case 3:
        Serial.println("Bad Message - Bad Checksum");
        badMessages++;
        consecutiveGoodMessages = 0;
        resetSoftSerialBuffer();
        break;
      default:

        Serial.print("Bad Message - Unknown Return Code =");
        Serial.println(interpretResult);
        badMessages++;
        consecutiveGoodMessages = 0;
        resetSoftSerialBuffer();
        break;
    }

    Serial.print("Good Messages: ");
    Serial.print(goodMessages);
    Serial.print(" Bad Messages: ");
    Serial.println(badMessages);

    clearBufferArray(buflen);              // call clearBufferArray function to clear the stored data from the array


    int i;



    buflen = 0;
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);

  }

}

void clearBufferArray(int buflen)              // function to clear buffer array
{
  for (int i = 0; i < buflen; i++)
  {
    buffer[i] = NULL; // clear all index of array with command NULL
  }
}

void resetSoftSerialBuffer()
{

  Serial.println("Resetting SoftSerial Buffer");
  // delay 1 second
  delay(1000);

  // clear Softserial buffer

  char discard;

  while (SoftSerial.available() > 0)
  {
    discard = SoftSerial.read();

  }
}



