/*
  SD card read/write

 This example shows how to read and write data to and from an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
 *
 *HC 05 module connection:
 ** TX - pin 0 (default)[can be changed if Softwareserial is being used ]
 ** RX - pin 1 (default) [can be changed if Softwareserial is being used ]
 * 
 *  The circuit: 
 * RX is digital pin 5 (connect to TX of other device)
 * TX is digital pin 6 (connect to RX of other device)

 */

#include <SPI.h>
#include <SD.h>
#include <dht.h>
#include <DS3231.h>

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX

// Init the DS3231 using the hardware interface
DS3231  rtc(SDA, SCL);

#include <Wire.h>
#include <LSM303.h>

LSM303 compass;

File myFile;
dht DHT;

#define DHT11_PIN A0

const int buttonPin = 8;     // the number of the pushbutton pin
//const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

char bluInput;               //bluetooth signal to read data from SD card
String bluInputStr;

float Xm, Ym, Zm, Xa, Ya, Za, roll, pitch, roll_print, pitch_print, Heading, fXm_comp, fYm_comp;
 int r0 = 0;      //value of select pin at the 4051 (s0)

  int r1 = 0;      //value of select pin at the 4051 (s1)

  int r2 = 0;      //value of select pin at the 4051 (s2)

  int sensorPin = A0;


  int s0 = 2;

  int s1 = 3;



  int count = 0;   //which y pin we are selecting

struct compassOp
{
  float head, rollOp, pitchOp;
};

struct compassOp compOp;

void setup() {

  // initialize the LED pin as an output:
  //pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  //pinMode(4, OUTPUT);
    pinMode(s0, OUTPUT);

    pinMode(s1, OUTPUT);

  Wire.begin();
  compass.init();
  compass.enableDefault();

  // Initialize the rtc object
  rtc.begin();

  compass.m_min = (LSM303::vector<int16_t>){-654,   -835,   -424};
  compass.m_max = (LSM303::vector<int16_t>){+694,   +527,   +739};
  
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("Type,\tHumidity(%),\tTemperature(C)");
  sdIni();
  
  //mySerial.println("Type,\tHumidity(%),\tTemperature(C)");

//DHTAcq();
//sdCardWrite("test3.txt");
//sdCardRead("test3.txt");

}

  float compassSoft(); //function declaration for the structure to work. function has to be invoked before using structure in the function


  void sdWrite()
  {

    
      // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("Details4.csv", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    //Serial.print("Writing to Details4.csv...");
        //myFile.println("Type,\tHumidity(%),\tTemperature(C)");
        //myFile.print(rtc.getDOWStr());
        //myFile.print(" ");
        // Send date
        compassSoft();
        myFile.print(rtc.getDateStr());
        myFile.print(" ");
        // Send time
        myFile.print(rtc.getTimeStr());
        myFile.print(" ");
        //myFile.print("Val:\t");
        int chk = DHT.read11(DHT11_PIN);        
        myFile.print(DHT.humidity, 1);
        myFile.print(",\t\t");
        myFile.print(DHT.temperature, 1);
        myFile.print(",\t\t");
        myFile.print(compOp.head/10);
        Serial.println(compOp.head/10);
        myFile.print(",\t\t");
        myFile.print(compOp.rollOp);
        myFile.print(",\t\t");
        myFile.println(compOp.pitchOp);
        
    // close the file:
    myFile.close();
    //Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening Details4.csv");
  }
  }
  void sdIni()
  {
    /*while (!Serial) 
    {
    ; // wait for serial port to connect. Needed for native USB port only
    }*/


    Serial.print("Initializing SD card...");
  
    if (!SD.begin(4)) {
      Serial.println("initialization failed!");
      return;
    }
    Serial.println("initialization done."); 
  }


 void sdRead()
  {
    
  // re-open the file for reading:
  myFile = SD.open("Details4.csv");
  if (myFile) {
    //Serial.println("Details4.csv:");
    //mySerial.println("Details4.csv:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      //Serial.write(myFile.read());
      mySerial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    //Serial.println("error opening Details4.csv");
    mySerial.println("error opening Details4.csv");
  }
  }


void loop() {
  // nothing happens after setup
  //Serial.println("test 1.. 2.. 3");
  //delay(1000);

  buttonState = digitalRead(buttonPin);
  
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    Serial.println("button pressed");
     MuxVal();
    //head = compassSoft();
    sdWrite();
    //sdRead();    
      
    // turn LED on:
    //digitalWrite(ledPin, HIGH);
    delay(500);    
    
  } else {
    // turn LED off:
    //digitalWrite(ledPin, LOW);
  }

  if(mySerial.available())
  {
    delay(10);
    bluInput = mySerial.read();
    Serial.println(bluInput);
    if(bluInput == '#')
    {
      //break;
    }
    bluInputStr = bluInput;
  
    if (bluInputStr.length() >0)
    {
      
      Serial.println(bluInputStr);
      if(bluInputStr == "a")
      {
        
        sdRead();
      }
      bluInputStr="";
    }
  }

  
}

float compassSoft()
{

  
  compass.read();

 /*     snprintf(report, sizeof(report), "A: %6d %6d %6d    M: %6d %6d %6d",
        compass.a.x, compass.a.y, compass.a.z,
        compass.m.x, compass.m.y, compass.m.z);
      Serial.println(report);
      */
  //Acceralerometer
  
  Xm = compass.m.x;
  Ym = compass.m.y;
  Zm = compass.m.z;

  //Acceralerometer

  Xa = compass.a.x;
  Ya = compass.a.y;
  Za = compass.a.z;

//roll and pitch

  roll = atan2(Ya, sqrt(Xa*Xa + Za*Za ));
  pitch = atan2(Xa, sqrt(Ya*Ya + Za*Za));

  roll_print = roll*180.0/M_PI;
  pitch_print = pitch*180.0/M_PI;

// Tilt compensated magnetic sensor measurements

fXm_comp = Xm*cos(pitch)+Zm*sin(pitch);
fYm_comp = Xm*sin(roll)*sin(pitch)+Ym*cos(roll)-Zm*sin(roll)*cos(pitch);


// Arctangent of y/x
Heading = (atan2(fYm_comp,fXm_comp)*180.0)/M_PI;
if (Heading < 0)
Heading += 360;

compOp.head = Heading;
compOp.rollOp = roll_print;
compOp.pitchOp = pitch_print;

  

  //Serial.print(roll_print, 2);Serial.print("\t"); Serial.print(pitch_print, 2); Serial.print("\t");
  //Serial.print("Heading "); Serial.println(Heading);     

  delay(1000);
}



void MuxVal()
{
  for (count=0; count<=2; count++) {



      // select the bit  

      r0 = bitRead(count,0);    // use this with arduino 0013 (and newer versions)     

      r1 = bitRead(count,1);    // use this with arduino 0013 (and newer versions)     





      //r0 = count & 0x01;      // old version of setting the bits

      //r1 = (count>>1) & 0x01;      // old version of setting the bits

      //r2 = (count>>2) & 0x01;      // old version of setting the bits



      digitalWrite(s0, r0);

      digitalWrite(s1, r1);

      int val = analogRead(sensorPin);    

       Serial.print("the value of photodiode  ");
       Serial.print(count);
       Serial.print("\t");
       Serial.println(val);

    }  

}



