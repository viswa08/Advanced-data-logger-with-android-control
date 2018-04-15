/*
 Circuit connections for Arduino Mega to modules
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 51 [pin 11 - arduino uno]
 ** MISO - pin 50 [pin 12 - arduino uno]
 ** CLK  - pin 52 [pin 13 - arduino uno]
 ** CS   - pin 4  [pin 2 - arduino uno]  
 *
 *HC 05 module connection:
 ** TX - pin 0 (default) [software serial pin 5 - arduino Mega and uno ]
 ** RX - pin 1 (default) [software serial pin 6 - arduino Mega and uno ]
 * 
 *  The circuit: 
 * RX is digital pin 5 (connect to TX of other device)
 * TX is digital pin 6 (connect to RX of other device)
 * 
 * LSMD DLHC (compass) module connection:
 * SDA - pin 20 (mega) [pin A4 - arduino uno]
 * SCL - pin 21 (mega) [pin A5 - arduino uno]
 * 
 * RTC module connection:
 * SDA - pin 20 (mega) [pin A4 - arduino uno]
 * SCL - pin 21 (mega) [pin A5 - arduino uno]

 */

#include <SPI.h>
#include <SD.h>
#include <dht.h>
#include <DS3231.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LSM303.h>

DS3231  rtc(SDA, SCL);// Init the DS3231 using the hardware interface
SoftwareSerial mySerial(10, 11); // RX, TX
LSM303 compass;
File myFile;
dht DHT;
#define DHT11_PIN A0
const int buttonPin = 8;     // the number of the pushbutton pin
//const int ledPin =  13;      // the number of the LED pin
// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

String bluInputStr;
float Xm, Ym, Zm, Xa, Ya, Za, roll, pitch, roll_print, pitch_print, Heading, fXm_comp, fYm_comp;
int r0 = 0;      //value of select pin at the 4052 (s0)
int r1 = 0;      //value of select pin at the 4052 (s1)
int r2 = 0;      //value of select pin at the 4052 (s2)
int r3 = 0;
int r4 = 0;
int r5 = 0;
int sensorPin = A0;
int s0 = 22;
int s1 = 23;
int s2 = 24;
int s3 = 25;
int s4 = 26;
int s5 = 270;
int count = 0;   //which y pin we are selecting
String fileName = "Details.csv";
struct compassOp
{
  float head, rollOp, pitchOp;
};
struct compassOp compOp;
String compassSoft(); //function declaration for the structure to work. function has to be invoked before using structure in the function

void setup() {
  Serial.begin(9600); //DEBUG
  mySerial.begin(9600);//bluetooth
  //TODO: initialize the LED pin as an output:
  //pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  Wire.begin();
  rtc.begin();// Initialize the rtc object
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-654,   -835,   -424};
  compass.m_max = (LSM303::vector<int16_t>){+694,   +527,   +739};
  sdIni();
  MuxInit();
}

//sd card code starts here//
void sdIni()
{
    //Serial.print("Initializing SD card...");  
    if (!SD.begin(4)) 
    {
      Serial.println("initialization failed!");
    }
    else
    {
      Serial.println("initialization done."); 
    }    
}

void sdWrite(String sensorData, String compassData)
{    
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  // if the file opened okay, write to it:
  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile) 
  {
    Serial.println("Writing to Details.csv...");   //DEBUG     
    myFile.print(rtc.getDateStr());
    myFile.print(",");
    myFile.print(rtc.getTimeStr());
    myFile.print(",");
    myFile.print(sensorData);
    //myFile.print(","); // to be removed
    myFile.println(compassData);
    Serial.println("done.");//DEBUG
    myFile.close();// close the file
  } 
  else 
  {
    // if the file didn't open, print an error:
    Serial.println("error opening Details.csv");//DEBUG
    mySerial.println("error opening Details.csv");
  }
}

void sdRead()
{ 
  //Serial.begin(9600);//bluetooth
  // re-open the file for reading:
  if (SD.exists(fileName))
  {
    myFile = SD.open(fileName);
    File archiveFile;
    if (myFile) 
    {
      Serial.println("Read from file."); //DEBUG
      // read from the file until there's nothing else in it:
      String d=rtc.getDateStr(FORMAT_SHORT);
      String t=rtc.getTimeStr(FORMAT_SHORT);
      String dateTime = (String)d[0]+(String)d[1]+(String)d[3]+(String)d[4]+(String)t[0]+(String)t[1]+(String)t[3]+(String)t[4];
      String fileArchive = dateTime+".csv";
      //Serial.println(fileArchive); //DEBUG
      archiveFile = SD.open(fileArchive, FILE_WRITE);
      if (archiveFile)
      {
        while (myFile.available()) 
        {
          char c = myFile.read();
          //Serial.println(c); //DEBUG
          archiveFile.print(c);
          mySerial.write(c);
        }
        archiveFile.close();
        myFile.close();// close the file
        SD.remove(fileName);
      }
      else
      {
        //Serial.println("error creating archive");//DEBUG
        mySerial.println("error creating archive");
        myFile.close();// close the file
      }
    } 
    else 
    {
      // if the file didn't open, print an error:
      //Serial.println("error opening Details.csv");//DEBUG
      mySerial.println("error opening Details.csv");
    }
  }
  else
  {
    mySerial.println("Details file not exist!");
  }
}
//sd card code ends here

void loop() 
{
  buttonState = digitalRead(buttonPin);
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) 
  {
    Serial.println("button pressed");//DEBUG
    String sensorData = MuxVal();
    String compassData = compassSoft();
    sdWrite(sensorData,compassData);   
    // turn LED on:
    //digitalWrite(ledPin, HIGH);
    delay(500);    
  } 
  else 
  {
    // turn LED off:
    //digitalWrite(ledPin, LOW);
  }

  if(mySerial.available())
  {
    delay(10);
    char bluInput = mySerial.read();
    bluInputStr = bluInput; 
    if (bluInputStr.length() >0)
    { 
      Serial.println(bluInputStr); //DEBUG
      if(bluInputStr == "a")
      {
        sdRead();
      }
      bluInputStr="";
    }
  }
  else
  {
      //Serial.println("bluetooth connection not available!"); //DEBUG
  }
}

String compassSoft()
{  
  String compassData="";
  compass.read();
  
  //Magnetometer
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
  
  compassData+=(String)Heading;
  compassData+=(String)roll_print;
  compassData+=(String)pitch_print;
  delay(1000);
  return compassData;
}

void MuxInit()
{
  //mux logic initialisation starts here
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(s4, OUTPUT);
  pinMode(s5, OUTPUT);
  //mux logic initialisation ends here
}

String MuxVal()
{
  String muxOp="";
  for (count=0; count<=44; count++) 
  {
      // select the bit
      r0 = bitRead(count,0);    // use this with arduino uno and mega (and newer versions)
      r1 = bitRead(count,1);    // use this with arduino uno and mega (and newer versions)
      r2 = bitRead(count,2);
      r3 = bitRead(count,3);
      r4 = bitRead(count,4);
      r5 = bitRead(count,5);
            

      //r0 = count & 0x01;      // old version of setting the bits
      //r1 = (count>>1) & 0x01; // old version of setting the bits
      //r2 = (count>>2) & 0x01; // old version of setting the bits      

      digitalWrite(s0, r0);
      digitalWrite(s1, r1);
      digitalWrite(s2, r2);
      digitalWrite(s3, r3);
      digitalWrite(s4, r4);
      digitalWrite(s5, r5);
      
      int val = analogRead(sensorPin);    
      //Serial.print("the value of photodiode  ");//DEBUG
      //Serial.print(count);//DEBUG
      //Serial.print("\t");//DEBUG
      //Serial.println(val); //DEBUG
      muxOp = muxOp + (String)val+",";
    }
    delay(250); // 
    return muxOp;
}

