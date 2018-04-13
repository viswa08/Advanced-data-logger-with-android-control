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
int sensorPin = A0;
int s0 = 2;
int s1 = 3;
int count = 0;   //which y pin we are selecting

struct compassOp
{
  float head, rollOp, pitchOp;
};
struct compassOp compOp;
float compassSoft(); //function declaration for the structure to work. function has to be invoked before using structure in the function

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

void sdWrite(String sensorData)
{    
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  // if the file opened okay, write to it:
  myFile = SD.open("Details5.csv", FILE_WRITE);
  if (myFile) 
  {
    Serial.println("Writing to Details5.csv...");   //DEBUG     
    compassSoft();
    myFile.print(rtc.getDateStr());
    myFile.print(",");
    myFile.print(rtc.getTimeStr());
    myFile.print(",");
    myFile.print(sensorData);
    //myFile.print(","); // to be removed
    myFile.print(compOp.head/10);
    //Serial.println(compOp.head/10); //DEBUG
    myFile.print(",");
    myFile.print(compOp.rollOp);
    myFile.print(",");
    myFile.println(compOp.pitchOp);
    Serial.println("done.");//DEBUG
    myFile.close();// close the file
  } 
  else 
  {
    // if the file didn't open, print an error:
    Serial.println("error opening Details5.csv");//DEBUG
    mySerial.println("error opening Details5.csv");
  }
}

void sdRead()
{ 
  //Serial.begin(9600);//bluetooth
  // re-open the file for reading:
  myFile = SD.open("Details5.csv");
  if (myFile) 
  {
    Serial.println("Read from file."); //DEBUG
    // read from the file until there's nothing else in it:
    while (myFile.available()) 
    {
      //Serial.write(myFile.read()); //DEBUG
      mySerial.write(myFile.read());
    }
    myFile.close();// close the file
  } 
  else 
  {
    // if the file didn't open, print an error:
    //Serial.println("error opening Details5.csv");//DEBUG
    mySerial.println("error opening Details5.csv");
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
    sdWrite(sensorData);   
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

float compassSoft()
{  
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
  delay(1000);
}

void MuxInit()
{
  //mux logic initialisation starts here
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  //mux logic initialisation ends here
}

String MuxVal()
{
  String muxOp="";
  for (count=0; count<=2; count++) 
  {
      // select the bit
      r0 = bitRead(count,0);    // use this with arduino uno and mega (and newer versions)
      r1 = bitRead(count,1);    // use this with arduino uno and mega (and newer versions)      

      //r0 = count & 0x01;      // old version of setting the bits
      //r1 = (count>>1) & 0x01; // old version of setting the bits
      //r2 = (count>>2) & 0x01; // old version of setting the bits      

      digitalWrite(s0, r0);
      digitalWrite(s1, r1);
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

