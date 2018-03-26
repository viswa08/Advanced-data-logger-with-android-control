#include <dht.h>
#include <Stepper.h>
dht DHT;
int relay =7;
#define relay1 13
#define DHT11_PIN A0
const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
String voice;

void setup()
{
  Serial.begin(9600);
  Serial.println("DHT PROGRAM");
  Serial.print("LIBRARY VERSION: ");
  Serial.println(DHT_LIB_VERSION);
  Serial.println();
  Serial.println("Type,\tStatus,\tHumidity(%),\tTemperature(C)");
  myStepper.setSpeed(60);
 
}
void loop()
{
  ReadSensors();
  MotorAction();
  AndroidAction();
 delay(1000);
}
void ReadSensors()
{
  Serial.print("DHT11, \t");
  int chk = DHT.read11(DHT11_PIN);
  Serial.print(DHT.humidity, 1);
  Serial.print(",\t");
  Serial.print(DHT.temperature, 1);
  delay(2000);

if (DHT.temperature > 10 && DHT.humidity > 10)
  {
    digitalWrite(7, HIGH);
    
  }
  else
  {
    digitalWrite(7, LOW);
    
  }
  delay(500);
}
void MotorAction()
{
  Serial.println("clockwise");
  myStepper.step(stepsPerRevolution);
  delay(500);
  
}
void AndroidAction()
{
  while(Serial.available())    //Check if there are available bytes to read
  {
    delay(10);                 //Delay to make it stable
    char c = Serial.read();    //Conduct a serial read
    if (c == '#'){
      break;                   //Stop the loop once # is detected after a word
    }
    voice += c;                //Means voice = voice + c
  }
    if (voice.length() >0)
    {
      Serial.println(voice);
      if(voice == "*power on"){
        switchon();
      }               //Initiate function switchon if voice is switch on
      else if(voice == "*switch off"){
        switchoff();
      }               //Initiate function switchoff if voice is switch off
      else if(voice == "*cooler on"){   
//You can replace 'lamp on' with anything you want...same applies to others
        digitalWrite(relay1, HIGH);
      }
      else if(voice == "*cooler off"){
        digitalWrite(relay1, LOW);
      }
      
      
      
      voice="";
       }
}
void switchon()               //Function for turning on relays
{
  digitalWrite(relay1, HIGH);
  Serial.println("fan on,\t");
  
}
void switchoff()              //Function for turning on relays
{
  digitalWrite(relay1, LOW);
  Serial.println("fan off,\t");
}
