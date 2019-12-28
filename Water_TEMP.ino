

//__________________________________________________________________________________________________________________________________________________________________________
//                                                                    LIBRARIES
//___________________________________________________________________________________________________________________________________________________________________________

#include <EEPROM.h>
#include <GravityTDS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <SPI.h>
#include "DFRobot_SHT20.h"
//__________________________________________________________________________________________________________________________________________________________________________
//                                                                     PINS
//___________________________________________________________________________________________________________________________________________________________________________

#define TdsSensorPin A0
#define SensorPin A1            //pH meter Analog output to Arduino Analog Input 0
#define ONE_WIRE_BUS 2         // for temperature Pin
#define Offset -2.00          //deviation compensate
#define LED 13
#define samplingInterval 60
#define printInterval 10000
#define ArrayLenth  1000     //times of collection
#define BAUDRATE    9600
#define SEALEVELPRESSURE_HPA (1013.25)


//__________________________________________________________________________________________________________________________________________________________________________
//                                                                     OTHERS
//___________________________________________________________________________________________________________________________________________________________________________

GravityTDS gravityTds;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DFRobot_SHT20   sht20;//I2C //for the BME280
//__________________________________________________________________________________________________________________________________________________________________________
//                                                                     VARIABLES
//___________________________________________________________________________________________________________________________________________________________________________
//for TDS 
float temperature,tdsValue = 0;

//For PH values
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;
float test;
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;


  
//for water FLow

byte sensorInterrupt = 0;     // 0 = digital pin 2
byte sensorPin       = 3;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
// litre/minute of flow.
float calibrationFactor = 4.5;
volatile byte pulseCount;  
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;
//for temperature
float humd ,temp;                

//__________________________________________________________________________________________________________________________________________________________________________
//                                                                     SETUP
//___________________________________________________________________________________________________________________________________________________________________________

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(BAUDRATE);
  Serial1.begin(BAUDRATE);
  Serial.println("STRT");
  Serial1.println("complete!"); 
  // for the temperature 
    sht20.initSHT20();                                  // Init SHT20 Sensor
    delay(100);
   // sht20.checkSHT20();                                 

  // for TDS
    gravityTds.setPin(TdsSensorPin);
    gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
    gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
    gravityTds.begin();  //initialization
    sensors.begin();
    //for PH
    pinMode(LED,OUTPUT);
    //for the Water flow 
    pinMode(sensorPin, INPUT);
    digitalWrite(sensorPin, HIGH);

  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;

  // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a FALLING state change (transition from HIGH
  // state to LOW state)
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

}

void loop()
{
  // put your main code here, to run repeatedly:
TDS();
delay(500);
PH();
delay(500);
Flow();
delay(500);
SHT20VALUES();//taking SHT20 values
delay(1000);
//for print on serial 

    Serial.print(String(temp, 1));
    Serial.print("|");
    Serial.print(humd, 1);
      Serial.print("|");
    Serial.print(pHValue,2);
      Serial.print("|");
    Serial.print(tdsValue,0);
      Serial.print("|");
    Serial.print (temperature);
      Serial.print("|"); 
    Serial.print(totalMilliLitres/1000);
       Serial.println();
    //need to add oxygen
    delay(10000);
 

}


void TDS()
{
  sensors.requestTemperatures(); 
  
  Serial1.print("Celsius temperature: ");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial1.print(sensors.getTempCByIndex(0)+0.7); 
  Serial1.print(" - Fahrenheit temperature: ");
  Serial1.println(sensors.getTempFByIndex(0));
  delay(1000);



  
    temperature = sensors.getTempCByIndex(0)+0.7; //add your temperature sensor and read it
    gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate
    tdsValue = gravityTds.getTdsValue();  // then get the value
    Serial1.print(tdsValue,0);
    Serial1.println("ppm");
    delay(1000);
}


void PH()
{
   if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5/1024;
      if (voltage<2.4)
      {
      pHValue = 3.5*voltage-Offset;
      }
      else if (voltage>2.4)
      {
      pHValue = 3.5*voltage+Offset;
      }
       test=14-pHValue;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {     Serial1.println();
        Serial1.print("Voltage:");
        Serial1.print(voltage,2);
        Serial1.print("    pH value: ");
        Serial1.print(pHValue,2);
        Serial1.print("      reversed value: ");
        Serial1.println(test,2);
        digitalWrite(LED,digitalRead(LED)^1);
        printTime=millis();
  }
}
//for pH p  
double avergearray(int* arr, int number)
{
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

//for Flow
void Flow()
{
   if((millis() - oldTime) > 1000)    // Only process counters once per second
  { 
    // Disable the interrupt while calculating flow rate and sending the value to
    // the host
    detachInterrupt(sensorInterrupt);
        
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    
    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    oldTime = millis();
    
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;
    
    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
      
    unsigned int frac;
    
    // Print the flow rate for this second in litres / minute
    Serial1.print("Flow rate: ");
    Serial1.print(int(flowRate));  // Print the integer part of the variable
    Serial1.print("L/min");
    Serial1.print("\t");       // Print tab space

    // Print the cumulative total of litres flowed since starting
    Serial1.print("Output Liquid Quantity: ");        
    Serial1.print(totalMilliLitres);
    Serial1.println("mL"); 
    Serial1.print("\t");       // Print tab space
    Serial1.print(totalMilliLitres/1000);
    Serial1.print("L");
    

    // Reset the pulse counter so we can start incrementing again
    pulseCount = 0;
    
    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  }
}


//Insterrupt Service Routine
void pulseCounter()
{
  // Increment the pulse counter
  pulseCount++;

}

//for temperature
void SHT20VALUES()
{
      humd = sht20.readHumidity();                  // Read Humidity
      temp = sht20.readTemperature();               // Read Temperature
    Serial1.print(" Temperature:");
    Serial1.print(temp, 1);
    Serial1.print("C");
    Serial1.print(" Humidity:");
    Serial1.print(humd, 1);
    Serial1.print("%");
    Serial1.println();
    delay(1000);
}


