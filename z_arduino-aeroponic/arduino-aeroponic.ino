boolean runViaPython = true;   // Switch to determine whether running via python (to gcloud), or run via timers set below


int relaySolenoidPin = 4;   // The output pin on the Arduino used for the solenoid relay
int relayGrowLightsPin = 3;   // The output pin on the Arduino used for the grow lights
int relayPumpPin = 2;       // The output pin on the Arduino used for the pump

int tempWaterPin = 8;   // The input pin on the Arduino used for the water temperature sensor
int humidChamberPin = 9;   // The input pin on the Arduino used for the chamber humidity sensor
int tempChamberPin = 10;    // The input pin on the Arduino used for the chamber temperature sensor

int phTransPin = 12;		// Transitor pin for PH probe	
int ecTransPin = 13;		// Transitor pin for EC probe

int pumpState = HIGH;         // Set the Pump state to HIGH
int solenoidState = LOW;     // Set the Solenoid state to HIGH

int potPin = A0;		// The input pin on the Arduino for the Potentiometer sensor
int pressurePin = A1;  		// The input pin on the Arduino used for the water pressure sensor
int tempAirPin = A2;      	// The input pin on the Arduino used for the air temperature sensor

int phPin = A4;				// The input pin on the Arduino used for the ph sensor
int ecPin = A5;				// The input pin on the Arduino used for the ec sensor

int potVal = 0;		// Potentiometer sensor value
int pressureVal = 0;        // Water pressure sensor value
int tempAirVal = 0;			// Air Temp sensor value
int phVal = 0;				// PH sensor value
float ecVal = 0.0;				// EC sensor value

int potCaseVal = 0;	// Cases to use in switch

unsigned long oldSolenoidTime = 0;  // used to store last time solenoid state was changed
unsigned long oldPumpTime = 0;  // used to store last time pump state was changed

unsigned long oldPressureTime = 0;  // used to store last time pressure was checked
unsigned long oldTempAirTime = 0;  // used to store last time the air temperature was checked
unsigned long oldPhTime = 0;  // used to store last time PH was measured
unsigned long oldEcTime = 0;  // used to store last time EC was measured


// Set intervals
const long solenoidIntervalOn = 5000;   // change solenoid state every X milliseconds (1000 = 1 second)
const long solenoidIntervalOff = 300000;   // change solenoid state every X milliseconds (300000 = 5 minutes)
const long pumpInterval = 10000;   // change pump state every X milliseconds (1000 = 1 second)
const long pressureReadInterval = 1000;   // read pressure sensor every X milliseconds (1000 = 1 second)
const long tempAirReadInterval = 1000;   // read air temperature sensor every X milliseconds (1000 = 1 second)
const long phReadInterval = 1000;   // read PH sensor every X milliseconds (1000 = 1 second)
const long ecReadInterval = 1000;   // read EC sensor every X milliseconds (1000 = 1 second)

long solenoidInterval = solenoidIntervalOn;   // set first interval to ON

char ch = 0;           // Serial Input string
char sendBuffer[32];  // set up string for output to Raspberry Pi
String inputString = "";
boolean stringComplete=false;

// PH sensor variables:
#define Offset 0.22            //deviation compensate
#define samplingIntervalPH 20
#define printIntervalPH 800
#define ArrayLength  40    //times of collection
int pHArray[ArrayLength];   //Store the average value of the sensor feedback
int pHArrayIndex=0;

// Gravity TDS Library
#include <EEPROM.h>
#include "GravityTDS.h"
GravityTDS gravityTds;
float tempTDS = 25,tdsValue = 0;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  
  // Gravity TDS Library
  gravityTds.setPin(ecPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization

  // OUTPUTS
  pinMode(phTransPin,OUTPUT);			// Sets the ph transitor pin as an output
  pinMode(ecTransPin,OUTPUT);			// Sets the ec transitor pin as an output
  
  pinMode(relayPumpPin, OUTPUT);   // Sets the pump pin as an output
  pinMode(relaySolenoidPin, OUTPUT);   // Sets the solenoid pin as an output
  pinMode(LED_BUILTIN, OUTPUT);   // Set up LED pin as an output
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  unsigned long currentTime = millis();    // Get the current time
    
  // SOLENOID:  If current time is greater than interval
  if (currentTime > (oldSolenoidTime + solenoidInterval)){
    
    // Save new time:
    oldSolenoidTime = currentTime;

    // Switch state:
    solenoidState = !solenoidState;

    // Set next interval:
    if (solenoidState == LOW){
      solenoidInterval = solenoidIntervalOn;
    } else {
      solenoidInterval = solenoidIntervalOff;
    }
    
    // Set digital pin state:
    digitalWrite(relaySolenoidPin, solenoidState);
    digitalWrite(LED_BUILTIN, solenoidState);   // TEMP - also change LED on arduino
    
    // Write to log
    if (runViaPython==false){
      Serial.print("solenoidState: "); 
      Serial.println(solenoidState);
    };  
  };
  
  // Do different things depending on potentiameter:
  
  potVal = analogRead(potPin);
  potCaseVal = round(potVal / (1024 / 5));
  
  switch (potCaseVal){
	
	case 0:
	  
	  
	  // OPEN TO SEND TELEMETRY (i.e. IoT sensor readings) TO GOOGLE CLOUD:
	  
	  // Turn off other sensors:
	  digitalWrite(phTransPin,LOW);
	  digitalWrite(ecTransPin,LOW);
	  
	  if (Serial.available()){
	    ch = Serial.read();
	    // Pressure reading
	    if (ch == '1'){
		  sendTelemetry();  // TO UPDATE
		};
		// Air temp reading
		
	    // PH reading
	    if (ch == '4'){
		  sendTelemetry();  // TO UPDATE
		};
	    // EC reading
	    if (ch == '5'){
		 sendTelemetry();  // TO UPDATE
		};   
	  };
	  break;
	
	case 1:
	// PRESSURE SENSOR:  If current time has passed the interval
	  if (currentTime > (oldPressureTime + pressureReadInterval)){
		
		// Turn off other sensors:
		digitalWrite(phTransPin,LOW);
		digitalWrite(ecTransPin,LOW);
		
		// Save new time:
		oldPressureTime = currentTime;

		// Read analogue value:
		pressureVal = analogRead(pressurePin);

		float voltage = (pressureVal*5.0)/1024.0;                          // Convert reading to voltage
		float pressure_pascal = (3.0*((float)voltage-0.475))*1000000.0;   // Convert to pascal
		float pressure_bar = pressure_pascal/10e5;                        // Convert to bar
		int pressure_psi = pressure_bar*14.503773773;                   // Convert to PSI
	  
		// Print pressure sensor values
		Serial.print("Pressure Sensor Value: ");
		Serial.print(pressureVal);
		Serial.print(";  Pressure = ");
		Serial.print(pressure_psi);
		Serial.println(" PSI");
	  };
	  break;
	
	case 2:
	  // AIR TEMP SENSOR:  If current time has passed the interval
	  if (currentTime > (oldTempAirTime + tempAirReadInterval)){
		
		// Turn off other sensors:
		digitalWrite(phTransPin,LOW);
		digitalWrite(ecTransPin,LOW);
		
		// Save new time:
		oldTempAirTime = currentTime;

		// Read analogue value:
		tempAirVal = analogRead(tempAirPin);

		float voltageTempAir = ((float)tempAirVal*5.0)/1024;   // Convert reading to voltage
		float tempAirCelcius = (voltageTempAir-0.5)*100;       // Convert to celcius
	  
		// Print pressure sensor values
		Serial.print("AirTemp Sensor Value: ");
		Serial.print(tempAirVal);
		Serial.print(";  Celsius = ");
		Serial.print(tempAirCelcius);
		Serial.println(" C");  
	  };
	  break;
	
	case 3:
	  // PH SENSOR
	  
	  // Turn on PH sensor:
	  digitalWrite(ecTransPin,LOW); // Turn off EC sensor
	  digitalWrite(phTransPin,HIGH);
	  
	  static unsigned long samplingTime = millis();
	  static unsigned long printTime = millis();
	  static float pHValue,voltage;
	  if(millis()-samplingTime > samplingIntervalPH){
		  pHArray[pHArrayIndex++]=analogRead(phPin);
		  if(pHArrayIndex==ArrayLength)pHArrayIndex=0;
		  voltage = averageArray(pHArray, ArrayLength)*5.0/1024;
		  pHValue = 3.5*voltage+Offset;
		  samplingTime=millis();
	  }
	  //Every 800 milliseconds, print the PH reading
	  if(millis() - printTime > printIntervalPH){
		Serial.print("Voltage:");
		Serial.print(voltage,2);
		Serial.print("    pH value: ");
		Serial.println(pHValue,2);
		printTime=millis();
	  }
	  
	  break;
	
    case 4:
	  // TDS/EC SENSOR:  If current time has passed the interval
	  if (currentTime > (oldEcTime + ecReadInterval)){

		// Turn on EC sensor:
		digitalWrite(phTransPin,LOW);  // Turn off PH sensor
		digitalWrite(ecTransPin,HIGH);
		
		// Save new time:
		oldEcTime = currentTime;

		//tempTDS = readTemperature();  //add your temperature sensor and read it
		gravityTds.setTemperature(tempTDS);  // set the temperature and execute temperature compensation
		gravityTds.update();  //sample and calculate
		tdsValue = gravityTds.getTdsValue();  // then get the value
		ecVal = tdsValue*2/1000; // get to EC value

		// Print out values:
		Serial.print("EC value: ");
		Serial.print(ecVal);
		Serial.println(" mS/cm");
		
	  };
	  break;

		
	default:
	  // If nothing else matches
	  Serial.print("No match... ");
	  Serial.print("Potentiometer value = ");
	  Serial.print(potVal);
	  Serial.print("; case value = ");
	  Serial.println(potCaseVal);
      break;
          
    };

  /*
  // PUMP:  If current time is greater than interval
  if (currentTime > (oldPumpTime + pumpInterval)){
    
    // Save new time:
    oldPumpTime = currentTime;

    // Switch state:
    pumpState = !pumpState;
    
    // Set digital pin state:
    digitalWrite(relayPumpPin, pumpState);
    
    // Write to log
    Serial.print("pumpState: ");
    Serial.println(pumpState);
  };
  */
};



void sendTelemetry(){
  
      // Read analogue values:
    pressureVal = analogRead(pressurePin);

    /*
    float voltage = (pressureVal*5.0)/1024.0;                          // Convert reading to voltage
    float pressure_pascal = (3.0*((float)voltage-0.475))*1000000.0;   // Convert to pascal
    float pressure_bar = pressure_pascal/10e5;                        // Convert to bar
    int pressure_psi = pressure_bar*14.503773773;                   // Convert to PSI
    */
    // Compile string to send to Raspberry PI
    //memset(sendBuffer, 0, sizeof(sendBuffer));
    //sprintf(sendBuffer, "X psi:%d temp:%d ph:%d ec:%d z:%d", pressure_psi, 0, 0, 0, 0);
    //sprintf(sendBuffer, "X p:%d t:%d", pressure_psi, 0);
    
    Serial.print("X p:");
    Serial.println(pressureVal);
    
    /*
    sprintf(sendBuffer, "X p:%d", pressureVal);
    Serial.println(sendBuffer);
    */
}

double averageArray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to average!/n");
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
};
