
int tempPropagatorPin = A2; // The input pin on the Arduino used for the air temperature sensor
int relayHeatmatPin = 7;    // The output pin on the Arduino used for the heatmat relay

int heatmatState = HIGH;    // Set the Heatmat state to HIGH

int tempAirVal = 0;			// Air tempC sensor value
double tempC = 0;        	//The variable we will use to store temperature in degrees. 

double tempC_wantMin = 15;
double tempC_wantMax = 20;
double tempC_adjLag = 1;

double tempC_wantMinAdj = 0;
double tempC_wantMaxAdj = 0;

// Set intervals
const long delayRead = 1000;   // check tempC every X milliseconds


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  // OUTPUTS
  pinMode(relayHeatmatPin, OUTPUT);   // Sets the heatmat pin as an output
  pinMode(LED_BUILTIN, OUTPUT);   // Set up LED pin as an output
  
  // Set up adjusted
  
}

void loop() {
  
  // put your main code here, to run repeatedly: 
  sensorInput = analogRead(A0);    			//read the analog sensor and store it
  tempC = (double)sensorInput / 1024;       //find percentage of input reading
  tempC = tempC * 5;                 		//multiply by 5V to get voltage
  tempC = tempC - 0.5;               		//Subtract the offset 
  tempC = tempC * 100;               		//Convert to degrees 
 
  Serial.print("Current Temperature: ");
  Serial.print(tempC);
  Serial.println(" degrees C ");
  
  // calculate temperature boundaries 
  tempC_wantMinAdj = tempC_wantMin + tempC_adjLag;
  tempC_wantMaxAdj = tempC_wantMax - tempC_adjLag;
  
  
  if (tempC < tempC_wantMin){
	heatmatState = LOW;
	digitalWrite(relayHeatmatPin, heatmatState);
  };

  if (tempC > tempC_wantMax){
	heatmatState = HIGH;
	digitalWrite(relayHeatmatPin, heatmatState);
  };  
  
  delay(delayRead);    // Get the current time
    
};


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
