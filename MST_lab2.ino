int sensorIN_1 = A0; 
int sensorIN_2 = A1;
int sensorIN_3 = A3;
int sensorIN_4 = A4;

int reading_1 = 0;
int reading_2 = 0;
int reading_3 = 0;
int reading_4 = 0;

int windowSize = 100;

int * sensorBuf_1 = new int[windowSize];
int * sensorBuf_2 = new int[windowSize];
int * sensorBuf_3 = new int[windowSize];
int * sensorBuf_4 = new int[windowSize];
int * sensorStateArr = new int[4];
int * tmpArr = new int[4]; 
int * multArr = new int[4];

int treshold = 50; // what

int idx = 0;

void setup() {
Serial.begin(9600); // baud 9600
}

void loop() {
if(idx>windowSize){
  
 // float var1 = calcVar(sensorBuf_1,windowSize,calcAverage(sensorBuf_1,windowSize));
  
 // float var2 = calcVar(sensorBuf_2,windowSize,calcAverage(sensorBuf_2,windowSize));
  
 // float var3 = calcVar(sensorBuf_3,windowSize,calcAverage(sensorBuf_3,windowSize));
  
 // float var4 = calcVar(sensorBuf_4,windowSize,calcAverage(sensorBuf_4,windowSize));

   idx = 0; 
}

reading_1 = analogRead(sensorIN_1);
reading_2 = analogRead(sensorIN_2);
reading_3 = analogRead(sensorIN_3);
reading_4 = analogRead(sensorIN_4);

  if(abs(reading_1-reading_2)>treshold){
	  sensorStateArr[0]++;
	  sensorStateArr[1]++; 
  }
  
  if(abs(reading_1-reading_3)>treshold){
    sensorStateArr[0]++;
	  sensorStateArr[2]++;
  }
  
  if(abs(reading_1-reading_4)>treshold){
    sensorStateArr[0]++;
	  sensorStateArr[3]++;
  }
  
  if(abs(reading_2-reading_3)>treshold){
    sensorStateArr[1]++;
	  sensorStateArr[2]++;
  }
  
  if(abs(reading_2-reading_4)>treshold){
    sensorStateArr[1]++;
	  sensorStateArr[3]++;
  }

  if(abs(reading_3-reading_4)>treshold){
    sensorStateArr[2]++;
	  sensorStateArr[3]++;
  }

tmpArr[0] = reading_1;
tmpArr[1] = reading_2;
tmpArr[2] = reading_3;
tmpArr[3] = reading_4;

// calc average according to array with multipliers -> 0*sensor reading for a borken sensor anv 1*sensor reading for functioning sensor
float avg = calcAverage(tmpArr,verifySensors(),4);
Serial.println(avg);
resetState();

sensorBuf_1[idx] = reading_1;
sensorBuf_2[idx] = reading_2;
sensorBuf_3[idx] = reading_3;
sensorBuf_4[idx] = reading_4;

idx++;

//sensorValue = analogRead(sensorPin); // read the value from the sensor
//Serial.println(sensorValue); //prints the values coming from the sensor on the screen

delay(1000);
// store values as integers (in buffer size = 100/sensor) conv to float when needed ( avg calc) 
}

int* verifySensors(){

  multArr[0] = 1;
  multArr[1] = 1;
  multArr[2] = 1;
  multArr[3] = 1;
  
  if(sensorStateArr[0] == 2 && sensorStateArr[1] == 2 && sensorStateArr[2] == 2 && sensorStateArr[3] == 2){

    Serial.println("weird state");
      return multArr;
  }
  
  int oksensors = 0;
  
  for(int i=0;i<4;i++){
    
	  if(sensorStateArr[i] == 3){
      Serial.print("Faulty sensor: ");
		  Serial.println(i+1);
		  multArr[i] = 0;
   	}
	  else{
      oksensors++;
      Serial.println(sensorStateArr[i]);
	  }
  }
  
  Serial.print("oksensors: ");
  Serial.println(oksensors);
  return multArr;
}

float calcVar(int* a,int len, int avg){
  float val = 0;
  float tmp = 0;
  for(int i=0;i<len;i++){
    tmp = float(a[i]-avg);
    val += pow(tmp,2);
  }
  val = val/len;
 return val; 
}

// params: array with values to be calculated, array with multipliers for value array, lenght of arrays
float calcAverage(int* a,int* multArr,int len){
  float val = 0;
  int divider = 0;
  for(int i=0;i<len;i++){
    val+=a[i]*multArr[i];
    divider+=multArr[i];
  }
  return float(val/divider);
}

void resetState(){ 
 sensorStateArr[0] = 0;
 sensorStateArr[1] = 0;
 sensorStateArr[2] = 0;
 sensorStateArr[3] = 0;
}