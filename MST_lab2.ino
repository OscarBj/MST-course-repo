int sensorIN_1 = A0; // select the input pin for LDR
int sensor1_Points = 0;
int sensorIN_2 = A1;
int sensor2_Points = 0;
int sensorIN_3 = A3;
int sensor3_Points = 0;
int sensorIN_4 = A4;
int sensor4_Points = 0;

int windowSize = 100;

int * sensorBuf_1 = new int[windowSize];
int * sensorBuf_2 = new int[windowSize];
int * sensorBuf_3 = new int[windowSize];
int * sensorBuf_4 = new int[windowSize];
int * tmpArr = new int[4]; 
int * sensorStateArr = new int[4];
int * sensorPoints = new int[4];

int treshold = 200;

int idx = 0;

int pair12 = 0;
int pair13 = 0;
int pair14 = 0;
int pair23 = 0;
int pair24 = 0;
int pair34 = 0;

void setup() {
Serial.begin(9600); //sets serial port for communication
}

void loop() {
if(idx>windowSize){
  
  float var1 = calcVar(sensorBuf_1,windowSize,calcAverage(sensorBuf_1,windowSize));
  
  float var2 = calcVar(sensorBuf_2,windowSize,calcAverage(sensorBuf_2,windowSize));
  
  float var3 = calcVar(sensorBuf_3,windowSize,calcAverage(sensorBuf_3,windowSize));
  
  float var4 = calcVar(sensorBuf_4,windowSize,calcAverage(sensorBuf_4,windowSize));

   idx = 0; 
}

sensorBuf_1[idx] = analogRead(sensorIN_1);

sensorBuf_2[idx] = analogRead(sensorIN_2);

sensorBuf_3[idx] = analogRead(sensorIN_3);

sensorBuf_4[idx] = analogRead(sensorIN_4);

  pair12 = abs(sensorBuf_1[idx]-sensorBuf_2[idx]);
  if(pair12>treshold){
    sensor1_Points++;
    sensor2_Points++;
  }
  pair13 = abs(sensorBuf_1[idx]-sensorBuf_3[idx]);
  if(pair13>treshold){
    sensor1_Points++;
    sensor3_Points++;
  }
  pair14 = abs(sensorBuf_1[idx]-sensorBuf_4[idx]);
  if(pair14>treshold){
    sensor1_Points++;
    sensor4_Points++;
  }
  pair23 = abs(sensorBuf_2[idx]-sensorBuf_3[idx]);
  if(pair23>treshold){
    sensor2_Points++;
    sensor3_Points++;
  }
  pair24 = abs(sensorBuf_2[idx]-sensorBuf_4[idx]);
  if(pair24>treshold){
    sensor2_Points++;
    sensor4_Points++;
  }
  pair34 = abs(sensorBuf_3[idx]-sensorBuf_4[idx]);
  if(pair34>treshold){
    sensor3_Points++;
    sensor4_Points++;
  }

  
  for(int i=0;i<4;i++){
    
  }
  
  
  Serial.println(sensor1_Points);
  Serial.println(sensor2_Points);
  Serial.println(sensor3_Points);
  Serial.println(sensor4_Points);
  
  verifySensors();
  
  resetState();

tmpArr[0] = sensorBuf_1[idx];
tmpArr[1] = sensorBuf_2[idx];
tmpArr[2] = sensorBuf_3[idx];
tmpArr[3] = sensorBuf_4[idx];

int avg = calcAverage();

//Serial.println(avg);
//Serial.println(calcVar(tmpArr,4,avg));

idx++;

//sensorValue = analogRead(sensorPin); // read the value from the sensor
//Serial.println(sensorValue); //prints the values coming from the sensor on the screen

delay(100);
// store values as integers (in buffer size = 100/sensor) conv to float when needed ( avg calc) 
}

void verifySensors(){
  if(sensor1_Points == 2 && sensor2_Points == 2 && sensor3_Points == 2 && sensor3_Points == 2){
    sensorStateArr = {1,1,1,1}; // special case check variance
  }
  if(sensor1_Points == 3){
    sensorStateArr[0] = 0
  }
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

float calcAverage(int* a,int len){
  float val = 0;
  for(int i=0;i<len;i++){
    val+=a[i];
  }
  return float(val/len);
}

void resetState(){
 
  pair12 = 0;
  pair13 = 0;
  pair14 = 0;
  pair23 = 0;
  pair24 = 0;
  pair34 = 0;
  
  sensor1_Points = 0;
  sensor2_Points = 0;
  sensor3_Points = 0;
  sensor4_Points = 0;
  // reset buffers???????
}
// print avg for working ones and sensor not wokring
