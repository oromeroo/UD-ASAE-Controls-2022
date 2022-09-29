
// Declaration of Variables -------------------------
int data_1;
int data_2;
int data_3;

String outputData;

// Data Collection ----------------------------------
String data_collect(){
  outputData = ""; 

// Collecting data
  data_1 = 1;
  data_2 = 2;
  data_3 = 3;
  
// Putting Data in a Packet

  outputData += String(data_1); 
  outputData +="," + String(data_2);
  outputData +="," + String(data_3);

  return outputData;
}

void setup() {
  Serial.begin(115200); 
}

void loop() {

  Serial.println(data_collect()); 
  delay(100);
}
