#include "BluetoothSerial.h"

//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234"; // Change this to more secure PIN.

String device_name = "ESP32-BT-Slave";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;


int trigPin = 19;      // trig pin of HC-SR04
int echoPin = 18;     // Echo pin of HC-SR04

int revleft4 = 22;       // REVerse motion of Left motor
int fwdleft5 = 23;       // ForWarD motion of Left motor
int revright6 = 32;      // REVerse motion of Right motor
int fwdright7 = 33;      // ForWarD motion of Right motor
int enablerPinA = 12;
int enablerPinB = 14;

int vacuumMotor = 2;
unsigned char GOFLAG = 0;
uint8_t dataRead = 0;
unsigned long queue[10];
int qIndex = 0;

void setup() {
    Serial.begin(115200);
  SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  //Serial.printf("The device with name \"%s\" and MAC address %s is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), SerialBT.getMacString()); // Use this after the MAC method is implemented
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif
  //Serial.begin(9600);
  pinMode(revleft4, OUTPUT);      // set Motor pins as output
  pinMode(fwdleft5, OUTPUT);
  pinMode(revright6, OUTPUT);
  pinMode(fwdright7, OUTPUT);
  pinMode(vacuumMotor, OUTPUT);
  pinMode(trigPin, OUTPUT);         // set trig pin as output
  pinMode(echoPin, INPUT);          // set echo pin as input to capture reflected waves
  pinMode(enablerPinA, OUTPUT);
  pinMode(enablerPinB, OUTPUT);

}

void moveForward() {

  digitalWrite(fwdright7, HIGH);  // move forward
  digitalWrite(revright6, LOW);
  digitalWrite(fwdleft5, HIGH);
  digitalWrite(revleft4, LOW);
  digitalWrite(vacuumMotor, HIGH);

}
void turnRight() {
  digitalWrite(fwdright7, LOW);
  digitalWrite(revright6, HIGH);
  digitalWrite(fwdleft5, LOW);
  digitalWrite(revleft4, LOW);
  delay(200);  // Adjust the delay based on the actual turning time for a 90-degree turn
  brake();     // Stop the motors after turning
}


void brake() {
  digitalWrite(fwdright7, HIGH);
  digitalWrite(revright6, HIGH);
  digitalWrite(fwdleft5, HIGH);
  digitalWrite(revleft4, HIGH);
}

void sort(unsigned long arr[], int arr_size) {
  int i, j;
  for (i = 0; i < arr_size - 1; i++) {
    for (j = 0; j < arr_size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        int tmp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = tmp;  
      }
    }  
  }
}


unsigned long getMedian() {
  // Copy the original queue to a local array
  unsigned long queue_sorted[10];
  int i = 0;
  for (i = 0; i < 10; i++) {
    queue_sorted[i] = queue[i];
  }
  sort(queue_sorted, 10);
  
  return queue_sorted[10/2];
}


void loop() {
  digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);     // send waves for 10 us
    delayMicroseconds(10);
    unsigned long duration = pulseIn(echoPin, HIGH); // receive reflected waves
    if (duration == 0) duration = 10000;
    //Serial.print("duration: ");
    //Serial.println(duration);
    unsigned long raw_distance = duration / 58;   // convert to distance
    queue[qIndex] = raw_distance;
    qIndex = (qIndex+1) % 10;
    unsigned long medianDistance = getMedian();
//    Serial.print("distance: ");
    Serial.println(raw_distance);
    //delay(10);
  if (SerialBT.available()) {
    dataRead = SerialBT.read();
    if (dataRead == '1') GOFLAG=1;
    else if (dataRead == '0') GOFLAG = 0;
  }
  Serial.print("dataRead: ");
  Serial.println(dataRead,HEX);
  Serial.print("GOFLAG: ");
  Serial.println(GOFLAG);
  if(GOFLAG && medianDistance > 15){
    moveForward();
    }
  if(GOFLAG && medianDistance <= 15){
          // Stop and turn around 180 degrees to the right while avoiding the wall
      brake();
      turnRight();
    }
    else if(GOFLAG==0){
      brake();
      digitalWrite(vacuumMotor, LOW);
    }
 }
