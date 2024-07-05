#include <Wire.h>
#include <RTClib.h>
#include <DFRobot_SIM7000.h>
#include <SoftwareSerial.h>
#include <arduinoFFT.h> 
#include <TimerOne.h> 

#define THINGSPEAK_API_KEY "8LUTHG19097RVDRH"

#define PIN_TX 7
#define PIN_RX 10


#define VOLTAGE_PIN A0
#define CURRENT_PIN A1

#define VOLTAGE_PIN A0
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03


SoftwareSerial mySerial(PIN_RX, PIN_TX);
DFRobot_SIM7000 sim7000(&mySerial);

RTC_DS3231 rtc; 


const uint16_t samples = 256;
const uint16_t samplingFrequency = 500;

float vReal[samples];
float vImag[samples];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency);

struct Reading {
  
  float voltage;
  float current;
   unsigned long timestamp;
};

Reading readings[samples];
volatile int readingCount = 0;

void initSIM7000();
Reading takeReading();
void printReading(Reading reading);
void performFFT();
void sendDataToThingspeak();

void setup() {
  Serial.begin(9600);
  mySerial.begin(19200);

  initSIM7000();  
  Timer1.initialize(2000);  // Initialize Timer1 to trigger every 2ms
  Timer1.attachInterrupt(timerISR);  // Attach the timerISR function to the interrupt
  Timer1.start();
 
 delay(1000);
}
void loop() {

float iPeak=0;
  float vPeak=0;
  float vAngle=0;
  float iAngle=0;
  float iMaxMagnitude=0;
  float vMaxMagnitude=0;
  int maxIndex=0;
   float freqV=0;
    float freqC=0;
  // Inside the loop() function after processing the voltage signal
  if (readingCount == 256) {
     for (int i = 0; i < samples; i++) {
       printReading(readings[i]);
        vReal[i] = readings[i].voltage;
        vImag[i] = 0.0; // Ensure imaginary part is zeroed for voltage
    }
  }

// Perform FFT computations for the voltage signal
FFT.dcRemoval();
FFT.windowing(vReal, samples, FFTWindow::Hamming, FFTDirection::Forward);
FFT.compute(FFTDirection::Forward);

// Voltage Magnitude at Frec HZ is example
FFT.complexToMagnitude();
Serial.println("Voltage Computed magnitudes:");
PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);



for (uint16_t i = 0; i < samples / 2; i++) {
    // Compute frequency
    freqV = float(i * samplingFrequency) / samples;
   
    vPeak = FFT.majorPeak(vReal,samples,samplingFrequency);
    vAngle = atan2(vImag[i], vReal[i]);
    vAngle = vAngle * 180.0 / PI;
  
}


zeroI();
// Process FFT for the current signal
for (int i = 0; i < samples; i++) {
    vReal[i] = readings[i].current;
    vImag[i] = 0.0; // Ensure imaginary part is zeroed for current
}
FFT.dcRemoval();
FFT.windowing(vReal, samples, FFTWindow::Hamming, FFTDirection::Forward);
FFT.compute(FFTDirection::Forward);
FFT.complexToMagnitude();
Serial.println("Current Computed magnitudes:");
PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
// Analyze and print magnitude spectrum for current signal
for (uint16_t i = 0; i < samples / 2; i++) {
    freqC = float(i * samplingFrequency) / samples;
    
    iPeak = FFT.majorPeak(vReal,samples,samplingFrequency);
    iAngle = atan2(vImag[i], vReal[i]);
    iAngle = iAngle * 180.0 / PI;
    
    
}

Serial.print("Voltage Max Frequency=");
Serial.print(vPeak, 6);
Serial.println("Hz");


Serial.print("Current Max Frequency==");
Serial.print(iPeak, 6);
Serial.println("Hz");


Serial.print("vAngle=");
Serial.print(vAngle, 6);
Serial.println("Hz");


Serial.print("iAngle=");
Serial.print(iAngle, 6);
Serial.println("Hz");

sendDataToThingspeak();
 
readingCount = 0;
  }

void sendDataToThingspeak() {
  int currentIndex = 0;

  while (currentIndex < 256) {
    if (sim7000.openNetwork(DFRobot_SIM7000::eTCP, "api.thingspeak.com", 80) != 0) {
      Serial.println("Failed to open network connection. Retrying...");
      delay(5000);
      continue;
    }

    String url = "GET /update?api_key=" + String(THINGSPEAK_API_KEY) + "&field1=" + String(readings[currentIndex].voltage, 2) + "&field2=" + String(readings[currentIndex].current, 2)
     + "&field3=" + String(readings[currentIndex].timestamp) + " HTTP/1.1\r\nHost: api.thingspeak.com\r\nConnection: close\r\n\r\n";

    Serial.print("Sending data to ThingSpeak: ");
    Serial.println(url);

    if (sim7000.send((char*)url.c_str())) {
      sim7000.closeNetwork();
      currentIndex++;
    } else {
      Serial.println("Failed to send data. Moving to the next data point...");
      delay(5000);
      sim7000.closeNetwork();
      currentIndex++;
    }
  }
}

Reading takeReading() {
 
  float voltage = analogRead(VOLTAGE_PIN) * (5.0 / 1023.0);
  float current = analogRead(CURRENT_PIN) * (5.0 / 1023.0);


 return{ voltage, current, millis()};
}

void printReading(Reading reading) {
 
  Serial.print(reading.voltage, 2); // Print voltage with 2 decimal places
  Serial.print(",");
  Serial.print(reading.current, 3); // Print current with 2 decimal places
  Serial.print(",");
  Serial.println(reading.timestamp);
}

void zeroI() {
  for (uint16_t i = 0; i < samples; i++) {
    vImag[i] = 0.0;
  } 
}

void plot(float value, bool last) {
 
  Serial.print(value);
  if(last==false) Serial.print(",");
  else Serial.println();
}

void initSIM7000() {
  // SIM7000 initialization and setup here
}

void PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType) {
  for (uint16_t i = 0; i < bufferSize; i++) {
    float abscissa;
    switch (scaleType) {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
        break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

void timerISR() {
  if (readingCount < samples) {
    Reading reading = takeReading();
    readings[readingCount] = reading;
    readingCount++;
  }
}





