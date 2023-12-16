#include "ProximitySleep.h"
#include <TimeLib.h>  
#include <Arduino_LSM9DS1.h> // library for 9-axis IMU
#include <Arduino_LPS22HB.h> // library to read Pressure
#include <Arduino_HTS221.h> // library to read Temperature and Humidity
#include <Arduino_APDS9960.h> // library for colour, proximity and gesture recognition
#include <Arduino_BMI270_BMM150.h>  // library to read the accelerometer, magnetometer and gyroscope
#include <PDM.h> // library for microphone input
#include <ArduinoBLE.h> // library for Bluetooth communication
#include <SPI.h> // library for communication with SPI (Serial Peripheral Interface) devices
#include <SD.h> // library for SD card control
#include <math.h>

const int CHIP_SELECT = 10; // pin for Chip Select (CS) signal for the SPI communication with SD
const int NUM_MS = 2000; // number of milliseconds to delay
const float GEOMETHER_THRESHOLD = 0.03;
const float ANGLES_VAR_THRESHOLD = 0.02;
File dataFile;
// int gesture = -999; // hand gesture
int r = -1, g = -1, b = -1, luminance = -1; // color channels data
float ax = -999.0, ay = -999.0, az = -999.0; // linear acceleration
double roll = -999.00; // x axis angle
double pitch = -999.00; // y axis angle

float gx = -999.0, gy = -999.0, gz = -999.0, angularVelocity = -999.0; // angular velocity (°/sec)
float mx = -999.0, my = -999.0, mz = -999.0; // magnetic field (μT)
float pressure = -999.0, temperature = -999.0, humidity = -999.0;
float sound = -999.0; // sound volume of environment noise (dB)
static const char channels = 1; // default number of output channels
static const int frequency = 16000; // default PCM output frequency
short sampleBuffer[512]; // Buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // Number of audio samples read
bool SDEmpty = true; // Is SD card empty
bool isCharacteristicsRead = false; // Is APP reading characteristics

// Define BLE service and characteristics using UUID
BLEService myBLEService("fd09f5b1-5ebe-4df9-b2ef-b6d778ece98c");
// BLEIntCharacteristic hourBLE("917d5312-529b-459d-b708-eb81812935fa", BLERead | BLENotify);
// BLEIntCharacteristic minuteBLE("7831d64f-b9eb-4d63-8e75-2cccfd22c925", BLERead | BLENotify);
// BLEIntCharacteristic secondBLE("022e2c37-6a6a-41c6-bc06-1fae2bb53909", BLERead | BLENotify);
BLEIntCharacteristic rBLE("69ef4849-ed83-4665-9fe0-852f3fc9f330", BLERead | BLENotify);
BLEIntCharacteristic gBLE("1a7a4154-bf0b-40a5-820e-0307aaf259b7", BLERead | BLENotify);
BLEIntCharacteristic bBLE("a5807b3f-8de8-4916-aa32-b7d4f82cd7d6", BLERead | BLENotify);
BLEIntCharacteristic luminanceBLE("1d3430e9-675a-4e8a-a2ce-2d9b3ca7edc2", BLERead | BLENotify);
// BLEFloatCharacteristic axBLE("13ca0bab-f75f-4975-8862-2fad91384809", BLERead | BLENotify);
// BLEFloatCharacteristic ayBLE("4f60ab18-1824-4c26-be73-5b0c30efa225", BLERead | BLENotify);
// BLEFloatCharacteristic azBLE("640587e1-f24c-4d1d-b67d-73cee1c28382", BLERead | BLENotify);
BLEDoubleCharacteristic rollBLE("355ade2a-3451-4455-bf04-436f3c70af2b", BLERead | BLENotify ); 
BLEDoubleCharacteristic pitchBLE("6164171a-e232-407d-885f-e373cfc24554", BLERead | BLENotify ); 
BLEFloatCharacteristic gxBLE("7c4cca54-3033-490a-a2ac-cb4b8c82fc8b", BLERead | BLENotify);
BLEFloatCharacteristic gyBLE("ba581012-f1a4-4ecc-b226-5a5d0f8ab22b", BLERead | BLENotify);
BLEFloatCharacteristic gzBLE("7c2e28e8-830d-4e16-aa96-fc0f4bcbcc67", BLERead | BLENotify);
// BLEFloatCharacteristic mxBLE("a159e7ef-001b-46d5-b64a-d46f42757f1a", BLERead | BLENotify);
// BLEFloatCharacteristic myBLE("55f7adfd-1dee-4048-b46a-989b13175bef", BLERead | BLENotify);
// BLEFloatCharacteristic mzBLE("0340c189-10f8-4805-bbe8-a53c97f4b685", BLERead | BLENotify);
BLEFloatCharacteristic angularVelocityBLE("529865f7-8da6-4bd7-863c-c4028df668f8", BLERead | BLENotify);
BLEFloatCharacteristic temperatureBLE("d8fb2c21-5808-4bd8-b178-a8c587de4286", BLERead | BLENotify);
// BLEFloatCharacteristic humidityBLE("", BLERead | BLENotify);
BLEFloatCharacteristic soundBLE("125dd222-6a88-4f3f-bde8-4f428c54c4e0", BLERead | BLENotify);
BLEBoolCharacteristic isCharacteristicsReadBLE("9336826c-6f1d-42c9-9db6-7441b6254539", BLERead | BLENotify);

void setup(){
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Program started.");
  delay(200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Time you start (Hour Minute Second Day Month Year )
  setTime(0, 0, 0, 1, 5 , 2023);

  // Initialize Colour, Proximity and Gesture sensor
  if (!APDS.begin()) { 
    Serial.println("Failed to initialize Colour, Proximity and Gesture Sensor!");
    while (1);
  }
  // APDS.setGestureSensitivity(80); 

  // Initialize IMU sensor
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Accelerometer sample rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  // Initialize Pressure sensor (Barometer)
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

  // Initialize temperature and humidity sensor
  // if (!HTS.begin()) {
  //   Serial.println("Failed to initialize humidity temperature sensor!");
  //   while (1);
  // }

  // Initialize the microphone
  PDM.onReceive(onPDMdata); // Configure the data receive callback
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
  
  // Initialize SD card
  // if (!SD.begin(CHIP_SELECT)) {
  //   Serial.println("Initialization of SD card failed.");
  //   while (1);
  // }
  // dataFile = SD.open("data.csv", FILE_WRITE);
  // if (dataFile) {
  //   Serial.print("Writing to data.txt...");
  //   // dataFile.println("time,r,g,b,luminance,ax,ay,az,roll,pitch,gx,gy,gz,mx,my,mz,pressure,temperature,sound");
  //   dataFile.close();
  //   Serial.println("done.");
  // } else {
  //   Serial.println("Error opening data.txt");
  // }

  // Initialize Bluetooth Low Energy (BLE)
  if (!BLE.begin()) {
    Serial.println("Initialization of BLE module failed.");
    while (1); 
  } else {
    //Define BLE Service name and add all characteristics 
    BLE.setLocalName("Asthma Inhaler"); 
    BLE.setAdvertisedService(myBLEService); 

    // myBLEService.addCharacteristic(hourBLE); 
    // myBLEService.addCharacteristic(minuteBLE); 
    // myBLEService.addCharacteristic(secondBLE); 
    myBLEService.addCharacteristic(rBLE); 
    myBLEService.addCharacteristic(gBLE); 
    myBLEService.addCharacteristic(bBLE); 
    myBLEService.addCharacteristic(luminanceBLE); 
    // myBLEService.addCharacteristic(axBLE); 
    // myBLEService.addCharacteristic(ayBLE); 
    // myBLEService.addCharacteristic(azBLE); 
    myBLEService.addCharacteristic(rollBLE); 
    myBLEService.addCharacteristic(pitchBLE); 
    myBLEService.addCharacteristic(gxBLE); 
    myBLEService.addCharacteristic(gyBLE); 
    myBLEService.addCharacteristic(gzBLE); 
    
    // myBLEService.addCharacteristic(mxBLE); 
    // myBLEService.addCharacteristic(myBLE); 
    // myBLEService.addCharacteristic(mzBLE); 
    // myBLEService.addCharacteristic(pressureBLE); 
    myBLEService.addCharacteristic(angularVelocityBLE); 
    myBLEService.addCharacteristic(temperatureBLE); 
    myBLEService.addCharacteristic(soundBLE); 

    BLE.addService(myBLEService); 

    // Set initial default characteristics 
    // hourBLE.writeValue(-1); 
    // minuteBLE.writeValue(-1); 
    // secondBLE.writeValue(-1); 
    rBLE.writeValue(r); 
    gBLE.writeValue(g); 
    bBLE.writeValue(b); 
    luminanceBLE.writeValue(luminance); 
    // axBLE.writeValue(ax); 
    // ayBLE.writeValue(ay); 
    // azBLE.writeValue(az); 
    rollBLE.writeValue(roll); 
    pitchBLE.writeValue(pitch); 
    gxBLE.writeValue(gx); 
    gyBLE.writeValue(gy); 
    gzBLE.writeValue(gz); 
    angularVelocityBLE.writeValue(angularVelocity);
    // mxBLE.writeValue(mx); 
    // myBLE.writeValue(my); 
    // mzBLE.writeValue(mz); 
    // pressureBLE.writeValue(pressure); 
    temperatureBLE.writeValue(temperature); 
    soundBLE.writeValue(sound); 

    //Start advertising 
    BLE.advertise(); 
    Serial.println("Bluetooth Device active, waiting for connections..."); 
  }

}


void loop() {
  Serial.println("Discovering smartphone...");
  BLEDevice central = BLE.central(); // central device (smartphone)
  if (central) { // Transmit data through BLE
    Serial.print("Connected to smartphone! Device MAC address: ");
    Serial.println(central.address());
    if (!SDEmpty) {
      // transferSDData();
      // clearSDData();
      SDEmpty = true;
    }    
    while (central.connected()) { //
      printTime();     
      updateData();
      sendBluetooth();
      anglesJudgement(); // Go to sleep or not
      delay(NUM_MS);
    }
    Serial.println("Bluetooth disconnect!");
  } else { // Record data by SD card temporarily
    Serial.println("Writing to SD card...");
    printTime();     
    updateData();
    // writeSD();
    SDEmpty = false;
    anglesJudgement(); // Go to sleep or not
    delay(NUM_MS);
  }
}

void anglesJudgement() {
  int numReadings = 10;
  float averageRoll, varianceRoll, averagePitch, variancePitch;
  while (1) {
    double totalRoll = 0.0; 
    double totalPitch = 0.0; 
    double readingsRoll[numReadings];
    double readingsPitch[numReadings];
    for (int i = 0; i < numReadings; i++) {
      while (!IMU.accelerationAvailable()) {
        delay(5);
      }
      IMU.readAcceleration(ax, ay, az); 
      ax = double(ax); ay = double(ay); az = double(az); 

      roll = atan2(ay , az) * RAD_TO_DEG; // Calculate roll angle
      readingsRoll[i] = roll;
      totalRoll += roll;

      pitch = atan2((- ax) , sqrt(ay * ay + az * az)) * RAD_TO_DEG; // Calculate pitch angle 
      readingsPitch[i] = pitch;
      totalPitch += pitch;

      delay(100);
    }

    averageRoll = totalRoll / (float)numReadings;
    varianceRoll = 0.0;
    for (int i = 0; i < numReadings; i++) {
      varianceRoll += pow(readingsRoll[i] - averageRoll, 2);
    }
    varianceRoll /= numReadings;

    averagePitch = totalPitch / (float)numReadings;
    variancePitch = 0.0;
    for (int i = 0; i < numReadings; i++) {
      variancePitch += pow(readingsPitch[i] - averagePitch, 2);
    }
    variancePitch /= numReadings;
    
    // Serial.print("Average Roll: ");
    // Serial.print(averageRoll);
    // Serial.print(" | Variance Roll: ");
    // Serial.println(varianceRoll);
    // Serial.print("Average Pitch: ");
    // Serial.print(averagePitch);
    // Serial.print(" | Variance Pitch: ");
    // Serial.println(variancePitch);

    if (varianceRoll < ANGLES_VAR_THRESHOLD && variancePitch < ANGLES_VAR_THRESHOLD) {
      // Serial.println("Y");
      IMU.readGyroscope(gx, gy, gz);
      float geoSqr = gx * gx + gy * gy + gz * gz; // squre of total angular velocity
      // Serial.println(geoSqr);
      if (geoSqr > GEOMETHER_THRESHOLD) {
        Serial.println("Motion data not accurate. Try agian...");
        continue;
      } else {
        Serial.println("Device is laying statically. Go to sleep...");
        // Sleep
        ProximitySleep::sleep(0, 50);
      }
    } else {
      break;
    }
  }

  if (averageRoll < -15 || averageRoll > 15 || averagePitch < -10 || averagePitch > 10) {
    Serial.println("Device is laying non-statically. Go to sleep...");
    // Sleep
    ProximitySleep::sleep(0, 50);
  }
}

void updateData() {
  // Get gesture motion
  // while (!APDS.gestureAvailable()) {
  //   delay(5);
  // }
  // gesture = APDS.readGesture(); 
  // Serial.print("Gesture: "); Serial.println(gesture);
  // switch (gesture) {
  //   case GESTURE_UP:
  //     Serial.println("Detected UP gesture");
  //     break;

  //   case GESTURE_DOWN:
  //     Serial.println("Detected DOWN gesture");
  //     break;

  //   case GESTURE_LEFT:
  //     Serial.println("Detected LEFT gesture");
  //     break;

  //   case GESTURE_RIGHT:
  //     Serial.println("Detected RIGHT gesture");
  //     break;

  //   default:
  //     // ignore
  //     break;
  // }

  // Get color channels data
  while (!APDS.colorAvailable()) {
    delay(5);
  }
  APDS.readColor(r, g, b, luminance); 
  Serial.print("Color [R G B Luminance]: "); 
  Serial.print(r); 
  Serial.print(" "); 
  Serial.print(g); 
  Serial.print(" "); 
  Serial.print(b); 
  Serial.print(" "); 
  Serial.println(luminance);

  // Get linear acceleration data
  while (!IMU.accelerationAvailable()) {
    delay(5);
  }
  IMU.readAcceleration(ax, ay, az); 
  ax = double(ax); ay = double(ay); az = double(az); 
  Serial.print("Acceleration [X Y Z]: "); 
  Serial.print(ax); 
  Serial.print(" "); 
  Serial.print(ay); 
  Serial.print(" "); 
  Serial.println(az);
  roll = atan2(ay , az) * RAD_TO_DEG; // Calculate roll angle
  pitch = atan2((- ax) , sqrt(ay * ay + az * az)) * RAD_TO_DEG; // Calculate pitch angle 
  Serial.print("Angles [Roll Pitch]: "); 
  Serial.print(roll); 
  Serial.print(" "); 
  Serial.println(pitch);

  // Get angular velocity data
  IMU.readGyroscope(gx, gy, gz); 
  angularVelocity = sqrt(gx * gx + gy * gy + gz * gz);
  Serial.print("Gyroscope [X Y Z]: "); 
  Serial.print(gx); 
  Serial.print(" "); 
  Serial.print(gy); 
  Serial.print(" "); 
  Serial.println(gz);
  Serial.print("Angular Velocity: "); 
  Serial.println(angularVelocity);

  // Get magnetometer data
  IMU.readMagneticField(mx, my, mz); 
  Serial.print("Magnetometer [X Y Z]: "); 
  Serial.print(mx); 
  Serial.print(" "); 
  Serial.print(my); 
  Serial.print(" "); 
  Serial.println(mz);

  // Get barometer data
  pressure = BARO.readPressure(); 
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" kPa");

  // Get temperature data
  temperature = BARO.readTemperature(); 
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  // Get humidity data
  // humidity = HTS.readHumidity(); 
  // Serial.print("Humidity: ");
  // Serial.print(humidity);
  // Serial.println(" %");

  // Get sound data
  volatile float squareSum = 0.0;
  if (samplesRead) {
    for (int i = 0; i < samplesRead; i++) {
      squareSum += sq(sampleBuffer[i]) ;
    }
  }
  sound = 20 * log10(sqrt(squareSum / samplesRead));
  Serial.print("noise: ");
  Serial.print(sound);
  Serial.println(" dB");
}

void sendBluetooth() {
  // Write new data to the characteristics
  // hourBLE.writeValue(hour());
  // minuteBLE.writeValue(minute());
  // secondBLE.writeValue(second());
  rBLE.writeValue(r);
  gBLE.writeValue(g);
  bBLE.writeValue(b);
  luminanceBLE.writeValue(luminanceBLE);
  // axBLE.writeValue(ax);
  // ayBLE.writeValue(ay);
  // azBLE.writeValue(az);
  rollBLE.writeValue(roll);
  pitchBLE.writeValue(pitch);
  gxBLE.writeValue(gx);
  gyBLE.writeValue(gy);
  gzBLE.writeValue(gz);
  // mxBLE.writeValue(mx);
  // myBLE.writeValue(my);
  // mzBLE.writeValue(mz);
  angularVelocityBLE.writeValue(angularVelocity);
  temperatureBLE.writeValue(temperature);
  soundBLE.writeValue(sound);

  delay(200); // give enough time to data trasmission
}

void writeSD() {
  dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) { // 
    dataFile.print(hour());
    dataFile.print(":");
    dataFile.print(minute());
    dataFile.print(":");
    dataFile.print(second());
    dataFile.print(",");
    dataFile.print(r);
    dataFile.print(",");
    dataFile.print(g);
    dataFile.print(",");
    dataFile.print(b);
    dataFile.print(",");
    // dataFile.print(luminance);
    // dataFile.print(",");
    // dataFile.print(ax);
    // dataFile.print(",");
    // dataFile.print(ay);
    // dataFile.print(",");
    // dataFile.print(az);
    // dataFile.print(",");
    dataFile.print(roll);
    dataFile.print(",");
    dataFile.print(pitch);
    dataFile.print(",");
    // dataFile.print(gx);
    // dataFile.print(",");
    // dataFile.print(gy);
    // dataFile.print(",");
    // dataFile.print(gz);
    // dataFile.print(",");
    dataFile.print(angularVelocity);
    dataFile.print(",");
    // dataFile.print(mx);
    // dataFile.print(",");
    // dataFile.print(my);
    // dataFile.print(",");
    // dataFile.print(mz);
    // dataFile.print(",");
    // dataFile.print(pressure);
    // dataFile.print(",");
    dataFile.print(temperature);
    dataFile.print(",");
    dataFile.println(sound);
    dataFile.close();
  } else {
    Serial.println("Error opening data.txt");
  }
}

void transferSDData() {
  // delay(80000);
  dataFile = SD.open("data.csv");
  if (dataFile) {
    while (dataFile.available()) {
        String line = dataFile.readStringUntil('\n');
        parseLine(line);
        sendBluetooth();
    }
    dataFile.close();
  }
}

void parseLine(String line) {
  int lastIndex = 0;
  int nextIndex = 0;

  nextIndex = line.indexOf(',', lastIndex);
  lastIndex = nextIndex + 1;

  nextIndex = line.indexOf(',', lastIndex);
  r = line.substring(lastIndex, nextIndex).toInt();
  lastIndex = nextIndex + 1;

  nextIndex = line.indexOf(',', lastIndex);
  g = line.substring(lastIndex, nextIndex).toInt();
  lastIndex = nextIndex + 1;

  nextIndex = line.indexOf(',', lastIndex);
  b = line.substring(lastIndex, nextIndex).toInt();
  lastIndex = nextIndex + 1;

  nextIndex = line.indexOf(',', lastIndex);
  roll = line.substring(lastIndex, nextIndex).toFloat();
  lastIndex = nextIndex + 1;

  nextIndex = line.indexOf(',', lastIndex);
  pitch = line.substring(lastIndex, nextIndex).toFloat();
  lastIndex = nextIndex + 1;

  nextIndex = line.indexOf(',', lastIndex);
  angularVelocity = line.substring(lastIndex, nextIndex).toFloat();
  lastIndex = nextIndex + 1;
  
  nextIndex = line.indexOf(',', lastIndex);
  temperature = line.substring(lastIndex, nextIndex).toFloat();
  lastIndex = nextIndex + 1;

  // nextIndex = line.indexOf(',', lastIndex);
  sound = line.substring(lastIndex).toFloat();
  // lastIndex = nextIndex + 1;
}

void clearSDData() {
  if (SD.exists("data.csv")) {
    SD.remove("data.csv");
  }
  dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.close(); 
  }
}

void onPDMdata() {
  int bytesAvailable = PDM.available(); // Query the number of available bytes
  PDM.read(sampleBuffer, bytesAvailable); // Read into the sample buffer
  samplesRead = bytesAvailable / 2; // 16-bit, 2 bytes per sample
}

void printTime() {
  Serial.print("================");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.print(second());
  Serial.println("================");
}