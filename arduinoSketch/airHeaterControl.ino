#include "PID_BV.h"
#include "Lowpass_BV.h"
#include <ESP8266WiFi.h>
#include "secrets.h"
#include "ThingSpeak.h"
#include <ModbusIP_ESP8266.h>

#define MBread_raw_PV 0
#define MBread_Filtered_pv 1
#define MBread_Output 2
#define MBread_spare3 3
#define MBread_spare3 4
#define MBread_spare3 5
#define MBread_spare3 6
#define MBread_spare3 7
#define MBread_spare3 8
#define MBread_spare3 9

#define MBwrite_Setpoint 10
#define MBwrite_P 11
#define MBwrite_I 12
#define MBwrite_D 13


// Work for IOT lab with Arduino https://www.halvorsen.blog/documents/teaching/courses/iot/lab_arduino.php
// Thingspeak (over Wifi)
// Modbus for Pid Param tuning
// RC filter for output. (AND variation with DAC)

// GUI in Ignition?


//-- MODBUS --//
#define n_floats 20               // Number of floats to transfer
ModbusTCP mb;                     //ModbusIP object
float mbv_floats[n_floats] = {};  //Array for modbus floating point values


// ---------------- CONTROLLER PARAMS ---------------- //
double PID1_Setpoint, PID1_Input, PID1_Output;
double Kp = 0.5, Ti = 10000, Td = 0;
PID_BV myController(&PID1_Input, &PID1_Output, &PID1_Setpoint, Kp, Ti, Td, true);
int sampleTimeMS = 200;
Lowpass_BV lowpass_pv(sampleTimeMS / 1000.0, 0.5);


// ---------------- SIM PARAMS ---------------- //

unsigned long sim_lastTime = 0;
float sim_tankLevel = 0;
int delayTimeMS = 2000;  // Set your desired delay time

// Function prototypes
float SimulateWaterTank(float controlSignal);
void UpdateControlSignalQueue(float controlSignal);

// Circular buffer for control signals
float *controlSignalQueue;
int controlSignalQueueSize;
int rear = 0;  // Declare rear as a global variable

// ---------------- SIM PARAMS END ---------------- //

//-- WIFI --//
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password
int keyIndex = 0;           // your network key Index number (needed only for WEP)
WiFiClient client;

//-- THINGSPEAK --//
unsigned long myChannelNumber = SECRET_CH_ID;
const char *myWriteAPIKey = SECRET_WRITE_APIKEY;


// ---------------- TIMING ---------------- //
unsigned long tol_print;           // Time of last print
unsigned long tol_thingspeak = 0;  //time of last thingspeak sending. lastConnectionTime
unsigned long updateInterval_thingspeak = 20000;


// ---------------- IO-signals ---------------- //
const int analogInPin = A0;
const int analogOutPin = 14;
bool doSim = true;


void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  ThingSpeak.begin(client);  // Initialize ThingSpeak

  myController.SetOutputLimits(0, 5);
  myController.SetSampleTime(sampleTimeMS);

  if (doSim) {
    initSimulation();
  } else {
    pinMode(analogOutPin, OUTPUT);
    pinMode(analogInPin, INPUT);
  }



  Serial.println("setpoint, input, raw, output");

  // mb.server();  //Start Modbus IP
  // mb.addIreg(100);  // Add SENSOR_IREG register - Use addIreg() for analog Inputs
  // mb.addHreg(0, 0, 1);
  init_Modbus_data();    // Synchronizing the array with variable data.
  mb.onConnect(cbConn);  // Add callback on connection event
  mb.server(502);

  if (!mb.addHreg(0, 0xF0F0, n_floats * 2)) Serial.println("Error adding HREG");  // Add Hregs
  mb.onGetHreg(0, cbRead, n_floats * 2);                                          // Add callback on Coils value get
  mb.onSetHreg(0, cbWrite, n_floats * 2);

  mb.onRequestSuccess(cbRequestSuccess);
}

void initSimulation() {
  // Initialize controlSignalQueue with dynamic memory allocation
  controlSignalQueueSize = delayTimeMS / sampleTimeMS;
  controlSignalQueue = new float[controlSignalQueueSize];

  // Initialize the circular buffer with zeros
  for (int i = 0; i < controlSignalQueueSize; i++) {
    controlSignalQueue[i] = 0.0;
  }
}

bool Metronome(unsigned long &last, unsigned long maxInterval) {
  unsigned long now = millis();
  if (now - last >= maxInterval) {
    last = now;
    return true;
  }
  return false;
}

int VoltageToPWM(float voltage) {
  float byteValue = voltage / 5 * 200;
  return 255 - int(byteValue);
}

void loop() {

  WifiReconnect();
  mb.task();  // Process modbus task

  PID1_Setpoint = mbv_floats[MBwrite_Setpoint];  // Setpoint water level


  bool doCompute = myController.Compute();

  //if (millis() - sim_lastTime > sampleTimeMS) {
  if (doCompute) {

    float rawPv = 0;
    if (doSim) {
      // Update the control signal queue
      UpdateControlSignalQueue(PID1_Output);
      // Use the delayed control signal from the queue
      rawPv = SimulateWaterTank(controlSignalQueue[rear]);

    } else {  // Real world measurements.
      float out_Capped = min(5.0 , PID1_Output);  // Maximum allowed output voltage is 5 volt.
      analogWrite(analogOutPin, VoltageToPWM(out_Capped)); // WRite analog signal to output pin

      float norm = (float(analogRead(analogInPin))-204.8) / 819.2;  // Read signal // signal from 0 to 0,8
      rawPv = norm * 30 + 20;                                        // temperature value;
    }

    float filteredPv = lowpass_pv.update(rawPv);
    PID1_Input = filteredPv;


    mbv_floats[MBread_raw_PV] = rawPv;
    mbv_floats[MBread_Filtered_pv] = filteredPv;
    mbv_floats[MBread_Output] = PID1_Output;
    mbv_floats[3] = PID1_Setpoint;  // Duplicate
  }

  if (Metronome(tol_print, 500)) {  // Print if timer has elapsed
    // Serial.print(PID1_Setpoint);
    // Serial.print(", ");
    // Serial.print(PID1_Input);
    // Serial.print(", ");
    Serial.println(myController.GetKp());
  }

  if (Metronome(tol_thingspeak, updateInterval_thingspeak))  // Write to thingspeak if timer has elapsed
  {
    Thingspeak_Write();
  }
}

float SimulateWaterTank(float controlSignal) {
  unsigned long now = millis();
  float dt = float(now - sim_lastTime) / 1000.0;  // time in seconds
  sim_lastTime = now;

  float effectiveControlSignal = controlSignal;
  float inflowRate = effectiveControlSignal * 0.5;  // Arbitrary factor for simulation
  float outflowRate = 0.8;

  sim_tankLevel += (inflowRate - outflowRate) * dt;  // Time step of the simulation

  // Ensure the water level doesn't go negative
  if (sim_tankLevel < 0.0) {
    sim_tankLevel = 0.0;
  }
  return sim_tankLevel + (random(-1000, 1000) / 1000.0);
}

void UpdateControlSignalQueue(float controlSignal) {
  controlSignalQueue[rear] = controlSignal;
  rear = (rear + 1) % controlSignalQueueSize;
}

void Thingspeak_Write() {
  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
  // pieces of information in a channel.  Here, we write to field 1.
  //int x = ThingSpeak.writeField(myChannelNumber, 1, number, myWriteAPIKey);

  ThingSpeak.setField(1, float(PID1_Input));
  ThingSpeak.setStatus("Good");
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  if (x == 200) {
    Serial.println("Channel update successful.");
  } else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
}


void WifiReconnect() {
  // Connect or reconnect to WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      client.stop();
      WiFi.begin(ssid, pass);
      Serial.print(".");
      delay(5000);
      attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nFailed to connect to WiFi. Please check your credentials.");
      return;
    }

    Serial.println("\nConnected.");
    Serial.println(WiFi.localIP());
  }
}



// MODBUS: Callback function to read corresponding float values
uint16_t cbRead(TRegister *reg, uint16_t val) {
  if (reg->address.address >= 0 && reg->address.address < n_floats * 2) {
    uint16_t floatNum = reg->address.address / 2;
    uint16_t byteNum = reg->address.address % 2;

    if (byteNum == 0) {
      uint32_t myuint32 = reinterpret_cast<uint32_t &>(mbv_floats[floatNum]);
      val = static_cast<uint16_t>(myuint32 & 0xFFFF);  // Get first 16 bits
    } else if (byteNum == 1) {
      uint32_t myuint32 = reinterpret_cast<uint32_t &>(mbv_floats[floatNum]);
      val = static_cast<uint16_t>(myuint32 >> 16);  // Shift right by 16 bits
    }
  }
  return val;
}

// MODBUS: Callback function to write float values
uint16_t cbWrite(TRegister *reg, uint16_t val) {
  if (reg->address.address >= 0 && reg->address.address < n_floats * 2) {
    uint16_t floatNum = reg->address.address / 2;
    uint16_t byteNum = reg->address.address % 2;

    if (byteNum == 0) {
      uint32_t myuint32 = reinterpret_cast<uint32_t &>(mbv_floats[floatNum]);
      myuint32 &= 0xFFFF0000;                                        // Clear the lower 16 bits
      myuint32 |= static_cast<uint32_t>(val);                        // Write the first 16 bits of val
      mbv_floats[floatNum] = *reinterpret_cast<float *>(&myuint32);  // Update the float
    } else if (byteNum == 1) {
      uint32_t myuint32 = reinterpret_cast<uint32_t &>(mbv_floats[floatNum]);
      myuint32 &= 0x0000FFFF;                                        // Clear the upper 16 bits
      myuint32 |= (static_cast<uint32_t>(val) << 16);                // Write the next 16 bits of val
      mbv_floats[floatNum] = *reinterpret_cast<float *>(&myuint32);  // Update the float
    }
  }
  return val;
}

// MODBUS: Callback function for client connect. Returns true to allow connection.
bool cbConn(IPAddress ip) {
  Serial.println(ip);
  return true;
}

// Function checking when we are writing to PID controller settings through modbus.
Modbus::ResultCode cbRequestSuccess(Modbus::FunctionCode fc, const Modbus::RequestData data) {

  uint16_t writeRangeStart = 10;
  uint16_t writeRangeEnd = 20;
  uint16_t WriteEnd = data.reg.address + data.regCount;
  Serial.println("Updating parameters");
  //if (writeRangeEnd > data.reg.address && WriteEnd > writeRangeStart && fc == Modbus::FC_WRITE_REGS) {
  // We are writing to one of the WRITE registers.
  // Updating


  PID1_Setpoint = mbv_floats[MBwrite_Setpoint];
  Kp = mbv_floats[MBwrite_P];
  Ti = mbv_floats[MBwrite_I];
  Td = mbv_floats[MBwrite_D];
  myController.SetTunings(Kp, Ti, Td);
  //}

  return Modbus::EX_SUCCESS;
}

void init_Modbus_data() {
  mbv_floats[MBwrite_Setpoint] = PID1_Setpoint;
  mbv_floats[MBwrite_P] = Kp;
  mbv_floats[MBwrite_I] = Ti;
  mbv_floats[MBwrite_D] = Td;
}
