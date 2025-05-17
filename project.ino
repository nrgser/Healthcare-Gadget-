#include <Arduino.h>
#include <Adafruit_ADXL345_U.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <math.h>
#include <SoftwareSerial.h>

// ----------------------------------------------------
//  Configuration
// ----------------------------------------------------
#define ECG_SAMPLE_INTERVAL   500   
#define ECG_BUFFER_SIZE       4096    
#define ECG_PUBLISH_INTERVAL  60000
#define ACC_DETECT_INTERVAL   500
#define EMG_PHONE_NUMBER      ""
#define USER_ID 1

#define SENSOR A0

// GPRS APN
#define APN_NAME "CMNET"

// MQTT server details
#define SERVER_IP ""
#define SERVER_PORT 12345

// Define the mutex
portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

SemaphoreHandle_t tcpMutex;



// Accelerometer threshold for fall detection
float accThreshold = 12.0;

// GPS location storage
String last_location = "";


// ECG buffer method
static int ecgBufferA[ECG_BUFFER_SIZE];
static unsigned long ecgTimestampsA[ECG_BUFFER_SIZE];

static int ecgBufferB[ECG_BUFFER_SIZE];
static unsigned long ecgTimestampsB[ECG_BUFFER_SIZE];

static volatile int fillCountA = 0;
static volatile int fillCountB = 0;

static volatile bool usingBufferA = true;

// ----------------------------------------------------
//  Global Serial Objects
// ----------------------------------------------------
SoftwareSerial mc60Serial(16, 17); // RX=16, TX=17 

// Accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// ----------------------------------------------------
//  Forward Declarations
// ----------------------------------------------------
void setupMC60();
bool openTCPConnection();
void closeTCPConnection();
bool sendTCP(const String &data);
bool sendBufferOverTCP(int *ecgBuf, unsigned long *tsBuf, int count);
void sendFallAlertToServer();
void sendLocationToServer(const char* location);

void recordECG(void* parameter);
void publishECGTask(void* parameter);
void recordAccelerometer(void* parameter);
void gpsTask(void* parameter);

void callTheSupervisor();
void getGPSLocation();
void sendATCommand(const char* cmd, int delayMs = 500);

// ----------------------------------------------------
//  Setup
// ----------------------------------------------------
void setup() {
  // Setup the serial ports (UART)
  Serial.begin(9600);
  delay(500);

  mc60Serial.begin(9600);
  delay(500);

  while (!Serial) {}
  while (!mc60Serial) {}

  Serial.println(">> Start...");

  tcpMutex = xSemaphoreCreateMutex();
  
  // Setup the MC60 and connect to MQTT broker
  setupMC60();

  // Settings for ECG tracker
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

  // Setup the ADXL
  if (!accel.begin()) {
    Serial.println("Failed to initialize ADXL345!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);

// Create tasks
  xTaskCreatePinnedToCore(recordECG,          "ECG Sensing Task",   4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(publishECGTask,     "ECG Publishing Task",4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(recordAccelerometer,"Accelerometer Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(gpsTask,            "GPS Task",           4096, NULL, 1, NULL, 1);  
}

// ----------------------------------------------------
//  Loop
// ----------------------------------------------------
void loop() {
  
}

String readSerialResponse() {
    String response = "";
    while (mc60Serial.available()) {
        char c = (char)mc60Serial.read();
        response += c;
    }
    return response;
}

void serial_print(String m){
  Serial.println(m);
  delay(50);
}

void recordECG(void* parameter) {
  while (true) {
    int ecgValue = adc1_get_raw(ADC1_CHANNEL_0);
    unsigned long timestamp = millis();
    taskENTER_CRITICAL(&myMutex);
    {
    // Store in buffer
      if (usingBufferA) {
        // Buffer A is active
        if (fillCountA < ECG_BUFFER_SIZE) {
          ecgBufferA[fillCountA]     = ecgValue;
          ecgTimestampsA[fillCountA] = timestamp;
          fillCountA++;
        } else {
          serial_print("ECG Buffer A Overflow!");
          fillCountA = ECG_BUFFER_SIZE - 1;
        }
      } else {
        // Buffer B is active
        if (fillCountB < ECG_BUFFER_SIZE) {
          ecgBufferB[fillCountB]     = ecgValue;
          ecgTimestampsB[fillCountB] = timestamp;
          fillCountB++;
        } else {
          serial_print("ECG Buffer B Overflow!");
          fillCountB = ECG_BUFFER_SIZE - 1;
        }
      }
    }
    taskEXIT_CRITICAL(&myMutex);

    vTaskDelay(pdMS_TO_TICKS(ECG_SAMPLE_INTERVAL)); 
  }
}

void publishECGTask(void* parameter) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(ECG_PUBLISH_INTERVAL)); // wait 1 minute
    
    serial_print("Starting sending ECG data.");
  
    delay(50);
    // 1) Determine which buffer is inactive
    bool bufferToSendIsA;
    int  samplesToSend = 0;

    // Temporarily disable interrupts or ensure atomic switch
    // We'll do a very quick "critical section" or use a mutex
    taskENTER_CRITICAL(&myMutex);
    {
      bufferToSendIsA = usingBufferA;
      usingBufferA = !usingBufferA;

      // Reset the inactive buffer's fill count to 0
      if (bufferToSendIsA) {
        fillCountB = 0; // Reset buffer B
      } else {
        fillCountA = 0; // Reset buffer A
      }
    }
    taskEXIT_CRITICAL(&myMutex);

    // 2) Now get the fill count from whichever buffer is inactive
    if (bufferToSendIsA) {
      // We are going to send buffer A
      samplesToSend = fillCountA;
      fillCountA = 0; // Reset for next usage
    } else {
      // We are going to send buffer B
      samplesToSend = fillCountB;
      fillCountB = 0;
    }
    if (samplesToSend == 0) {
      serial_print("No samples to send.");
      continue;
    }
    Serial.print("Samples to send: ");
    Serial.println(samplesToSend);

    // 3) Actually send the data from that buffer
    bool sendOk = false;
    if(xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(20000)) == pdTRUE){
      if (bufferToSendIsA) {
        if (samplesToSend > 0) {
          sendOk = sendBufferOverTCP(ecgBufferA, ecgTimestampsA, samplesToSend);
        } else {
          serial_print("No samples in Buffer A to send");
          sendOk = true; // or do nothing
        }
      } else {
          if (samplesToSend > 0) {
            sendOk = sendBufferOverTCP(ecgBufferB, ecgTimestampsB, samplesToSend);
          } else {
            serial_print("No samples in Buffer B to send");
            sendOk = true;
          }
      }

      if (!sendOk) {
        serial_print("Send failed.");
      }

      xSemaphoreGive(tcpMutex);
    }
  } 
}

void recordAccelerometer(void* parameter) {
  sensors_event_t event;
  while (true) {
    accel.getEvent(&event);
    float totalAcc = sqrt(
      pow(event.acceleration.x, 2) +
      pow(event.acceleration.y, 2) +
      pow(event.acceleration.z, 2)
    );

    if (totalAcc > accThreshold) {
      serial_print("Fall Detected!");
      // 1) Send a message to server
      if(xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(20000)) == pdTRUE){
        sendFallAlertToServer();
        xSemaphoreGive(tcpMutex);
      }

      // 2) Call supervisor
      callTheSupervisor();

      
      
      
    }

    vTaskDelay(ACC_DETECT_INTERVAL);
  }
}

void gpsTask(void* parameter) {
  while (true) {
    getGPSLocation();
    // Optionally send location to the server
    if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(20000)) == pdTRUE){
        sendLocationToServer(last_location);
        xSemaphoreGive(tcpMutex);
    }
    
    // Wait 5 minutes
    
    vTaskDelay(pdMS_TO_TICKS(4UL * 70UL * 1000UL));
  }
}

// ----------------------------------------------------
//  MC60 Setup (GPRS Attach)
// ----------------------------------------------------
void setupMC60() {
  serial_print("Setting up MC60 module...");
  
  sendATCommand("AT");                  // Test communication
  sendATCommand("ATE0");                // Enable echo
  sendATCommand("AT+CMEE=2");           // Enable verbose errors
  sendATCommand("AT+CPIN?");            // Check SIM card
  sendATCommand("AT+CSQ");              // Check signal strength
  sendATCommand("AT+CREG?");            // Check network registration

  sendATCommand("AT+QICSGP=1,\"CMNET\"",3000); // Set APN (replace "CMNET" with your APN)

  sendATCommand("AT+QIREGAPP", 3000);         // Register GPRS application
  sendATCommand("AT+QIACT", 5000);            // Activate PDP context
  sendATCommand("AT+QIMODE=0");
  sendATCommand("AT+QILOCIP"); 

  sendATCommand("AT+QAUDCH=1");

  sendATCommand("AT+QGNSSC=1", 2000);

  
}

// ----------------------------------------------------
//  TCP Functions
// ----------------------------------------------------

// Opens a TCP socket to the server
bool openTCPConnection() {
  // Close any previous connection if open
  closeTCPConnection();

  serial_print("Openning Connection ...");
  
  sendATCommand("ATV1", 1000);
  sendATCommand("AT+QIHEAD=0", 1000);
  sendATCommand("AT+QIDNSIP=0", 1000);
  
  String connectCommand = String("AT+QIOPEN=\"TCP\",\"") + SERVER_IP + "\"," + SERVER_PORT;
  serial_print("Generated Command: " + connectCommand);

  mc60Serial.println(connectCommand);
  delay(5000);
  String response = readSerialResponse();
  serial_print("Response: " + response);
  if (response.indexOf("CONNECT OK") != -1 || response.indexOf("ALREADY CONNECT") != -1) {
        serial_print("TCP Connection Successful.");
        return true;
    } else if (response.indexOf("ERROR") != -1 || response.indexOf("CME ERROR") != -1) {
        serial_print("TCP Connection Failed.");
        return false;
    }

    serial_print("No valid response for TCP connection.");
    return false;

}

// Closes the TCP connection
void closeTCPConnection() {
  sendATCommand("AT+QICLOSE", 2000);
}

// Send data (string) over the open TCP connection
bool sendTCP(const String &data) {
  // We can use AT+QISEND to send data
  // e.g.: AT+QISEND=<length>
  // Then we wait for ">" prompt, send the data, and end with 0x1A (CTRL+Z).
  // char cmd[32];
  // snprintf(cmd, sizeof(cmd), "AT+QISEND=%d", data.length());
  while (mc60Serial.available()) {
  mc60Serial.read(); // Clear the buffer
  }

  serial_print("Sending TCP Data...");

  sendATCommand("AT+QISACK", 1000);

  mc60Serial.println("AT+QISEND"); // Send the AT+QISEND command
  // mc60Serial.println(strlen(data.c_str()));

  delay(2000);
  String response = readSerialResponse();
  serial_print("Response: " + response);

  if (response.indexOf(">") == -1) {
      serial_print("No prompt received. Aborting.");
      return false;
  }
  serial_print("Sending Data: " + String(strlen(data.c_str())));
  serial_print("Data: " + data);
  mc60Serial.print(data.c_str());
  delay(2000);
  mc60Serial.write(0x1A);
  delay(5000);
  response = readSerialResponse();
  serial_print("Final Response: " + response);
  
  sendATCommand("AT+QISACK", 1000);
  closeTCPConnection();

  if (response.indexOf("SEND OK") != -1) {
      serial_print("Data Sent Successfully.");
      return true;
    } else {
      serial_print("Data Send Failed.");
      return false;
  }
  
   
}

// ----------------------------------------------------
// routines to send events/data
// ----------------------------------------------------
bool sendBufferOverTCP(int *ecgBuf, unsigned long *tsBuf, int count) {
  // 1) Build JSON
  const int maxItemsPerChunk = 200;
  bool allChunksSent = true;
  int current_count = count;
  int i = 0;
  while (current_count > 0) {
    int currentChunkSize = min(maxItemsPerChunk, current_count);

      // Build JSON for the current chunk
    String payload = "{\"user_id\":\"1\",\"ecg_record\": [";
    for (int j = 0; j < currentChunkSize; j++) {
        payload += String(ecgBuf[i * maxItemsPerChunk + j]);

        if (j < currentChunkSize - 1) {
              payload += ",";
        }
    }
    payload += "]}";
    // payload = String(payload.c_str());
    // Open TCP connection for this chunk
    if (!openTCPConnection()) {
      serial_print("Failed to open TCP connection for chunk starting at index: " + String(i * maxItemsPerChunk));
      allChunksSent = false;
      break; // Exit if the connection fails
    }
    if (!sendTCP(payload)) {
      serial_print("Failed to send chunk starting at index: " + String(i * maxItemsPerChunk));
      allChunksSent = false;
    } else {
      serial_print("Chunk sent successfully, starting at index: " + String(i * maxItemsPerChunk));
    }

    delay(2000);
    i += 1;
    current_count -= currentChunkSize;
  }
  return allChunksSent;
}

void sendFallAlertToServer() {
  String msg = "{\"user_id\":\"1\",\"has_fallen\":true}";
  if (openTCPConnection()) {
    if (sendTCP(msg)) {
        serial_print("TCP data sent successfully.");
    } else {
        serial_print("Failed to send TCP data.");
    }
  } else {
      serial_print("Failed to open TCP connection.");
  }
  
}

void sendLocationToServer(String location) {
  String payload = "{\"user_id\":\"1\",\"location\":\"";
  payload += location;
  payload += "\"}";
  if (openTCPConnection()){
      sendTCP(payload);

  }
}

// ----------------------------------------------------
//  MC60 AT Command Helper
// ----------------------------------------------------
void sendATCommand(const char* cmd, int delayMs) {
  serial_print("Sending: " + String(cmd));
  mc60Serial.println(cmd);
  delay(delayMs);
  String response = readSerialResponse();
  serial_print("Response: " + response);

}

// ----------------------------------------------------
//  FALL + SUPERVISOR CALL
// ----------------------------------------------------
void callTheSupervisor() {
  serial_print("Calling the supervisor...");
  String command = "ATD";
  command += String(EMG_PHONE_NUMBER);
  command += ";";
  sendATCommand(command.c_str());
  // For example:
  // sendATCommand("ATDphonenumber;");  // dial phone number
  // AT+QAUDCH=1
  // Delay or wait for ring, etc.
  // Then you might hang up with "ATH" after a while
}

// ----------------------------------------------------
//  GPS (Stub example)
// ----------------------------------------------------
void getGPSLocation() {
  String command = "AT+QGNSSRD=\"NMEA/RMC\"";
  serial_print("Generated Command: " + command);
  mc60Serial.println(command);

  delay(2000);
  last_location = readSerialResponse();
  serial_print("Response: " + last_location);
}

