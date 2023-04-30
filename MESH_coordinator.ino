/*
 * Project MESH_coordinator
 * Description:
 * Author: Kathleen Yang
 * Date:
 */

/* Notes:
1. setup() and loop() are MANDATORY structures.
    setup() runs once, when the device is first turned on.
    loop() runs over and over again, as quickly as it can execute.

2. UART:
  * Minimum receive (Rx) buffer size is 6 bytes for UART (default size = 128 bytes)
  * Can only use Serial1 for UART on Argon board 
    (Serial is a USB serial emulation, not a hardware UART)
  * Use SerialLogHandler forwriting debug messages (is thread safe)
  * The first 30 seconds or so after the device is flashed may print gibberish
 
3. SPI:
  * Only 1 SPI interface 
  * Pins: 
      SS => A5 (D14) (but can use any available GPIO)
      SCK => SCK (D13)
      MISO => MISO (D11)
      MOSI => MOSI (D12)

4. I2C (Wire) -- Commented out since it wasn't working
  * Pins: 
      SCL => D1
      SDA => D0
  * 2nd optional I2C interface on D2 and D3 for the Argon
  * Both SCL and SDA pins are open-drain outputs that only pull LOW 
    and typically operate with 3.3V logic. Connect a pull-up resistor 
    (1.5k to 10k) on the SDA line to 3V3. Connect a pull-up resistor 
    (1.5k to 10k) on the SCL line to 3V3. 
  * Default speed is 100 KHz
*/

// *******************************************************
//                  Initial declarations
// *******************************************************
// The following line is optional, but recommended in most firmware.
// It allows your code to run before the cloud is connected.
SYSTEM_THREAD(ENABLED); 

// SD Card Includes
#include "SdFat.h" // Need to download this to your project from Particle
#include "ParticleWebLog.h"
#include <sstream>
#include <iostream>
ParticleWebLog particleWebLog;

// SD card variables
#define SD_CS_PIN SS
String dataFileName = "sp23demo.txt";
SdFat SD;
File myFile;

// Data logging variables
String start_message;
String temp_message;
String soil_message;
String light_message;

// UART variables

/* Make this the length of your payload; this is used for parsing when printing
   Our debugging: We read 3 ADCs into 16-bit values -> YOUR_LENGTH = 6 */
#define YOUR_LENGTH 6 //3 

/* Make this the appropriate number for uart_read_blocking;
   1 byte payload -> arrayLength = 8 */
#define ARRAY_LENGTH 8 + YOUR_LENGTH // 7

char data[ARRAY_LENGTH]; // Array for holding data received from the mesh
uint16_t sensorData[YOUR_LENGTH]; // Array for holding parsed sensor data
char nodeData[2];
uint16_t nodeDataCombined;
char * nodeIDcharbuff;

// Variables for raw ADC readings (data parsing)
uint16_t soil_raw, temp_raw, light_raw = 0;
uint8_t temp_final_int, temp_final_decimal;
float temp_voltage;
float soil_final, light_final, temp_final = 0;

uint8_t count=0;
// 12-bit ADC
const float conversion_factor = 3.3f / (float)65535;

// Define address for the LoRa microcontroller slave 
#define LORA_SLAVE_ADDR 1

// WiFi credentials
const char* ssid = "ATT-WIFI-WRE";         // change SSID
const char* password = "MIST1234";    // change password

// Function prototypes
void fillData();


// Put initialization like pinMode and begin functions here.
void setup() {
  /* Start Serial for USB UART debugging messages
      - Can view messages in PuTTY or other Serial software
      - Check your device manager (Windows) to see which COM port the Argon registers as when configuring PuTTY
  */
  Serial.begin(9600);

  // Start UART to receive data from MESH
  // By default, Rx and Tx pins are allocated for UART with the below code line  
  Serial1.begin(115200, SERIAL_8N1); // Baud rate of 115,200 and 8 data bits, no parity, 1 stop bit

  // SD Card Initialization
  SPI.begin(SS);  // Starting SPI. SS (aka CS) is by default pin D14, aka pin A5 for Argon board
  SD_card_init();
  // remove files if they exist
  //SD.remove(file_name); // For reference

  // I2C intialization (to connect to LoRa's micrcontroller)
  //Wire.begin(); // If no address specified in parenthesis, it joins as an i2c master

  // Connecting to WiFi
  Serial.print("Connecting to wifi: ");
  Serial.println(ssid);

  WiFi.setCredentials(ssid, password);
  WiFi.connect(); // Tries to connect to WiFi
  while (WiFi.connecting()) // Returns false once connected
  { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("connected!");
}


// The core of your code will likely live here.
void loop() 
{
  fillData();
  
  // Open the SD card file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(dataFileName, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("Successfully opened file.");
  } 
  else {
    // If the file didn't open, print an error:
    Serial.println("Failed to open file for writing");
  }

  // Creates a string variable to combine string literal message + the variables 
  // Note: you need to declare the message variable globally
  myFile.print("The following data is from Node 0x");
  myFile.print(nodeData[0], HEX);
  myFile.println(nodeData[1], HEX);
  nodeDataCombined = (nodeData[0] << 8) | nodeData[1];
  String nodeIDString = String(nodeDataCombined, HEX);

  // Convert the payload data (raw ADC readings) into voltages and write meaningful messages to the SD card 
  soil_raw |= (sensorData[0] << 8) | (sensorData[1]);
  soil_final = (float)soil_raw * conversion_factor;

  temp_raw |= (sensorData[2] << 8) | (sensorData[3]);
  //temp_final = (((float)temp_raw * conversion_factor) - 0.5f)*100; // For TMP36 conversion --> Celsius
  temp_voltage = (float)temp_raw * conversion_factor; 
  temp_final = 1/((log10(temp_voltage)-1)/4400 + 1/298.15) -273.15; // For NTC 10k thermistor --> FARENHEIT

  
  temp_final_int = (uint8_t)(floor(temp_final));
  temp_final_decimal = (uint8_t)(floor((temp_final - temp_final_int)*100));

  light_raw |= (sensorData[4] << 8) | (sensorData[5]);
  light_final = (float)light_raw * conversion_factor;
  Serial.println(light_raw);

  // Temperature message
  temp_message = "Temperature: " + String(temp_final_int)+ "." + String(temp_final_decimal) + "°F \r\n";
  myFile.println(temp_message);
  
  // Soil message
  soil_message = "Soil sensor raw reading: 0x";
  myFile.print(soil_message);
  myFile.println(soil_raw, HEX);

  // Light message
  if (light_final < 0.0125) {
    light_message = "Light level: Dark";
  }
  else if (light_final >= 0.0125 && light_final < 0.0375) {
    light_message = "Light level: Medium-Bright";
  }
  else if (light_final >= 0.0375) {
    light_message = "Light level: Bright";
  }
  //write_to_File(SD, dataFileName.c_str(), light_message.c_str());
  myFile.println(light_message);
  myFile.println(" ");
  myFile.println("*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*");
  myFile.println(" ");

// I2C SECTION CURRENTLY NOT WORKING
  // Send sensor readings to LoRa microcontroller via I2c
  // Wire.beginTransmission(LORA_SLAVE_ADDR); // Max is 32 bytes that can be queued 
  // Wire.write ("Node ID (hex): ");   // 15 bytes
  // Wire.write(nodeData[0]);          // 1 byte
  // Wire.write(nodeData[1]);          // 1 byte
  // Wire.endTransmission(); // Stop transmitting
  // // Use to convert from float to bytes to be able to transmit via I2C
  // union floatToBytes {
  //   char buffer[4];
  //   float sensorReading;
  // } converter;
  // converter.sensorReading = temp_final;
  // Wire.beginTransmission(LORA_SLAVE_ADDR); // Max is 32 bytes that can be queued 
  // Wire.write("Temp (float): ");     // 14 bytes
  // // Send the 4 bytes of the float
  // for (int i = 0; i < 4; i++)
  // {  
  //   Wire.write(converter.buffer[i]);    
  // }
  // Wire.endTransmission(); // Sends the queue
  // Wire.beginTransmission(LORA_SLAVE_ADDR); // Max is 32 bytes that can be queued 
  // Wire.write("Soil (hex): ");         // 12 bytes
  // Wire.write(soil_raw);               // 2 bytes
  // Wire.endTransmission(); // Sends the queue
  // Wire.beginTransmission(LORA_SLAVE_ADDR); // Max is 32 bytes that can be queued 
  // Wire.write(light_message);        // 17-26 bytes
  // Wire.endTransmission(); // Sends the queue

  // Log data to loggly.com
  const char* log_message = "ID: 0x" + nodeIDString + ", " + temp_message + ", soil = " + String(soil_raw, HEX) + ", " + light_message; 
  Log.info(log_message);

  //clear variables for next transmission
  soil_raw = 0;
  temp_raw = 0;
  light_raw = 0;
  myFile.close();
  Serial.println("File written to and closed");

  delay(2000); // delay 5 sec
}

// *******************************************************
//                    Custom Functions
// *******************************************************
void fillData()
{
	while (Serial1.available() <= 0)
  {
    //Serial.println("Not receiving MESH data"); // Debugging line, can use if needed

    // KEEP THIS DELAY IN -- otherwise data streaming in becomes jumbled and false data 
    // registers as a transmission
    delay(1000); // 1000 ms (1 sec) delay
  }
  for (int i = 0; i < ARRAY_LENGTH; i++){
    data[i] = Serial1.read();
    Serial.print("data[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(data[i], HEX);
  }
  Serial.println ("The Argon has read all the data");

  // Data parsing
	if(data[0] == 0x52){ //in a normal transmission, 0x52 is the first byte sent
		for(int i=0; i<YOUR_LENGTH; i++){
			sensorData[i] = (uint16_t)data[i+7];
		}
		nodeData[0] = data[2];
		nodeData[1] = data[3];
	}
	else if(data[0] == 0xFF){ //transmission issues arise where the last byte is sent as the first transmission to the Pico, so we need a case for this where all data is one element off
		for(int i=0; i<YOUR_LENGTH; i++){
			sensorData[i] = (uint16_t)data[i+8];
		}
		nodeData[0] = data[3];
		nodeData[1] = data[4];
	}
	else{
		Serial.print("ERROR in reading and sorting data \n\r");
		for(int i=0; i<YOUR_LENGTH; i++){
			sensorData[i] = 0;
		}
		nodeData[0] = 0;
		nodeData[1] = 0;
	}

	//for debugging, but also does not print to SD card with out this forsome reason (Fall 2022)
	for(int i=0; i<YOUR_LENGTH; i++){
    Serial.print ("SensorData[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(sensorData[i], HEX);
	}

  // For debugging (can comment out when no longer needed)
  Serial.print ("nodeData[0] = ");
  Serial.println(nodeData[0], HEX);
  Serial.print ("nodeData[1] = ");
  Serial.println(nodeData[1], HEX);
  Serial.println("***********************");
}

void SD_card_init()
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

}
