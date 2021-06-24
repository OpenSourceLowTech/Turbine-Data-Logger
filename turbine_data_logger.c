//
// WindTurbineMonitor
// by CodeCoordination 2021
//
//

#include <Arduino.h>

// Library Headers
#include <Adafruit_INA260.h>
#include <SPIFFS.h>
#include <HTTPClient.h>
#include <ezTime.h>
#include <string.h>
#include <HX711_ADC.h>

// Choose 1 of these for Serial logging level.
#define DEBUG 2
#define LOGGING 1
#define SILENT 0

//#define CLEAR_LOGS // Define to clear logs of number of restarts.
#define DEBUGLEVEL LOGGING // Choose between DEBUG, LOGGING and SILENT
//#define LOG_ACTUAL_HTTP_REQUEST // If we have to debug why bad http request. Comment out otherwise.
//!!!! Make sure to set TEST to 0 or ina260 won't work !!!!\\/
#define TEST 0 // set to 0 for Summerhall. // 1 no ina260 & 2 with ina260...
#define WIFI_TEST_MODE // Gives regular wifi strength updates. comment out to not use.

#define NUMBER_OF_BEAM_BREAKERS 8

#define SOFT_DEBOUNCE
#ifdef SOFT_DEBOUNCE
#define SOFT_DEBOUNCE_TIME_TURBINE (70 / NUMBER_OF_BEAM_BREAKERS) //ms
#define SOFT_DEBOUNCE_TIME_ANEMOMETER 20 //ms
#endif

// To Recalibrate   -   Set value to 1.0f
//				  -   Upload and run with no mass.
//				  -   Put known mass on load cell.
//				  -   Divide output by known mass.
//				  -   Enter this value into LOAD_CELL_CALIBRATION_VALUE.
//				  -   Re-upload and repeat or adjust as desired.
#define LOAD_CELL_CALIBRATION_VALUE 208.3134020618f
// Set to true if you want to tare the load cell.  False otherwise.
#define LOAD_CELL_TARE true // Start with no load calbrates current load as zero.  Maybe can be false.
#define LOAD_CELL_STABILIZING_TIME 6000 // Delay at start to stabilize.

/* *** !!! Important !!! *** */
#define ANEMOMETER_PIN_INTERRUPT_MODE RISING // RISING FALLING CHANGE etc. CHANGE would be double frequency.
#define TURBINE_PIN_INTERRUPT_MODE FALLING // Or RISING depends what you want.

// Macro wrapper for Serial.println and Serial.print... Allowing for 2 levels of verbosity...
#if ( DEBUGLEVEL == DEBUG )
	#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
	#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
	#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
	#define LOG_PRINT(...) Serial.print(__VA_ARGS__)
	#define LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
	#define LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
#elif ( DEBUGLEVEL == LOGGING )
	#define LOG_PRINT(...) Serial.print(__VA_ARGS__)
	#define LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
	#define LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
	#define DEBUG_PRINT(...)
	#define DEBUG_PRINTLN(...)
	#define DEBUG_PRINTF(...)
#else 
	#define DEBUG_PRINT(...)
	#define DEBUG_PRINTLN(...)
	#define DEBUG_PRINTF(...)
	#define LOG_PRINT(...)
	#define LOG_PRINTLN(...)
	#define LOG_PRINTF(...)
#endif

#define INTERNAL_PULLUP 0x05
#define NO_PULLUP 0x01

// Pins for ESP32
// Good pins 2, 4, 5, 13 - 33
namespace PIN {
	const uint8_t turbineDigitalIn = 4;
	const uint8_t anemometerInput = 16;
	//const uint8_t pressureSensorAnalogInput = 35;
	const uint8_t loadCellDataOut = 32;
	const uint8_t loadCellSck = 33;
	// If external pullup then don't use internal pullup.
	const uint8_t AnemometerInputStyle = NO_PULLUP;
	const uint8_t TurbineInputStyle = NO_PULLUP;
}

const uint8_t badPins[] = { 0, 1, 3, 6, 7, 8, 9, 10, 11, 12 };
const uint8_t gpios[] = { PIN::turbineDigitalIn, PIN::anemometerInput, PIN::loadCellDataOut, PIN::loadCellSck };

namespace SETTINGS {

	const int ReadingInterval_mS = 1000; // 1 second
	const int TransmitInterval_mS = 30000;
	const int UpdateLoadCellInterval_mS = 200;
#if TEST != 0
	const char wifiSSID[] = "Chaos2FullNRG";
	const char wifiPassword[] = "*********";
	const char APIKey[] = "4b1defeb877d01598d7f77c3ebb1097ae1f02ca8df";
	const char FeedID[] = "152554148820184427";
#else
	const char wifiSSID[] = "Summerhall Public WiFi";
	const char wifiPassword[] = "";
	const char APIKey[] = "36ae2ca0c989184b86e576d3c7fe8d8baa22063f47";
	const char FeedID[] = "988624456562171095";
#endif
	const char URLPrefix[] = "http://iotplotter.com/api/v2/feed/";
	const char URLSuffix[] = ".csv";
	const int DataQueueLength = 60;
	const int ReadPriority = 2;
	const int SendPriority = 1;
	const int FlashPriority = 0;
	const int UpdateLoadCellPriority = 0;
};

// Struct to store a reading.
struct _DATA {
	time_t timestamp_s;
	float voltage_V;
	float current_A;
	float power_W;
	float energy_Wh;
	float pressure_g;
	float windspeed_mps;
	float turbineSpeed_rpm;
} DATA;

// FUNCTIONS
bool connectToWifi();
void logWIFI();
_DATA readData();
void flashLED(int nTimes);
void checkGPIOS();
void http_to_string(char* const returnString, const int code);

// Tasks for concurrent operation.
void flashLED_task(void*);
void readData_task(void*);
void transmitData_task(void*);
void updateLoadCell_task(void*);

// MultiThreading handles and queues;
TaskHandle_t flashLED_handle;
TaskHandle_t readData_handle;
TaskHandle_t transmitData_handle;
TaskHandle_t updateLoadCell_handle;
QueueHandle_t dataQueue;
QueueHandle_t flashQueue;
QueueHandle_t loadCellQueue;

// Interrupts for reading rotating devices.
//uint32_t nTriggers = 0;
void ICACHE_RAM_ATTR takeAnemometerReading();
void ICACHE_RAM_ATTR takeTurbineReading();

// CLASSES

/***** This is where to find stuff relating to anemometer ****/
class Anemometer {

	volatile int reading;
	volatile int previousReading;
	volatile bool hasBeenRead;

public:
	Anemometer();

	void start();

	void takeReading();
// This is where wind speed is calculated...
	const float getWindSpeed();
	const unsigned long getCurrReading() const;
};

// Initialize the load cell... It is needed by the Turbine class.

// Turbine declaration.  Definitions below.
class Turbine {

	HX711_ADC loadCell;
	volatile unsigned long rotationalReading;
	volatile unsigned long previousRotationalReading;
	volatile bool rotationHasBeenRead;

	const float convertAnalogPressureReadingToGrams(uint16_t reading) const;

public:
	Turbine();
	void start();

	// Called by readData() to get the data.
	const float getRPM();
	const float getLoadCellReading() const;

	// Called by the interrupt functions to update the data.
	void takeReading();
	void updateLoadCellReading();
	// Used by turbine rpm interrupt to check time of previous turbine reading.
	const unsigned long getCurrReading() const;
};

// SPIFFS - Files stored for logging... 
class Logger {
	bool SPIFFSInitialized;
	bool initSpiffs();
	void clearLogs() const;
public:
	Logger();
	~Logger();
	bool init();
	explicit operator bool() const;
	void readAndUpdateRestartsLog() const;
	void log(const char msg[]);
	void printLogs();
	void printSpace();
};


// Global vars
Adafruit_INA260 ina260 = Adafruit_INA260();
Anemometer anemometer;
Turbine turbine;
Timezone ukTime;
Logger logger;

// Setup Function
void setup() {

	pinMode(LED_BUILTIN, OUTPUT);

#if ( DEBUGLEVEL != SILENT )
	Serial.begin(115200);
	while (!Serial) delay(10);
#endif
	LOG_PRINTLN(F("\n\n\nWelcome to Wind Turbine Monitor by Codecoordination.\nA simple utility for remotely monitoring wind turbine data.\n\nRunning on an "));
#ifdef ESP32
	LOG_PRINTLN(F("Espressif ESP32.  "));
	LOG_PRINT(F("On Core: "));
	LOG_PRINTLN(xPortGetCoreID());
#endif
	LOG_PRINT(F("Current Clock speed is: "));
	LOG_PRINT(ESP.getCpuFreqMHz());
	LOG_PRINTLN(F(" MHz\n"));
	
	// Take a count of restarts...  Higher than expected count = issues.
	if (logger.init()) {
		DEBUG_PRINTLN("Logger initialized.");
		logger.printSpace();;
		logger.readAndUpdateRestartsLog();
		logger.printLogs();
	}

	while (!connectToWifi()) {
		delay(2500);
	}

// Get time synced...
	while (!waitForSync()) ;
	ukTime.setLocation("gb");
	LOG_PRINT(F("The current Time is: "));
	LOG_PRINTLN(ukTime.dateTime());
	LOG_PRINTLN();
	

#if ( TEST != 1 )
	LOG_PRINTLN(F("Connecting to INA260..."));

	Wire.setPins(22,23);

	while (!ina260.begin(INA260_I2CADDR_DEFAULT, &Wire)) {

		LOG_PRINTLN(F("INA260 failed to connect. Retrying."));
		delay(1500);
	}
	ina260.setMode(INA260_MODE_CONTINUOUS);  // Runs continuously.
	ina260.setAveragingCount(INA260_COUNT_4);  // Average of 4 samples.

	LOG_PRINTLN(F("Connected to IN260.\n"));
#endif

	turbine.start(); // Put this here as it delays anyway.

	//vTaskDelay(5500 * portTICK_PERIOD_MS);

// Multithreading - start 3 tasks with a data queue in between.
	// Data is thrown into here.
	dataQueue = xQueueCreate(SETTINGS::DataQueueLength, sizeof(DATA));
	// LED flashes are thrown in here.
	flashQueue = xQueueCreate(5, sizeof(int));
	// Keep one data element in queue.
	loadCellQueue = xQueueCreate(1, sizeof(float));

	// Data read task.
	xTaskCreatePinnedToCore(	readData_task,
								"Read data",
								2560,
								NULL,
								SETTINGS::ReadPriority,
								&readData_handle,
								1
							);
	// Data transmit task.
	xTaskCreatePinnedToCore(	transmitData_task,
								"Transmit data",
								15360,
								NULL,
								SETTINGS::SendPriority,
								&transmitData_handle,
								1
							);
	// Flash LED task.
	xTaskCreatePinnedToCore(	flashLED_task,
								"Flash LED",
								1024,
								NULL,
								SETTINGS::FlashPriority,
								&flashLED_handle,
								1
							);
	xTaskCreatePinnedToCore(	updateLoadCell_task,
								"Update Load Cell",
								1024,
								NULL,
								SETTINGS::UpdateLoadCellPriority,
								&updateLoadCell_handle,
								1
							);

	// Attach the interrupts to the pins.
	anemometer.start();
}


/// Other Functions

// Connect to the WIFI.
bool connectToWifi() {

	WiFi.mode(WIFI_STA);	// Wifi station mode.
	WiFi.begin(SETTINGS::wifiSSID, SETTINGS::wifiPassword);

	if (WiFi.waitForConnectResult() != WL_CONNECTED) {
		char status[20] = "\0";
		char msg[60] = "Connecting to WIFI failed.\nReason: \0";
		
		switch (WiFi.status()) {
			case WL_NO_SSID_AVAIL: strncpy(status, "No SSID", 19); break;
			case WL_SCAN_COMPLETED: strncpy(status, "Scanning", 19); break;
			case WL_CONNECT_FAILED: strncpy(status, "Failed to connect", 19); break;
			case WL_CONNECTION_LOST: strncpy(status, "Connection lost", 19); break;
			case WL_DISCONNECTED: strncpy(status, "Disconnected", 19); break;
			default: strncpy(status, "Other", 19);
		}
		strcat(msg, status);
		LOG_PRINTLN(msg);
		logger.log(msg);
		return false;
	}

	DEBUG_PRINT(F("Wifi successfully connected to "));
	DEBUG_PRINTLN(SETTINGS::wifiSSID);
	DEBUG_PRINT(F("Your IP Address is: "));
	DEBUG_PRINTLN(WiFi.localIP());
	DEBUG_PRINT(F("Wifi signal strength is: "));
	DEBUG_PRINT(WiFi.RSSI());
	DEBUG_PRINTLN(" dBm");
	DEBUG_PRINT("BSSID: ");
	DEBUG_PRINTLN(WiFi.BSSIDstr());
	DEBUG_PRINTLN();

	return true;
}

// Read all the data and return readings as _DATA object.
_DATA readData() {
	
	_DATA DATA;

	// Take some readings.
	LOG_PRINTLN(F("Taking Measurements."));
	// Pulse the LED.
	flashLED(2);
	// Timestamp the data.
	DATA.timestamp_s = now();

#if ( TEST != 1 )
	// Read the voltage.
	DATA.voltage_V = ina260.readBusVoltage() * 0.001 * 3 * 1.007;
	// Read the current.
	DATA.current_A = ina260.readCurrent() * 0.001;
	// Calculate the power.
	DATA.power_W = DATA.voltage_V * DATA.current_A;
#else
	// Only for testing without ina260 connected.
	DATA.voltage_V = 5;
	DATA.current_A = 100.0;
	DATA.power_W = 500;
#endif
	// Take windspeed reading.
	DATA.windspeed_mps = anemometer.getWindSpeed();
	// Take turbine rpm reading.
	DATA.turbineSpeed_rpm = turbine.getRPM();

	DATA.pressure_g = turbine.getLoadCellReading();

	// Calculate total Wh.
	float interval_Hours = SETTINGS::ReadingInterval_mS / 3600000.0;
	float power_W = DATA.power_W;
	DATA.energy_Wh = power_W * interval_Hours;

	// Return a data object.
	return DATA;
}

// Queue flashes for LED flashing task.
void flashLED(const int nTimes) {
	xQueueSend(flashQueue, &nTimes, portMAX_DELAY);
}

void checkGPIOS() {
	for (int16_t i = 0; i < sizeof(gpios); ++i) {
		for (int16_t j = 0; j < sizeof(badPins); ++j) {
			if (gpios[i] == badPins[j]) {
				LOG_PRINTLN("Don't use this gpio as there may be a problem.");
			}
		}
	} 
}

void logWIFI() {
	LOG_PRINT("Wifi signal strength is: ");
	LOG_PRINT(WiFi.RSSI());
	LOG_PRINTLN(" dBm");
	LOG_PRINT("BSSID: ");
	LOG_PRINTLN(WiFi.BSSIDstr());
}

// Processor Task Functions.
// The following 4 tasks run concurrently on core 1.
void flashLED_task(void* p) {
	int nFlashes = 0;
	while(1) {
		xQueueReceive(flashQueue, &nFlashes, portMAX_DELAY);
		for (int i = 0; i < nFlashes; ++i) { 
			digitalWrite(LED_BUILTIN, ((i % 2) != 0));
			vTaskDelay(50 / portTICK_PERIOD_MS);
		}
	}
}

// Sets up repetition and then calls readData().  Also logs some debugging info.
void readData_task(void* p) {

	// Set up the task repetition frequency.
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = SETTINGS::ReadingInterval_mS / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	while(true) {

		vTaskDelayUntil(&xLastWakeTime, xFrequency);

#ifdef WIFI_TEST_MODE   // Check WIFI strength.
		logWIFI();
#endif

// Check connection if not connected reset.
		if (WiFi.status() != WL_CONNECTED) {

			LOG_PRINTLN(F("Wifi is not connected... Initiating reset."));
			const char logError[50] = "Error: Wifi is not connected restarting.\0";
			logger.log(logError);
			ESP.restart();
		}
		
		auto data = readData();

#if ( DEBUGLEVEL == DEBUG )
		char readings[200];
		sprintf(readings, "Voltage: %.3f V\nCurrent: %.3f A\nPower: %.3f W\nEnergy: %.3f Wh\nWindpeed: %.1f mpS\nRPM: %.2f rpm\n\nForce: %.0f g", data.voltage_V, data.current_A, data.power_W, data.energy_Wh, data.windspeed_mps, data.turbineSpeed_rpm, data.pressure_g);
		DEBUG_PRINTLN(readings);
		DEBUG_PRINTLN();
#endif
#if ( DEBUGLEVEL == LOGGING )
		char readings[100];
		sprintf(readings, "Windspeed: %.1f mpS\nRPM %.2f rpm\nForce: %.0f g", data.windspeed_mps, data.turbineSpeed_rpm, data.pressure_g);
		LOG_PRINTLN(readings);
		LOG_PRINTLN();
#endif
		
		// Queue the data.
		xQueueSend(dataQueue, &data, portMAX_DELAY);

		// If crashes due to not enough ram...
		// DEBUG_PRINT("RAM remaining in readData_task: ");
		// DEBUG_PRINTLN(uxTaskGetStackHighWaterMark(NULL));
	}
}

void transmitData_task(void* p) {

// Needed stuff
	static HTTPClient http;
	static WiFiClient client;

// Make sure big enough to contain string for line.
	const int LINE_LENGTH = 200;
	const int CONTENT_LENGTH = LINE_LENGTH * SETTINGS::DataQueueLength;

// Parse the address.
	char url[110] = "\0";
	sprintf(url, "%s%s%s", SETTINGS::URLPrefix, SETTINGS::FeedID, SETTINGS::URLSuffix);

	while (true) {

		// Delay until
		vTaskDelay(SETTINGS::TransmitInterval_mS / portTICK_PERIOD_MS);

		LOG_PRINTLN(F("Making POST request to URL: "));
		LOG_PRINTLN(url);
		LOG_PRINTLN();

// Parse the data.
		char content[CONTENT_LENGTH] = "\0";

		// Assuming delay on reading means we can't keep loading the queue while emptying...
		while (uxQueueMessagesWaiting(dataQueue)) {

			char line[LINE_LENGTH];
			_DATA data;
			
			xQueueReceive(dataQueue, &data, portMAX_DELAY);
			
			auto ts = data.timestamp_s;
			sprintf(line, "%lu,Voltage,%.3f\n%lu,Current,%.3f\n%lu,Power,%.3f\n%lu,Energy,%.3f\n%lu,Windspeed,%.1f\n%lu,Turbine_RPM,%.2f\n%lu,Turbine_Force,%.0f\n",
							ts, data.voltage_V,
							ts, data.current_A,
							ts, data.power_W,
							ts, data.energy_Wh, 
							ts, data.windspeed_mps,
							ts, data.turbineSpeed_rpm,
							ts, data.pressure_g
			);

			strcat(content, line);
			flashLED(2);
			// Check if buffer overrun on line.
			//LOG_PRINT(F("Strlen line: "));
			//LOG_PRINTLN(strlen(line));
		}

		if (strlen(content)) {

			DEBUG_PRINTLN(F("Sending data string:"));
			DEBUG_PRINTLN(content);

	// Building the POST request.
			char length[6];
			sprintf(length, "%d", strlen(content));
			http.begin(client, url);
			http.addHeader(F("Content-Length"), length);
			http.addHeader(F("api-key"), SETTINGS::APIKey);

			int httpResponseCode = 0;

			for (int i = 0; httpResponseCode != 200 && i < 3; ++i) {
				// Sending the request.
				httpResponseCode = http.POST(content);
				
				// Checking it worked.
				if (httpResponseCode != 200) {

					LOG_PRINT(F("There was an error with the HTTP POST request. Error Code: "));
					LOG_PRINTLN(httpResponseCode);
					LOG_PRINTLN();
					char logm[60] = "\0";
					char httpErrorType[25] = "\0";
					http_to_string(httpErrorType, httpResponseCode);

					sprintf(logm, "HTTP request error code: %d (%s)", httpResponseCode, httpErrorType);
					logger.log(logm);
#ifdef LOG_ACTUAL_HTTP_REQUEST
					logger.log("Content of request was: ");
					logger.log(content);
#endif
					vTaskDelay(3000 / portTICK_PERIOD_MS);
				}
			}
			if (httpResponseCode == 200) LOG_PRINTLN(F("Request successful. ; )\n"));
			else LOG_PRINTLN(F("Attempt to POST data failed after 3 attempts.  Binning data."));
			http.end();
		}
//	  If crashes can be due to not enough RAM see tasks in setup.
		// DEBUG_PRINT("RAM left in Transmit: ");
		// DEBUG_PRINTLN(uxTaskGetStackHighWaterMark(NULL));
	}
}

// Takes a load cell reading so that the load cell measurement queue gets updated.
void updateLoadCell_task(void* p) {

	while(true) {
		// Set up the task repetition frequency.
		vTaskDelay(SETTINGS::UpdateLoadCellInterval_mS / portTICK_PERIOD_MS);
		turbine.updateLoadCellReading();
	}
}

// Interrupt functions.
void takeAnemometerReading() {
#ifdef SOFT_DEBOUNCE
	auto cTime = millis();
	if (cTime - anemometer.getCurrReading() < SOFT_DEBOUNCE_TIME_ANEMOMETER) return;
#endif
	anemometer.takeReading();
}

void takeTurbineReading() {
#ifdef SOFT_DEBOUNCE
	int cTime = millis();
	if (cTime - turbine.getCurrReading() < SOFT_DEBOUNCE_TIME_TURBINE) return;
#endif
	turbine.takeReading();
}

// Anemometer Class Functions
Anemometer::Anemometer() : reading(millis()), previousReading(reading), hasBeenRead(true) {

	LOG_PRINTLN("Anemometer initialized...");
	pinMode(PIN::anemometerInput, PIN::AnemometerInputStyle);
}

void Anemometer::start() {
	reading = millis() - 1200000;
	previousReading = reading - 1200000;	// Start with a slow speed.
	attachInterrupt(digitalPinToInterrupt(PIN::anemometerInput), takeAnemometerReading, ANEMOMETER_PIN_INTERRUPT_MODE);
}

void Anemometer::takeReading() { 
	previousReading = reading; 
	reading = millis(); 
	hasBeenRead = false; 
}

// This is where wind speed is calculated...
const float Anemometer::getWindSpeed() {

// if (hasBeenRead) then we have not received new data so we don't enter a recorded reading but take the last reading and time now.
	auto time_ms = (hasBeenRead) ? millis() - reading : reading - previousReading;
	LOG_PRINT("ms between anemometer clicks: ");
	LOG_PRINTLN(time_ms);

// Time for one rotation or period is time_ms
// Frequency is 1/period.
	float time_s = static_cast<float>(time_ms) * 0.001;
	float frequency_hz = 1.0 / time_s;
	hasBeenRead = true;

// Transfer function for m/s.
	return 1.25 * frequency_hz;
}

const unsigned long Anemometer::getCurrReading() const { return reading; }

// Turbine Class Functions

Turbine::Turbine() : loadCell(PIN::loadCellDataOut, PIN::loadCellSck), rotationalReading(millis()), previousRotationalReading(rotationalReading), rotationHasBeenRead(true) {

	LOG_PRINTLN("Turbine sensor initialized...");
	pinMode(PIN::turbineDigitalIn, PIN::TurbineInputStyle);
}

void Turbine::start() {

	rotationalReading = millis() - 1200000; // Imaginary last reading was 20 minutes ago.
	previousRotationalReading = rotationalReading - 1200000; // And the previous reading was 20 minutes before that.
	attachInterrupt(digitalPinToInterrupt(PIN::turbineDigitalIn), takeTurbineReading, TURBINE_PIN_INTERRUPT_MODE);
	loadCell.begin();
	LOG_PRINTLN("Starting load cell.");
	loadCell.start(LOAD_CELL_STABILIZING_TIME, LOAD_CELL_TARE);
	LOG_PRINTLN("Load cell started");
	if (loadCell.getTareTimeoutFlag()) {
		LOG_PRINTLN("Timeout, check MCU>HX711 wiring and pin designations");
		while (1);
	} else {
		loadCell.setCalFactor(LOAD_CELL_CALIBRATION_VALUE); // set calibration value (float)
		LOG_PRINTLN("Startup is complete");
	}
}

void Turbine::takeReading() { 
	previousRotationalReading = rotationalReading;
	rotationalReading = millis();
	rotationHasBeenRead = false;
}

void Turbine::updateLoadCellReading() {
	if (!loadCell.update()) return;
	float newValue = loadCell.getData();
	xQueueOverwrite(loadCellQueue, &newValue);
}

const float Turbine::getLoadCellReading() const {
	float reading = 0.0;
	xQueueReceive(loadCellQueue, &reading, portMAX_DELAY);
	return reading;
}

const float Turbine::getRPM() {

	// If no change has been detected things are slowing down.  We'll call it a reading
	// and so the speed will fade away.
	// time between clicks ie one revolution.   
	auto int time_ms;
//	if (rotationHasBeenRead)
//		time_ms = millis() - previousRotationalReading;
//	else
		time_ms = rotationalReading - previousRotationalReading;

	LOG_PRINT("ms between turbine clicks: ");
	LOG_PRINTLN(time_ms);

	// time in secs.
	float time_s = static_cast<float>(time_ms) * 0.001;
	rotationHasBeenRead = true;

	// inverse should give rpm.
	return (NUMBER_OF_BEAM_BREAKERS / time_s) * 60;

}

const float Turbine::convertAnalogPressureReadingToGrams(uint16_t reading) const {
	uint16_t inverted = 4095 - reading;
	//Serial.printf("Number of triggers: %u\n", nTriggers);
	Serial.printf("Analog reading: %hu\n", inverted);
	// expecting 0 to 4095...
	return 0.0;
}

const unsigned long Turbine::getCurrReading() const { 
	return rotationalReading; 
}

// Logger Class Functions
Logger::Logger() : SPIFFSInitialized(false) {}

Logger::~Logger() { SPIFFSInitialized = false; SPIFFS.end(); }

bool Logger::init() {

	if (this->initSpiffs()) { 
		SPIFFSInitialized = true; 
#ifdef CLEAR_LOGS
		this->clearLogs();
#endif
	}
	return this->SPIFFSInitialized;
}

Logger::operator bool() const { return SPIFFSInitialized; }

bool Logger::initSpiffs() {

	if (!SPIFFS.begin()) {
		DEBUG_PRINTLN(F("Failed to mount SPIFFS filesystem."));
		DEBUG_PRINTLN(F("Attempting to format SPIFFS."));

		if (!SPIFFS.format()) return false;
		DEBUG_PRINTLN(F("SPIFFS filesystem successfully formatted."));
	}
	return true;
}

// brash but effective.
void Logger::readAndUpdateRestartsLog() const {
	int count;

	if (SPIFFS.exists("/restarts.txt")) {
		File restartsFile = SPIFFS.open("/restarts.txt", "r");
		count = atoi(restartsFile.readString().c_str());
		count++;
		restartsFile.close();
		restartsFile = SPIFFS.open("/restarts.txt", "w");
		restartsFile.println(count);
		restartsFile.close();
	} else {
		File restartsFile = SPIFFS.open("/restarts.txt", "w");
		if (restartsFile) {
			restartsFile.println("0");
			restartsFile.close();
		}
		count = 0;
	}
	
	DEBUG_PRINT(F("Number of restarts: "));
	DEBUG_PRINTLN(count);
	DEBUG_PRINTLN();
}

void Logger::clearLogs() const {
	SPIFFS.format();
}

void Logger::log(const char msg[]) {

	if (*this || this->initSpiffs()) {

		File lf = SPIFFS.open("/log.txt", FILE_APPEND);
		char logLine[300] = "\0";
		sprintf(logLine, "%s - %s", ukTime.dateTime().c_str(), msg);
		logLine[strlen(logLine)] = '\0';
		lf.println(logLine);
		lf.close();
	}
}

void Logger::printLogs() {

	if (*this || this->initSpiffs()) {

		File lf = SPIFFS.open("/log.txt", FILE_READ);

		LOG_PRINTLN(F("******** Log Content ********\n"));
		while (lf.available()) {
			String line = lf.readString();
			LOG_PRINTLN(line);
		}
		LOG_PRINTLN(F("*****************************\n"));
		lf.close();
	}
}

void Logger::printSpace() {

	if (*this || initSpiffs()) {

		DEBUG_PRINTLN(F("SPIFFS filesystem space: "));
		DEBUG_PRINT(F("SPIFFS total bytes: "));
		DEBUG_PRINTLN(SPIFFS.totalBytes());
		DEBUG_PRINT(F("SPIFFS bytes used: "));
		DEBUG_PRINTLN(SPIFFS.usedBytes());
		DEBUG_PRINT(F("SPIFFS bytes free: "));
		DEBUG_PRINTLN(SPIFFS.totalBytes() - SPIFFS.usedBytes());
		DEBUG_PRINTLN();
	}
}

void http_to_string(char* const returnString, const int code) {
	enum HTTP_RESPONSE_CODE {
		success = 200,
		bad_data = 400,
		bad_api_key = 401,
		no_permission = 403,
		no_feed = 404,
		bad_data2 = 406,
		api_removed = 410,
		too_many_requests = 429,
		unexpected_error = 500,
		iotPlotter_offline = 503
	};
	switch (code) {
		case success: strcpy(returnString, "Success"); break;
		case bad_data: strcpy(returnString, "Bad data"); break;
		case bad_api_key: strcpy(returnString, "Bad api key"); break;
		case no_permission: strcpy(returnString, "No permission"); break;
		case no_feed: strcpy(returnString, "No feed"); break;
		case bad_data2: strcpy(returnString, "Other bad data"); break;
		case api_removed: strcpy(returnString, "API removed"); break;
		case too_many_requests: strcpy(returnString, "Too many requests"); break;
		case unexpected_error: strcpy(returnString, "Unexpected error"); break;
		case iotPlotter_offline: strcpy(returnString, "IOTPlotter offline"); break;
		default: strcpy(returnString, "Other");
	};
}

// The main loop! ( or not )
// Not used when running on ESP32
void loop() {
	// Deleted.
	vTaskDelete(NULL);
}
