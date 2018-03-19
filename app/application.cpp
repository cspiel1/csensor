#include <user_config.h>
#include <version.h>
#include <ota_update.h>

#include <SmingCore/SmingCore.h>
#include <Libraries/OneWire/OneWire.h>

#include <Libraries/DS18S20/ds18s20.h>
#include <Adafruit_BME280_Library/Adafruit_BME280.h>

#define WIFI_SSID "OpenWrt-chris"
#define WIFI_PWD  "zippezappe538"
#define SERVER_IP "10.1.0.22"
#define SERVER_PORT 5588

enum SensorType {
    S_None,
    S_DS18S20,
    S_BME280,
    S_Num
};

SensorType _sensorType=S_None;

DS18S20 ReadTemp;
Adafruit_BME280 bme; // I2C
TwoWire wire1;

enum BME280_State {
    BME280_None,
    BME280_Temp,
    BME280_Press,
    BME280_Hum,
    BME280_Done
};
BME280_State _BME280_state;

#define SEALEVELPRESSURE_HPA (1013.25)

//**********************************************************
// DS18S20 example, reading
// You can connect multiple sensors to a single port
// (At the moment 4 pcs - it depends on the definition in the library)
// Measuring time: 1.2 seconds * number of sensors
// The main difference with the previous version of the demo:
//  - Do not use the Delay function () which discourages by manufacturer of ESP8266
//  - We can read several sensors
// Usage:
//  Call Init to setup pin eg. ReadTemp.Init(2);   //pin 2 selected
//  Call ReadTemp.StartMeasure();
//   if ReadTemp.MeasureStatus() false read sensors
//   You can recognize sensors by the ID or index.
//***********************************************************
//
Timer sleepTimer;
Timer readTimer;
void read_DS18S20();
void read_BME280();
void retrieveSensorType();

bool _update_running=false;

typedef struct {
    int set;
    SensorType stype;
} RtcUserData;

void writeRTC();

void deep_sleep() {
    if (_update_running) return;
    if (!sleepTimer.isStarted()) {
	Serial.printf("%s SleepTimer is not running!\n", __PRETTY_FUNCTION__);
	return;
    }

    Serial.println("Deep sleep...");
    delay(100);
    System.deepSleep(1200000); // 20min
    delay(100);
}

int _rssi=0;
// Will be called when WiFi station network scan was completed
void listNetworks(bool succeeded, BssList list) {
    if (!succeeded) {
        Serial.println("Failed to scan networks");
        _rssi=0;
        return;
    }

    Serial.println("WiFi Scan Result:");
    for (int i = 0; i < list.count(); i++) {
        Serial.print("\tWiFi: ");
        Serial.print(list[i].ssid);
        Serial.print(", ");
        Serial.print(list[i].getAuthorizationMethodName());
        Serial.print(", rssi=");
        Serial.print(list[i].rssi);
        if (list[i].hidden) Serial.print(" (hidden)");
        Serial.println();

        if (list[i].ssid==WIFI_SSID) {
            _rssi=list[i].rssi;
        }
    }
    if (_rssi==-1)
        _rssi=0;

    Serial.printf("%s END\n", __PRETTY_FUNCTION__);

}

void on_TCPCompleted(TcpClient& client, bool successful);
void on_TCPReadyToSend(TcpClient& client, TcpConnectionEvent sourceEvent);
bool on_TCPReceive(TcpClient& client, char *buf, int size);

TcpClient _client(on_TCPCompleted, on_TCPReadyToSend, on_TCPReceive);
String _txBuffer;

void sendWifi() {
    _client.connect(SERVER_IP, SERVER_PORT);
}

void connectOk(IPAddress ip, IPAddress mask, IPAddress gateway) {
    // debug msg
    Serial.println("I'm CONNECTED to WiFi");
	Serial.printf("ip:      %s\n", ip.toString().c_str());
	Serial.printf("mask:    %s\n", mask.toString().c_str());
	Serial.printf("gateway: %s\n", gateway.toString().c_str());
    sendWifi();
}

void connectFail(String ssid, uint8_t ssidLength, uint8_t *bssid, uint8_t reason) {
    Serial.println("I'm NOT CONNECTED!");
    Serial.printf("reason: %u\n", reason);
    sleepTimer.start(false);
}

void on_TCPCompleted(TcpClient& client, bool successful)
{
    // debug msg
    Serial.printf("%s successful: %d\n", __PRETTY_FUNCTION__, successful);
    if (!successful) {
	sleepTimer.initializeMs(500, deep_sleep).start(false);
	return;
    }
    if (_sensorType==S_BME280) {
	Serial.printf("%s _BME280_state=%d\n", __PRETTY_FUNCTION__, _BME280_state);
	if (_BME280_state<BME280_Done) {
	    sleepTimer.stop();
	    readTimer.initializeMs(2000, read_BME280).start(false);
	} else {
	    sleepTimer.initializeMs(500, deep_sleep).start(false);
	}
    } else {
	sleepTimer.initializeMs(500, deep_sleep).start(false);
    }
}

void on_TCPReadyToSend(TcpClient& client, TcpConnectionEvent sourceEvent)
{
    // debug msg
    if(sourceEvent == eTCE_Connected) {
        client.sendString(_txBuffer, false);
    }
}

bool on_TCPReceive(TcpClient& client, char *buf, int size)
{
    // debug msg
    Serial.printf("%s response: %s\n", __PRETTY_FUNCTION__, buf);
    client.close();

    if (size>=13 && !strncmp(buf, "HELLO UPGRADE", 13)) {
	_update_running=true;
        String v(buf+14);
        otaUpdate(v);
    }

    return true;
}

void read_BME280() {
    switch (_BME280_state) {
	case BME280_Temp: {
	    float temp=bme.readTemperature();
	    Serial.printf("Temperature = %f *C\n", temp);
	    _txBuffer=String("CID=")+String(system_get_chip_id())+String(", VER=")+
		VERSION+", TEMP[C]=" + String(temp,1) +	", RSSI="+_rssi;
	    _BME280_state=BME280_Press;
	    sleepTimer.stop();
	    WifiStation.config(WIFI_SSID, WIFI_PWD, false);
	    WifiStation.enable(true);
	    WifiStation.connect();
	}
	break;
	case BME280_Press: {
	    float press=bme.readPressure() / 100.0F;
	    Serial.printf("Pressure = %f hPa\n", press);
	    _txBuffer=String("CID=")+String(system_get_chip_id())+String(", VER=")+
		VERSION+", PRESS[hPa]=" + String(press,1) +	", RSSI="+_rssi;
	    _BME280_state=BME280_Hum;
	    sleepTimer.stop();
	    sendWifi();
        }
	break;
	case BME280_Hum: {
	    float hum=bme.readHumidity();
	    Serial.printf("Humidity = %f \%\n", hum);
	    _txBuffer=String("CID=")+String(system_get_chip_id())+String(", VER=")+
		VERSION+", HUMI[%]=" + String(hum,1) +	", RSSI="+_rssi;
	    _BME280_state=BME280_Done;
	    sleepTimer.stop();
	    sendWifi();
	}
	break;
	default:
	break;
    }
}

void read_DS18S20() {
    uint8_t a;
    uint64_t info;

    // the last measurement completed
    if (!ReadTemp.MeasureStatus()) {
	// is minimum 1 sensor detected ?
	if (ReadTemp.GetSensorsCount()) {
	    if (_sensorType==S_None) {
		_sensorType=S_DS18S20;
		Serial.println("DS18S20 sensor found!");
//                writeRTC();
	    }
	    Serial.println("******************************************");
	    Serial.println(" Reading temperature DEMO");
	    for(a=0;a<ReadTemp.GetSensorsCount();a++) {
		Serial.print(" T");
		Serial.print(a+1);
		Serial.print(" = ");
		// temperature read correctly ?
		if (ReadTemp.IsValidTemperature(a)) {
		    Serial.print(ReadTemp.GetCelsius(a));
		    Serial.print(" Celsius, (");
		    Serial.print(ReadTemp.GetFahrenheit(a));
		    Serial.println(" Fahrenheit)");
		    _txBuffer=String("CID=")+String(system_get_chip_id())+String(", VER=")+
			VERSION+", TEMP[C]=" + String(ReadTemp.GetCelsius(a),1) +
			", RSSI="+_rssi;
		}
		else {
		    Serial.println("Temperature not valid");
		    _txBuffer=String("CID=")+String(system_get_chip_id())+String(", VER=")+
			VERSION+", TEMP[C]=invalid" +
			", RSSI="+_rssi;
		}

		Serial.print(" <Sensor id.");

		info=ReadTemp.GetSensorID(a)>>32;
		Serial.print((uint32_t)info,16);
		Serial.print((uint32_t)ReadTemp.GetSensorID(a),16);
		Serial.println(">");
	    }
	    sleepTimer.stop();
	    Serial.println("******************************************");
	    WifiStation.config(WIFI_SSID, WIFI_PWD, false);
	    WifiStation.enable(true);
	    WifiStation.connect();
	} else {
	    sleepTimer.stop();
	    sleepTimer.initializeMs(6500, deep_sleep).start(false);
	    Serial.println("No valid Measure so far! wait please");
	    readTimer.initializeMs(1000, read_DS18S20).start(false);
	}
    }
}

bool _ds18S20_started=false;

void writeRTC() {
    RtcUserData data;
    data.set=123456;
    data.stype=_sensorType;
    system_rtc_mem_write(80, &data, sizeof(data));
}

void retrieveSensorType() {
    // read from RTC memory
//    uint32_t clock=RTC.getRtcSeconds();
//    if (clock>6) {
//        RtcUserData data;
//        system_rtc_mem_read(80, &data, sizeof(data));
//        if (data.set==123456 && data.stype>S_None && data.stype<S_Num) {
//            _sensorType=data.stype;
//            Serial.printf("Read sensortype from RTC. Type=%d\n", _sensorType);
//        }
//    }
    // check through GPIO
    sleepTimer.initializeMs(6500, deep_sleep).start(false);
    if (_sensorType==S_None) {
	wire1.pins(14,13);
	if (bme.begin(0x76,&wire1)) {
	    _sensorType=S_BME280;
	    Serial.println("BME280 sensor found!");
//            writeRTC();
	    _BME280_state=BME280_Temp;
	    readTimer.initializeMs(2000, read_BME280).start(false);
	    return;
	}

	if (!ReadTemp.MeasureStatus() && !_ds18S20_started) {
	    _ds18S20_started=true;
	    ReadTemp.Init(2);        // select PIN It's required for one-wire initialization!
	    ReadTemp.StartMeasure(); // first measure start,result after 1.2 seconds * number of sensors
	    readTimer.initializeMs(2000, read_DS18S20).start(false);
	    return;
	}
    }
}

void ShowInfo() {
    Serial.printf("\r\nSDK: v%s\r\n", system_get_sdk_version());
    Serial.printf("Free Heap: %d\r\n", system_get_free_heap_size());
    Serial.printf("CPU Frequency: %d MHz\r\n", system_get_cpu_freq());
    Serial.printf("System Chip ID: %x\r\n", system_get_chip_id());
    Serial.printf("SPI Flash ID: %x\r\n", spi_flash_get_id());
    Serial.println("Version: " VERSION);
    //Serial.printf("SPI Flash Size: %d\r\n", (1 << ((spi_flash_get_id() >> 16) & 0xff)));
}

void process_serial(const char* str) {
    if (!strcmp(str, "ota")) {
	_update_running=true;
	sleepTimer.stop();
	readTimer.stop();

        otaUpdate();
    } else if (!strcmp(str, "con")) {
	sleepTimer.stop();
	readTimer.stop();
	WifiStation.config(WIFI_SSID, WIFI_PWD, false);
	WifiStation.enable(true);
	WifiStation.connect();
    } else if (!strcmp(str, "ip")) {
        Serial.printf("ip: %s mac: %s\r\n",
                WifiStation.getIP().toString().c_str(),
                WifiStation.getMAC().c_str());
    } else if (!strcmp(str, "info")) {
        ShowInfo();
    } else if (!strcmp(str, "help")) {
        Serial.println();
        Serial.println("available commands:");
        Serial.println("  help    - display this message");
        Serial.println("  ip      - show current ip address");
        Serial.println("  con     - connect wifi");
        Serial.println("  ota     - perform ota update, switch rom and reboot");
        Serial.println("  info    - show esp8266 info");
        Serial.println();
    }
}

void cb_serialReceive(Stream& stream, char arrivedChar,
        unsigned short availableCharsCount) {

    if (arrivedChar<=13) {
        char str[20];
        int j=0;
        for (int i = 0; i < availableCharsCount; i++) {
            char c = Serial.read();
            if ( (c>='a' && c<='z') ||
                    (c>='0' && c<='9') ||
                    (c>='A' && c<='Z') ) {
                str[j] = c;
		Serial.printf("%c",c);
                j++;
                if (j==20) break;
            }
        }
        str[j]=0;

        process_serial(str);
    }
}

void init()
{
//    WifiStation.startScan(listNetworks);
    WifiAccessPoint.enable(false);

    // Callback when assigned IP.
    WifiEvents.onStationGotIP(connectOk);

    // Callback when disconnected or connection attempt failed.
    WifiEvents.onStationDisconnect(connectFail);

    Serial.begin(SERIAL_BAUD_RATE); // 115200 by default
    Serial.systemDebugOutput(false); // Allow debug output to serial
    Serial.setCallback(cb_serialReceive);

    retrieveSensorType();
}


