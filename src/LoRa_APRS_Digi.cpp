#include <map>
#include <deque>
#include <IotWebConf.h>

#include <Arduino.h>
#include <APRS-Decoder.h>
#include <LoRa_APRS.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <APRS-IS.h>

#include "pins.h"
#include "settings.h"
#include "display.h"

#if defined(ARDUINO_T_Beam) && !defined(ARDUINO_T_Beam_V0_7)
#include "power_management.h"
PowerManagement powerManagement;
#endif

const char thingName[] = "APRSLoRaDigi";

// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "aprsrocks";

#define STRING_LEN 128
#define NUMBER_LEN 32

// -- Configuration specific key. The value should be modified if config structure was changed.
#define CONFIG_VERSION "dem2"


// -- Callback method declarations.
void configSaved();
boolean formValidator();

DNSServer dnsServer;
WebServer server(80);

char ccall[STRING_LEN];
char caprsishost[STRING_LEN];
char caprsisport[NUMBER_LEN];
char cpasskey[STRING_LEN];
char cbeaconmessage[STRING_LEN];
char cbeacontimeout[NUMBER_LEN];
char cbeaconlat[NUMBER_LEN];
char cbeaconlon[NUMBER_LEN];
char cbeaconmsg[STRING_LEN];

#define FORWARD_TIMEOUT 5

IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);
IotWebConfParameter callParam = IotWebConfParameter("Callsign", "callParam", ccall, STRING_LEN);
IotWebConfParameter latParam = IotWebConfParameter("Latitude", "latParam", cbeaconlat, NUMBER_LEN, "number", "e.g. 50.411", NULL, "step='0.00001'");
IotWebConfParameter lonParam = IotWebConfParameter("Longitude", "lonParam", cbeaconlon, NUMBER_LEN, "number", "e.g. 3.123", NULL, "step='0.00001'");
IotWebConfSeparator separator1 = IotWebConfSeparator("APRS IS");
IotWebConfParameter aprsishostParam = IotWebConfParameter("APRS IS Host", "aprsishostParam", caprsishost, STRING_LEN);
IotWebConfParameter aprsisportParam = IotWebConfParameter("APRS IS Port", "aprsisportParam", caprsisport, NUMBER_LEN, "number", NULL, "14580", NULL);
IotWebConfParameter passkeyParam = IotWebConfParameter("APRSIS-Passkey", "passkeyParam", cpasskey, STRING_LEN);
IotWebConfSeparator separator2 = IotWebConfSeparator("Beacon");
IotWebConfParameter beaconmsgParam = IotWebConfParameter("Beacon Message", "beaconmsgParam", cbeaconmessage, STRING_LEN);
IotWebConfParameter beacontimeoutParam = IotWebConfParameter("Beacon Timeout", "beacontimeoutParam", cbeacontimeout, NUMBER_LEN, "number", "1..100", "5", "min='1' step='1'");
// -- We can add a legend to the separator

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * timer = NULL;
volatile uint secondsSinceLastTX = 0;
volatile uint secondsSinceStartup = 0;

LoRa_APRS lora_aprs;

String create_lat_aprs(double lat);
String create_long_aprs(double lng);

// WiFiMulti WiFiMulti;
String BeaconMsg;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, 60*60);
APRS_IS * aprs_is = 0;
bool haveWiFi=false;
void setup_lora();
void setup_wifi();
void setup_ntp();
void setup_aprs_is();

std::map<uint, std::shared_ptr<APRSMessage> > lastMessages;
std::deque<std::pair<String, std::shared_ptr<APRSMessage> > > lastnMessages;
void configSaved()
{
  Serial.println("Configuration was updated.");
}

void wifiConnected()
{
	haveWiFi = true;
    Serial.println("WiFi was connected.");
    setup_ntp();
}

void IRAM_ATTR onTimer()
{
	portENTER_CRITICAL_ISR(&timerMux);
	secondsSinceLastTX++;
	secondsSinceStartup++;
	portEXIT_CRITICAL_ISR(&timerMux);
}


// cppcheck-suppress unusedFunction
void setup()
{
	
	Serial.begin(115200);

#if defined(ARDUINO_T_Beam) && !defined(ARDUINO_T_Beam_V0_7)
	Wire.begin(SDA, SCL);
	if (!powerManagement.begin(Wire))
	{
		Serial.println("LoRa-APRS / Init / AXP192 Begin PASS");
	} else {
		Serial.println("LoRa-APRS / Init / AXP192 Begin FAIL");
	}
	powerManagement.activateLoRa();
	powerManagement.activateOLED();
	powerManagement.deactivateGPS();
#endif

	setup_display();
	
	delay(500);
	Serial.println("[INFO] LoRa APRS Digi by OE5BPA (Peter Buchegger), WebConf by DB5SB");
	show_display(ccall, "LoRa APRS Digi", "", 2000);

	setup_wifi();
	setup_lora();
	//setup_lora();
	setup_aprs_is();


	timer = timerBegin(0, 80, true);
	timerAlarmWrite(timer, 1000000, true);
	timerAttachInterrupt(timer, &onTimer, true);
	timerAlarmEnable(timer);
	
	delay(500);
	if (strlen(cbeacontimeout)==0) strcpy(cbeacontimeout, "5");
	if (strlen(ccall)==0) strcpy(ccall, "N0CALL");
}

// cppcheck-suppress unusedFunction
void loop()
{
	iotWebConf.doLoop();
	static bool send_update = true;
	if(secondsSinceLastTX >= (atoi(cbeacontimeout)*60))
	{
		portENTER_CRITICAL(&timerMux);
		secondsSinceLastTX -= (atoi(cbeacontimeout)*60);
		portEXIT_CRITICAL(&timerMux);
		send_update = true;
	}

	if(WiFi.isConnected() && WiFi.getMode() != WIFI_AP && !aprs_is->connected())
	{
		Serial.print("[INFO] connecting to server: ");
		Serial.print(caprsishost);
		Serial.print(" on port: ");
		Serial.println(caprsisport);
		show_display("INFO", "Connecting to server");
		if(!aprs_is->connect(caprsishost, atoi(caprsisport)))
		{
			Serial.println("[ERROR] Connection failed.");
			Serial.println("[INFO] Waiting 5 seconds before retrying...");
			show_display("ERROR", "Server connection failed!", "waiting 5 sec");
			delay(5000);
			return;
		}
		Serial.println("[INFO] Connected to server!");
	}

	if(send_update && strcmp(ccall, "N0CALL")!=0)
	{
		send_update = false;

		std::shared_ptr<APRSMessage> msg = std::shared_ptr<APRSMessage>(new APRSMessage());
		msg->setSource(ccall);
		msg->setDestination("APLG0");
		String lat = create_lat_aprs(atof(cbeaconlat));
		String lng = create_long_aprs(atof(cbeaconlon));
		msg->getAPRSBody()->setData(String("=") + lat + "R" + lng + "#" + cbeaconmessage);
		String data = msg->encode();
		Serial.print(data);
		show_display(ccall, "<< Beaconing myself >>", data);
		lora_aprs.sendMessage(msg);
		if (WiFi.isConnected() && WiFi.getMode() != WIFI_AP) aprs_is->sendMessage(msg);
		Serial.println("finished TXing...");
		lastnMessages.push_back(std::pair<String, std::shared_ptr<APRSMessage>>(timeClient.getFormattedTime(), msg));
	}

	if(lora_aprs.hasMessage())
	{
		std::shared_ptr<APRSMessage> msg = lora_aprs.getMessage();

		if(msg->getSource().indexOf(ccall) != -1)
		{
			Serial.print("Message already received as repeater: '");
			Serial.print(msg->toString());
			Serial.print("' with RSSI ");
			Serial.print(lora_aprs.getMessageRssi());
			Serial.print(" and SNR ");
			Serial.println(lora_aprs.getMessageSnr());
			return;
		}

		// lets try not to flood the LoRa frequency in limiting the same messages:
		std::map<uint, std::shared_ptr<APRSMessage>>::iterator foundMsg = std::find_if(lastMessages.begin(), lastMessages.end(), [&](std::pair<const unsigned int, std::shared_ptr<APRSMessage> > & old_msg)
			{
				if(msg->getSource() == old_msg.second->getSource() &&
					msg->getDestination() == old_msg.second->getDestination() &&
					msg->getAPRSBody()->getData() == old_msg.second->getAPRSBody()->getData())
				{
					return true;
				}
				return false;
			});

		if(foundMsg == lastMessages.end())
		{
			show_display(ccall, "RSSI: " + String(lora_aprs.getMessageRssi()) + ", SNR: " + String(lora_aprs.getMessageSnr()), msg->toString(), 0);
			Serial.print("Received packet '");
			Serial.print(msg->toString());
			Serial.print("' with RSSI ");
			Serial.print(lora_aprs.getMessageRssi());
			Serial.print(" and SNR ");
			Serial.println(lora_aprs.getMessageSnr());
			msg->setPath(String(ccall) + "*");
			Serial.println("Digipeating the Message via HF");
			lora_aprs.sendMessage(msg);
			Serial.println("finished TXing...");
			Serial.println("Digipeating the Message via APRSIS");
			aprs_is->sendMessage(msg->encode());
			Serial.println("finished Uploading...");
			lastMessages.insert({secondsSinceStartup, msg});
			lastnMessages.push_back(std::pair<String, std::shared_ptr<APRSMessage>>(timeClient.getFormattedTime(), msg));
		}
		else
		{
			Serial.print("Message already received (timeout): '");
			Serial.print(msg->toString());
			Serial.print("' with RSSI ");
			Serial.print(lora_aprs.getMessageRssi());
			Serial.print(" and SNR ");
			Serial.println(lora_aprs.getMessageSnr());
		}
		return;
	}

	for(std::map<uint, std::shared_ptr<APRSMessage>>::iterator iter = lastMessages.begin(); iter != lastMessages.end(); )
	{
		if(secondsSinceStartup >= iter->first + FORWARD_TIMEOUT*60)
		{
			iter = lastMessages.erase(iter);
		}
		else
		{
			iter++;
		}
	}
	while(lastnMessages.size()>100) { Serial.println("Purging LastNMessages: "+lastnMessages.size()); lastnMessages.pop_front(); }

	static int _secondsSinceLastTX = 0;
	if(secondsSinceLastTX != _secondsSinceLastTX)
	{
		show_display(ccall, "Time to next beaconing: " + String((atoi(cbeacontimeout)*60) - secondsSinceLastTX));
	}
}

void setup_lora()
{
	lora_aprs.tx_frequency = LORA_RX_FREQUENCY;
	//lora_aprs.rx_frequency = LORA_TX_FREQUENCY; // for debugging
	if (!lora_aprs.begin())
	{
		Serial.println("[ERROR] Starting LoRa failed!");
		show_display("ERROR", "Starting LoRa failed!");
		while (1);
	}
	Serial.println("[INFO] LoRa init done!");
	show_display("INFO", "LoRa init done!", 2000);
}

String create_lat_aprs(double lat)
{
	char str[20];
	char n_s = 'N';
	if(lat < 0)
	{
		n_s = 'S';
	}
	lat = std::abs(lat);
	sprintf(str, "%02d%05.2f%c", (int)lat, (lat - (double)((int)lat)) * 60.0, n_s);
	String lat_str(str);
	return lat_str;
}

String create_long_aprs(double lng)
{
	char str[20];
	char e_w = 'E';
	if(lng < 0)
	{
		e_w = 'W';
	}
	lng = std::abs(lng);
	sprintf(str, "%03d%05.2f%c", (int)lng, (lng - (double)((int)lng)) * 60.0, e_w);
	String lng_str(str);
	return lng_str;
}

void setup_ntp()
{
	Serial.println("setting up NTP");
	timeClient.begin();
	if(!timeClient.forceUpdate())
	{
		Serial.println("[WARN] NTP Client force update issue!");
		show_display("WARN", "NTP Client force update issue!", 2000);
	}
	Serial.println("[INFO] NTP Client init done!");
	show_display("INFO", "NTP Client init done!", 2000);
}

void setup_aprs_is()
{
	Serial.println("setting up APRS-IS");
	aprs_is = new APRS_IS(ccall, cpasskey , "DB5SB-LoRaDigi", "0.2");

	APRSMessage msg;
    String lat = create_lat_aprs(atof(cbeaconlat));
	String lng = create_long_aprs(atof(cbeaconlon));

	msg.setSource(ccall);
	msg.setDestination("APLG0");
	msg.getAPRSBody()->setData(String("=") + lat + "I" + lng + "&" + cbeaconmessage);
	BeaconMsg = msg.encode();
}
void handleRoot()
{
	// -- Let IotWebConf test and handle captive portal requests.
	if (iotWebConf.handleCaptivePortal())
	{
		// -- Captive portal request were already served.
		return;
	}
	Serial.println("Sending Homepage");
	String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
	s += "<title>Digi Web Config</title></head><body>";
	s += "Go to <a href='config'>configure page</a> to change values.";
	s += "<h3>Last 100 Messages</h3>";
	s += "<table border=\"1\" cellspacing=\"0\"><thead><tr><th>Time</th><th>Call</th><th>Message</th></tr></thead><tbody>\n";
	for(std::deque<std::pair<String, std::shared_ptr<APRSMessage>>>::iterator iter = lastnMessages.begin(); iter != lastnMessages.end(); iter++)
	{
 		Serial.println("Adding Row to Table 1");
		s+= "<tr>\n";

		s+= "<td>";
		s+= (*iter).first;
		s+= "</td>";

		s+= "<td>";
		s+= (*iter).second->getSource();
		s+= "</td>";

		s+= "<td>";
		s+= (*iter).second->toString();
		s+= "</td>";

		s+= "</tr>\n";
	}
	s += "</tbody></table>\n";
	s += "<h3>Last 5 Minutes</h3>";
	s += "<table border=\"1\" cellspacing=\"0\"><thead><tr><th>Time since startup</th><th>Call</th><th>Message</th></tr></thead><tbody>\n";
	for(std::map<uint, std::shared_ptr<APRSMessage>>::iterator iter = lastMessages.begin(); iter != lastMessages.end(); iter++)
	{
 		Serial.println("Adding Row to Table 2");
		s+= "<tr>\n";

		s+= "<td>";
		s+= iter->first;
		s+= "</td>";

		s+= "<td>";
		s+= iter->second->getSource();
		s+= "</td>";

		s+= "<td>";
		s+= iter->second->toString();
		s+= "</td>";

		s+= "</tr>\n";
	}
	s += "</tbody></table>\n";
	s += "</body></html>\n";
 	server.send(200, "text/html", s);
	Serial.println("Homepage sent.");
}

void setup_wifi()
{
//	WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
//	WiFi.setHostname(CALL);
//	WiFiMulti.addAP(WIFINAME, WIFIPASS);
	Serial.print("[INFO] Waiting for WiFi");
	show_display("INFO", "Waiting for WiFi");
  iotWebConf.addParameter(&callParam);
  iotWebConf.addParameter(&latParam);
  iotWebConf.addParameter(&lonParam);
  iotWebConf.addParameter(&separator1);
  iotWebConf.addParameter(&aprsishostParam);
  iotWebConf.addParameter(&aprsisportParam);
  iotWebConf.addParameter(&passkeyParam);
  iotWebConf.addParameter(&separator2);
  iotWebConf.addParameter(&beaconmsgParam);
  iotWebConf.addParameter(&beacontimeoutParam);
  iotWebConf.setWifiConnectionCallback(&wifiConnected);
  iotWebConf.init();
	
  server.on("/", handleRoot);
  server.on("/config", []{ iotWebConf.handleConfig(); });
  server.onNotFound([](){ iotWebConf.handleNotFound(); });

}