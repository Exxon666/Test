/*
  On Test#LifeBit do
  delay 10
  Publish '*%uptime%,[Test#Emergency],2,2,2,%rssi%,%sysday%,%sysmonth%,%sysyear%,%syshour%,%sysmin%@'
  Publish test
  endon
*/

#include <DateTime.h>
#include <DS323x.h>

#include <AltSoftSerial.h>
#include <Adafruit_MCP23X17.h>
#include <EEPROM.h>
#include <Wire.h>
#include <PWFusion_TCA9548A.h>
#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;
DS323x rtc;
TCA9548A i2cMux;
AltSoftSerial esp8266;        // RX pin 8, TX pin 9 -> to RX on esp8266
Adafruit_MCP23X17 mcp;

const byte numChars = 45;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
byte StateTerarium[5] = {2, 2, 2, 2, 2};
byte Terarium[5];
byte ventilace = 0;

boolean newData = false;

String MQTThost = "Terarium";
String MQTTHeatMap = MQTThost + "/HeatMap";
String MQTTInfo = MQTThost + "/Info";

unsigned long time_pauza = 0;
unsigned long pauza_Period = 1800000;        // maximum 65,535
unsigned int MQTT_Period = 28012;        // maximum 65,535
unsigned long AHT_Period = 30230;
unsigned int fade_Period = 1000;          // maximum 65,535
unsigned long Terarium_Period = 60124;
unsigned int Watchdog_Timeout = 40017;   // maximum 65,535
unsigned int lifebit_Period = 9800;      // maximum 65,535
unsigned long ventilace_Period = 303211;   // maximum 65,535
unsigned long timesync_Period = 1800000;  // 1800000
unsigned long time_lifebit = 0;
unsigned long time_timesync = 0;
unsigned long time_ventilace = 0;
unsigned long time_mqtt = 0;
unsigned long time_aht = 0;
unsigned long time_watchdog = 0;
unsigned long time_terarium = 0;
unsigned long startTime = 0;
unsigned long time_fade = 0;

int life_bit_counter = 0;
int life_bit_esp = 0;
int life_bit_arduino = 0;
byte fadeValue = 0;
byte ZapinaniTeraria = 0;
byte VypinaniTeraria = 0;

int TerarkoMinutesOn = 510;  // 8:30 = 8 * 60 + 30 = 510
int TerarkoMinutesOff = 1125;  // 18:45 = 18 * 60 + 45 = 1125
int AktualMinutes = 0;

int TerarkoMinutesOn_temp = 0;
int TerarkoMinutesOff_temp = 0;
byte WriteToEeprom = 0;

byte EspDay = 0;
byte EspMonth = 0;
int EspYear = 0;
byte EspHour = 0;
byte EspMinute = 0;
byte EspSeconds = 0;

float tempe[10];
float humi[10];
sensors_event_t humidity, temp;

#define PWM1 6      // Led osvetleni
//#define PWM2 6
//#define PWM3 5
#define ResetEsp 7      // Reset ESP-easy (Arduino)

int Rele[8] = {0, 1, 2, 3, 4, 5, 6, 7};  //(MCP23017)

#define  Min_On_Eeprom  0
#define  Min_Off_Eeprom  8

//============

void setup() {
  esp8266.begin(19200);
//  Serial.begin(19200);
  mcp.begin_I2C();

  Wire.begin();
  i2cMux.begin();
  aht.begin();

  rtc.attach(Wire);

  //EEPROM.put(Min_On_Eeprom, TerarkoMinutesOn);
  //EEPROM.put(Min_Off_Eeprom, TerarkoMinutesOff);
  EEPROM.get(Min_On_Eeprom,  TerarkoMinutesOn);
  EEPROM.get(Min_Off_Eeprom, TerarkoMinutesOff);

  pinMode(PWM1, OUTPUT);
  //pinMode(PWM2, OUTPUT);
  //pinMode(PWM3, OUTPUT);
  mcp.pinMode(Rele[0], OUTPUT);
  mcp.pinMode(Rele[1], OUTPUT);
  mcp.pinMode(Rele[2], OUTPUT);
  mcp.pinMode(Rele[3], OUTPUT);
  mcp.pinMode(Rele[4], OUTPUT);
  mcp.pinMode(Rele[5], OUTPUT);
  mcp.pinMode(Rele[6], OUTPUT);
  mcp.pinMode(Rele[7], OUTPUT);

  //  mcp.pinMode(OK, INPUT_PULLUP);
}

//============

void loop() {

  //=========== Cteni teplot ===========
  // Cte jedno cidlo po druhem pred kazdym ctenim se prepne multiplex I2C

  if (millis() - time_aht > AHT_Period) {

    i2cMux.setChannel(CHAN0);
    ReadAht(0);
    i2cMux.setChannel(CHAN1);
    ReadAht(1);
    i2cMux.setChannel(CHAN2);
    ReadAht(2);
    i2cMux.setChannel(CHAN3);
    ReadAht(3);
    i2cMux.setChannel(CHAN4);
    ReadAht(4);
    i2cMux.setChannel(CHAN5);
    ReadAht(5);
    i2cMux.setChannel(CHAN6);
    ReadAht(6);
    i2cMux.setChannel(CHAN7);
    ReadAht(7);
    i2cMux.setChannel(CHAN_NONE);

    time_aht = millis();
  }

  //=========== Synchronizace casu ===========
  // provede se pouze pokud je rozdil v hodine nebo minute Arduino vs ESP
  // rok musi byt vetsi nez 2021 (esp bez nastaveneho casu posila 1970)

  DateTime now = rtc.now();
  if (millis() - time_timesync > timesync_Period) {
    if (now.hour() != EspHour || now.minute() != EspMinute) {
      if (EspYear > 2021) {
        rtc.now(DateTime(EspYear, EspMonth, EspDay, EspHour, EspMinute, EspSeconds));
      }
    }
    time_timesync = millis();
  }

  //=========== Prijem dat z ESP-Easy ===========

  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    //  showParsedData();
    newData = false;
  }

  //=========== Watchdog ESP ===========
  // pokud neprijde life bit z esp po stanovenou dobu (Watchdog_Timeout)
  // ESP bude restartovano

  if (life_bit_esp != life_bit_counter) {   // porovnavani zda prisel novy lifebit
    life_bit_counter = life_bit_esp;
    time_watchdog = millis();            // reset watchdogu
  }

  if (millis() - time_watchdog > Watchdog_Timeout) {
    digitalWrite(ResetEsp, 1);
    delay (150);
    digitalWrite(ResetEsp, 0);
    delay (1000);
    time_watchdog = millis();
  }


  //========= Odeslani MQTT dat do ESP-Easy ==========

  if (millis() - time_mqtt > MQTT_Period) {

    sendMQTT(MQTTHeatMap, ((String)"'" + (String)"{\"T1\":" + (tempe[0]) + (String)",\"H1\":" + (humi[0]) + (String)",\"T2\":" + (tempe[1]) + (String)",\"H2\":" + (humi[1]) + (String)"}'"));
    delay(50);
    sendMQTT(MQTTHeatMap, ((String)"'" + (String)"{\"T3\":" + (tempe[2]) + (String)",\"H3\":" + (humi[2]) + (String)",\"T4\":" + (tempe[3]) + (String)",\"H4\":" + (humi[3]) + (String)"}'"));
    delay(50);
    sendMQTT(MQTTHeatMap, ((String)"'" + (String)"{\"T5\":" + (tempe[4]) + (String)",\"H5\":" + (humi[4]) + (String)",\"T6\":" + (tempe[5]) + (String)",\"H6\":" + (humi[5]) + (String)"}'"));
    delay(50);
    sendMQTT(MQTTHeatMap, ((String)"'" + (String)"{\"T7\":" + (tempe[6]) + (String)",\"H7\":" + (humi[6]) + (String)",\"T8\":" + (tempe[7]) + (String)",\"H8\":" + (humi[7]) + (String)"}'"));

//    Serial.println ((String)"'" + (String)"{\"T1\":" + (tempe[0]) + (String)",\"H1\":" + (humi[0]) + (String)",\"T2\":" + (tempe[1]) + (String)",\"H2\":" + (humi[1]) + (String)"}'");
//    delay(50);
//    Serial.println ((String)"'" + (String)"{\"T3\":" + (tempe[2]) + (String)",\"H3\":" + (humi[2]) + (String)",\"T4\":" + (tempe[3]) + (String)",\"H4\":" + (humi[3]) + (String)"}'");
//    delay(50);
//    Serial.println ((String)"'" + (String)"{\"T5\":" + (tempe[4]) + (String)",\"H5\":" + (humi[4]) + (String)",\"T6\":" + (tempe[5]) + (String)",\"H6\":" + (humi[5]) + (String)"}'");
//    delay(50);
//    Serial.println ((String)"'" + (String)"{\"T7\":" + (tempe[6]) + (String)",\"H7\":" + (humi[6]) + (String)",\"T8\":" + (tempe[7]) + (String)",\"H8\":" + (humi[7]) + (String)"}'");

    //Serial.println("Poslano mqtt");
    time_mqtt = millis();
  }

  //========= Odeslani LifeBitu do ESP-Easy ==========

  if (millis() - time_lifebit > lifebit_Period) {
    //   esp8266.println("Test");
    esp8266.print("taskvalueset,1,1,");
    esp8266.println(life_bit_arduino);
    esp8266.println("taskRun,1");
    life_bit_arduino++;
    if (life_bit_arduino > 9999) {
      life_bit_arduino = 0;
    }
    time_lifebit = millis();
  }

  //========= Ovladani terarii ==========

  if (millis() - time_terarium > Terarium_Period) {
    OvladaniTerarium(0);
    OvladaniTerarium(1);
    OvladaniTerarium(2);
    OvladaniTerarium(3);
    OvladaniTerarium(4);
    time_terarium = millis();
  }


  //========= Zapinani velkeho teraria ==========

  if (ZapinaniTeraria == 1) {
    VypinaniTeraria = 0;

    if (millis() - time_fade > fade_Period) {
      if (fadeValue < 120) {
        fadeValue++;
        //Serial.println (fadeValue);
        analogWrite(PWM1, fadeValue);
        if (fadeValue == 120) {
          time_pauza = millis();
        }
      } else {
        if (millis() - time_pauza > pauza_Period) {
          ZapinaniTeraria = 0;
          mcp.digitalWrite(Rele[4], 1);
          mcp.digitalWrite(Rele[5], 1);
        }
        // fadeValue = 0;
        //ZapinaniTeraria = 0;
        //mcp.digitalWrite(Rele[4], 1);
        //mcp.digitalWrite(Rele[5], 1);
      }
      time_fade = millis();
    }
  }


  //========= Vypinani velkeho teraria ==========

  if (VypinaniTeraria == 1) {
    ZapinaniTeraria = 0;
    if (millis() - time_pauza > pauza_Period) {
      if (millis() - time_fade > fade_Period) {
        if (fadeValue > 0) {
          fadeValue--;
          // Serial.println (fadeValue);
          analogWrite(PWM1, fadeValue);
          // mcp.digitalWrite(Rele[4], 0);
          // mcp.digitalWrite(Rele[5], 0);
        } else {
          VypinaniTeraria = 0;
        }
        time_fade = millis();
      }
    }
  }
  //========= Ovladani ventilatoru ==========

  if (StateTerarium[0] == 1 || StateTerarium[1] == 1 || StateTerarium[2] == 1 || StateTerarium[3] == 1 || StateTerarium[4] == 1) {
    mcp.digitalWrite(Rele[7], 1);
    ventilace = 1;
  } else {
    if (millis() - time_ventilace > ventilace_Period) {
      if (ventilace == 0) {
        mcp.digitalWrite(Rele[7], 1);
        ventilace = 1;
        sendMQTT(MQTTInfo, ((String)"'" + (String)"\"Vent1\":" + (ventilace) + (String)"'"));
      } else {
        mcp.digitalWrite(Rele[7], 0);
        ventilace = 0;
        sendMQTT(MQTTInfo, ((String)"'" + (String)"\"Vent1\":" + (ventilace) + (String)"'"));
      }
      time_ventilace = millis();
    }
  }

  //========= Kontrola aktualnosti prijatych a nastavenych dat (časů) ==========
  //========= Pokud jsou rozdílná a je nastaven požadavek pro přepsání přepíšou se v paměti ==========

  if (WriteToEeprom == 42) {
    if (TerarkoMinutesOn_temp != TerarkoMinutesOn && TerarkoMinutesOn_temp <= 600 && TerarkoMinutesOn_temp >= 300) {
      EEPROM.put(Min_On_Eeprom, TerarkoMinutesOn_temp);
      TerarkoMinutesOn = TerarkoMinutesOn_temp;
    }
    if (TerarkoMinutesOff_temp != TerarkoMinutesOff && TerarkoMinutesOff_temp <= 1370 && TerarkoMinutesOff_temp >= 780) {
      EEPROM.put(Min_Off_Eeprom, TerarkoMinutesOff_temp);
      TerarkoMinutesOff = TerarkoMinutesOff_temp;
    }
    esp8266.println("taskvalueset,1,4,0");
    WriteToEeprom = 0;
  }

}


// ---------------------------------------------------------------------------------------------
// ------------------------------- Konec Loopu ---------------------------------
// ---------------------------------------------------------------------------------------------

//========= Cteni teploty a vlhkosti z cidel AHT10x ==========

void ReadAht(int ii) {
  temp.temperature = 0;
  humidity.relative_humidity = 0;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  tempe[ii] = temp.temperature;
  humi[ii] = humidity.relative_humidity;
}

//========= Ovladani svetel v terariu ==========

void OvladaniTerarium(int num) {
  DateTime now = rtc.now();
  AktualMinutes = now.hour() * 60 + now.minute();
  //Serial.println(AktualMinutes);
  //Serial.println(TerarkoMinutesOff);

  if (AktualMinutes >= TerarkoMinutesOn && AktualMinutes <= TerarkoMinutesOff) {
    if (Terarium[num] == 1 && StateTerarium[num] != 1) {
      if (num != 4) {                      // pokud se nejedna o velke terarium
        mcp.digitalWrite(Rele[num], 1);
        StateTerarium[num] = 1;
        sendMQTT(MQTTInfo, ((String)"'" + (String)"\"Terarko" + (num) + (String)"\":" + (1) + (String)"'"));
      } else if (num == 4) {
        ZapinaniTeraria = 1;  // Zapinani velkeho teraria
        StateTerarium[num] = 1;
        fadeValue = 0;        // Rozsveceni PWM
        sendMQTT(MQTTInfo, ((String)"'" + (String)"\"Terarko" + (num) + (String)"\":" + (1) + (String)"'"));
      }
    }
  } else if (StateTerarium[num] != 0) {
    if (num != 4) {                      // pokud se nejedna o velke terarium
      mcp.digitalWrite(Rele[num], 0);
      sendMQTT(MQTTInfo, ((String)"'" + (String)"\"Terarko" + (num) + (String)"\":" + (0) + (String)"'"));
      StateTerarium[num] = 0;
    } else if (num == 4) {
      VypinaniTeraria = 1;  // Vypinani velkeho teraria
      time_pauza = millis();
      fadeValue = 120;        // Zhasinani PWM
      sendMQTT(MQTTInfo, ((String)"'" + (String)"\"Terarko" + (num) + (String)"\":" + (0) + (String)"'"));
      StateTerarium[num] = 0;
      mcp.digitalWrite(Rele[4], 0);
      mcp.digitalWrite(Rele[5], 0);
    }
  }
}

//========= Ulozeni a rozdeleni prijatych dat ==========

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '*';
  char endMarker = '@';
  char rc;

  while (esp8266.available() > 0 && newData == false) {
    rc = esp8266.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

//========= Parsovani prijatych dat a ulozeni do promenych ==========

void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ","); // this continues where the previous call left off
  life_bit_esp = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  Terarium[0] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  Terarium[1] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  Terarium[2] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  Terarium[3] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  Terarium[4] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  EspDay = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  EspMonth = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  EspYear = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  EspHour = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  EspMinute = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  EspSeconds = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  TerarkoMinutesOn_temp = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  TerarkoMinutesOff_temp = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  WriteToEeprom = atoi(strtokIndx);     // convert this part to an integer

}

//========= Funkce pro odesilani MQTT dat ==========

String sendMQTT(String MQTTpublish, String MQTTvalue) {
  String response = "OK";
  String MQTTdata = "publish " + MQTTpublish + "," + MQTTvalue;
  esp8266.println(MQTTdata);
  return response;
}
