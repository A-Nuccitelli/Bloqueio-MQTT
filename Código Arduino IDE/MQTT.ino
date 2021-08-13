//------------------------------------------ Begin Wifi Manager (include) -----------------------------------------
#define ESP_WIFIMANAGER_VERSION_MIN_TARGET     "ESP_WiFiManager v1.7.3"
#define _WIFIMGR_LOGLEVEL_    3
#include <FS.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;

#define USE_LITTLEFS      true
  #if USE_LITTLEFS
    #include <LittleFS.h>
    FS* filesystem = &LittleFS;
    #define FileFS    LittleFS
    #define FS_Name       "LittleFS"
  #else
    FS* filesystem = &SPIFFS;
    #define FileFS    SPIFFS
    #define FS_Name       "SPIFFS"
  #endif
  //////
  
#define ESP_getChipId()   (ESP.getChipId())
#define LED_ON      LOW
#define LED_OFF     HIGH

//PIN_D0 can't be used for PWM/I2C
#define PIN_D0            16        // Pin D0 mapped to pin GPIO16/USER/WAKE of ESP8266. This pin is also used for Onboard-Blue LED. PIN_D0 = 0 => LED ON
#define PIN_D1            5         // Pin D1 mapped to pin GPIO5 of ESP8266
#define PIN_D2            4         // Pin D2 mapped to pin GPIO4 of ESP8266
#define PIN_D3            0         // Pin D3 mapped to pin GPIO0/FLASH of ESP8266
#define PIN_D4            2         // Pin D4 mapped to pin GPIO2/TXD1 of ESP8266
#define PIN_LED           2         // Pin D4 mapped to pin GPIO2/TXD1 of ESP8266, NodeMCU and WeMoS, control on-board LED

const int TRIGGER_PIN   = 13; // D3 on NodeMCU and WeMos.
const char* CONFIG_FILE = "/ConfigSW.json";
bool readConfigFile();
bool writeConfigFile();
String ssid = "MKF_" + String(ESP_getChipId(), HEX);
String password;
String Router_SSID;
String Router_Pass;

#define FORMAT_FILESYSTEM         false
#define MIN_AP_PASSWORD_SIZE    8
#define SSID_MAX_LEN            32
#define PASS_MAX_LEN            64

typedef struct
{
  char wifi_ssid[SSID_MAX_LEN];
  char wifi_pw  [PASS_MAX_LEN];
}  WiFi_Credentials;

typedef struct
{
  String wifi_ssid;
  String wifi_pw;
}  WiFi_Credentials_String;

#define NUM_WIFI_CREDENTIALS      2
#define TZNAME_MAX_LEN            50
#define TIMEZONE_MAX_LEN          50

typedef struct
{
  WiFi_Credentials  WiFi_Creds [NUM_WIFI_CREDENTIALS];
  char TZ_Name[TZNAME_MAX_LEN];     // "America/Toronto"
  char TZ[TIMEZONE_MAX_LEN];        // "EST5EDT,M3.2.0,M11.1.0"
  uint16_t checksum;
} WM_Config;

WM_Config         WM_config;

#define  CONFIG_FILENAME              F("/wifi_cred.dat")
bool initialConfig = false;
#define USE_AVAILABLE_PAGES     false
#define USE_ESP_WIFIMANAGER_NTP     true
#define USING_AMERICA       true
#define USE_CLOUDFLARE_NTP          false
#define USING_CORS_FEATURE          true

#if (defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP)
  // Force DHCP to be true
  #if defined(USE_DHCP_IP)
    #undef USE_DHCP_IP
  #endif
  #define USE_DHCP_IP     true
#else
  #define USE_DHCP_IP     false
#endif

#if ( USE_DHCP_IP )
  #warning Using DHCP IP
  IPAddress stationIP   = IPAddress(0, 0, 0, 0);
  IPAddress gatewayIP   = IPAddress(192, 168, 1, 1);
  IPAddress netMask     = IPAddress(255, 255, 255, 0);
#else
  #warning Using static IP
  #ifdef ESP32
    IPAddress stationIP   = IPAddress(192, 168, 1, 232);
  #else
    IPAddress stationIP   = IPAddress(192, 168, 1, 186);
  #endif
    IPAddress gatewayIP   = IPAddress(192, 168, 1, 1);
  IPAddress netMask     = IPAddress(255, 255, 255, 0);
#endif

#define USE_CONFIGURABLE_DNS      true

IPAddress dns1IP      = gatewayIP;
IPAddress dns2IP      = IPAddress(8, 8, 8, 8);

#define USE_CUSTOM_AP_IP          false

IPAddress APStaticIP  = IPAddress(192, 168, 100, 1);
IPAddress APStaticGW  = IPAddress(192, 168, 100, 1);
IPAddress APStaticSN  = IPAddress(255, 255, 255, 0);

#include <ESP_WiFiManager.h>              //https://github.com/khoih-prog/ESP_WiFiManager

uint8_t connectMultiWiFi();
WiFi_AP_IPConfig  WM_AP_IPconfig;
WiFi_STA_IPConfig WM_STA_IPconfig;

void initAPIPConfigStruct(WiFi_AP_IPConfig &in_WM_AP_IPconfig)
{
  in_WM_AP_IPconfig._ap_static_ip   = APStaticIP;
  in_WM_AP_IPconfig._ap_static_gw   = APStaticGW;
  in_WM_AP_IPconfig._ap_static_sn   = APStaticSN;
}

void initSTAIPConfigStruct(WiFi_STA_IPConfig &in_WM_STA_IPconfig)
{
  in_WM_STA_IPconfig._sta_static_ip   = stationIP;
  in_WM_STA_IPconfig._sta_static_gw   = gatewayIP;
  in_WM_STA_IPconfig._sta_static_sn   = netMask;
#if USE_CONFIGURABLE_DNS  
  in_WM_STA_IPconfig._sta_static_dns1 = dns1IP;
  in_WM_STA_IPconfig._sta_static_dns2 = dns2IP;
#endif
}

void displayIPConfigStruct(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
  LOGERROR3(F("stationIP ="), in_WM_STA_IPconfig._sta_static_ip, ", gatewayIP =", in_WM_STA_IPconfig._sta_static_gw);
  LOGERROR1(F("netMask ="), in_WM_STA_IPconfig._sta_static_sn);
#if USE_CONFIGURABLE_DNS
  LOGERROR3(F("dns1IP ="), in_WM_STA_IPconfig._sta_static_dns1, ", dns2IP =", in_WM_STA_IPconfig._sta_static_dns2);
#endif
}

void configWiFi(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
  #if USE_CONFIGURABLE_DNS  
    // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn, in_WM_STA_IPconfig._sta_static_dns1, in_WM_STA_IPconfig._sta_static_dns2);  
  #else
    // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn);
  #endif 
}

uint8_t connectMultiWiFi()
{
  #define WIFI_MULTI_1ST_CONNECT_WAITING_MS             2200L
  #define WIFI_MULTI_CONNECT_WAITING_MS                   500L

  uint8_t status;
  LOGERROR(F("ConnectMultiWiFi with :"));

  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Flash-stored Router_SSID = "), Router_SSID, F(", Router_Pass = "), Router_Pass );
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass );
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
  }

  for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
  {
    // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
    if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
    {
      LOGERROR3(F("* Additional SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
    }
  }

  LOGERROR(F("Connecting MultiWifi..."));

  //WiFi.mode(WIFI_STA);

#if !USE_DHCP_IP
  configWiFi(WM_STA_IPconfig);
#endif

  int i = 0;
  status = wifiMulti.run();
  delay(WIFI_MULTI_1ST_CONNECT_WAITING_MS);

  while ( ( i++ < 20 ) && ( status != WL_CONNECTED ) )
  {
    status = WiFi.status();

    if ( status == WL_CONNECTED )
      break;
    else
      delay(WIFI_MULTI_CONNECT_WAITING_MS);
  }

  if ( status == WL_CONNECTED )
  {
    LOGERROR1(F("WiFi connected after time: "), i);
    LOGERROR3(F("SSID:"), WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
    LOGERROR3(F("Channel:"), WiFi.channel(), F(",IP address:"), WiFi.localIP() );
  }
  else
  {
    LOGERROR(F("WiFi not connected"));
/*  
#if ESP8266      
    ESP.reset();
#else
    ESP.restart();
#endif  */
  }

  return status;
}

#if USE_ESP_WIFIMANAGER_NTP

void printLocalTime()
{
#if ESP8266
  static time_t now;
  
  now = time(nullptr);
  
  if ( now > 1451602800 )
  {
    Serial.print("Local Date/Time: ");
    Serial.print(ctime(&now));
  }
#else
  struct tm timeinfo;

  getLocalTime( &timeinfo );
  if (timeinfo.tm_year > 100 )
  {
    Serial.print("Local Date/Time: ");
    Serial.print( asctime( &timeinfo ) );
  }
#endif
}

#endif

void heartBeatPrint()
{
#if USE_ESP_WIFIMANAGER_NTP
  printLocalTime();
#else
  static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    Serial.print(F("H"));        // H means connected to WiFi
  else
    Serial.print(F("F"));        // F means not connected to WiFi

  if (num == 80)
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    Serial.print(F(" "));
  }
#endif  
}

void check_WiFi()
{
  if ( (WiFi.status() != WL_CONNECTED) )
  {
    Serial.println(F("\nWiFi lost. Call connectMultiWiFi in loop"));
    connectMultiWiFi();
  }
}  

void check_status()
{
  static ulong checkstatus_timeout  = 0;
  static ulong checkwifi_timeout    = 0;

  static ulong current_millis;

#define WIFICHECK_INTERVAL    1000L

#if USE_ESP_WIFIMANAGER_NTP
  #define HEARTBEAT_INTERVAL    60000L
#else
  #define HEARTBEAT_INTERVAL    10000L
#endif

  current_millis = millis();
  
  if ((current_millis > checkwifi_timeout) || (checkwifi_timeout == 0))
  {
    check_WiFi();
    checkwifi_timeout = current_millis + WIFICHECK_INTERVAL;
  }

  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((current_millis > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint();
    checkstatus_timeout = current_millis + HEARTBEAT_INTERVAL;
  }
}

int calcChecksum(uint8_t* address, uint16_t sizeToCalc)
{
  uint16_t checkSum = 0;
  
  for (uint16_t index = 0; index < sizeToCalc; index++)
  {
    checkSum += * ( ( (byte*) address ) + index);
  }

  return checkSum;
}

bool loadConfigData()
{
  File file = FileFS.open(CONFIG_FILENAME, "r");
  LOGERROR(F("LoadWiFiCfgFile "));

  memset((void *) &WM_config,       0, sizeof(WM_config));

  // New in v1.4.0
  memset((void *) &WM_STA_IPconfig, 0, sizeof(WM_STA_IPconfig));
  //////

  if (file)
  {
    file.readBytes((char *) &WM_config,   sizeof(WM_config));

    // New in v1.4.0
    file.readBytes((char *) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
    //////

    file.close();
    LOGERROR(F("OK"));

    if ( WM_config.checksum != calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) ) )
    {
      LOGERROR(F("WM_config checksum wrong"));
      
      return false;
    }
    
    // New in v1.4.0
    displayIPConfigStruct(WM_STA_IPconfig);
    //////

    return true;
  }
  else
  {
    LOGERROR(F("failed"));

    return false;
  }
}

void saveConfigData()
{
  File file = FileFS.open(CONFIG_FILENAME, "w");
  LOGERROR(F("SaveWiFiCfgFile "));

  if (file)
  {
    WM_config.checksum = calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) );
    
    file.write((uint8_t*) &WM_config, sizeof(WM_config));

    displayIPConfigStruct(WM_STA_IPconfig);

    // New in v1.4.0
    file.write((uint8_t*) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
    //////

    file.close();
    LOGERROR(F("OK"));
  }
  else
  {
    LOGERROR(F("failed"));
  }
}

bool readConfigFile()
{
  // this opens the config file in read-mode
  File f = FileFS.open(CONFIG_FILE, "r");

  if (!f)
  {
    Serial.println(F("Configuration file not found"));
    return false;
  }
  else
  {
    size_t size = f.size();
    std::unique_ptr<char[]> buf(new char[size + 1]);
    f.readBytes(buf.get(), size);
    f.close();

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
    DynamicJsonDocument json(1024);
    auto deserializeError = deserializeJson(json, buf.get());
    if ( deserializeError )
    {
      Serial.println(F("JSON parseObject() failed"));
      return false;
    }
    serializeJson(json, Serial);
#else
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.parseObject(buf.get());
    if (!json.success())
    {
      Serial.println(F("JSON parseObject() failed"));
      return false;
    }
    json.printTo(Serial);
#endif
  }
  Serial.println(F("\nConfig file was successfully parsed"));
  return true;
}

bool writeConfigFile()
{
  Serial.println(F("Saving config file"));

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  DynamicJsonDocument json(1024);
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
#endif

  File f = FileFS.open(CONFIG_FILE, "w");
  if (!f)
  {
    Serial.println(F("Failed to open config file for writing"));
    return false;
  }

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  serializeJsonPretty(json, Serial);
  // Write data to file and close it
  serializeJson(json, f);
#else
  json.prettyPrintTo(Serial);
  // Write data to file and close it
  json.printTo(f);
#endif

  f.close();

  Serial.println(F("\nConfig file was successfully saved"));
  return true;
}
//------------------------------------------ End Wifi Manager (include) -------------------------------------------
//------------------------------------------ Begin EEPROM (include) -----------------------------------------------
#include <EEPROM.h>
//------------------------------------------ End EEPROM (include) -------------------------------------------------
//------------------------------------------ Begin MQTT (include)  ------------------------------------------------
#include <PubSubClient.h>
//------------------------------------------ End MQTT (include)  --------------------------------------------------
//------------------------------------------ Begin MQTT (define) --------------------------------------------------
#define TOPICO_SUBSCRIBE "MQTTMKFenvio-reg01"
#define TOPICO_PUBLISH   "MQTTMKFrecebe-reg01"
#define ID_MQTT  "MQTTMKF-reg01"
//------------------------------------------ End MQTT (define) ----------------------------------------------------
//------------------------------------------ Begin MQTT (Variaveis) -----------------------------------------------
const char* BROKER_MQTT0 = "test.mosquitto.org";
const char* BROKER_MQTT1 = "broker.emqx.io";
const char* BROKER_MQTT2 = "anuccitelli.ddns.net:80";
int BROKER_PORT = 1883;
int c = 0;
int g = 0;
WiFiClient espClient;
PubSubClient MQTT(espClient);
//------------------------------------------ End MQTT (Variaveis) -------------------------------------------------
//------------------------------------------ Begin Millis (Variaveis) ---------------------------------------------
unsigned long millisTarefa1 = millis();
char* buf = "0";
//------------------------------------------ End Millis (Variaveis) -----------------------------------------------
//------------------------------------------ Begin Trava (Variaveis) ----------------------------------------------
//const int botao1 = 4;
//------------------------------------------ End trava (Variaveis) ------------------------------------------------
//------------------------------------------ begin eeprom (Variaveis) ---------------------------------------------
uint8_t   trava; // 0 a 100  (inteiro)     - Ex.: 56
          // 0 a 255 - byte - 8 bits 1 byte

uint8_t   EstadoSaida; // 0 a 100  (inteiro)     - Ex.: 56
          // 0 a 255 - byte - 8 bits 1 byte

uint16_t  tempo;      // 0 a 100  (1 decimal)   - Ex.: 57.8
          // 0 a 65535 - word - 16 bits 2 bytes
          
const   byte  CFG_TRAVA  =  0;
const   byte  CFG_ESAIDA =  1 + CFG_TRAVA;
const   byte  CFG_TEMPO  =  1 + CFG_ESAIDA;
const   byte  CFG_TOTAL  =  2 + CFG_TEMPO;

uint8_t getEEPROMUInt8(byte offset) {
  // Le dado UInt8 da EEPROM (0 a 255)
  return EEPROM.read(offset);
}

void setEEPROMInt8(byte offset, uint8_t v) {
  // Grava dado UInt8 na EEPROM (0 a 255)
  EEPROM.write(offset, v);
}

uint16_t getEEPROMUInt16(byte offset) {
  // Le dado UInt16 da EEPROM (0 a 65535)
  return word(EEPROM.read(offset), EEPROM.read(offset + 1));
}

void setEEPROMInt16(byte offset, uint16_t v) {
  // Grava dado UInt16 na EEPROM (0 a 65535)
  EEPROM.write(offset, highByte(v));
  EEPROM.write(offset + 1, lowByte(v));
}
//------------------------------------------ Begin Declara Void (MQTT e Serial) -----------------------------------
void initSerial();
void initMQTT();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void VerificaConexoesWiFIEMQTT(void);
void InitOutput(void);
//------------------------------------------ End Declara Void (MQTT e Serial) -------------------------------------
//------------------------------------------ Begin Declara Void (millis) ------------------------------------------
void initMillisTarefa();
//------------------------------------------ End Delara Void (millis) ---------------------------------------------
//------------------------------------------ Begin Declara Void (trava) -------------------------------------------
void initTrava();
//------------------------------------------ End Declara Void (trava) ---------------------------------------------
//------------------------------------------ Begin Declara Void (eeprom) ------------------------------------------
void initEeprom();
//------------------------------------------ End Declara Void (eeprom) --------------------------------------------
//------------------------------------------ Begin Declara Void (Wifi) --------------------------------------------
void initWiFi();
void verificaWifi();
//------------------------------------------ End Declara Void (Wifi) ----------------------------------------------
//------------------------------------------ Begin Void (Setup) ---------------------------------------------------
void setup() 
{
    Serial.begin(115200);
    InitOutput();//Serial.println("Passo 1 setup");
    //initSerial();Serial.println("Passo 2 setup");
    initEeprom();//Serial.println("Passo 2 setup");
    initMillisTarefa();//Serial.println("Passo 3 setup");
    initTrava();//Serial.println("Passo 4 setup");
    initWiFi();//Serial.println("Passo 5 setup");
    initMQTT();//Serial.println("Passo 6 setup");
}

void InitOutput(void)
{
    EEPROM.begin(CFG_TOTAL);
    pinMode(4, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    //pinMode(botao1, INPUT_PULLUP);
    digitalWrite(4, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
}
/*
void initSerial() 
{
    Serial.begin(115200);
}*/
//------------------------------------------ End Void (Setup) -----------------------------------------------------
//------------------------------------------ Begin Void (millis) --------------------------------------------------
void initMillisTarefa(){
 // Serial.println("Passo 1 loop");
  if((millis() - millisTarefa1) > 7200100){
    //if((millis() - millisTarefa1) > 10000){
      if((tempo <= 140 && trava == 1) || (tempo <= 336 && trava == 4) || (tempo <= 3912 && trava == 2) || (tempo <= 8760 && trava == 3)){
      tempo++;
      setEEPROMInt16(CFG_TEMPO, tempo);
      EEPROM.commit();
     /* Serial.println("\n--- Dados da EEPROM");
      Serial.print("Trava = "); Serial.println(trava);
      Serial.print("Tempo      = "); Serial.println(tempo);
      Serial.print("EstadoSaida = "); Serial.println(EstadoSaida);*/
      millisTarefa1 = millis();
      }
    }
}
//------------------------------------------ End Void (millis) ----------------------------------------------------
//------------------------------------------ Begin Void (trava) ---------------------------------------------------
void initTrava(){
  //Serial.println("Passo 2 loop");

  if((tempo > 140 && trava == 1) || (tempo > 336 && trava == 4) || (tempo > 3912 && trava == 2) || (tempo > 8760 && trava == 3) || (EstadoSaida == 1)){
    digitalWrite(4, LOW);digitalWrite(LED_BUILTIN, LOW);
    }else {
       digitalWrite(4, HIGH);digitalWrite(LED_BUILTIN, HIGH);
    }

  /*if(digitalRead(botao1) == LOW){
    tempo = tempo + 1;
    Serial.println("botao pressionado, tempo");
    Serial.println(tempo);
    Serial.println("botao pressionado, trava");
    Serial.println(trava);
  }*/
}
//------------------------------------------- End Void (trava) ----------------------------------------------------
//------------------------------------------- Begin Void (eeprom) -------------------------------------------------
void initEeprom(){

  // Le dados da EEPROM
  trava  = getEEPROMUInt8(CFG_TRAVA);
  EstadoSaida  = getEEPROMUInt8(CFG_ESAIDA);
  tempo  = getEEPROMUInt16(CFG_TEMPO);
/*
  Serial.println("\n--- Dados da EEPROM");
  Serial.print("Trava = "); Serial.println(trava);
  Serial.print("EstadoSaida = "); Serial.println(EstadoSaida);
  Serial.print("Tempo      = "); Serial.println(tempo);*/
}
//------------------------------------------- End Void (eeprom) ---------------------------------------------------
//------------------------------------------- Begin Void (Wifi) ---------------------------------------------------
void initWiFi(){ 
while (!Serial);

  delay(200);
/*
  Serial.print(F("\nStarting ConfigOnSwichFS using ")); Serial.print(FS_Name);
  Serial.print(F(" on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP_WIFIMANAGER_VERSION);

  if ( String(ESP_WIFIMANAGER_VERSION) < ESP_WIFIMANAGER_VERSION_MIN_TARGET )
  {
    Serial.print(F("Warning. Must use this example on Version equal or later than : "));
    Serial.println(ESP_WIFIMANAGER_VERSION_MIN_TARGET);
  }*/

  // Initialize the LED digital pin as an output.
  pinMode(PIN_LED, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  if (FORMAT_FILESYSTEM)
  {
    Serial.println(F("Forced Formatting."));
    FileFS.format();
  }
#ifdef ESP32
  if (!FileFS.begin(true))
#else
  if (!FileFS.begin())
#endif
  {
#ifdef ESP8266
    FileFS.format();
#endif

    Serial.println(F("SPIFFS/LittleFS failed! Already tried formatting."));
  
    if (!FileFS.begin())
    {     
      delay(100);
      
#if USE_LITTLEFS
      Serial.println(F("LittleFS failed!. Please use SPIFFS or EEPROM. Stay forever"));
#else
      Serial.println(F("SPIFFS failed!. Please use LittleFS or EEPROM. Stay forever"));
#endif

      while (true)
      {
        delay(1);
      }
    }
  }

  if (!readConfigFile())
  {
    Serial.println(F("Failed to read ConfigFile, using default values"));
  }

  unsigned long startedAt = millis();
  
  initAPIPConfigStruct(WM_AP_IPconfig);
  initSTAIPConfigStruct(WM_STA_IPconfig);

  ESP_WiFiManager ESP_wifiManager("ConfigOnSwichFS");

  ESP_wifiManager.setDebugOutput(true);

#if USE_CUSTOM_AP_IP

  ESP_wifiManager.setAPStaticIPConfig(WM_AP_IPconfig);
  
#endif

  ESP_wifiManager.setMinimumSignalQuality(-1);

  ESP_wifiManager.setConfigPortalChannel(0);

#if !USE_DHCP_IP    

    ESP_wifiManager.setSTAStaticIPConfig(WM_STA_IPconfig);

#endif

#if USING_CORS_FEATURE
  ESP_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");
#endif

  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();

  Serial.println("ESP Self-Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  ssid.toUpperCase();
  password = "MKF123";

  bool configDataLoaded = false;
  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass);
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
    
    ESP_wifiManager.setConfigPortalTimeout(120); 
    Serial.println(F("Got ESP Self-Stored Credentials. Timeout 120s for Config Portal"));
  }
  
  if (loadConfigData())
  {
    configDataLoaded = true;
    
    ESP_wifiManager.setConfigPortalTimeout(120);
    Serial.println(F("Got stored Credentials. Timeout 120s for Config Portal")); 

#if USE_ESP_WIFIMANAGER_NTP      
    if ( strlen(WM_config.TZ_Name) > 0 )
    {
      LOGERROR3(F("Current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

  #if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org"); 
  #else
      //configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
  #endif   
    }
    else
    {
      Serial.println(F("Current Timezone is not set. Enter Config Portal to set."));
    } 
#endif
  }
  else
  {
    // Enter CP only if no stored SSID on flash and file
    Serial.println(F("Open Config Portal without Timeout: No stored Credentials."));
    initialConfig = true;
  }

  if (initialConfig)
  {
    Serial.print(F("Starting configuration portal @ "));
    
#if USE_CUSTOM_AP_IP    
    Serial.print(APStaticIP);
#else
    Serial.print(F("192.168.4.1"));
#endif

    Serial.print(F(", SSID = "));
    Serial.print(ssid);
    Serial.print(F(", PWD = "));
    Serial.println(password);

    digitalWrite(PIN_LED, LED_ON); // Turn led on as we are in configuration mode.

    //sets timeout in seconds until configuration portal gets turned off.
    //If not specified device will remain in configuration mode until
    //switched off via webserver or device is restarted.
    //ESP_wifiManager.setConfigPortalTimeout(600);

    // Starts an access point
    if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password.c_str()))
      Serial.println(F("Not connected to WiFi but continuing anyway."));
    else
    {
      Serial.println(F("WiFi connected...yeey :)"));
    }

    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));
    
    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      String tempSSID = ESP_wifiManager.getSSID(i);
      String tempPW   = ESP_wifiManager.getPW(i);
  
      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);

      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);  

      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

#if USE_ESP_WIFIMANAGER_NTP      
    String tempTZ   = ESP_wifiManager.getTimezoneName();

    if (strlen(tempTZ.c_str()) < sizeof(WM_config.TZ_Name) - 1)
      strcpy(WM_config.TZ_Name, tempTZ.c_str());
    else
      strncpy(WM_config.TZ_Name, tempTZ.c_str(), sizeof(WM_config.TZ_Name) - 1);

    const char * TZ_Result = ESP_wifiManager.getTZ(WM_config.TZ_Name);
    
    if (strlen(TZ_Result) < sizeof(WM_config.TZ) - 1)
      strcpy(WM_config.TZ, TZ_Result);
    else
      strncpy(WM_config.TZ, TZ_Result, sizeof(WM_config.TZ_Name) - 1);
         
    if ( strlen(WM_config.TZ_Name) > 0 )
    {
      LOGERROR3(F("Saving current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

#if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org"); 
#else
      //configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
#endif
    }
    else
    {
      LOGERROR(F("Current Timezone Name is not set. Enter Config Portal to set."));
    }
#endif

    // New in v1.4.0
    ESP_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig);
    //////
    
    saveConfigData();
  }

  digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.

  startedAt = millis();

  if (!initialConfig)
  {
    // Load stored data, the addAP ready for MultiWiFi reconnection
    if (!configDataLoaded)
      loadConfigData();

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    if ( WiFi.status() != WL_CONNECTED ) 
    {
      Serial.println(F("ConnectMultiWiFi in setup"));
     
      connectMultiWiFi();
    }
  }

  Serial.print(F("After waiting "));
  Serial.print((float) (millis() - startedAt) / 1000);
  Serial.print(F(" secs more in setup(), connection result is "));

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print(F("connected. Local IP: "));
    Serial.println(WiFi.localIP());
  }
  else
    Serial.println(ESP_wifiManager.getStatus(WiFi.status()));
}

//------------------------------------------ End Void (Wifi) -----------------------------------------------------
//------------------------------------------ Begin Void (MQTT) ---------------------------------------------------
void initMQTT() 
{
    MQTT.setServer(BROKER_MQTT0, BROKER_PORT);
    MQTT.setCallback(mqtt_callback);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
    String msg;

    for(int i = 0; i < length; i++) 
    {
       char c = (char)payload[i];
       msg += c;
    }
       // Serial.println(msg);
    if (msg.equals("L"))
    {
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(4,LOW);
        EstadoSaida = 1;
        setEEPROMInt8(CFG_ESAIDA, EstadoSaida);
        trava = 5;
        setEEPROMInt8(CFG_TRAVA, trava);
        EEPROM.commit();
        g = 0;
        EnviaEstadoOutputMQTT();
    }else if (msg.equals("D"))
    {
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(4,HIGH);
        EstadoSaida = 0;
        setEEPROMInt8(CFG_ESAIDA, EstadoSaida);
        g = 0;
        trava = 0;
        setEEPROMInt8(CFG_TRAVA, trava);
        EEPROM.commit();
        EnviaEstadoOutputMQTT();
    }else if (msg.equals("R"))
    {
        EstadoSaida = 2;
        setEEPROMInt8(CFG_ESAIDA, EstadoSaida);
        g = 0;
        tempo = 0;
        setEEPROMInt8(CFG_TEMPO, tempo);
        EEPROM.commit();
        digitalWrite(4, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
        EnviaEstadoOutputMQTT();
    }else if (msg.equals("S28A"))
    {
        trava = 1;
        EstadoSaida = 4;
        setEEPROMInt8(CFG_ESAIDA, EstadoSaida);
        setEEPROMInt8(CFG_TRAVA, trava);
        EEPROM.commit();
        g = 0;
        EnviaEstadoOutputMQTT();
    }else if (msg.equals("S28C"))
    {
        trava = 4;
        EstadoSaida = 7;
        setEEPROMInt8(CFG_ESAIDA, EstadoSaida);
        setEEPROMInt8(CFG_TRAVA, trava);
        EEPROM.commit();
        g = 0;
        EnviaEstadoOutputMQTT();
    }else if (msg.equals("S183"))
    {
        trava = 2;
        EstadoSaida = 5;
        setEEPROMInt8(CFG_ESAIDA, EstadoSaida);
        setEEPROMInt8(CFG_TRAVA, trava);
        EEPROM.commit();
        g = 0;
        EnviaEstadoOutputMQTT();
    }else if (msg.equals("S365"))
    {
        trava = 3;
        EstadoSaida = 6;
        setEEPROMInt8(CFG_ESAIDA, EstadoSaida);
        setEEPROMInt8(CFG_TRAVA, trava);
        EEPROM.commit();
        g = 0;
        EnviaEstadoOutputMQTT();
    }else{
    //  Serial.println("Comando desconhecido");
      EstadoSaida = 3;
      setEEPROMInt8(CFG_ESAIDA, EstadoSaida);;
      EEPROM.commit();
      g = 0;
      EnviaEstadoOutputMQTT();
    }
}

void reconnectMQTT() 
{
    while (!MQTT.connected() && digitalRead(TRIGGER_PIN) == HIGH && WiFi.status() == WL_CONNECTED) 
    {
      //  Serial.print("* Tentando se conectar ao Broker MQTT: ");
        digitalWrite(LED_BUILTIN, LOW);

        if(c==0)
        {
          MQTT.setServer(BROKER_MQTT0, BROKER_PORT);
          //Serial.println(BROKER_MQTT0);
          //Serial.println(c);
          if (MQTT.connect(ID_MQTT)) 
          {
              //Serial.println("Conectado com sucesso ao broker MQTT!");
              digitalWrite(LED_BUILTIN, HIGH);
             /* Serial.println("tempo corrido");
              Serial.println(tempo);
              Serial.println("Trava Atual");
              Serial.println(trava);*/
              MQTT.subscribe(TOPICO_SUBSCRIBE); 
          } 
          else
          {
              digitalWrite(LED_BUILTIN, LOW);
             /* Serial.println("Falha ao reconectar no broker.");
              Serial.println("Havera nova tentatica de conexao em 2s");*/
              c++;
             // Serial.println(c);
              delay(250);
              digitalWrite(LED_BUILTIN, HIGH);
          }
         }else if (c==1)
         {
          MQTT.setServer(BROKER_MQTT1, BROKER_PORT);
          //Serial.println(BROKER_MQTT1);
          if (MQTT.connect(ID_MQTT)) 
          {
              //Serial.println("Conectado com sucesso ao broker MQTT!");
              digitalWrite(LED_BUILTIN, HIGH);
             /* Serial.println("tempo corrido");
              Serial.println(tempo);
              Serial.println("Trava Atual");
              Serial.println(trava);*/
              MQTT.subscribe(TOPICO_SUBSCRIBE); 
          } 
          else
          {
              digitalWrite(LED_BUILTIN, LOW);
             /* Serial.println("Falha ao reconectar no broker.");
              Serial.println("Havera nova tentatica de conexao em 2s");*/
              c++;
             // Serial.println(c);
              delay(250);
              digitalWrite(LED_BUILTIN, HIGH);
          }
          }else
          {
              MQTT.setServer(BROKER_MQTT2, BROKER_PORT);
             // Serial.println(BROKER_MQTT2);
              if (MQTT.connect(ID_MQTT)) 
          {
             // Serial.println("Conectado com sucesso ao broker MQTT!");
              digitalWrite(LED_BUILTIN, HIGH);
            /*  Serial.println("tempo corrido");
              Serial.println(tempo);
              Serial.println("Trava Atual");
              Serial.println(trava);*/
              MQTT.subscribe(TOPICO_SUBSCRIBE); 
          } 
              else
          {
              digitalWrite(LED_BUILTIN, LOW);
            /*  Serial.println("Falha ao reconectar no broker.");
              Serial.println("Havera nova tentatica de conexao em 2s");
              Serial.println(c);*/
              c=0;
              delay(250);
              digitalWrite(LED_BUILTIN, LOW);
          }
          }
    }
}
//------------------------------------------ End Void (MQTT) -----------------------------------------------------
//------------------------------------------ Begin Void (Verifica Conexão) ---------------------------------------
void VerificaConexoesWiFIEMQTT(void)
{
 // Serial.println("Passo 3 loop");
    if (!MQTT.connected())reconnectMQTT();
}

void verificaWifi()
{
//  Serial.println("Passo 4 loop");
       if ((digitalRead(TRIGGER_PIN) == LOW))
  {
    Serial.println(F("\nConfiguration portal requested."));
    digitalWrite(PIN_LED, LED_ON);
    ESP_WiFiManager ESP_wifiManager("ConfigOnSwitchFS");
    Serial.println(F("Opening configuration portal. "));
    Router_SSID = ESP_wifiManager.WiFi_SSID();
    Router_Pass = ESP_wifiManager.WiFi_Pass();
    Serial.println("ESP Self-Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);
    
    if ( (Router_SSID != "") && (Router_Pass != "") )
    {
      LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass);
      wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
      
      ESP_wifiManager.setConfigPortalTimeout(120);
      Serial.println(F("Got ESP Self-Stored Credentials. Timeout 120s for Config Portal"));
    }
    else if (loadConfigData())
    {      
      ESP_wifiManager.setConfigPortalTimeout(120);
      Serial.println(F("Got stored Credentials. Timeout 120s for Config Portal")); 
    }
    else
    {

      Serial.println(F("Open Config Portal without Timeout: No stored Credentials."));
      initialConfig = true;
    }

    ESP_wifiManager.setMinimumSignalQuality(-1);
    ESP_wifiManager.setConfigPortalChannel(0);

#if USE_CUSTOM_AP_IP
  //set custom ip for portal
  // New in v1.4.0
  ESP_wifiManager.setAPStaticIPConfig(WM_AP_IPconfig);
  //////
#endif
    
#if !USE_DHCP_IP    
  #if USE_CONFIGURABLE_DNS  
    ESP_wifiManager.setSTAStaticIPConfig(stationIP, gatewayIP, netMask, dns1IP, dns2IP);  
  #else
    ESP_wifiManager.setSTAStaticIPConfig(stationIP, gatewayIP, netMask);
  #endif 
#endif  

#if USING_CORS_FEATURE
  ESP_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");
#endif

    if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password.c_str()))
    {
      Serial.println(F("Not connected to WiFi but continuing anyway."));
    }
    else
    {
      //if you get here you have connected to the WiFi
      Serial.println(F("connected...yeey :)"));
      Serial.print(F("Local IP: "));
      Serial.println(WiFi.localIP());
    }

    if ( String(ESP_wifiManager.getSSID(0)) != "" && String(ESP_wifiManager.getSSID(1)) != "" )
    {
      // Stored  for later usage, from v1.1.0, but clear first
      memset(&WM_config, 0, sizeof(WM_config));
      
      for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
      {
        String tempSSID = ESP_wifiManager.getSSID(i);
        String tempPW   = ESP_wifiManager.getPW(i);
    
        if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
          strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
        else
          strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);
    
        if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
          strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
        else
          strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);  
    
        // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
        if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
        {
          LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
          wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
        }
      }

    #if USE_ESP_WIFIMANAGER_NTP      
      String tempTZ   = ESP_wifiManager.getTimezoneName();
  
      if (strlen(tempTZ.c_str()) < sizeof(WM_config.TZ_Name) - 1)
        strcpy(WM_config.TZ_Name, tempTZ.c_str());
      else
        strncpy(WM_config.TZ_Name, tempTZ.c_str(), sizeof(WM_config.TZ_Name) - 1);
  
      const char * TZ_Result = ESP_wifiManager.getTZ(WM_config.TZ_Name);
      
      if (strlen(TZ_Result) < sizeof(WM_config.TZ) - 1)
        strcpy(WM_config.TZ, TZ_Result);
      else
        strncpy(WM_config.TZ, TZ_Result, sizeof(WM_config.TZ_Name) - 1);
           
      if ( strlen(WM_config.TZ_Name) > 0 )
      {
        LOGERROR3(F("Saving current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

  #if ESP8266
       configTime(WM_config.TZ, "pool.ntp.org"); 
  #else
        configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
  #endif
      }
      else
      {
        LOGERROR(F("Current Timezone Name is not set. Enter Config Portal to set."));
      }
#endif
      ESP_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig);
      saveConfigData();
    }
    writeConfigFile();
    digitalWrite(PIN_LED, LED_OFF); // Turn LED off as we are not in configuration mode.
  }
  check_status();
}
//------------------------------------------ End Void (Verifica Conexão) -----------------------------------------
//------------------------------------------ Begin Void (Verifica Status) ----------------------------------------
void EnviaEstadoOutputMQTT(void)
{

  for(g = 0; g < 5; g++){
    if (EstadoSaida == 0)
      MQTT.publish(TOPICO_PUBLISH, "Desligada a trava");
 
    if (EstadoSaida == 1)
      MQTT.publish(TOPICO_PUBLISH, "Ligada a trava");

    if (EstadoSaida == 3)
      MQTT.publish(TOPICO_PUBLISH, "Comando desconhecido");

    if (EstadoSaida == 2)
      MQTT.publish(TOPICO_PUBLISH, "Tempo resetado");

    if (EstadoSaida == 4)
      MQTT.publish(TOPICO_PUBLISH, "Tempo setado para 28 dias (ALUGUEL)");

    if (EstadoSaida == 7)
      MQTT.publish(TOPICO_PUBLISH, "Tempo setado para 28 dias (COMPLETO)");

    if (EstadoSaida == 5)
      MQTT.publish(TOPICO_PUBLISH, "Tempo setado para 183 dias");

    if (EstadoSaida == 6)
      MQTT.publish(TOPICO_PUBLISH, "Tempo setado para 365 dias");
    delay(500);
  }/*
    Serial.println("- Estado da saida D0 enviado ao broker!");
    Serial.print("tempo corrido = ");Serial.println(tempo);
    Serial.print("Trava Atual = ");Serial.println(trava);
    Serial.print("EstadoSaida = "); Serial.println(EstadoSaida);*/
}
//------------------------------------------ End Void (Verifica Conexão) ------------------------------------------
//------------------------------------------ Begin Void (programa principal) --------------------------------------
void loop() 
{   
    initMillisTarefa();
    initTrava();
    VerificaConexoesWiFIEMQTT();
    verificaWifi();
    //EnviaEstadoOutputMQTT();
    MQTT.loop();
}
//------------------------------------------ End Void (programa principal) ----------------------------------------
