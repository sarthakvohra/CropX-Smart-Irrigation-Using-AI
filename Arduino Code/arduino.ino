#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <FirebaseESP8266.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <WiFiClientSecure.h>
#include "DHTesp.h"

WiFiClientSecure wifiClient;
DHTesp dht;
uint8_t motorPin = D2;

/* 1. Define the WiFi credentials */
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

// For the following credentials, see examples/Authentications/SignInAsUser/EmailPassword/EmailPassword.ino

/* 2. Define the API Key */
#define API_KEY ""

/* 3. Define the RTDB URL */
#define DATABASE_URL "" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app

/* 4. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL ""
#define USER_PASSWORD ""

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

// Variables
int count;
float sensorMoisture;
float h;
float t;
float moisture_percentage;
int sensor_analog;
float ET0;
float KCET0;
float Kc;
float PrecipProb;
float G;
float SumY;
float tempMin;
float tempMax;
float Pres;
float rh;
float solarRad;
float windSpd;
float dni;
float modeOfOperation;

////////

void setup()
{

  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  // Or use legacy authenticate method
  // config.database_url = DATABASE_URL;
  // config.signer.tokens.legacy_token = "<database secret>";

  // To connect without auth in Test Mode, see Authentications/TestMode/TestMode.ino

  //////////////////////////////////////////////////////////////////////////////////////////////
  // Please make sure the device free Heap is not lower than 80 k for ESP32 and 10 k for ESP8266,
  // otherwise the SSL connection will fail.
  //////////////////////////////////////////////////////////////////////////////////////////////

  Firebase.begin(&config, &auth);

  // Comment or pass false value when WiFi reconnection will control by your code or third party library
  Firebase.reconnectWiFi(true);

  Firebase.setDoubleDigits(5);  //////////////////////////////////////////////////////////////////////////////////////////////commentes due to error
  dht.setup(13, DHTesp::DHT11); // GPIO14 or D5, GPIO13 or D7
  /** Timeout options.

  //WiFi reconnect timeout (interval) in ms (10 sec - 5 min) when WiFi disconnected.
  config.timeout.wifiReconnect = 10 * 1000;

  //Socket connection and SSL handshake timeout in ms (1 sec - 1 min).
  config.timeout.socketConnection = 10 * 1000;

  //Server response read timeout in ms (1 sec - 1 min).
  config.timeout.serverResponse = 10 * 1000;

  //RTDB Stream keep-alive timeout in ms (20 sec - 2 min) when no server's keep-alive event data received.
  config.timeout.rtdbKeepAlive = 45 * 1000;

  //RTDB Stream reconnect timeout (interval) in ms (1 sec - 1 min) when RTDB Stream closed and want to resume.
  config.timeout.rtdbStreamReconnect = 1 * 1000;

  //RTDB Stream error notification timeout (interval) in ms (3 sec - 30 sec). It determines how often the readStream
  //will return false (error) when it called repeatedly in loop.
  config.timeout.rtdbStreamError = 3 * 1000;

  Note:
  The function that starting the new TCP session i.e. first time server connection or previous session was closed, the function won't exit until the
  time of config.timeout.socketConnection.

  You can also set the TCP data sending retry with
  config.tcp_data_sending_retry = 1;

  */
}

void loop()
{
  if (count % 20 == 0)
  {
    if (Firebase.ready())
      Serial.println("DOne");

    //=========================================IRRISTAT API============================//
    HTTPClient httpAgro; // Declare an object of class HTTPClient

    //  httpAgro.begin(wifiClient, "");//("");  //Specify request destination
    httpAgro.begin(wifiClient, "");
    //  std::begin(httpAgro, "");
    int httpAPICode = httpAgro.GET(); // Send the request

    if (httpAPICode > 0)
    { // Check the returning code

      String payload = httpAgro.getString(); // Get the request response payload

      int ind = payload.indexOf('}');
      ///////////////////JSON PARSINNG///////////////////////////////
      Serial.println("\n");
      Serial.println(payload.substring(0, ind + 1)); // Print the response payload
      String final = payload.substring(0, ind + 1) + "]}";
      DynamicJsonDocument jsonBuffer(907);

      char json[final.length() + 1];
      final.toCharArray(json, final.length() + 1);

      //  JsonObject root = deserializeJson(jsonBuffer, json);
      auto error = deserializeJson(jsonBuffer, json, final.length() + 1);
      if (error)
      {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(error.c_str());
        return;
      }
      Serial.println("aaabbbccc");
      JsonObject root = jsonBuffer["data"][0];
      //  Serial.println("Printing JSON: ");
      //  root.printTo(Serial);
      //  Serial.println(final);
      //  serializeJson(root, Serial);
      //  JsonObject root = jsonBuffer.parseObject(json);
      // Test if parsing succeeds.
      //  if (!root.success()) {
      //    Serial.println("parseObject() failed");
      //    return;
      //  }
      ET0 = root["evapotranspiration"];
      Serial.print(ET0);
      //  Serial.print(root["data"][0]["v_soilm_40_100cm"]);

      //  Kc = Firebase.getFloat(fbdo, "/Kc");
      if (fbdo, F("/Kc"))
      {
        Kc = fbdo.to<float>();
      }
      else
      {
        Kc = 0;
        Serial.println(fbdo.errorReason().c_str());
      }
      KCET0 = ET0 * Kc;
      Firebase.setFloat(fbdo, "/KCET0", KCET0);
      PrecipProb = root["precip"];
      G = root["soilt_0_10cm"];
      Firebase.pushFloat(fbdo, "/ET0", ET0);
      Firebase.pushFloat(fbdo, "/GSoilFlux", G);
      Firebase.setFloat(fbdo, "/CurrentET0", ET0);
      SumY = Firebase.getFloat(fbdo, "/SumY");
      SumY = SumY + ET0;
      Firebase.setFloat(fbdo, "/SumY", SumY);
      Firebase.setFloat(fbdo, "/PrecipProbability", PrecipProb);
      // Print values.
      // Serial.println(ET0);
    }
    httpAgro.end(); // Close connection

    //====================================OpenWeatherAPI==========================================//

    HTTPClient http; // Declare an object of class HTTPClient

    //  http.begin(wifiClient, "");//Specify request destination
    http.begin(wifiClient, ""); // Specify request destination
    int httpCode = http.GET();  // Send the request

    if (httpCode > 0)
    { // Check the returning code

      String openW = http.getString(); // Get the request response payload
      Serial.println(openW);
      Serial.println("1");

      ///////////////////JSON PARSINNG///////////////////////////////
      //  DynamicJsonDocument jsonBufferO(782);
      DynamicJsonDocument jsonBufferO(907);
      Serial.println("2");
      char jsonO[openW.length() + 1];
      Serial.println("3");
      openW.toCharArray(jsonO, openW.length() + 1);
      Serial.println("4");

      auto error1 = deserializeJson(jsonBufferO, (const char *)jsonO);
      Serial.println("5");
      if (error1)
      {
        Serial.println("6");
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(error1.c_str());
        return;
      }
      Serial.println("7");
      JsonObject rootWeather = jsonBufferO["weather"][0];
      Serial.println("8");
      //  JsonObject& rootO = jsonBufferO.parseObject(jsonO);
      //
      //  // Test if parsing succeeds.
      //  if (!rootO.success()) {
      //    Serial.println("parseObject() failed");
      //    return;
      //  }
      const char *desc = rootWeather["description"];
      Serial.println(desc);
      //    bool result = Firebase.getString(fbdo, "/CurrentWeather");
      //    Firebase.setStringAsync(fbdo, "/CurrentWeather", desc); ///Just to check
      //    yield();
      //  if(Firebase.setString(fbdo, "/CurrentWeather", desc)){
      //   Serial.println("Success!"); }
      //   else{
      //      Serial.print("setting /message failed:");
      //      Serial.println(fbdo.errorReason());
      //      return;
      //  }
      Serial.println("10");

      JsonObject rootMain = jsonBufferO["main"];
      tempMin = (rootMain["temp_min"]);
      tempMax = (rootMain["temp_max"]);
      tempMin = tempMin - 273;
      tempMax = tempMax - 273;
      Serial.println("abbbbbbbbbbbbbbbbbb");

      Firebase.pushFloat(fbdo, "/dataTempMin", tempMin);
      Firebase.pushFloat(fbdo, "/dataTempMax", tempMax);
      Firebase.setFloat(fbdo, "/DisplayTempMin", tempMin);
      Firebase.setFloat(fbdo, "/DisplayTempMax", tempMax);
      Serial.println("11");

      float disPres = rootMain["pressure"];
      float disHum = rootMain["humidity"];
      Serial.println("12");
      Firebase.setFloat(fbdo, "/DisplayPressure", disPres);
      Firebase.setFloat(fbdo, "/DisplayHumidity", disHum);
      Serial.println("13");
    }

    http.end(); // Close connection

    //=======================================WEATHERBIT API===================================================================//

    HTTPClient httpWeatherBit; // Declare an object of class HTTPClient

    // httpWeatherBit.begin("");  //Specify request
    // httpWeatherBit.begin(wifiClient, "");  //Specify request destination
    httpWeatherBit.begin(wifiClient, "");   // Specify request destination
    int httpBitCode = httpWeatherBit.GET(); // Send the request

    if (httpBitCode > 0)
    { // Check the returning code

      String WeatherBit = httpWeatherBit.getString(); // Get the request response payload
      Serial.println(WeatherBit);

      ///////////////////JSON PARSINNG///////////////////////////////
      DynamicJsonDocument jsonBufferW(782);

      char jsonW[WeatherBit.length() + 1];
      WeatherBit.toCharArray(jsonW, WeatherBit.length() + 1);

      //  JsonObject& rootW = jsonBufferW.parseObject(jsonW);
      //
      //  // Test if parsing succeeds.
      //  if (!rootW.success()) {
      //    Serial.println("parseObject() failed");
      //    return;
      //  }

      auto error = deserializeJson(jsonBufferW, jsonW);
      if (error)
      {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(error.c_str());
        return;
      }
      JsonObject rootW = jsonBufferW["data"][0];
      Pres = rootW["pres"];
      rh = rootW["rh"];
      solarRad = rootW["solar_rad"];
      windSpd = rootW["wind_spd"];
      dni = rootW["dni"];
      //  Pres = rootW["data"][0]["pres"];
      //  rh = rootW["data"][0]["rh"];
      //  solarRad = rootW["data"][0]["solar_rad"];
      //  windSpd = rootW["data"][0]["wind_spd"];
      //  dni = rootW["data"][0]["dni"];

      Firebase.pushFloat(fbdo, "/PressureCalc", Pres);
      Firebase.pushFloat(fbdo, "/RelativeHumidity", rh);
      Firebase.pushFloat(fbdo, "/SolarRad", solarRad);
      Firebase.pushFloat(fbdo, "/WindSpeed", windSpd);
      Firebase.setFloat(fbdo, "/DisplayWindSpeed", windSpd);
      Firebase.pushFloat(fbdo, "/DirectNSI", dni);
    }

    httpWeatherBit.end(); // Close connection

    //====================PLSR ALGO===================================//

    float tempMean = (tempMin + tempMax) / 2;
    float p = (17.27 * tempMean) / (tempMean + 273.3);
    float del = (4098 * (0.6108 * pow(2.71828, p))) / pow((tempMean + 237.3), 2);
    float gama = 0.000665 * Pres;
    float Rn = solarRad;
    float u2 = windSpd;
    float p1 = (17.27 * tempMax) / (tempMax + 273.3);
    float p2 = (17.27 * tempMin) / (tempMin + 273.3);
    float eTmax = 0.6108 * (pow(2.71828, p1));
    float eTmin = 0.6108 * (pow(2.71828, p2));
    float es = (eTmax + eTmin) / 2;
    float ea = (eTmin) * (rh / 100);

    float X1 = (del * (Rn - G)) / (del + (gama * (1 + 0.34 * u2)));
    float X2 = (gama * u2 * (es - ea)) / ((tempMean + 273) * (del + (gama * (1 + 0.34 * u2))));
    Firebase.pushFloat(fbdo, "/X1", X1);
    Firebase.pushFloat(fbdo, "/X2", X2);
    float Y = ET0;

    float X1Y = X1 * Y;
    float X2square = X2 * X2;
    float X1X2 = X1 * X2;
    float X2Y = X2 * Y;
    float X1square = X1 * X1;

    float SumX1 = Firebase.getFloat(fbdo, "/SumX1");
    SumX1 = SumX1 + X1;
    float SumX2 = Firebase.getFloat(fbdo, "/SumX2");
    SumX2 = SumX2 + X2;
    float SumX1Y = Firebase.getFloat(fbdo, "/SumX1Y");
    SumX1Y = SumX1Y + X1Y;
    float SumX2square = Firebase.getFloat(fbdo, "/SumX2square");
    SumX2square = SumX2square + X2square;
    float SumX1X2 = Firebase.getFloat(fbdo, "/SumX1X2");
    SumX1X2 = SumX1X2 + X1X2;
    float SumX2Y = Firebase.getFloat(fbdo, "/SumX2Y");
    SumX2Y = SumX2Y + X2Y;
    float SumX1square = Firebase.getFloat(fbdo, "/SumX1square");
    SumX1square = SumX1square + X1square;

    Firebase.setFloat(fbdo, "/SumX1", SumX1);
    Firebase.setFloat(fbdo, "/SumX2", SumX2);
    Firebase.setFloat(fbdo, "/SumX1Y", SumX1Y);
    Firebase.setFloat(fbdo, "/SumX2square", SumX2square);
    Firebase.setFloat(fbdo, "/SumX1X2", SumX1X2);
    Firebase.setFloat(fbdo, "/SumX2Y", SumX2Y);
    Firebase.setFloat(fbdo, "/SumX1square", SumX1square);

    int c = Firebase.getInt(fbdo, "/count");

    float b1 = ((SumX2square * SumX1Y) - (SumX1X2 * SumX2Y)) / ((SumX1square * SumX2square) - pow(SumX1X2, 2));
    float b2 = ((SumX1square * SumX2Y) - (SumX1X2 * SumX1Y)) / ((SumX1square * SumX2square) - pow(SumX1X2, 2));
    float b0 = (SumY / c) - (b1 * (SumX1 / c)) - (b2 * (SumX2 / c));

    float YFinal = b0 + (b1 * X1) + (b2 * X2);
    float ET0F = (((0.408 * X1) + (900 * X2)) * 2) / 10;

    Firebase.pushFloat(fbdo, "/ET0F", ET0F);
    Firebase.pushFloat(fbdo, "/b1", b1);
    Firebase.pushFloat(fbdo, "/b2", b2);
    Firebase.pushFloat(fbdo, "/b0", b0);
  }

  //===================================SENSOR VALUES TO CLOUD===================================//
  if (count % 5 == 0)
  {
    // DHT11 data
    h = dht.getHumidity();
    t = dht.getTemperature();
    Serial.print("Status: ");
    Serial.print(dht.getStatusString());
    Serial.print("\t");
    Serial.print("Humidity (%): ");
    Serial.print(h, 1);
    Serial.print("\t");
    Serial.print("Temperature (C): ");
    Serial.print(t, 1);
    Serial.print("\t");
    Serial.println();
    Firebase.pushFloat(fbdo, "/SensorTemperature", t);
    Firebase.pushFloat(fbdo, "/SensorHumidity", h);

    // Soil Moisture Sensor Data
    sensor_analog = analogRead(A0);
    Serial.println(sensor_analog);
    int map_high = 375;
    int map_low = 1024;
    moisture_percentage = map(sensor_analog, map_low, map_high, 0, 100);
    Serial.println(moisture_percentage);
    Firebase.pushFloat(fbdo, "/SensorMoistureSoil", moisture_percentage);
  }

  // Motor
  Serial.printf("Get float... %s\n", Firebase.getFloat(fbdo, F("/mode")) ? String(fbdo.to<float>()).c_str() : fbdo.errorReason().c_str());
  if (fbdo, F("/mode"))
  {
    modeOfOperation = fbdo.to<float>();
  }
  else
  {
    modeOfOperation = 0;
    Serial.println(fbdo.errorReason().c_str());
  }
  Serial.print("Mode of operation: ");
  Serial.println(modeOfOperation);
  if (modeOfOperation == 2)
  {
    Serial.println("Boost mode has been activated!!!");
    analogWrite(motorPin, 255);
  }

  else if (count % 10 == 0)
  {
    sensorMoisture = ((float)(moisture_percentage) / 10);

    if (sensorMoisture < 6 && sensorMoisture < KCET0 && PrecipProb < KCET0)
    {
      Serial.println("\MotorON");
      analogWrite(motorPin, 255);
      delay(2000);
    }
    while (sensorMoisture < 7 && sensorMoisture < KCET0 && PrecipProb < KCET0)
    {
      sensor_analog = analogRead(A0);
      //  moisture_percentage = ( 100 - ( ((sensor_analog-300)/724.00) * 100 ) );
      sensorMoisture = ((float)(moisture_percentage) / 10);
      Serial.println("Updated Moisture:");
      Serial.println(sensorMoisture);
      delay(5000);
    }
    analogWrite(motorPin, 0);
    Serial.println("\MOTOR OFF");
  }
  count++;
  Serial.println(count);
  delay(1000);
  //  if(Firebase.ready())
  //  Serial.println("DOne");
  //  delay(1000);
}
