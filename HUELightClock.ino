/**
 * Emulate Philips Hue Bridge
 **/
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <aJSON.h> // Replace avm/pgmspace.h with pgmspace.h there and set #define PRINT_BUFFER_LEN 4096 ################# IMPORTANT
#include "params.h"
#include "secrets.h"
#include <LightService.h>

// librairies
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <virtuabotixRTC.h>

#define NBELEMS(x)  (sizeof(x) / sizeof((x)[0]))

// Definition of PINs
#define PIN_LUM A0
#define PIN_BTN D8

enum ProgramState {
  STATE_INIT,
  STATE_CONNECT,
  STATE_STANDBY,
  STATE_MENU_DATE,
  STATE_MENU_DAY,
  STATE_MENU_LUM
};
ProgramState state = STATE_INIT;


// LED matrix backpack
Adafruit_8x8matrix m[] = {
  Adafruit_8x8matrix(),
  Adafruit_8x8matrix(),
  Adafruit_8x8matrix(),
  Adafruit_8x8matrix()
};

// RTC module
// SCLK, I/O, CE
virtuabotixRTC myRTC(D0,D3,D4);

// PIN Leds (Relay)
const int PIN_LED[] = {D5,D6,D7};

class LedLight : public Light {
  public:
    LedLight(char* n, LightType t) : Light(n,t) {}
    
    void handleQuery(HueLightInfo info) {
      this->info = info;
      this->infosToVar();
      this->update();
    }

    void nextLed() {
      if (++led > NBELEMS(PIN_LED))
        led = 0;
      this->varToInfos();
      this->update();
    }

    /**
     * Turn on/off leds
     */
    void update() {
      uint8_t i;
      for(i = 0; i < this->led; ++i)
        digitalWrite(PIN_LED[i], !HIGH);
      for(i; i < NBELEMS(PIN_LED); ++i)
        digitalWrite(PIN_LED[i], !LOW);
    }

  private:
    uint8_t led = 0;
    const int led_bri[4] = {0,84,169,254};

    void infosToVar() {
      if (this->info.on)
        if (this->info.brightness > this->led_bri[2])
          this->led = 3;
        else if (this->info.brightness > this->led_bri[1])
          this->led = 2;
        else
          this->led = 1;
      else
        this->led = 0;
    }

    void varToInfos() {
      this->info.brightness = this->led_bri[led];
      this->info.on = this->led > 0;
    }
};

LedLight myLight("Vase LED",DIMMABLE_LIGHT);

void setup() {
  Serial.begin(9600);

  // Connect I2C matrix and print 'W'
  initMatrix(&m[0],0x70);
  initMatrix(&m[1],0x71);
  initMatrix(&m[2],0x74);
  initMatrix(&m[3],0x75);

  // LED Lights
  pinMode(PIN_LED[0], OUTPUT);
  pinMode(PIN_LED[1], OUTPUT);
  pinMode(PIN_LED[2], OUTPUT);
  myLight.update();

  // Button
  pinMode(PIN_BTN, INPUT);
  
  LightService.addLight(&myLight);
  LightGroup *group = LightService.getGroup();
  group->setName("Reveil");
  group->setType(ROOM);
  group->setClass(BEDROOM);
  group->addLight(&myLight);

  delay(1000);
}


const long  DELAY_LONG_PRESS = 1000,//ms
            DELAY_MENU = 5000,//ms
            DELAY_BRIGHTNESS = 1200,//ms
            TIMEOUT_RECONNECT = 10000,//ms
            DELAY_RECONNECT = 300000;//ms
            
boolean btn, short_press, long_press, first = true;// is button pressed ?
unsigned long last_press,// last millis the button was pressed
              last_autobrightness = -DELAY_BRIGHTNESS,
              last_reconnect;
boolean auto_brightness = true;
uint8_t brightness = 16, calc_bri;// 0 to 15
unsigned int lum;// luminosity (sensor)

const char  ERR_NOTASTATE = '1',
            ERR_NOTADAY = '2';

//String utc_time = "";

void loop() {
  if ((WiFi.status() == WL_CONNECTED))
    LightService.update();
  else if (millis() - last_reconnect > DELAY_RECONNECT)
    setState(STATE_CONNECT);
    
  readValues();

  switch(state) {
    case STATE_STANDBY:
      writeTime();
      if (short_press) myLight.nextLed();
      else if (long_press) setState(STATE_MENU_DATE);
      // TODO
      // If not connected to Wifi : try to connect (not each time)
      break;
    case STATE_MENU_DATE:
      writeDate();
      if (short_press) setState(STATE_MENU_DAY);//setState(STATE_MENU_DAY);
      else if (long_press) setState(STATE_MENU_LUM);
      else delayMenu();
      break;
    case STATE_MENU_DAY:
      writeDay();
      if (short_press) setState(STATE_MENU_DATE);
      else if (long_press) setState(STATE_MENU_LUM);
      else delayMenu();
      break;
    case STATE_MENU_LUM:
      if (auto_brightness) writeChars('A','U','T','O');
      else writeChars(
        'L',
        'M',
        getFirstDec(brightness) + '0',
        getSecDec(brightness) + '0'
      );
      if (short_press) {
        setBrightness(auto_brightness?0:(brightness+1));
        auto_brightness = (brightness == 16);
      }
      else if (long_press) setState(STATE_STANDBY);
      else delayMenu();
      break;
    case STATE_CONNECT:
      connectHUE();
      setState(STATE_STANDBY);
      break;
    case STATE_INIT:
      setState(STATE_CONNECT);
      break;
    default:
      writeError(ERR_NOTASTATE);
  }
  
  apply();
  yield();
}

/**
 * grab information from sensors and calculate logical informations
 */
void readValues() {
  btn = digitalRead(PIN_BTN);
  lum += analogRead(PIN_LUM);
  lum /= 2;
  delay(50);
  myRTC.updateTime();
  // check logical value of our button
  long_press = false; short_press = false;
  if (btn) {
    if (first) {
      last_press = millis();
      first = false;
    }
  } else {
    if (!first) {
      if ( (millis()-last_press) > DELAY_LONG_PRESS ) {
        long_press = true;
      } else {
        short_press = true;
      }
      first = true;
      last_press = millis();
    }
  }
}

void apply() {
  // adjust brightness
  if (auto_brightness && (millis() - last_autobrightness) >= DELAY_BRIGHTNESS) {
    uint8_t cb = calculateBrightness();
    // We only change brightness after two similar results (its more stable)
    if (calc_bri == cb)
      setBrightness(calc_bri);
    else
      calc_bri = cb;
    last_autobrightness = millis();
  }
}

static const uint8_t PROGMEM
  smile_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10100101,
    B10011001,
    B01000010,
    B00111100 };

/**
 * @param *matrix reference to the matrix that will be initialized 
 * @param addr I2C address of the matrix
 * Connect and configure the matrix 
 */
void initMatrix(Adafruit_8x8matrix *matrix, byte addr) {
  matrix->begin(addr);
  matrix->clear();
  matrix->setTextSize(1);
  matrix->setTextWrap(false);
  matrix->setTextColor(LED_ON);
  matrix->setRotation(1);
  matrix->drawBitmap(0,0,smile_bmp,8,8,LED_ON);
  matrix->writeDisplay();
}

/**
 * @param Brightness from 0 (auto) , 1 (min) to 15 (max)
 */
void setBrightness(uint8_t b) {
  if (brightness != b) {
    while ( b > 16 ) b -= 16;       
    for(int i = 0; i < NBELEMS(m); ++i)
      m[i].setBrightness(b);
    brightness = b;
  }
}

/**
 * Calculate appropriates brightness according on ambient light
 */
uint8_t calculateBrightness() {
  float f = ( -0.91 + 0.17 * exp( (float)lum / 221.0 ) ) + 0.5;
  uint8_t fint = (uint8_t)( f + 0.5 );
  if (fint < 0)
    fint = 0;
  return fint;
}

void delayMenu() {
  if ( (millis() - last_press) > DELAY_MENU )
    setState(STATE_STANDBY);
}

/**
 * @param c Char to write on matrix
 * @param matrix Matrix on which to write the char
 */
void writeOnMatrix(Adafruit_8x8matrix *matrix, char *c, boolean now) {
  matrix->clear();
  matrix->setCursor(1,1);
  matrix->print(*c);
  if (now)
    matrix->writeDisplay();
}

void writeOnMatrix(Adafruit_8x8matrix *matrix, char *c) {
  writeOnMatrix(matrix,c,true);
}

void writeChars(char a, char b, char c, char d, boolean now) {
  // Write matrix
  writeOnMatrix(&m[0],&a,now);
  writeOnMatrix(&m[1],&b,now);
  writeOnMatrix(&m[2],&c,now);
  writeOnMatrix(&m[3],&d,now);
}

void writeChars(char a, char b, char c, char d) {
  writeChars(a,b,c,d,true);
}

void writeError(const char code) {
  writeChars('E','R','R',code);
}


static const uint8_t PROGMEM
  numbers[][8] = {
    { 
    B01111100,
    B01111100,
    B01101100,
    B01101100,
    B01101100,
    B01101100,
    B01111100,
    B01111100
    },
    { 
    B00001100,
    B00001100,
    B00001100,
    B00001100,
    B00001100,
    B00001100,
    B00001100,
    B00001100
    },
    { 
    B01111100,
    B01111100,
    B00001100,
    B01111100,
    B01111100,
    B01100000,
    B01111100,
    B01111100
    },
    { 
    B01111100,
    B01111100,
    B00001100,
    B01111100,
    B01111100,
    B00001100,
    B01111100,
    B01111100
    },
    { 
    B01101100,
    B01101100,
    B01101100,
    B01111100,
    B01111100,
    B00001100,
    B00001100,
    B00001100
    },
    {
    B01111100,
    B01111100,
    B01100000,
    B01111100,
    B01111100,
    B00001100,
    B01111100,
    B01111100
    },
    { 
    B01111100,
    B01111100,
    B01100000,
    B01111100,
    B01111100,
    B01101100,
    B01111100,
    B01111100
    },
    { 
    B01111100,
    B01111100,
    B00001100,
    B00001100,
    B00001100,
    B00001100,
    B00001100,
    B00001100
    },
    { 
    B01111100,
    B01111100,
    B01101100,
    B01111100,
    B01111100,
    B01101100,
    B01111100,
    B01111100
    },
    { 
    B01111100,
    B01111100,
    B01101100,
    B01111100,
    B01111100,
    B00001100,
    B01111100,
    B01111100
    }    
  };
void writeTime() {
  // write numbers
  /*writeChars(
    (myRTC.hours > 9) ? (getFirstDec(myRTC.hours) + '0') : ' ',
    getSecDec(myRTC.hours) + '0',
    getFirstDec(myRTC.minutes) + '0',
    getSecDec(myRTC.minutes) + '0',
    false
  );*/
  for(int i = 0; i < NBELEMS(m); ++i) {
    m[i].clear();
  }
  
  if (myRTC.hours > 9) m[0].drawBitmap(0,0,numbers[getFirstDec(myRTC.hours)],8,8,LED_ON);
  m[1].drawBitmap(0,0,numbers[getSecDec(myRTC.hours)],8,8,LED_ON);
  m[2].drawBitmap(0,0,numbers[getFirstDec(myRTC.minutes)],8,8,LED_ON);
  m[3].drawBitmap(0,0,numbers[getSecDec(myRTC.minutes)],8,8,LED_ON);

  m[1].drawPixel(7, 2, LED_ON);
  m[1].drawPixel(7, 5, LED_ON);
  
  // write displays
  for(int i = 0; i < NBELEMS(m); ++i)
    m[i].writeDisplay();
}

void writeDate() {
  // write numbers
  writeChars(
    getFirstDec(myRTC.dayofmonth) + '0',
    getSecDec(myRTC.dayofmonth) + '0',
    getFirstDec(myRTC.month) + '0',
    getSecDec(myRTC.month) + '0',
    false
  );
  // draw separation
  m[1].drawPixel(7, 7, LED_ON);
  // write displays
  for(int i = 0; i < NBELEMS(m); ++i)
    m[i].writeDisplay();
}

void writeDay() {
  switch(myRTC.dayofweek) {
    case 0:
      writeChars('D','I','M','.');
      break;
    case 1:
      writeChars('L','U','N','.');
      break;
    case 2:
      writeChars('M','A','R','.');
      break;
    case 3:
      writeChars('M','E','R','.');
      break;
    case 4:
      writeChars('J','E','U','.');
      break;
    case 5:
      writeChars('V','E','N','.');
      break;
    case 6:
      writeChars('S','A','M','.');
      break;
    default:
      writeError(ERR_NOTADAY);
  }
}

void setState(ProgramState STATE) {
  state = STATE;
}

int getFirstDec(int dec) {
  return (int)(dec/10);
}

int getSecDec(int dec) {
  return (dec-((int)(dec/10)*10));
}

void connectHUE() {
  writeChars('W','i','F','i');
  last_reconnect = millis();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.println("Connecting");
  while ( (millis() - last_reconnect) < TIMEOUT_RECONNECT && WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wifi connected");
  
    connectTime();
    
    LightService.begin();
  }

  last_reconnect = millis();
}


void connectTime() {
  const char* host = "projetw.xyz";
  const char* api_url = "/api_test/";
  const unsigned int httpPort = 80;
  const unsigned long HTTP_TIMEOUT = 10000;  // max respone time from server
  WiFiClient client;

  Serial.print("Connecting to ");
  Serial.println(host);
  
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  
  Serial.print("Requesting URL: ");
  Serial.println(api_url);

  client.print("GET ");
  client.print(api_url);
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(host);
  client.println("Connection: close");
  client.println();

  char endOfHeaders[] = "\r\n\r\n";

  client.setTimeout(HTTP_TIMEOUT);

  if (!client.find(endOfHeaders)) {
    Serial.println("No response or invalid response!");
    return;
  }
  
  Serial.println("Getting response");
  
  String line = "";
  while(client.available()){
    line = client.readStringUntil('\r');
    if (line.length() > 2) break;
  }

  Serial.println(line);
    
  aJsonObject* root = aJson.parse(( char*) line.c_str());
  if (root) {
    aJsonObject* jSeconds = aJson.getObjectItem(root, "seconds");
    aJsonObject* jMinutes = aJson.getObjectItem(root, "minutes");
    aJsonObject* jHours = aJson.getObjectItem(root, "hours");
    aJsonObject* jDay = aJson.getObjectItem(root, "day");
    aJsonObject* jDate = aJson.getObjectItem(root, "date");
    aJsonObject* jMonth = aJson.getObjectItem(root, "month");
    aJsonObject* jYear = aJson.getObjectItem(root, "year");
    if (jSeconds && jMinutes && jHours && jDay && jDate && jMonth && jYear)
      myRTC.setDS1302Time(jSeconds->valueint,jMinutes->valueint,jHours->valueint,jDay->valueint,jDate->valueint,jMonth->valueint,jYear->valueint);
    aJson.deleteItem(root);
  }
}

/*void animationWakeUp() {
  setBrightness(15);
  writeChars('W','A','K','E');
  delay(1000);
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < NBELEMS(m); ++j)
      {
        m[j].fillRect(0,0,8,8,i%2==0);
        m[j].writeDisplay();
      }
    delay(500);
  }
}*/
