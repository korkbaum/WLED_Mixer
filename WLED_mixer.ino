// on-off, beim starten status nicht ändern
// 2d-Effekte raus
// nur aktivierte fx + pals holen....
// Invalide/leere wledjson files
// ret-values nach update_nextion ??

#define DEBUG 1     // 0 very basic Serial debug messages, 1 = more than just basic, 2 = full

#include <i2cEncoderMiniLib.h>  //for 2x rotary encoders via I2C 

#define I2C_ADDRESS_ROT1 0x21
#define I2C_ADDRESS_ROT2 0x30 

//Class initialization with the I2C addresses
i2cEncoderMiniLib encoder[2] = { i2cEncoderMiniLib(I2C_ADDRESS_ROT1), i2cEncoderMiniLib(I2C_ADDRESS_ROT2)};

#define INTPIN 23        // mini encoder interrupt pin
#define MIDSPEED  5      // threshold for medium speed of rot encoder
#define FASTSPEED 20     // threshold for high speed of rot encoder
#define QUICK 5          // rot value increase medium value 
#define VERYQUICK 15     // rot value increase high value 
#define JITTER   3       // tolerance before pot reading changes are taken serious

int rot_data[2] = {0};   // Current positions of rot encoders
int rot_data_old[2] = {0};

// for calculating encoder rotation speed in rotspeed() function
int myspeed[2] = {0};
int newPosition[2] = {0};
int oldPosition[2] = {0};
unsigned long newTime[2] = {0};
unsigned long oldTime[2] = {0};
float linearSpeed_old;


// 2x ADS1115 I2C Hardware extension board for multiple potentiometer readings
#include<ADS1115_WE.h> 
#include<Wire.h>
#define I2C_ADDRESS1 0x48
#define I2C_ADDRESS2 0x4A  // ADDR Pin to SDA on 2nd ADS1115 board changes its I2C address to 0x4A
ADS1115_WE adc1 = ADS1115_WE(I2C_ADDRESS1);
ADS1115_WE adc2 = ADS1115_WE(I2C_ADDRESS2);


// network & network data exchange
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoOTA.h>

#include <ArduinoJson.h>
#include <StreamUtils.h>
#include "wled_mixer_spiffs.h"
#include "wled_mixer_json2arrays.h"
#include "wled_mixer_uploads.h"

// Nextion touch display
#include "EasyNextionLibrary.h"
EasyNex myNex(Serial2);

//Nextion touch display button variables
bool bt0, bt1, bt2, bt3 = 0;
int selected_lamp = 0;
bool butSC = 0;
bool butOnOff = 0;
bool paletteORsolid[MAX_LAMPS] = {0};

uint32_t fx_speed = 128;
uint32_t fx_intensity = 128;
bool fx_go = 0;

unsigned long lastTime = 0;
unsigned long timerDelay = 100;

int lamp_count = 0;
int lamp_id[4] = {-1};      // ids are given by wledjson file postfix in SPIFFs. -1 = undefined
int voltages[9] = {0};      // for pot value readings. 2x4 from 2xADS1115 over I2C, 1* on GPIO 14
int voltages_old[9] = {0};

//Pre-Declarations
void startOTA();
void read_json_files();
void adjust_brightness(int pot_ID, int voltage);
void adc_init();
void init_encoder(int enc_ID);
float readChannel_adc1(ADS1115_MUX channel);
float readChannel_adc2(ADS1115_MUX channel);


////////////////////////////////////////////////////////////////

void setup() {

  if (DEBUG >= 1) Serial.begin(115200);

  // for reading I2C, potentiometer values over ADS1115 (2x) and rot encoder values (2x)
  Wire.begin(21, 22);
  adc_init();

  //Rot encoder init (I2C)
  pinMode(INTPIN, INPUT);
  init_encoder(0);
  init_encoder(1);

  // for Nextion display communication 
  Serial2.begin(38400, SERIAL_8N1, 16, 17);
  myNex.begin(38400); delay(200);
  myNex.writeStr("page 0"); 

  // Connect to wifi
  WiFiManager wifiManager;
  //  wifiManager.resetSettings();
  wifiManager.autoConnect("WLED_Mixer");

  startOTA();
  
  myNex.writeStr("ip_address.txt", "Config URL: http://" + WiFi.localIP().toString() );
  
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
      Serial.println("SPIFFS Mount Failed");
      return;
  }
 
  MDNS.begin("WLED_MIXER");
  
  bool files = readFile(SPIFFS, "/wledjson0", 0);
  if (!files) {
    // call up page on nextion
    myNex.writeStr("page 2");
    Serial.println("No wledjson files found. \r\nPlease upload valid json files via \r\nhttp://" + WiFi.localIP().toString() );
  } else {
    read_json_files();
  }

  // webserver configs
  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.on("/update", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleUpdate);
  
  server.begin();
  Serial.println("HTTP server started");

  if ((DEBUG == 1) || (DEBUG == 2)) {
    Serial.println("*************setup************");
     for (int j = 0; j < lamp_count; j++){  
      int fxid = search_fxchar_id_array( j, info_fxcount [j], state_seg_fx[j] );
      Serial.printf("j: %d ID: %d  name: %s IP: %s FX: %d FX Name: %s \r\n", j, lamp_id[j], info_name_char[j], info_ip_char[j], state_seg_fx[j], fx_char[j][fxid] );
      if (DEBUG == 2) {
        for (int i = 0; i < info_fxcount[j]; i++) { 
          Serial.printf("fx_char[j][i]:  fx_char[%d][%d]: %s\r\n", j, i, fx_char[j][i] );
        }
        Serial.printf( "\r\nlamp_id: %d palettes \r\n", j);
        for (int i = 0; i < info_palcount[j]; i++) {
          Serial.printf("pal_char[j][i]:  pal_char[%d][%d]: %s\r\n", j, i, pal_char[j][i] );
        }
      }
      Serial.println("********** setup end **************");
     }
  }
  
} //setup end

///////////////////////////////

void loop() {

  server.handleClient();
  ArduinoOTA.handle();

  //Nextion: check for toggle events in mynex serial reads
  myNex.NextionListen();
  while (Serial2.available()) {   //reads data from serial2 and sends to serial0 (serial monitor)
    Serial.print(char(Serial2.read()));
  }
  
  // Rotary encoder updates
  if (digitalRead(INTPIN) == LOW) {
    // Check the status of the encoder and call the callback 
    encoder[0].updateStatus();
    encoder[1].updateStatus();
  }

  if ((millis() - lastTime) > timerDelay) {
    encoder[0].writeStep((int32_t) 1);  //reset rot enc steps to standard after a little timeout
    encoder[1].writeStep((int32_t) 1);
    
    // ADS1115: Update potentiometer readings
    for (int j = 0; j < 9; j++) { 
      voltages_old[j] = voltages[j];    // keep last readings to find any changes
    }
    voltages[0] = map(readChannel_adc1(ADS1115_COMP_0_GND), 0.0, 4860, 255, 1); // value range between 0.0 and 4860 mV
    voltages[1] = map(readChannel_adc1(ADS1115_COMP_1_GND), 0.0, 4860, 255, 1); // dont go to zero to distinguish between off atate and low brightness
    voltages[2] = map(readChannel_adc1(ADS1115_COMP_2_GND), 0.0, 4860, 255, 1);
    voltages[3] = map(readChannel_adc1(ADS1115_COMP_3_GND), 0.0, 4860, 255, 1);
    voltages[4] = map(readChannel_adc2(ADS1115_COMP_0_GND), 0.0, 4860, 1, 255);
    voltages[5] = map(readChannel_adc2(ADS1115_COMP_1_GND), 0.0, 4860, 1, 255);
    voltages[6] = map(readChannel_adc2(ADS1115_COMP_2_GND), 0.0, 4860, 1, 255);
    voltages[7] = map(readChannel_adc2(ADS1115_COMP_3_GND), 0.0, 4860, 1, 255);
    voltages[8] = map(analogRead(36), 0, 4095, 255, 0);    // value range between 0 and 4095
    lastTime = millis();
  }

  static bool master_onoff = 0;     // crossfader (voltages[8]) acts as master switch
  
  for (int j = 0; j < 9; j++) { 
    if ((voltages[j] < voltages_old[j] - JITTER) || (voltages[j] > voltages_old[j] + JITTER)) {  
      //if (DEBUG == 2) 
      Serial.printf("pot changed: %d  all values: %d %d %d %d %d %d %d %d %d\r\n", j, voltages[0], voltages[1], voltages[2], voltages[3], voltages[4], voltages[5], voltages[6], voltages[7], voltages[8]);
      adjust_brightness(j, voltages[j]);       // now we know what value has chaned, let it send out 

      if (voltages[8] < 50) {     // crossfader servers as master on/off switch
        if (master_onoff == 1) {
          if ((DEBUG == 1) || (DEBUG == 2)) Serial.println("Master Power OFF");
          master_onoff = 0;
          for ( int i = 0; i < lamp_count; i++ ){
            send_udp (info_ip_char[i], (String)"{\"on\":false,\"PWRon\":false}" );
            state_on[i] = 0;
            state_PWRon[i] = 0;
          }
          myNex.writeNum("butOnOff.val", 0);
        }
      } 
      else if (voltages[8] > 200) {
        if (master_onoff == 0) {
          if ((DEBUG == 1) || (DEBUG == 2)) Serial.println("Master Power ON");
          master_onoff = 1;
          for ( int i = 0; i < lamp_count; i++ ){
            send_udp (info_ip_char[i], (String)"{\"on\":true,\"PWRon\":true}" );
            state_on[i] = 1;
            state_PWRon[i] = 1;
          }
          myNex.writeNum("butOnOff.val", 1);
        }
      }
    }
  }
  
} //loop end

///////////////////////////////////////////////////////////////////


void read_json_files(){
  
    if ((DEBUG == 1) || (DEBUG == 2)) Serial.print("files found: ");
    lamp_count = listDir(SPIFFS, "/", 0);
    
    for (int j = 0; j < lamp_count; j++){
      lamp_id[j] = j;     // in case we have less than 4 lamps, array values will remain -1 as initialized
      
      char buf[10];
      sprintf(buf, "%s%d", "/wledjson", j);      //generate filename
      if ((DEBUG == 1) || (DEBUG == 2)) Serial.print("call deserialization with: ");  Serial.println(buf);
      readFile(SPIFFS, buf, 0);
      File file = SPIFFS.open(buf);
      DeserializationError error = deserializeJson(doc, file);
      if (error) Serial.println(F("Failed to read file"));
      file.close();
      
      if ((DEBUG == 1) || (DEBUG == 2)) Serial.println("call json2array");
      json2array(j, 1);
    }
}


void update_wled_status(char * IP, int lamp_ID) {   // get current status such as fx, pal etc. for display on nextion
  HTTPClient http;
  String addr = "http://" + (String)IP + "/json";
  if (DEBUG == 2) Serial.println(addr);
  
  http.begin(addr);
  int httpResponseCode = http.GET();
  if (httpResponseCode == 200) {
    String httpstr = http.getString();
    DeserializationError error = deserializeJson(doc, httpstr);
    if (error) {              // Test if parsing succeeds.
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    json2array(lamp_ID, 0);
  }
  http.end();
}

////////////////

void adjust_brightness(int pot_ID, int voltage) {    // send changed brightness to respective lamp, based on potentiometer ID

  // pot_ID values
  // 0 = 1st wheel above 1st slider   pos = 0
  // 1 = 2nd wheel above 2nd slider   pos = 1
  // 2 = 3rd wheel above 3rd slider   pos = 2
  // 3 = 4th wheel above 4th slider   pos = 3
  // 4 = 4th slider (from right)      pos = 3
  // 5 = 3rd slider                   pos = 2
  // 6 = 2nd slider                   pos = 1
  // 7 = 1st slider                   pos = 0
  // 8 = crossfade / master slider handled in loop

  if (pot_ID < 4) {                         // wheel was moved, wheel are for bri
    if (lamp_id[pot_ID] == -1) return;
    char sendbri[15] = "\0";
    sprintf(sendbri, "%s%d%s","{\"bri\":", voltage, "}");
    if (DEBUG == 2) Serial.printf("\r\n Wheel Pot ID: %d  Name: %s  IP: %s  Position: %d  JSON: %s \r\n ", pot_ID, info_name_char[lamp_id[pot_ID]], info_ip_char[lamp_id[pot_ID]], lamp_id[pot_ID], sendbri);

    if (state_on[lamp_id[pot_ID]]) {     // jitter caused updates could turn lights on unintentionally
      send_udp(info_ip_char[lamp_id[pot_ID]], (String)sendbri);
    }
  } 
  else if (pot_ID >= 4 && pot_ID < 8) {   // slider was moved, sliders are for PWRbri
    if (lamp_id[7 - pot_ID] == -1) return;
    char sendPWRbri[20] = "\0";
    sprintf(sendPWRbri, "%s%d%s", "{\"PWRbri\":", voltage, "}");
    if (DEBUG == 2) Serial.printf("\r\n Slider Pot ID: %d  Name: %s  IP: %s  Position: %d  PWR JSON: %s \r\n ", pot_ID, info_name_char[lamp_id[7 - pot_ID]], info_ip_char[lamp_id[7 - pot_ID]], lamp_id[7 - pot_ID], sendPWRbri);

    if (state_on[lamp_id[7 - pot_ID]]) {     // jitter caused updates could turn lights on unintentionally
      send_udp(info_ip_char[lamp_id[7 - pot_ID]], (String)sendPWRbri);   // 7- because we have 8 wheels+sliders but only 4 positions
    }    
  } 
}


void send_udp (char * IP, String payload) {      // e.g. send_udp("{\"on\":false}");

  if ((WiFi.status() == WL_CONNECTED)) {
    WiFiUDP udp;
    if (DEBUG == 2) Serial.printf("send_udp: IP %s  payload: %s\r\n", IP, payload.c_str());
    udp.beginPacket(IP, 21324);
    udp.printf("%s", payload.c_str());
    udp.endPacket();
  }
} 


///////////// ADS1115 ADC over I2C converter
void adc_init() {
  if(!adc1.init()){
    Serial.println("adc1 not connected!");
  }
  adc1.setVoltageRange_mV(ADS1115_RANGE_6144); 
  adc1.setCompareChannels(ADS1115_COMP_0_GND);
  adc1.setMeasureMode(ADS1115_CONTINUOUS); 
  adc1.setConvRate(ADS1115_64_SPS);

  if(!adc2.init()){
    Serial.println("adc2 not connected!");
  }
  adc2.setVoltageRange_mV(ADS1115_RANGE_6144); 
  adc2.setCompareChannels(ADS1115_COMP_0_GND);
  adc2.setMeasureMode(ADS1115_CONTINUOUS);
  adc2.setConvRate(ADS1115_64_SPS);
}

// get ADS1115 values over I2C
float readChannel_adc1(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc1.setCompareChannels(channel);
  voltage = adc1.getResult_mV(); 
  return voltage;
}

float readChannel_adc2(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc2.setCompareChannels(channel);
  voltage = adc2.getResult_mV(); 
  return voltage;
}


////////// Rotary Encoder 

void init_encoder(int enc_ID){
  encoder[enc_ID].reset();
  encoder[enc_ID].begin(i2cEncoderMiniLib::WRAP_DISABLE
                | i2cEncoderMiniLib::DIRE_LEFT | i2cEncoderMiniLib::IPUP_ENABLE
                | i2cEncoderMiniLib::RMOD_X1 );

  encoder[enc_ID].writeCounter((int32_t) 0);      /* Reset the counter value */
  encoder[enc_ID].writeMax((int32_t) 32767);      /* Set the maximum threshold*/
  encoder[enc_ID].writeMin((int32_t) - 32767);    /* Set the minimum threshold */
  encoder[enc_ID].writeStep((int32_t) 1);         /* Set the step to 1*/
  encoder[enc_ID].id = enc_ID;
  
  // Definition of the events
  encoder[enc_ID].onIncrement = encoder_increment;
  encoder[enc_ID].onDecrement = encoder_decrement;
  //encoder[enc_ID].onMax = encoder_max;    //disabled, no limits to always get steps back for transmission
  //encoder[enc_ID].onMin = encoder_min;
  encoder[enc_ID].autoconfigInterrupt();    // Enable the I2C Encoder V2 interrupts
}

//Callback when the CVAL is incremented
void encoder_increment(i2cEncoderMiniLib* obj) {
  if ((obj->id))  set_rot(1, 1);
  else set_rot(0, -1);
}


//Callback when the CVAL is decremented
void encoder_decrement(i2cEncoderMiniLib* obj) {
  if ((obj->id)) set_rot(1, -1);
  else set_rot(0, 1);
}


void set_rot(int encID, int sign){    // determine speed level, set rot step value and sign of result (according to rot encoder direction)

  static int rot_data_old = 0;
  
  myspeed[encID] = rotspeed(rot_data[encID], encID);      //check how fast encoder knob was moved

  if (myspeed[encID] > FASTSPEED) {
    encoder[encID].writeStep((int32_t) VERYQUICK);
    rot_data[encID] = rot_data[encID] + (VERYQUICK * sign);
  }
  else if (myspeed[encID] >= MIDSPEED) {
    encoder[encID].writeStep((int32_t) QUICK);
    rot_data[encID] = rot_data[encID] + (QUICK * sign);
  }
  else if (myspeed[encID] < MIDSPEED) {
    encoder[encID].writeStep((int32_t) 1);
    rot_data[encID] = rot_data[encID] + (1 * sign);   
  }
  
  if (encID == 0) {           // encID = 0 is for palette or color changes
 
    if (paletteORsolid[selected_lamp]) {     // paletteORsolid: 1 = Solid Color, use rot encoder 0 as color wheel    
      if (rot_data[0] < 0) rot_data[0] = 360;
      else if (rot_data[0] > 360) rot_data[0] = 0;

      int color_tmp = color[selected_lamp] + (abs((rot_data[0] - rot_data_old)) * 554 * sign);
      if (color_tmp <= 0) color_tmp = 65500;
      else if (color_tmp > 65500) color_tmp = 0;
      
      color[selected_lamp] = color_tmp;
      Serial.printf("set_rot color[%d]: %d  rot_data[0]: %d \r\n", selected_lamp, color[selected_lamp], rot_data[0] );
      
      rot_data_old = rot_data[0];
      HSV2RGB( (float)color[selected_lamp], 100.0, 100.0 );    // converts current color value to RGB and sends it to selected_lamp via udp
    } 
    else {                     // paletteORsolid: 0 = palette changes
      if (rot_data[0] < 0) rot_data[0] = 0;
      if (rot_data[0] > info_palcount[selected_lamp] -1) rot_data[0] = info_palcount[selected_lamp] -1;   //don't overrun pal ID range
     
      int pallen = strlen(pal_char[selected_lamp][rot_data[0]]);
      char palbuf [25] = {0};
      for (int i = 0; i < pallen -4; i++) { palbuf[i] = pal_char[selected_lamp][rot_data[0]][i]; }   //eliminate _ID at the end of palette name
      myNex.writeStr("pal_name.txt", (String)palbuf );
    }
  }
  else {                      // encID = 1 is for fx changes only
    if (rot_data[1] < 0) rot_data[1] = 0;
    if (rot_data[1] > info_fxcount[selected_lamp] -1) rot_data[1] = info_fxcount[selected_lamp] -1;     //don't overrun fx ID range
    
    int fxlen = strlen(fx_char[selected_lamp][rot_data[1]]);
    char fxbuf [25] = {0};
    for (int i = 0; i < fxlen -4; i++) { fxbuf[i] = fx_char[selected_lamp][rot_data[1]][i]; }   //eliminate _ID at the end of fx name
    myNex.writeStr("fx_name.txt",  (String)fxbuf );
  }

}


//based on https://forum.arduino.cc/t/rotary-encoder-angular-velocity/703260
int rotspeed(int i, int encID) {      //i = current pos value of encoder[encID]

  float dPos = 0;
  float dTheta = 0;
  unsigned long dt = 0;
  float angularSpeed = 0;
  float linearSpeed = 0;
  float r = 0.09;
  
  newPosition[encID] = i;
  dPos = abs(newPosition[encID] - oldPosition[encID]);
  oldPosition[encID] = newPosition[encID];
  newTime[encID] = millis();
  dTheta = dPos*0.031415926535;
  dt = (newTime[encID] - oldTime[encID]);
  oldTime[encID] = newTime[encID];
  angularSpeed = (dTheta / dt)*1000;
  linearSpeed = r*angularSpeed*100;

  //Serial.print("rotspeed: *"); Serial.print(linearSpeed); Serial.println("*");
  if ((linearSpeed < linearSpeed/2) || (linearSpeed > linearSpeed*4)) {
    return (int)linearSpeed_old;
  } else { 
    linearSpeed_old = linearSpeed;
    return (int)linearSpeed;
  }
}


// Nextion Touch Display Trigger Functions
// https://github.com/Seithan/EasyNextionLibrary

void trigger1(){    //RadioButton "r0" Touch Release Event printh 23 02 54 01
  bt0 = myNex.readNumber("bt0.val");
  if ((DEBUG == 1) || (DEBUG == 2)) Serial.println((String)"bt0: " + bt0);
  selected_lamp = 0;
  update_nextion(selected_lamp);
}

void trigger2(){    //RadioButton "r1" Touch Release Event printh 23 02 54 02
  bt1 = myNex.readNumber("bt1.val");
  if ((DEBUG == 1) || (DEBUG == 2)) Serial.println((String)"bt1: " + bt1);
  selected_lamp = 1;
  update_nextion(selected_lamp);
}

void trigger3(){    //RadioButton "r2" Touch Release Event printh 23 02 54 03
  bt2 = myNex.readNumber("bt2.val");
  if ((DEBUG == 1) || (DEBUG == 2)) Serial.println((String)"bt2: " + bt2);
  selected_lamp = 2;
  update_nextion(selected_lamp);
}

void trigger4(){    //RadioButton "r3" Touch Release Event printh 23 02 54 04
  bt3 = myNex.readNumber("bt3.val");
  if ((DEBUG == 1) || (DEBUG == 2)) Serial.println((String)"bt3: " + bt3);
  selected_lamp = 3;
  update_nextion(selected_lamp);
}

void trigger5(){    //
}

void trigger6(){    //Button "butSC" Touch Release Event printh 23 02 54 06
  butSC = myNex.readNumber("butSC.val");
  myNex.writeNum("butSC.pco", 64512);
  myNex.writeNum("fx_go.pco", 54938);
  myNex.writeStr("pal_name.txt",  "(Color Wheel Mode)" );
  if ((DEBUG == 1) || (DEBUG == 2)) Serial.println((String)"6 butSC: " + butSC);
 
  paletteORsolid[selected_lamp] = 1;        // to control what rot_encoder[0] is doing
  // calculate current solid color
  //color[selected_lamp] = RGB2HSV( (float)state_col_R[selected_lamp], (float)state_col_G[selected_lamp], (float)state_col_B[selected_lamp] );
  // palid init?
  
  state_seg_fx[selected_lamp] = 0;        // set solid color mode fx=0
  send_udp (info_ip_char[selected_lamp], (String)"{\"seg\":[{\"id\":0, \"fx\":0}]}" );
  // Solid Color changes are handled in set_rot function
}

void trigger7(){    //Button "butOnOff" Touch Release Event printh 23 02 54 07
  String butOnOffstr = "false";
  butOnOff = myNex.readNumber("butOnOff.val");
  if ((DEBUG == 1) || (DEBUG == 2)) Serial.println((String)"7 butOnOff: " + butOnOff);
  state_on[selected_lamp] = butOnOff;
  state_PWRon[selected_lamp] = butOnOff;
  if (butOnOff == 0) butOnOffstr = "false";     // 0 and 1 don't work in JSON API call
  else butOnOffstr = "true";
  send_udp (info_ip_char[selected_lamp], (String)"{\"on\":" + butOnOffstr + ",\"PWRon\":" + butOnOffstr + "}" );
}

void trigger8(){    //Slider "fx_speed" Touch Move Event printh 23 02 54 08
  fx_speed = myNex.readNumber("fx_speed.val");
  if ((DEBUG == 1) || (DEBUG == 2)) Serial.print("8 fx_speed: "); Serial.println(fx_speed);
  state_seg_sx[selected_lamp] = fx_speed;
  send_udp (info_ip_char[selected_lamp], (String)"{\"seg\":[{\"id\":0, \"sx\":" + (String)fx_speed + "}]}" );
}

void trigger9(){    //Slider "fx_intensity" Touch Move Event printh 23 02 54 09
  fx_intensity = myNex.readNumber("fx_intensity.val");
  if ((DEBUG == 1) || (DEBUG == 2)) Serial.print("9 fx_intensity: "); Serial.println(fx_intensity);
  state_seg_ix[selected_lamp] = fx_speed;
  send_udp (info_ip_char[selected_lamp], (String)"{\"seg\":[{\"id\":0, \"ix\":" + (String)fx_speed + "}]}" );
}

void trigger10(){    //Button "fx_go" Touch Release Event printh 23 02 54 0A
  fx_go = myNex.readNumber("fx_go.val");
  myNex.writeNum("butSC.pco", 54938);
  myNex.writeNum("fx_go.pco", 64512);
  if ((DEBUG == 1) || (DEBUG == 2)) Serial.println((String)"10 (0A) fx_go: " + fx_go);
  paletteORsolid[selected_lamp] = 0;        // to control what rot_encoder[0] is doing
  
  // last 3 digits of fx_char[] name hold the ID of the effect fx_char[selected_lamp][rot_data[1]]
  int fxlen = strlen(fx_char[selected_lamp][rot_data[1]]);
  char fxID[4] = { fx_char[selected_lamp][rot_data[1]][fxlen-3], fx_char[selected_lamp][rot_data[1]][fxlen-2], fx_char[selected_lamp][rot_data[1]][fxlen-1] };  
  send_udp (info_ip_char[selected_lamp], "{\"seg\":[{\"id\":0, \"fx\":" + (String)fxID + "}]}" );
  state_seg_fx[selected_lamp] = atoi(fxID);

   // last 3 digits of pal_char[] name hold the ID of the effect
  int pallen = strlen(pal_char[selected_lamp][rot_data[0]]);
  char palID[4] = { pal_char[selected_lamp][rot_data[0]][pallen-3], pal_char[selected_lamp][rot_data[0]][pallen-2], pal_char[selected_lamp][rot_data[0]][pallen-1] };  
  send_udp (info_ip_char[selected_lamp], "{\"seg\":[{\"id\":0, \"pal\":" + (String)palID + "}]}" );
  state_seg_pal[selected_lamp] = atoi(palID);
}

void trigger11(){    //operation panel (nextion page 1) was openend, shall show current values. printh 23 02 54 0B
    update_nextion(0);
}


/////////////////////////

void update_nextion(int lamp_id){    // called if lamp was changed by lamp buttons 1-4 on nextion
  
  if (lamp_id == -1) return;         // position not allocated/assigned to a lamp; i.e. less than 4 lamps total

  char butbuf[10] = {0};
  sprintf (butbuf, "bt%d.pco2", lamp_id);  // show ongoing update with button text color change
  char butbuf_save[10] = {0};
  strncpy(butbuf_save, butbuf, 10);        // we need to know the current button id in the end again
  
  myNex.writeNum(butbuf, 65504);            // 54938 = white/normal text, 64512 = orange text, 65504 = yellow text
  for (int i=0; i< lamp_count; i++) {     
    if (i != lamp_id) {                     // reset any other buttons to normal text color
       sprintf (butbuf, "bt%d.pco2", i);
       myNex.writeNum(butbuf, 54938);
    }
  }

  update_wled_status(info_ip_char[lamp_id], lamp_id);  //pulls the current json status and updates parts of the global arrays
  
  myNex.writeNum("butOnOff.val", state_on[lamp_id]);
  myNex.writeStr("lamp_name.txt", (String)info_name_char[lamp_id] );

  // calculate and save current solid color
  color[lamp_id] = RGB2HSV( (float)state_col_R[lamp_id], (float)state_col_G[lamp_id], (float)state_col_B[lamp_id] );
Serial.println(" udate_nextion color[lamp_id]: " + (String)color[lamp_id] );
  // palette handling
  int palid = search_palchar_id_array( lamp_id, info_palcount[lamp_id], state_seg_pal[lamp_id] );
  int pallen = strlen(pal_char[lamp_id][palid]);
  char palbuf [25] = {0};
  for (int i = 0; i < pallen -4; i++) { palbuf[i] = pal_char[lamp_id][palid][i]; }   //eliminate _ID at the end of palette name
  
  // effect handling
  int fxid = search_fxchar_id_array( lamp_id, info_fxcount[lamp_id], state_seg_fx[lamp_id] );
  int fxlen = strlen(fx_char[lamp_id][fxid]);
  char fxbuf [25] = {0};
  for (int i = 0; i < fxlen -4; i++) { fxbuf[i] = fx_char[lamp_id][fxid][i]; }   //eliminate _ID at the end of fx name
  myNex.writeStr("fx_name.txt",  (String)fxbuf );
  rot_data[1] = fxid;   //start with last chosen fx for this lamp
    
  myNex.writeNum("fx_speed.val", state_seg_sx[lamp_id]);
  myNex.writeNum("fx_intensity.val", state_seg_ix[lamp_id]);

  if (state_seg_fx[lamp_id] != 0) {          // some fx is active, not solid color
    myNex.writeNum("fx_go.pco", 64512);      // orange text
    myNex.writeNum("butSC.pco", 54938);      // 54938 = white/normal text
    myNex.writeStr("pal_name.txt", (String)palbuf );
    paletteORsolid[lamp_id] = 0;
    rot_data[0] = palid;    // start with last chosen palette ID of this lamp
  } 
  else {  // Solid Color
    myNex.writeNum("butSC.pco", 64512);
    myNex.writeNum("fx_go.pco", 54938);
    myNex.writeStr("pal_name.txt",  "(Color Wheel Mode)" );
    paletteORsolid[lamp_id] = 1;
    rot_data[0] = color[lamp_id];            // start with last chosen color of this lamp
  }

  myNex.writeNum(butbuf_save, 64512);          // 54938 = white/normal text, 64512 = orange text, 65504 = yellow text
}

////////////////////////////

void HSV2RGB(float H, float S, float V) {      // called with H = rot encoder value, s, v = 100.0
  H = H * 0.0054931640625;    //map 0-65536 values to the 0-360 degree range
  float s = S/100.0;
  float v = V/100.0;
  float C = s * v;
  float X = C * (1-fabs(fmod(H/60.0, 2)-1));
  float m = v - C;
  float r , g, b;
  
  if (H >= 0 && H < 60)       {  r = C, g = X, b = 0;  }
  else if(H >= 60 && H < 120) {  r = X, g = C, b = 0;  }
  else if(H >= 120 && H < 180){  r = 0, g = C, b = X;  }
  else if(H >= 180 && H < 240){  r = 0, g = X, b = C;  }
  else if(H >= 240 && H < 300){  r = X, g = 0, b = C;  }
  else{        r = C, g = 0, b = X;    }
  
  int R = ((r+m)*255);
  int G = ((g+m)*255);
  int B = ((b+m)*255);
  
  char Rchar[4]; sprintf( Rchar, "%d", R );
  char Gchar[4]; sprintf( Gchar, "%d", G );
  char Bchar[4]; sprintf( Bchar, "%d", B );
  
  // JSON API call to set color e.g.: {"seg":[{"col":[[0,255,200]]}]}
  char payload[35] = "\0";
  sprintf( payload, "%s%s%s%s%s%s%s", "{\"seg\":[{\"col\":[[", Rchar, ",", Gchar, ",", Bchar, "]]}]}" );
  send_udp(info_ip_char[selected_lamp], (String)payload );

  state_col_R [selected_lamp] = R;  // save the current solid color
  state_col_G [selected_lamp] = G;
  state_col_B [selected_lamp] = B;
}


// from https://www.tutorialspoint.com/c-program-to-change-rgb-color-model-to-hsv-color-model
float max(float a, float b, float c) {
   return ((a > b)? (a > c ? a : c) : (b > c ? b : c));
}

float min(float a, float b, float c) {
   return ((a < b)? (a < c ? a : c) : (b < c ? b : c));
}

int RGB2HSV(float r, float g, float b) {
   // R, G, B values are divided by 255
   // to change the range from 0..255 to 0..1:
   float h, s, v;
   r /= 255.0;
   g /= 255.0;
   b /= 255.0;
   float cmax = max(r, g, b); // maximum of r, g, b
   float cmin = min(r, g, b); // minimum of r, g, b
   float diff = cmax-cmin; // diff of cmax and cmin.
   if (cmax == cmin)
      h = 0;
   else if (cmax == r)
      h = fmod((60 * ((g - b) / diff) + 360), 360.0);
   else if (cmax == g)
      h = fmod((60 * ((b - r) / diff) + 120), 360.0);
   else if (cmax == b)
      h = fmod((60 * ((r - g) / diff) + 240), 360.0);
   // if cmax equal zero
      if (cmax == 0)
         s = 0;
      else
         s = (diff / cmax) * 100;
   // compute v
   v = cmax * 100;
   
   return (int)h;
}


void startOTA(){
  // OTA stuff
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("WLED Mixer");
  ArduinoOTA.onStart([]() {    Serial.println("Start");  });
  ArduinoOTA.onEnd([]()   {    Serial.println("\nEnd");  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
}
