#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "espDMX_RDM.h"
#include "espArtNetRDM.h"
#include "wsFX.h"
#include <NeoPixelBus.h>
#include "config.h"
#include <ESPUI.h>
#include "DNSServer.h"
#include <Updater.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

extern "C" {
  #include "user_interface.h"
  extern struct rst_info resetInfo;
}

// #define DEBUG
// #define SOLE_OUTPUT
// #define OLED

#define TIMEOUT_AP                  30000       // Timeout [ms] before closing Access Point if standalone mode is false
#define TIMEOUT_CLEAR_COUNTERS      6000        // Timeout [ms] before clearing wdt and reset counters
#define TIMEOUT_DMX                 3000        // Timeout [ms] before pausing dmx output when no artnet data is received anymore
#define INTERVAL_STATUS_LED         1000        // Interval for updating Status LEDs
#define INTERVAL_STATUS_LED_UPDATE  500         // Interval for updating Status LEDs in case of OTA Update
#define LIM_PIXELS_PER_ARTNET_PORT  170         // Maximum allowed number of pixels per ArtNet port
#define STATUS_LED_COLOR_SAT        11          // Status LEDs brightness
#define DNS_PORT                    53          // Port used by DNS server

#ifdef OLED
  #define DISP_LOGO_WIDTH    128
  #define DISP_LOGO_HEIGHT   32
  const unsigned char dispLogo [] PROGMEM = {
    // 'Art-Net-DJ-logo-scaled, 128x32px
    0x03, 0x01, 0xfc, 0x1f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x01, 0x04, 0x80, 0x00, 0x00, 0x00, 0x00, 
    0x03, 0x81, 0x9f, 0x1f, 0xff, 0xc0, 0x00, 0x00, 0x11, 0x83, 0x07, 0x87, 0x00, 0x00, 0x00, 0x00, 
    0x06, 0x81, 0x81, 0x80, 0x60, 0x00, 0x00, 0x00, 0x19, 0x83, 0x07, 0x04, 0x0e, 0x00, 0x00, 0x00, 
    0x04, 0xc1, 0x81, 0x80, 0x60, 0x00, 0x00, 0x00, 0x19, 0x83, 0x83, 0x04, 0x0f, 0x00, 0x00, 0x00, 
    0x0c, 0x41, 0x80, 0x80, 0x60, 0x00, 0x00, 0x80, 0x1d, 0x87, 0x83, 0x07, 0x0f, 0x0f, 0x00, 0x00, 
    0x08, 0x61, 0x81, 0x80, 0x60, 0x00, 0x00, 0xc0, 0x1f, 0x87, 0x87, 0x04, 0x0e, 0x09, 0x00, 0x00, 
    0x18, 0x61, 0x83, 0x80, 0x60, 0x00, 0x00, 0xc0, 0x1e, 0x84, 0xc5, 0x84, 0x0e, 0x18, 0x08, 0x00, 
    0x18, 0x31, 0x9f, 0x00, 0x60, 0x00, 0x60, 0x40, 0x1e, 0x84, 0x44, 0x87, 0x0a, 0x17, 0x0d, 0x00, 
    0x3f, 0xf1, 0x8c, 0x00, 0x60, 0x00, 0xf0, 0x40, 0x12, 0x00, 0x00, 0x00, 0x0b, 0x1b, 0x0f, 0x00, 
    0x30, 0x11, 0x84, 0x00, 0x60, 0x00, 0x98, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x0e, 0x00, 
    0x20, 0x19, 0x86, 0x00, 0x60, 0x00, 0x98, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x0c, 0x00, 
    0x60, 0x09, 0x83, 0x00, 0x60, 0x00, 0xc8, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 
    0x40, 0x0d, 0x81, 0x80, 0x60, 0x00, 0x58, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x70, 0x00, 0x08, 0x04, 0x00, 0x04, 0x00, 0x00, 0x08, 0x00, 
    0xe0, 0x01, 0x8f, 0xfe, 0x60, 0x00, 0x60, 0x00, 0x04, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 
    0xf0, 0x03, 0x8f, 0xfe, 0x60, 0x00, 0x00, 0x00, 0x04, 0x04, 0x10, 0x04, 0x00, 0x00, 0x00, 0x00, 
    0xf8, 0x01, 0x8c, 0x00, 0x60, 0x00, 0x00, 0x40, 0x04, 0x24, 0x11, 0x04, 0x00, 0x10, 0x00, 0x00, 
    0xd8, 0x01, 0x8c, 0x00, 0x60, 0x00, 0x00, 0x40, 0x04, 0x20, 0x11, 0x04, 0x40, 0x10, 0x20, 0x00, 
    0xcc, 0x01, 0x8c, 0x00, 0x60, 0x00, 0x08, 0x20, 0x92, 0x20, 0x00, 0x00, 0x40, 0x30, 0x40, 0x00, 
    0xce, 0x01, 0x8c, 0x00, 0x60, 0x00, 0x08, 0x20, 0x90, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40, 0x00, 
    0xc7, 0x01, 0x8c, 0x00, 0x60, 0x00, 0x04, 0x20, 0x00, 0x3f, 0xff, 0xf6, 0x00, 0x28, 0x80, 0x00, 
    0xc3, 0x01, 0x8c, 0x00, 0x60, 0x00, 0x05, 0x10, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x0a, 0x80, 0x00, 
    0xc1, 0x81, 0x8f, 0xfe, 0x60, 0x00, 0x03, 0x00, 0xff, 0xff, 0xff, 0xf7, 0xff, 0x81, 0x00, 0x00, 
    0xc0, 0xc1, 0x8f, 0xfe, 0x60, 0x00, 0x02, 0x07, 0xff, 0x80, 0x00, 0x01, 0xff, 0xf0, 0x00, 0x00, 
    0xc0, 0xe1, 0x8c, 0x00, 0x60, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 0x00, 0x07, 0xfe, 0x00, 0x00, 
    0xc0, 0x61, 0x8c, 0x00, 0x60, 0x00, 0x01, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xc0, 0x00, 
    0xc0, 0x31, 0x8c, 0x00, 0x60, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 
    0xc0, 0x19, 0x8c, 0x00, 0x60, 0x1c, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x04, 
    0xc0, 0x1d, 0x8c, 0x00, 0x60, 0x1c, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x0e, 
    0xc0, 0x0f, 0x8c, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x0e, 
    0xc0, 0x07, 0x8f, 0xfe, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
    0xc0, 0x03, 0x8f, 0xfe, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
#endif

/* ---------------- Pin config ---------------- */
#define PIN_DMX_DIR_A               13  // D1 Mini: D7
#define PIN_DMX_DIR_B               16  // D1 Mini: D0
#define PIN_PORT_A                  1   // D1 Mini: TX
#define PIN_PORT_B                  2   // D1 Mini: D4

#define PIN_STATUS_LED              12  // D6, approved: 3 = RX (DMA support, but used for DMX input)
// Physical wiring order for status LEDs:
#define ADDR_STATUS_LED_S           0
#define ADDR_STATUS_LED_A           1
#define ADDR_STATUS_LED_B           2

#define PIN_RESET_CONFIG            14
/* -------------- Pin config end -------------- */

Config config, configActive;
esp8266ArtNetRDM artRDM;
DNSServer dnsServer;

uint8_t portA[5], portB[5];
uint8_t macAddr[6];
char macAddr_str[18];
uint32_t statusTimer = 0;
uint32_t artDmxReceiveTimer = 0;

char wifiStatus[100] = "";
bool accessPointStarted = false;
uint32_t nextNodeReport = 0;
char nodeError[ARTNET_NODE_REPORT_LENGTH] = "";
bool nodeErrorShowing = 1;
uint32_t nodeErrorTimeout = 0;
bool pixDone = true;
bool newDmxIn = false;
byte* dataIn;
bool doReboot = false;
bool dmxOutAPaused = false;
#ifndef SOLE_OUTPUT
  bool dmxOutBPaused = false;
#endif
bool artDmxReceiving = false;

#ifdef SOLE_OUTPUT
  NeoPixelBus<NeoGrbFeature, NeoEsp8266AsyncUart0Ws2812xMethod>* pixPortA = NULL; // uses shared interrupt -> conflicts with DMX output
  pixPatterns* pixFXA;
#else
  NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart0Ws2812xMethod>* pixPortA = NULL;
  NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart1Ws2812xMethod>* pixPortB = NULL;
  pixPatterns* pixFXA;
  pixPatterns* pixFXB;
#endif

// NeoPixelBus<NeoGrbFeature, NeoEsp8266DmaWs2812xMethod> statusStrip(3, PIN_STATUS_LED); // approved (DMA)
NeoPixelBus<NeoGrbFeature, NeoEsp8266BitBangWs2812xMethod> statusStrip(3, PIN_STATUS_LED);
RgbColor red(STATUS_LED_COLOR_SAT, 0, 0);
RgbColor green(0, STATUS_LED_COLOR_SAT, 0);
RgbColor yellow(STATUS_LED_COLOR_SAT, STATUS_LED_COLOR_SAT, 0);
RgbColor blue(0, 0, STATUS_LED_COLOR_SAT);
RgbColor cyan(0, STATUS_LED_COLOR_SAT, STATUS_LED_COLOR_SAT);
RgbColor white(STATUS_LED_COLOR_SAT);
RgbColor black(0);
RgbColor pink(STATUS_LED_COLOR_SAT, STATUS_LED_COLOR_SAT/11, STATUS_LED_COLOR_SAT/3);

#ifdef OLED
  Adafruit_SSD1306 display(128, 32, &Wire, -1);
#endif

uint16_t numWdtCounter;
uint16_t numRstCounter;
uint16_t lblNetStaNetworksAvail;
uint16_t lblNetStaBroadcast;
uint16_t lblNetApBroadcast;
String strListNetworksAvail = "";
uint16_t txtNetStaIPAddr;
uint16_t txtNetStaSubnetAddr;
uint16_t txtNetStaGatewayAddr;
uint16_t numNetApDelay;
uint16_t numPixelsA;
uint16_t selPixelModeA;
uint16_t numPixelFxStartChA;
uint16_t numPixelsB;
uint16_t selPixelModeB;
uint16_t numPixelFxStartChB;

void handleUpdate(AsyncWebServerRequest *request) {
  char html[] = "<form method='POST' action='/doUpdate' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
  request->send(200, "text/html", html);
}

void handleDoUpdate(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) {
    #ifdef DEBUG
      Serial.println("Update");
    #endif
    size_t content_len = request->contentLength();
    // if filename includes spiffs, update the spiffs partition
    int cmd = (filename.indexOf("spiffs") > -1) ? U_FS : U_FLASH;
    Update.runAsync(true);
    if (!Update.begin(content_len, cmd)) {
      #ifdef DEBUG
        Update.printError(Serial);
      #endif
    }
  }

#ifdef DEBUG
  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  } else {
    Serial.printf("Progress: %d%%\n", (Update.progress()*100)/Update.size());
  }
#endif

  if (final) {
    AsyncWebServerResponse *response = request->beginResponse(302, "text/plain", "Please wait while the device reboots");
    response->addHeader("Refresh", "20");  
    response->addHeader("Location", "/");
    request->send(response);
    if (!Update.end(true)) {
      #ifdef DEBUG
        Update.printError(Serial);
      #endif
    } else {
      #ifdef DEBUG
        Serial.println("Update complete");
        Serial.flush();
      #endif
      // ESP.restart();
      doReboot = true;
    }
  }
}

/* ---------------- Number callbacks ---------------- */

void cbNumNetApDelay(Control* sender, int type) {
  uint16_t val = (uint16_t)(sender->value.toInt());
  if (val > 180) {
    val = 180;
  }
  config.net_ap_delay = val;
}

void cbNumNetPortA(Control* sender, int type) {
  uint8_t val = (uint8_t)(sender->value.toInt());
  if (val > 127) {
    val = 127;
  }
  config.portA_net = val;
}

void cbNumSubnetPortA(Control* sender, int type) {
  uint8_t val = (uint8_t)(sender->value.toInt());
  if (val > 15) {
    val = 15;
  }
  config.portA_subnet = val;
}

void cbNumPixelsPortA(Control* sender, int type) {
  uint16_t val = (uint16_t)(sender->value.toInt());
  if (val > 680) {
    val = 680;
  }
  config.portA_pixel_count = val;
}

void cbNumPixelFxStartChPortA(Control* sender, int type) {
  uint16_t val = (uint16_t)(sender->value.toInt());
  if (val > 501) {
    val = 501;
  } else if (val < 1) {
    val = 1;
  }
  config.portA_pixel_startFX = val;
}

#ifndef SOLE_OUTPUT
void cbNumNetPortB(Control* sender, int type) {
  uint8_t val = (uint8_t)(sender->value.toInt());
  if (val > 127) {
    val = 127;
  }
  config.portB_net = val;
}

void cbNumSubnetPortB(Control* sender, int type) {
  uint8_t val = (uint8_t)(sender->value.toInt());
  if (val > 15) {
    val = 15;
  }
  config.portB_subnet = val;
}

void cbNumPixelsPortB(Control* sender, int type) {
  uint16_t val = (uint16_t)(sender->value.toInt());
  if (val > 680) {
    val = 680;
  }
  config.portB_pixel_count = val;
}

void cbNumPixelFxStartChPortB(Control* sender, int type) {
  uint16_t val = (uint16_t)(sender->value.toInt());
  if (val > 501) {
    val = 501;
  } else if (val < 1) {
    val = 1;
  }
  config.portB_pixel_startFX = val;
}
#endif

/* -------------- Number callbacks end -------------- */

/* ------------ Number extended callbacks ----------- */

void cbNumUniversePortA(Control* sender, int type, void* param) {
  uint8_t val = (uint8_t)(sender->value.toInt());
  if (val > 15) {
    val = 15;
  }
  config.portA_uni[(int)param] = val;
}

void cbNumSACNUniversePortA(Control* sender, int type, void* param) {
  uint16_t val = (uint16_t)(sender->value.toInt());
  if (val > 63999) {
    val = 63999;
  } else if (((int)param > 0) && (val < 1)) {
    val = 1;
  }
  config.portA_SACNuni[(int)param] = val;
}

#ifndef SOLE_OUTPUT
void cbNumUniversePortB(Control* sender, int type, void* param) {
  uint8_t val = (uint8_t)(sender->value.toInt());
  if (val > 15) {
    val = 15;
  }
  config.portB_uni[(int)param] = val;
}

void cbNumSACNUniversePortB(Control* sender, int type, void* param) {
  uint16_t val = (uint16_t)(sender->value.toInt());
  if (val > 63999) {
    val = 63999;
  } else if (((int)param > 0) && (val < 1)) {
    val = 1;
  }
  config.portB_SACNuni[(int)param] = val;
}
#endif

/* --------- Number extended callbacks end --------- */

/* ----------------- Text callbacks ---------------- */

void cbTxtNodeName(Control* sender, int type) {
  strlcpy(config.gen_nodeName, sender->value.c_str(), 18);
}

void cbTxtLongName(Control* sender, int type) {
  strlcpy(config.gen_longName, sender->value.c_str(), 64);
}

void cbTxtNetStaSSID(Control* sender, int type) {
  strlcpy(config.net_sta_ssid, sender->value.c_str(), 32);
}

void cbTxtNetStaPwd(Control* sender, int type) {
  // min. 8, max. 64 characters
  if (sender->value.length() < 8) {
    strcpy(config.net_sta_password, "");
  } else {
    strlcpy(config.net_sta_password, sender->value.c_str(), 64);
  }
}

void cbTxtNetStaIP(Control* sender, int type) {
  config.net_sta_ip.fromString(sender->value.c_str());
  config.net_sta_broadcast = {
    (uint8_t)((~config.net_sta_subnet[0]) | (config.net_sta_ip[0] & config.net_sta_subnet[0])), 
    (uint8_t)((~config.net_sta_subnet[1]) | (config.net_sta_ip[1] & config.net_sta_subnet[1])), 
    (uint8_t)((~config.net_sta_subnet[2]) | (config.net_sta_ip[2] & config.net_sta_subnet[2])), 
    (uint8_t)((~config.net_sta_subnet[3]) | (config.net_sta_ip[3] & config.net_sta_subnet[3]))  
  };
  ESPUI.updateLabel(lblNetStaBroadcast, config.net_sta_broadcast.toString());
}

void cbTxtNetStaSubnet(Control* sender, int type) {
  config.net_sta_subnet.fromString(sender->value.c_str());
  config.net_sta_broadcast = {
    (uint8_t)((~config.net_sta_subnet[0]) | (config.net_sta_ip[0] & config.net_sta_subnet[0])), 
    (uint8_t)((~config.net_sta_subnet[1]) | (config.net_sta_ip[1] & config.net_sta_subnet[1])), 
    (uint8_t)((~config.net_sta_subnet[2]) | (config.net_sta_ip[2] & config.net_sta_subnet[2])), 
    (uint8_t)((~config.net_sta_subnet[3]) | (config.net_sta_ip[3] & config.net_sta_subnet[3]))  
  };
  ESPUI.updateLabel(lblNetStaBroadcast, config.net_sta_broadcast.toString());
}

void cbTxtNetStaGateway(Control* sender, int type) {
  config.net_sta_gateway.fromString(sender->value.c_str());
}

void cbTxtNetApSSID(Control* sender, int type) {
  strlcpy(config.net_ap_ssid, sender->value.c_str(), 32);
}

void cbTxtNetApPwd(Control* sender, int type) {
  // min. 8, max. 64 characters
  if (sender->value.length() < 8) {
    strcpy(config.net_ap_password, "");
  } else {
    strlcpy(config.net_ap_password, sender->value.c_str(), 64);
  }
}

void cbTxtNetApIP(Control* sender, int type) {
  config.net_ap_ip.fromString(sender->value.c_str());
  config.net_ap_broadcast = {
    (uint8_t)((~config.net_ap_subnet[0]) | (config.net_ap_ip[0] & config.net_ap_subnet[0])), 
    (uint8_t)((~config.net_ap_subnet[1]) | (config.net_ap_ip[1] & config.net_ap_subnet[1])), 
    (uint8_t)((~config.net_ap_subnet[2]) | (config.net_ap_ip[2] & config.net_ap_subnet[2])), 
    (uint8_t)((~config.net_ap_subnet[3]) | (config.net_ap_ip[3] & config.net_ap_subnet[3]))  
  };
  ESPUI.updateLabel(lblNetApBroadcast, config.net_ap_broadcast.toString());
}

void cbTxtNetApSubnet(Control* sender, int type) {
  config.net_ap_subnet.fromString(sender->value.c_str());
  config.net_ap_broadcast = {
    (uint8_t)((~config.net_ap_subnet[0]) | (config.net_ap_ip[0] & config.net_ap_subnet[0])), 
    (uint8_t)((~config.net_ap_subnet[1]) | (config.net_ap_ip[1] & config.net_ap_subnet[1])), 
    (uint8_t)((~config.net_ap_subnet[2]) | (config.net_ap_ip[2] & config.net_ap_subnet[2])), 
    (uint8_t)((~config.net_ap_subnet[3]) | (config.net_ap_ip[3] & config.net_ap_subnet[3]))  
  };
  ESPUI.updateLabel(lblNetApBroadcast, config.net_ap_broadcast.toString());
}

void cbTxtNetDmxIn(Control* sender, int type) {
  config.net_dmxIn_broadcast.fromString(sender->value.c_str());
}

/* --------------- Text callbacks end -------------- */

/* ---------------- Button callbacks ---------------- */

void cbBtnReboot(Control* sender, int type) {
  switch (type) {
    case B_DOWN:
      break;
    case B_UP:
      delay(500);
      doReboot = true;
      break;
  }
}

void cbBtnSaveConfig(Control* sender, int type) {
  switch (type) {
    case B_DOWN:
      break;
    case B_UP:
      #ifdef DEBUG
        Serial.println("Button UP - Saving Configuration...");
      #endif
      config_save();
      break;
  }
}

void cbBtnResetConfig(Control* sender, int type) {
  switch (type) {
    case B_DOWN:
      break;
    case B_UP:
      #ifdef DEBUG
        Serial.println("Button UP - Resetting Configuration...");
      #endif
      config_init();
      config_save();
      delay(500);
      doReboot = true;
      break;
  }
}

/* -------------- Button callbacks end -------------- */

/* ---------------- Switch callbacks ---------------- */

void cbSwNetStaDHCP(Control* sender, int value) {
  switch (value) {
    case S_ACTIVE:
        config.net_sta_dhcp = true;
        ESPUI.setEnabled(txtNetStaIPAddr, false);
        ESPUI.setEnabled(txtNetStaSubnetAddr, false);
        ESPUI.setEnabled(txtNetStaGatewayAddr, false);
        if (config.net_ap_standalone == false) {
          ESPUI.setEnabled(numNetApDelay, true);
        }
        break;
    case S_INACTIVE:
        config.net_sta_dhcp = false;
        ESPUI.setEnabled(txtNetStaIPAddr, true);
        ESPUI.setEnabled(txtNetStaSubnetAddr, true);
        ESPUI.setEnabled(txtNetStaGatewayAddr, true);
        if (config.net_ap_standalone == false) {
          ESPUI.setEnabled(numNetApDelay, false);
        }
        break;
  }
}

void cbSwNetApStandalone(Control* sender, int value) {
  switch (value) {
    case S_ACTIVE:
        config.net_ap_standalone = true;
        if (config.net_sta_dhcp == true) {
          ESPUI.setEnabled(numNetApDelay, false);
        }
        break;
    case S_INACTIVE:
        config.net_ap_standalone = false;
        if (config.net_sta_dhcp == true) {
          ESPUI.setEnabled(numNetApDelay, true);
        }
        break;
  }
}

/* -------------- Switch callbacks end -------------- */

/* ---------------- Select callbacks ---------------- */

void cbSelModePortA(Control* sender, int value) {
  config.portA_mode = (uint8_t)(sender->value.toInt());
  if (config.portA_mode == PORT_TYPE_WS2812) {
    // ESPUI.updateVisibility(numPixelsA, true);
    // ESPUI.updateVisibility(selPixelModeA, true);
    // ESPUI.updateVisibility(numPixelFxStartChA, true);
    ESPUI.setEnabled(numPixelsA, true);
    ESPUI.setEnabled(selPixelModeA, true);
    ESPUI.setEnabled(numPixelFxStartChA, true);
  } else {
    // ESPUI.updateVisibility(numPixelsA, false);
    // ESPUI.updateVisibility(selPixelModeA, false);
    // ESPUI.updateVisibility(numPixelFxStartChA, false);
    ESPUI.setEnabled(numPixelsA, false);
    ESPUI.setEnabled(selPixelModeA, false);
    ESPUI.setEnabled(numPixelFxStartChA, false);
  }
}

void cbSelProtPortA(Control* sender, int value) {
  config.portA_prot = (uint8_t)(sender->value.toInt());
}

void cbSelMergePortA(Control* sender, int value) {
  config.portA_merge = (uint8_t)(sender->value.toInt());
}

void cbSelPixelModePortA(Control* sender, int value) {
  config.portA_pixel_mode = (uint8_t)(sender->value.toInt());
}

#ifndef SOLE_OUTPUT
void cbSelModePortB(Control* sender, int value) {
  config.portB_mode = (uint8_t)(sender->value.toInt());
  if (config.portB_mode == PORT_TYPE_WS2812) {
    // ESPUI.updateVisibility(numPixelsB, true);
    // ESPUI.updateVisibility(selPixelModeB, true);
    // ESPUI.updateVisibility(numPixelFxStartChB, true);
    ESPUI.setEnabled(numPixelsB, true);
    ESPUI.setEnabled(selPixelModeB, true);
    ESPUI.setEnabled(numPixelFxStartChB, true);
  } else {
    // ESPUI.updateVisibility(numPixelsB, false);
    // ESPUI.updateVisibility(selPixelModeB, false);
    // ESPUI.updateVisibility(numPixelFxStartChB, false);
    ESPUI.setEnabled(numPixelsB, false);
    ESPUI.setEnabled(selPixelModeB, false);
    ESPUI.setEnabled(numPixelFxStartChB, false);
  }
}

void cbSelProtPortB(Control* sender, int value) {
  config.portB_prot = (uint8_t)(sender->value.toInt());
}

void cbSelMergePortB(Control* sender, int value) {
  config.portB_merge = (uint8_t)(sender->value.toInt());
}

void cbSelPixelModePortB(Control* sender, int value) {
  config.portB_pixel_mode = (uint8_t)(sender->value.toInt());
}
#endif

/* -------------- Select callbacks end -------------- */

/* ----------------- Tab callbacks ----------------- */

void cbNetworkScanComplete(int count) {
  strListNetworksAvail = "";
  for (int i = 0; i < count; i++) {
    strListNetworksAvail = strListNetworksAvail + WiFi.SSID(i) + " (" + WiFi.RSSI(i) + " dBm)" + "<br>";
  }
  ESPUI.updateLabel(lblNetStaNetworksAvail, strListNetworksAvail);
}

void cbTabStation(Control* sender, int type) {
  WiFi.scanNetworksAsync(cbNetworkScanComplete);
}

/* --------------- Tab callbacks end -------------- */

/* ############### Init functions ############### */

void initWebServer() {
  yield();

  // Disable captive portal when access point is not started
  if (accessPointStarted == false) {
    ESPUI.captivePortal = false;
  }
  #ifdef DEBUG
    ESPUI.setVerbosity(Verbosity::VerboseJSON);
  #endif

  ESPUI.jsonUpdateDocumentSize = 2000;
  ESPUI.jsonInitialDocumentSize = 6000; // needs to be reduced in order to get content shown on ESP8266

  // Tabs:
  uint16_t tabHome = ESPUI.addControl(ControlType::Tab, "TbHome", "Home");
  uint16_t tabMonitor = ESPUI.addControl(ControlType::Tab, "TbMonitor", "Monitor");
  uint16_t tabNode = ESPUI.addControl(ControlType::Tab, "TbNode", "Node Config");
  uint16_t tabStation = ESPUI.addControl(ControlType::Tab, "TbStation", "Network Config", ControlColor::None, 0, cbTabStation);
  uint16_t tabAp = ESPUI.addControl(ControlType::Tab, "TbAP", "AP Config");
  uint16_t tabDmxIn = ESPUI.addControl(ControlType::Tab, "TbDmxIn", "DMX Input Config");
  uint16_t tabPortA = ESPUI.addControl(ControlType::Tab, "TbPortA", "Port A");
  #ifndef SOLE_OUTPUT
  uint16_t tabPortB = ESPUI.addControl(ControlType::Tab, "TbPortB", "Port B");
  #endif
  uint16_t tabUpdate = ESPUI.addControl(ControlType::Tab, "TbUpdate", "Update");
  // Home Tab content:
  // Info:
  ESPUI.addControl(ControlType::Label, "Version", configActive.gen_version, ControlColor::Peterriver, tabHome);
  ESPUI.addControl(ControlType::Label, "Node Name", configActive.gen_nodeName, ControlColor::Peterriver, tabHome);
  ESPUI.addControl(ControlType::Label, "MAC Address", macAddr_str, ControlColor::Emerald, tabHome);
  ESPUI.addControl(ControlType::Label, "Wifi Status", wifiStatus, ControlColor::Emerald, tabHome);
  if (configActive.net_ap_standalone == false && configActive.net_sta_ssid[0] != '\0') {
    ESPUI.addControl(ControlType::Label, "IP Address (Station)", configActive.net_sta_ip.toString(), ControlColor::Emerald, tabHome);
  } else {
    ESPUI.addControl(ControlType::Label, "IP Address (Access Point)", configActive.net_ap_ip.toString(), ControlColor::Emerald, tabHome);
  }
  // Actions:
  uint16_t btnGroupActions = ESPUI.addControl(ControlType::Button, "Actions", "Reboot", ControlColor::Alizarin, tabHome, cbBtnReboot);
  ESPUI.addControl(ControlType::Button, "", "Save Config", ControlColor::Alizarin, btnGroupActions, cbBtnSaveConfig);
  ESPUI.addControl(ControlType::Button, "", "Reset Config", ControlColor::Alizarin, btnGroupActions, cbBtnResetConfig);
  // Monitor Tab content:
  numWdtCounter = ESPUI.addControl(ControlType::Number, "Watchdog counter", String(configActive.wdtCounter), ControlColor::Wetasphalt, tabMonitor);
  ESPUI.setEnabled(numWdtCounter, false);
  numRstCounter = ESPUI.addControl(ControlType::Number, "Reset counter", String(configActive.resetCounter), ControlColor::Wetasphalt, tabMonitor);
  ESPUI.setEnabled(numRstCounter, false);
  // Node Config Tab content:
  uint16_t txtNodeName = ESPUI.addControl(ControlType::Text, "Short Name", configActive.gen_nodeName, ControlColor::Peterriver, tabNode, cbTxtNodeName);
	ESPUI.addControl(Max, "", "18", None, txtNodeName);
  uint16_t txtLongName = ESPUI.addControl(ControlType::Text, "Long Name", configActive.gen_longName, ControlColor::Peterriver, tabNode, cbTxtLongName);
  ESPUI.addControl(Max, "", "64", None, txtLongName);
  // Network Config Tab content:
  lblNetStaNetworksAvail = ESPUI.addControl(ControlType::Label, "List of available networks", strListNetworksAvail, ControlColor::Emerald, tabStation);
  uint16_t txtNetStaSSID = ESPUI.addControl(ControlType::Text, "SSID", configActive.net_sta_ssid, ControlColor::Emerald, tabStation, cbTxtNetStaSSID);
  ESPUI.addControl(Max, "", "32", None, txtNetStaSSID);
  uint16_t txtNetStaPwd = ESPUI.addControl(ControlType::Text, "Password", configActive.net_sta_password, ControlColor::Emerald, tabStation, cbTxtNetStaPwd);
  ESPUI.setInputType(txtNetStaPwd, "password");
  ESPUI.addControl(Min, "", "8", None, txtNetStaPwd);
  ESPUI.addControl(Min, "", "64", None, txtNetStaPwd);
  ESPUI.addControl(ControlType::Switcher, "DHCP", configActive.net_sta_dhcp ? "1" : "0", ControlColor::Emerald, tabStation, cbSwNetStaDHCP);
  txtNetStaIPAddr = ESPUI.addControl(ControlType::Text, "IP Address", configActive.net_sta_ip.toString(), ControlColor::Emerald, tabStation, cbTxtNetStaIP);
  txtNetStaSubnetAddr = ESPUI.addControl(ControlType::Text, "Subnet Address", configActive.net_sta_subnet.toString(), ControlColor::Emerald, tabStation, cbTxtNetStaSubnet);
  txtNetStaGatewayAddr = ESPUI.addControl(ControlType::Text, "Gateway Address", configActive.net_sta_gateway.toString(), ControlColor::Emerald, tabStation, cbTxtNetStaGateway);
  lblNetStaBroadcast = ESPUI.addControl(ControlType::Label, "Broadcast Address", configActive.net_sta_broadcast.toString(), ControlColor::Emerald, tabStation);
  if (configActive.net_sta_dhcp == true) {
    ESPUI.setEnabled(txtNetStaIPAddr, false);
    ESPUI.setEnabled(txtNetStaSubnetAddr, false);
    ESPUI.setEnabled(txtNetStaGatewayAddr, false);
  }
  // AP Config Tab content:
  uint16_t txtNetApSSID = ESPUI.addControl(ControlType::Text, "SSID", configActive.net_ap_ssid, ControlColor::Turquoise, tabAp, cbTxtNetApSSID);
  ESPUI.addControl(Max, "", "32", None, txtNetApSSID);
  uint16_t txtNetApPwd = ESPUI.addControl(ControlType::Text, "Password", configActive.net_ap_password, ControlColor::Turquoise, tabAp, cbTxtNetApPwd);
  ESPUI.setInputType(txtNetApPwd, "password");
  ESPUI.addControl(Min, "", "8", None, txtNetApPwd);
  ESPUI.addControl(Min, "", "64", None, txtNetApPwd);
  ESPUI.addControl(ControlType::Text, "IP Address", configActive.net_ap_ip.toString(), ControlColor::Turquoise, tabAp, cbTxtNetApIP);
  ESPUI.addControl(ControlType::Text, "Subnet Address", configActive.net_ap_subnet.toString(), ControlColor::Turquoise, tabAp, cbTxtNetApSubnet);
  lblNetApBroadcast = ESPUI.addControl(ControlType::Label, "Broadcast Address", configActive.net_ap_broadcast.toString(), ControlColor::Turquoise, tabAp);
  ESPUI.addControl(ControlType::Switcher, "Standalone", configActive.net_ap_standalone ? "1" : "0", ControlColor::Turquoise, tabAp, cbSwNetApStandalone);
  numNetApDelay = ESPUI.addControl(ControlType::Number, "Start Delay [s]", String(configActive.net_ap_delay), ControlColor::Turquoise, tabAp, cbNumNetApDelay);
  ESPUI.addControl(Min, "", "0", None, numNetApDelay);
	ESPUI.addControl(Max, "", "180", None, numNetApDelay);
  if ((configActive.net_ap_standalone == true) || (configActive.net_sta_dhcp == false)) {
    ESPUI.setEnabled(numNetApDelay, false);
  }
  // DMX Input Config Tab content:
  ESPUI.addControl(ControlType::Text, "Broadcast Address", configActive.net_dmxIn_broadcast.toString(), ControlColor::Dark, tabDmxIn, cbTxtNetDmxIn);
  yield();
  // Port A Tab content:
  uint16_t selModeA = ESPUI.addControl(ControlType::Select, "Mode", String(configActive.portA_mode), ControlColor::Sunflower, tabPortA, cbSelModePortA);
  ESPUI.addControl(ControlType::Option, "DMX Output", "0", ControlColor::None, selModeA);
  ESPUI.addControl(ControlType::Option, "DMX Output with RDM", "1", ControlColor::None, selModeA);
  ESPUI.addControl(ControlType::Option, "DMX Input", "2", ControlColor::None, selModeA);
  ESPUI.addControl(ControlType::Option, "LED Pixels - WS2812", "3", ControlColor::None, selModeA);
  uint16_t selProtA = ESPUI.addControl(ControlType::Select, "Protocol", String(configActive.portA_prot), ControlColor::Sunflower, tabPortA, cbSelProtPortA);
  ESPUI.addControl(ControlType::Option, "Artnet v4", "0", ControlColor::None, selProtA);
  ESPUI.addControl(ControlType::Option, "Artnet v4 with sACN DMX", "1", ControlColor::None, selProtA);
  uint16_t selMergeA = ESPUI.addControl(ControlType::Select, "Merge Mode", String(configActive.portA_merge), ControlColor::Sunflower, tabPortA, cbSelMergePortA);
  ESPUI.addControl(ControlType::Option, "Merge LTP", "0", ControlColor::None, selMergeA);
  ESPUI.addControl(ControlType::Option, "Merge HTP", "1", ControlColor::None, selMergeA);
  uint16_t numNetA = ESPUI.addControl(ControlType::Number, "Net", String(configActive.portA_net), ControlColor::Sunflower, tabPortA, cbNumNetPortA);
  ESPUI.addControl(Min, "", "0", None, numNetA);
	ESPUI.addControl(Max, "", "127", None, numNetA);
  uint16_t numSubnetA = ESPUI.addControl(ControlType::Number, "Subnet", String(configActive.portA_subnet), ControlColor::Sunflower, tabPortA, cbNumSubnetPortA);
  ESPUI.addControl(Min, "", "0", None, numSubnetA);
	ESPUI.addControl(Max, "", "15", None, numSubnetA);
  uint16_t numGroupUniverseA = ESPUI.addControl(ControlType::Number, "Universe", String(configActive.portA_uni[0]), ControlColor::Sunflower, tabPortA, cbNumUniversePortA, (void*)0);
	ESPUI.addControl(Min, "", "0", None, numGroupUniverseA);
	ESPUI.addControl(Max, "", "15", None, numGroupUniverseA);
  uint16_t numUniverseA1 = ESPUI.addControl(ControlType::Number, "", String(configActive.portA_uni[1]), ControlColor::Sunflower, numGroupUniverseA, cbNumUniversePortA, (void*)1);
	ESPUI.addControl(Min, "", "0", None, numUniverseA1);
	ESPUI.addControl(Max, "", "15", None, numUniverseA1);
  uint16_t numUniverseA2 = ESPUI.addControl(ControlType::Number, "", String(configActive.portA_uni[2]), ControlColor::Sunflower, numGroupUniverseA, cbNumUniversePortA, (void*)2);
	ESPUI.addControl(Min, "", "0", None, numUniverseA2);
	ESPUI.addControl(Max, "", "15", None, numUniverseA2);
  uint16_t numUniverseA3 = ESPUI.addControl(ControlType::Number, "", String(configActive.portA_uni[3]), ControlColor::Sunflower, numGroupUniverseA, cbNumUniversePortA, (void*)3);
  ESPUI.addControl(Min, "", "0", None, numUniverseA3);
	ESPUI.addControl(Max, "", "15", None, numUniverseA3);
  uint16_t numGroupSACNUniverseA = ESPUI.addControl(ControlType::Number, "sACN Universe", String(configActive.portA_SACNuni[0]), ControlColor::Sunflower, tabPortA, cbNumSACNUniversePortA, (void*)0);
	ESPUI.addControl(Min, "", "0", None, numGroupSACNUniverseA);
	ESPUI.addControl(Max, "", "63999", None, numGroupSACNUniverseA);
  uint16_t numSACNUniverseA1 = ESPUI.addControl(ControlType::Number, "", String(configActive.portA_SACNuni[1]), ControlColor::Sunflower, numGroupSACNUniverseA, cbNumSACNUniversePortA, (void*)1);
	ESPUI.addControl(Min, "", "1", None, numSACNUniverseA1);
	ESPUI.addControl(Max, "", "63999", None, numSACNUniverseA1);
  uint16_t numSACNUniverseA2 = ESPUI.addControl(ControlType::Number, "", String(configActive.portA_SACNuni[2]), ControlColor::Sunflower, numGroupSACNUniverseA, cbNumSACNUniversePortA, (void*)2);
	ESPUI.addControl(Min, "", "1", None, numSACNUniverseA2);
	ESPUI.addControl(Max, "", "63999", None, numSACNUniverseA2);
  uint16_t numSACNUniverseA3 = ESPUI.addControl(ControlType::Number, "", String(configActive.portA_SACNuni[3]), ControlColor::Sunflower, numGroupSACNUniverseA, cbNumSACNUniversePortA, (void*)3);
  ESPUI.addControl(Min, "", "1", None, numSACNUniverseA3);
	ESPUI.addControl(Max, "", "63999", None, numSACNUniverseA3);
  numPixelsA = ESPUI.addControl(ControlType::Number, "Number of pixels (680 max - 170 per universe)", String(configActive.portA_pixel_count), ControlColor::Sunflower, tabPortA, cbNumPixelsPortA);
  ESPUI.addControl(Min, "", "0", None, numPixelsA);
	ESPUI.addControl(Max, "", "680", None, numPixelsA);
  selPixelModeA = ESPUI.addControl(ControlType::Select, "Pixel mode", String(configActive.portA_pixel_mode), ControlColor::Sunflower, tabPortA, cbSelPixelModePortA);
  ESPUI.addControl(ControlType::Option, "Pixel Mapping", "0", ControlColor::None, selPixelModeA);
  ESPUI.addControl(ControlType::Option, "12 Channel FX", "1", ControlColor::None, selPixelModeA);
  numPixelFxStartChA = ESPUI.addControl(ControlType::Number, "Start channel (FX modes only)", String(configActive.portA_pixel_startFX), ControlColor::Sunflower, tabPortA, cbNumPixelFxStartChPortA);
  ESPUI.addControl(Min, "", "1", None, numPixelFxStartChA);
	ESPUI.addControl(Max, "", "501", None, numPixelFxStartChA);
  if (configActive.portA_mode != PORT_TYPE_WS2812) {
    // ESPUI.updateVisibility(numPixelsA, false);
    // ESPUI.updateVisibility(selPixelModeA, false);
    // ESPUI.updateVisibility(numPixelFxStartChA, false);
    ESPUI.setEnabled(numPixelsA, false);
    ESPUI.setEnabled(selPixelModeA, false);
    ESPUI.setEnabled(numPixelFxStartChA, false);
  }
  yield();
  // Port B Tab content:
  #ifndef SOLE_OUTPUT
  uint16_t selModeB = ESPUI.addControl(ControlType::Select, "Mode", String(configActive.portB_mode), ControlColor::Carrot, tabPortB, cbSelModePortB);
  ESPUI.addControl(ControlType::Option, "DMX Output", "0", ControlColor::None, selModeB);
  ESPUI.addControl(ControlType::Option, "DMX Output with RDM", "1", ControlColor::None, selModeB);
  // ESPUI.addControl(ControlType::Option, "DMX Input", "2", ControlColor::None, selModeB); // Not possible, so disable it
  ESPUI.addControl(ControlType::Option, "LED Pixels - WS2812", "3", ControlColor::None, selModeB);
  uint16_t selProtB = ESPUI.addControl(ControlType::Select, "Protocol", String(configActive.portB_prot), ControlColor::Carrot, tabPortB, cbSelProtPortB);
  ESPUI.addControl(ControlType::Option, "Artnet v4", "0", ControlColor::None, selProtB);
  ESPUI.addControl(ControlType::Option, "Artnet v4 with sACN DMX", "1", ControlColor::None, selProtB);
  uint16_t selMergeB = ESPUI.addControl(ControlType::Select, "Merge Mode", String(configActive.portB_merge), ControlColor::Carrot, tabPortB, cbSelMergePortB);
  ESPUI.addControl(ControlType::Option, "Merge LTP", "0", ControlColor::None, selMergeB);
  ESPUI.addControl(ControlType::Option, "Merge HTP", "1", ControlColor::None, selMergeB);
  uint16_t numNetB = ESPUI.addControl(ControlType::Number, "Net", String(configActive.portB_net), ControlColor::Carrot, tabPortB, cbNumNetPortB);
  ESPUI.addControl(Min, "", "0", None, numNetB);
	ESPUI.addControl(Max, "", "127", None, numNetB);
  uint16_t numSubnetB = ESPUI.addControl(ControlType::Number, "Subnet", String(configActive.portB_subnet), ControlColor::Carrot, tabPortB, cbNumSubnetPortB);
  ESPUI.addControl(Min, "", "0", None, numSubnetB);
	ESPUI.addControl(Max, "", "15", None, numSubnetB);
  uint16_t numGroupUniverseB = ESPUI.addControl(ControlType::Number, "Universe", String(configActive.portB_uni[0]), ControlColor::Carrot, tabPortB, cbNumUniversePortB, (void*)0);
  ESPUI.addControl(Min, "", "0", None, numGroupUniverseB);
	ESPUI.addControl(Max, "", "15", None, numGroupUniverseB);
	uint16_t numUniverseB1 = ESPUI.addControl(ControlType::Number, "", String(configActive.portB_uni[1]), ControlColor::Carrot, numGroupUniverseB, cbNumUniversePortB, (void*)1);
  ESPUI.addControl(Min, "", "0", None, numUniverseB1);
	ESPUI.addControl(Max, "", "15", None, numUniverseB1);
	uint16_t numUniverseB2 = ESPUI.addControl(ControlType::Number, "", String(configActive.portB_uni[2]), ControlColor::Carrot, numGroupUniverseB, cbNumUniversePortB, (void*)2);
  ESPUI.addControl(Min, "", "0", None, numUniverseB2);
	ESPUI.addControl(Max, "", "15", None, numUniverseB2);
	uint16_t numUniverseB3 = ESPUI.addControl(ControlType::Number, "", String(configActive.portB_uni[3]), ControlColor::Carrot, numGroupUniverseB, cbNumUniversePortB, (void*)3);
  ESPUI.addControl(Min, "", "0", None, numUniverseB3);
	ESPUI.addControl(Max, "", "15", None, numUniverseB3);
  uint16_t numGroupSACNUniverseB = ESPUI.addControl(ControlType::Number, "sACN Universe", String(configActive.portB_SACNuni[0]), ControlColor::Carrot, tabPortB, cbNumSACNUniversePortB, (void*)0);
	ESPUI.addControl(Min, "", "0", None, numGroupSACNUniverseB);
	ESPUI.addControl(Max, "", "63999", None, numGroupSACNUniverseB);
  uint16_t numSACNUniverseB1 = ESPUI.addControl(ControlType::Number, "", String(configActive.portB_SACNuni[1]), ControlColor::Carrot, numGroupSACNUniverseB, cbNumSACNUniversePortB, (void*)1);
	ESPUI.addControl(Min, "", "1", None, numSACNUniverseB1);
	ESPUI.addControl(Max, "", "63999", None, numSACNUniverseB1);
  uint16_t numSACNUniverseB2 = ESPUI.addControl(ControlType::Number, "", String(configActive.portB_SACNuni[2]), ControlColor::Carrot, numGroupSACNUniverseB, cbNumSACNUniversePortB, (void*)2);
	ESPUI.addControl(Min, "", "1", None, numSACNUniverseB2);
	ESPUI.addControl(Max, "", "63999", None, numSACNUniverseB2);
  uint16_t numSACNUniverseB3 = ESPUI.addControl(ControlType::Number, "", String(configActive.portB_SACNuni[3]), ControlColor::Carrot, numGroupSACNUniverseB, cbNumSACNUniversePortB, (void*)3);
  ESPUI.addControl(Min, "", "1", None, numSACNUniverseB3);
	ESPUI.addControl(Max, "", "63999", None, numSACNUniverseB3);
  numPixelsB = ESPUI.addControl(ControlType::Number, "Number of pixels (680 max - 170 per universe)", String(configActive.portB_pixel_count), ControlColor::Carrot, tabPortB, cbNumPixelsPortB);
  ESPUI.addControl(Min, "", "0", None, numPixelsB);
	ESPUI.addControl(Max, "", "680", None, numPixelsB);
  selPixelModeB = ESPUI.addControl(ControlType::Select, "Pixel mode", String(configActive.portB_pixel_mode), ControlColor::Carrot, tabPortB, cbSelPixelModePortB);
  ESPUI.addControl(ControlType::Option, "Pixel Mapping", "0", ControlColor::None, selPixelModeB);
  ESPUI.addControl(ControlType::Option, "12 Channel FX", "1", ControlColor::None, selPixelModeB);
  numPixelFxStartChB = ESPUI.addControl(ControlType::Number, "Start channel (FX modes only)", String(configActive.portB_pixel_startFX), ControlColor::Carrot, tabPortB, cbNumPixelFxStartChPortB);
  ESPUI.addControl(Min, "", "1", None, numPixelFxStartChB);
	ESPUI.addControl(Max, "", "501", None, numPixelFxStartChB);
  if (configActive.portB_mode != PORT_TYPE_WS2812) {
    // ESPUI.updateVisibility(numPixelsB, false);
    // ESPUI.updateVisibility(selPixelModeB, false);
    // ESPUI.updateVisibility(numPixelFxStartChB, false);
    ESPUI.setEnabled(numPixelsB, false);
    ESPUI.setEnabled(selPixelModeB, false);
    ESPUI.setEnabled(numPixelFxStartChB, false);
  }
  yield();
  #endif
  // Update Tab content:
  uint16_t lblUpdateHint = ESPUI.addControl(ControlType::Label, "Info", "For doing OTA updates, go to [node-ip]/update.", ControlColor::None, tabUpdate);
  ESPUI.setPanelWide(lblUpdateHint, true);

  // Initialize WebServer:
  ESPUI.begin(" ArtNetNode by DJ Maxergy (PB Matthew Tong) ");
  yield();

  // Add OTA Update requests:
  ESPUI.server->on("/update", HTTP_GET, [](AsyncWebServerRequest *request){handleUpdate(request);});
  ESPUI.server->on("/doUpdate", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data,
                  size_t len, bool final) {handleDoUpdate(request, filename, index, data, len, final);}
  );

  yield();
}

void convertMACtoStr(uint8_t* macArrayPtr, char* macStringPtr) {
  sprintf(macStringPtr, "%2X:%2X:%2X:%2X:%2X:%2X", macArrayPtr[0], macArrayPtr[1], macArrayPtr[2], macArrayPtr[3], macArrayPtr[4], macArrayPtr[5]);
}

void initAP() {
  yield();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(configActive.net_ap_ip, configActive.net_ap_ip, configActive.net_ap_subnet);
  WiFi.softAP(configActive.net_ap_ssid, configActive.net_ap_password);
  WiFi.softAPSSID().toCharArray(configActive.net_ap_ssid, sizeof(configActive.net_ap_ssid));
  WiFi.macAddress(macAddr);
  convertMACtoStr(macAddr, macAddr_str);

  // Set broadcast address just for information on website
  configActive.net_ap_broadcast = {
    (uint8_t)((~configActive.net_ap_subnet[0]) | (configActive.net_ap_ip[0] & configActive.net_ap_subnet[0])), 
    (uint8_t)((~configActive.net_ap_subnet[1]) | (configActive.net_ap_ip[1] & configActive.net_ap_subnet[1])), 
    (uint8_t)((~configActive.net_ap_subnet[2]) | (configActive.net_ap_ip[2] & configActive.net_ap_subnet[2])), 
    (uint8_t)((~configActive.net_ap_subnet[3]) | (configActive.net_ap_ip[3] & configActive.net_ap_subnet[3]))  
  };

  sprintf(wifiStatus, "Access Point started.<br />\nSSID: %s", configActive.net_ap_ssid);
  accessPointStarted = true;

  // Start mDNS for captive portal (only relevant for access point)
  dnsServer.start(DNS_PORT, "*", configActive.net_ap_ip);

  if (configActive.net_ap_standalone == true) {
    return;
  } else {
    // Start web server
    initWebServer();

    unsigned long timeout = millis() + TIMEOUT_AP;
    // Stay here if not in stand alone mode - no dmx or artnet
    while (timeout > millis() || WiFi.softAPgetStationNum() > 0) {
      dnsServer.processNextRequest();
      yield();
    }

    accessPointStarted = false;
    delay(500);
    ESP.restart();
  }
}

void initWifi() {
  wifi_set_sleep_type(NONE_SLEEP_T);

  // If it's the default WiFi Access Point SSID or it's empty, make it unique:
  if (strcmp(configActive.net_ap_ssid, CONF_NET_AP_SSID_DEF) == 0 || configActive.net_ap_ssid[0] == '\0') {
    sprintf(configActive.net_ap_ssid, "%s_%05u", CONF_NET_AP_SSID_DEF, (ESP.getChipId() & 0xFF));
  }
  
  if (configActive.net_ap_standalone == true) {
    // If standalone mode is activated, directly start access point:
    initAP();
  } else if (configActive.net_sta_ssid[0] != '\0') {
    // If WiFi station SSID is configured, start WiFi station:
    WiFi.mode(WIFI_STA);
    WiFi.hostname(configActive.gen_nodeName);
    WiFi.begin(configActive.net_sta_ssid, configActive.net_sta_password);
    WiFi.SSID().toCharArray(configActive.net_sta_ssid, sizeof(configActive.net_sta_ssid));
    WiFi.macAddress(macAddr);
    convertMACtoStr(macAddr, macAddr_str);
    
    unsigned long timeout = millis() + (configActive.net_ap_delay * 1000);
    if (configActive.net_sta_dhcp == true) {
      // WiFi station is configured for DHCP -> wait for established connection:
      while (WiFi.status() != WL_CONNECTED && timeout > millis()) {
        yield();
      }
      
      if (millis() >= timeout) {
        // If timeout reached, start Access Point:
        initAP();
      } else {
        // WiFi station is connected:
        sprintf(wifiStatus, "Wifi connected (DHCP).<br />SSID: %s", configActive.net_sta_ssid);

        configActive.net_sta_ip = WiFi.localIP();
        configActive.net_sta_subnet = WiFi.subnetMask();
        if (configActive.net_sta_gateway == INADDR_NONE)
          configActive.net_sta_gateway = WiFi.gatewayIP();
        configActive.net_sta_broadcast = {
          (uint8_t)((~configActive.net_sta_subnet[0]) | (configActive.net_sta_ip[0] & configActive.net_sta_subnet[0])), 
          (uint8_t)((~configActive.net_sta_subnet[1]) | (configActive.net_sta_ip[1] & configActive.net_sta_subnet[1])), 
          (uint8_t)((~configActive.net_sta_subnet[2]) | (configActive.net_sta_ip[2] & configActive.net_sta_subnet[2])), 
          (uint8_t)((~configActive.net_sta_subnet[3]) | (configActive.net_sta_ip[3] & configActive.net_sta_subnet[3]))  
        };
      }
    } else {
      WiFi.config(configActive.net_sta_ip, configActive.net_sta_gateway, configActive.net_sta_subnet);
      sprintf(wifiStatus, "Wifi connected (non-DHCP).<br />SSID: %s", configActive.net_sta_ssid);
    }
  } else {
    // No WiFi station SSID configured, start access point:
    initAP();
  }

  yield();
}

void cbArtDmxReceive(uint8_t group, uint8_t port, uint16_t numChans, bool syncEnabled) {
  uint16_t maxChans = 3*LIM_PIXELS_PER_ARTNET_PORT;
  uint8_t* dmxData = artRDM.getDMX(group, port);
  uint16_t pixStartIndex = port * maxChans;
  uint16_t pixEndIndex = 0;
  uint16_t indexDmxData = 0;

  artDmxReceiving = true;

  if (portA[0] == group) {
    // WS2812 mode:
    if (configActive.portA_mode == PORT_TYPE_WS2812) {
      statusStrip.SetPixelColor(ADDR_STATUS_LED_A, blue); //green

      // FX MAP mode:
      if (configActive.portA_pixel_mode == PIXEL_FX_MODE_MAP) {
        // Limit channels:
        if (numChans > maxChans) {
          numChans = maxChans;
        }
        
        // Copy DMX data to the pixels buffer:
        pixEndIndex = (pixStartIndex + numChans) / 3;
        indexDmxData = 0;
        if (pixPortA != NULL) {
          for (uint16_t i = pixStartIndex; i < pixEndIndex; i++) {
            pixPortA->SetPixelColor(i, RgbColor(dmxData[indexDmxData], dmxData[indexDmxData + 1], dmxData[indexDmxData + 2]));
            indexDmxData = indexDmxData + 3;
          }
        }
        
        // Output to pixel strip:
        if (!syncEnabled) {
          pixDone = false;
        }

        return;

      // FX 12 Mode:
      } else if (port == portA[1]) {
          uint16_t pixFXAddr = configActive.portA_pixel_startFX - 1;
          
          pixFXA->Intensity = dmxData[pixFXAddr + 0];
          pixFXA->setFX(dmxData[pixFXAddr + 1]);
          pixFXA->setSpeed(dmxData[pixFXAddr + 2]);
          pixFXA->Pos = dmxData[pixFXAddr + 3];
          pixFXA->Size = dmxData[pixFXAddr + 4];
          pixFXA->setColour1((dmxData[pixFXAddr + 5] << 16) | (dmxData[pixFXAddr + 6] << 8) | dmxData[pixFXAddr + 7]);
          pixFXA->setColour2((dmxData[pixFXAddr + 8] << 16) | (dmxData[pixFXAddr + 9] << 8) | dmxData[pixFXAddr + 10]);
          pixFXA->Size1 = dmxData[pixFXAddr + 11];
          //pixFXA->Fade = dmxData[pixFXAddr + 12];
          pixFXA->NewData = 1;
      }
    // DMX modes:
    } else if (configActive.portA_mode != PORT_TYPE_DMX_IN && port == portA[1]) {
      dmxA.chanUpdate(numChans);
      statusStrip.SetPixelColor(ADDR_STATUS_LED_A, blue);
    }
  #ifndef SOLE_OUTPUT
  } else if (portB[0] == group) {
    /* if (port == portB[1]) {
      dmxB.chanUpdate(numChans);
      statusStrip.SetPixelColor(ADDR_STATUS_LED_B, blue);
    } */
    // WS2812 mode:
    if (configActive.portB_mode == PORT_TYPE_WS2812) {
      statusStrip.SetPixelColor(ADDR_STATUS_LED_B, blue); //green
      
      // FX MAP mode:
      if (configActive.portB_pixel_mode == PIXEL_FX_MODE_MAP) {
        // Limit channels:
        if (numChans > maxChans) {
          numChans = maxChans;
        }
        
        // Copy DMX data to the pixels buffer:
        pixEndIndex = (pixStartIndex + numChans) / 3;
        indexDmxData = 0;
        if (pixPortB != NULL) {
          for (uint16_t i = pixStartIndex; i < pixEndIndex; i++) {
            pixPortB->SetPixelColor(i, RgbColor(dmxData[indexDmxData], dmxData[indexDmxData + 1], dmxData[indexDmxData + 2]));
            indexDmxData = indexDmxData + 3;
          }
        }
        
        // Output to pixel strip:
        if (!syncEnabled) {
          pixDone = false;
        }

        return;

      // FX 12 mode:
      } else if (port == portB[1]) {
        uint16_t pixFXAddr = configActive.portB_pixel_startFX - 1;
        
        pixFXB->Intensity = dmxData[pixFXAddr + 0];
        pixFXB->setFX(dmxData[pixFXAddr + 1]);
        pixFXB->setSpeed(dmxData[pixFXAddr + 2]);
        pixFXB->Pos = dmxData[pixFXAddr + 3];
        pixFXB->Size = dmxData[pixFXAddr + 4];
        pixFXB->setColour1((dmxData[pixFXAddr + 5] << 16) | (dmxData[pixFXAddr + 6] << 8) | dmxData[pixFXAddr + 7]);
        pixFXB->setColour2((dmxData[pixFXAddr + 8] << 16) | (dmxData[pixFXAddr + 9] << 8) | dmxData[pixFXAddr + 10]);
        pixFXB->Size1 = dmxData[pixFXAddr + 11];
        //pixFXB->Fade = dmxData[pixFXAddr + 12];
        pixFXB->NewData = 1;
      }
    // DMX modes:
    } else if (configActive.portB_mode != PORT_TYPE_DMX_IN && port == portB[1]) {
      dmxB.chanUpdate(numChans);
      statusStrip.SetPixelColor(ADDR_STATUS_LED_B, blue);
    }
  #endif
  }
}

void cbArtRdmReceive(uint8_t group, uint8_t port, rdm_data* c) {
  if (portA[0] == group && portA[1] == port)
    dmxA.rdmSendCommand(c);
  #ifndef SOLE_OUTPUT
    else if (portB[0] == group && portB[1] == port)
      dmxB.rdmSendCommand(c);
  #endif
}

void cbArtSync() {
  #ifdef SOLE_OUTPUT
  if (configActive.portA_mode == PORT_TYPE_WS2812) {
    rdmPause(1);
    // pixDone = pixDriver.show();
    if (pixPortA != NULL) {
      pixPortA->Show();
      pixDone = pixPortA->CanShow();
    }
    rdmPause(0);
  } else if (configActive.portA_mode != PORT_TYPE_DMX_IN) {
    dmxA.unPause();
  }
  #else
  if (configActive.portA_mode == PORT_TYPE_WS2812 || configActive.portB_mode == PORT_TYPE_WS2812) {
    rdmPause(1);
    if (pixPortA != NULL) {
      pixPortA->Show();
      pixDone = pixPortA->CanShow();
    }
    if (pixPortB != NULL) {
      pixPortB->Show();
      pixDone |= pixPortB->CanShow();
    }
    rdmPause(0);
  } else {
    if (configActive.portA_mode != PORT_TYPE_DMX_IN) {
      dmxA.unPause();
    }
    if (configActive.portB_mode != PORT_TYPE_DMX_IN) {
      dmxB.unPause();
    }
  }
  #endif
}

void cbArtIpChanged() {
  if (artRDM.getDHCP()) {
    config.net_sta_dhcp = true;
    config.net_sta_gateway = INADDR_NONE;
  } else {
    config.net_sta_dhcp = false;
    config.net_sta_ip = artRDM.getIP();
    config.net_sta_subnet = artRDM.getSubnetMask();
    config.net_sta_gateway = config.net_sta_ip;
    config.net_sta_gateway[3] = 1;
    config.net_sta_broadcast = {
      (uint8_t)((~config.net_sta_subnet[0]) | (config.net_sta_ip[0] & config.net_sta_subnet[0])), 
      (uint8_t)((~config.net_sta_subnet[1]) | (config.net_sta_ip[1] & config.net_sta_subnet[1])), 
      (uint8_t)((~config.net_sta_subnet[2]) | (config.net_sta_ip[2] & config.net_sta_subnet[2])), 
      (uint8_t)((~config.net_sta_subnet[3]) | (config.net_sta_ip[3] & config.net_sta_subnet[3]))  
    };
  }
  
  // Store new network configuration:
  config_save();

  doReboot = true;
}

void cbArtAddressChanged() {
  memcpy(&config.gen_nodeName, artRDM.getShortName(), ARTNET_SHORT_NAME_LENGTH);
  memcpy(&config.gen_longName, artRDM.getLongName(), ARTNET_LONG_NAME_LENGTH);

  // Port A:
  config.portA_net = artRDM.getNet(portA[0]);
  config.portA_subnet = artRDM.getSubNet(portA[0]);
  config.portA_uni[0] = artRDM.getUni(portA[0], portA[1]);
  config.portA_merge = artRDM.getMerge(portA[0], portA[1]);
  if (artRDM.getE131(portA[0], portA[1]) == true) {
    config.portA_prot = PORT_PROT_ARTNET_SACN;
  } else {
    config.portA_prot = PORT_PROT_ARTNET;
  }

  // Port B:
  #ifndef SOLE_OUTPUT
    config.portB_net = artRDM.getNet(portB[0]);
    config.portB_subnet = artRDM.getSubNet(portB[0]);
    config.portB_uni[0] = artRDM.getUni(portB[0], portB[1]);
    config.portB_merge = artRDM.getMerge(portB[0], portB[1]);
    if (artRDM.getE131(portB[0], portB[1]) == true) {
      config.portB_prot = PORT_PROT_ARTNET_SACN;
    } else {
      config.portB_prot = PORT_PROT_ARTNET;
    }
  #endif
  
  // Store new address configuration:
  config_save();
}

void cbDmxSendTodA() {
  artRDM.artTODData(portA[0], portA[1], dmxA.todMan(), dmxA.todDev(), dmxA.todCount(), dmxA.todStatus());
}

void cbDmxRdmReceiveA(rdm_data* c) {
  artRDM.rdmResponse(c, portA[0], portA[1]);
}

#ifndef SOLE_OUTPUT
void cbDmxSendTodB() {
  artRDM.artTODData(portB[0], portB[1], dmxB.todMan(), dmxB.todDev(), dmxB.todCount(), dmxB.todStatus());
}

void cbDmxRdmReceiveB(rdm_data* c) {
  artRDM.rdmResponse(c, portB[0], portB[1]);
}
#endif

void cbArtTodRequest(uint8_t group, uint8_t port) {
  if (portA[0] == group && portA[1] == port)
    artRDM.artTODData(portA[0], portA[1], dmxA.todMan(), dmxA.todDev(), dmxA.todCount(), dmxA.todStatus());
  #ifndef SOLE_OUTPUT
    else if (portB[0] == group && portB[1] == port)
      artRDM.artTODData(portB[0], portB[1], dmxB.todMan(), dmxB.todDev(), dmxB.todCount(), dmxB.todStatus());
  #endif
}

void cbArtTodFlush(uint8_t group, uint8_t port) {
  if (portA[0] == group && portA[1] == port)
    dmxA.rdmDiscovery();
  #ifndef SOLE_OUTPUT
    else if (portB[0] == group && portB[1] == port)
      dmxB.rdmDiscovery();
  #endif
}

void cbDmxInputReceive(uint16_t num) {
  // Double buffer switch
  byte* tmp = dataIn;
  dataIn = dmxA.getChans();
  dmxA.setBuffer(tmp);
  
  newDmxIn = true;
}

void initArtnet() {
  bool useE131 = false;

  // Initialize ArtNet:
  if (accessPointStarted)
    artRDM.init(configActive.net_ap_ip, configActive.net_ap_subnet, true, configActive.gen_nodeName, configActive.gen_longName, CONF_ARTNET_OEM, CONF_ESTA_MAN, macAddr);
  else
    artRDM.init(configActive.net_sta_ip, configActive.net_sta_subnet, configActive.net_sta_dhcp, configActive.gen_nodeName, configActive.gen_longName, CONF_ARTNET_OEM, CONF_ESTA_MAN, macAddr);

  // Set firmware:
  artRDM.setFirmwareVersion(CONF_ART_FIRM_VER);

  // ----- Port A -----
  // Add Group:
  portA[0] = artRDM.addGroup(configActive.portA_net, configActive.portA_subnet);

  // Add Port:
  // WS2812 uses PORT_TYPE_DMX_OUT - the rest use the value assigned
  if (configActive.portA_mode == PORT_TYPE_WS2812)
    portA[1] = artRDM.addPort(portA[0], 0, configActive.portA_uni[0], PORT_TYPE_DMX_OUT, configActive.portA_merge);
  else
    portA[1] = artRDM.addPort(portA[0], 0, configActive.portA_uni[0], configActive.portA_mode, configActive.portA_merge);

  // Set E131:
  useE131 = (configActive.portA_prot == PORT_PROT_ARTNET_SACN) ? true : false;
  artRDM.setE131(portA[0], portA[1], useE131);
  artRDM.setE131Uni(portA[0], portA[1], configActive.portA_SACNuni[0]);

  // Add extra Artnet ports for WS2812:
  if (configActive.portA_mode == PORT_TYPE_WS2812 && configActive.portA_pixel_mode == PIXEL_FX_MODE_MAP) {
    if (configActive.portA_pixel_count > LIM_PIXELS_PER_ARTNET_PORT) {
      portA[2] = artRDM.addPort(portA[0], 1, configActive.portA_uni[1], PORT_TYPE_DMX_OUT, configActive.portA_merge);
      artRDM.setE131(portA[0], portA[2], useE131);
      artRDM.setE131Uni(portA[0], portA[2], configActive.portA_SACNuni[1]);
    }
    if (configActive.portA_pixel_count > (2*LIM_PIXELS_PER_ARTNET_PORT)) {
      portA[3] = artRDM.addPort(portA[0], 2, configActive.portA_uni[2], PORT_TYPE_DMX_OUT, configActive.portA_merge);
      artRDM.setE131(portA[0], portA[3], useE131);
      artRDM.setE131Uni(portA[0], portA[3], configActive.portA_SACNuni[2]);
    }
    if (configActive.portA_pixel_count > (3*LIM_PIXELS_PER_ARTNET_PORT)) {
      portA[4] = artRDM.addPort(portA[0], 3, configActive.portA_uni[3], PORT_TYPE_DMX_OUT, configActive.portA_merge);
      artRDM.setE131(portA[0], portA[4], useE131);
      artRDM.setE131Uni(portA[0], portA[4], configActive.portA_SACNuni[3]);
    }
  }

  // ----- Port B -----
  #ifndef SOLE_OUTPUT
    // Add Group:
    portB[0] = artRDM.addGroup(configActive.portB_net, configActive.portB_subnet);

    // Add Port:
    // WS2812 uses PORT_TYPE_DMX_OUT - the rest use the value assigned
    if (configActive.portB_mode == PORT_TYPE_WS2812)
      portB[1] = artRDM.addPort(portB[0], 0, configActive.portB_uni[0], PORT_TYPE_DMX_OUT, configActive.portB_merge);
    else
      portB[1] = artRDM.addPort(portB[0], 0, configActive.portB_uni[0], configActive.portB_mode, configActive.portB_merge);

    // Set E131:
    useE131 = (configActive.portB_prot == PORT_PROT_ARTNET_SACN) ? true : false;
    artRDM.setE131(portB[0], portB[1], useE131);
    artRDM.setE131Uni(portB[0], portB[1], configActive.portB_SACNuni[0]);

    // Add extra Artnet ports for WS2812:
    if (configActive.portB_mode == PORT_TYPE_WS2812 && configActive.portB_pixel_mode == PIXEL_FX_MODE_MAP) {
      if (configActive.portB_pixel_count > LIM_PIXELS_PER_ARTNET_PORT) {
        portB[2] = artRDM.addPort(portB[0], 1, configActive.portB_uni[1], PORT_TYPE_DMX_OUT, configActive.portB_merge);
        artRDM.setE131(portB[0], portB[2], useE131);
        artRDM.setE131Uni(portB[0], portB[2], configActive.portB_SACNuni[1]);
      }
      if (configActive.portB_pixel_count > (2*LIM_PIXELS_PER_ARTNET_PORT)) {
        portB[3] = artRDM.addPort(portB[0], 2, configActive.portB_uni[2], PORT_TYPE_DMX_OUT, configActive.portB_merge);
        artRDM.setE131(portB[0], portB[3], useE131);
        artRDM.setE131Uni(portB[0], portB[3], configActive.portB_SACNuni[2]);
      }
      if (configActive.portB_pixel_count > (3*LIM_PIXELS_PER_ARTNET_PORT)) {
        portB[4] = artRDM.addPort(portB[0], 3, configActive.portB_uni[3], PORT_TYPE_DMX_OUT, configActive.portB_merge);
        artRDM.setE131(portB[0], portB[4], useE131);
        artRDM.setE131Uni(portB[0], portB[4], configActive.portB_SACNuni[3]);
      }
    }
  #endif

  // Add required callback functions:
  artRDM.setArtDMXCallback(cbArtDmxReceive);
  artRDM.setArtRDMCallback(cbArtRdmReceive);
  artRDM.setArtSyncCallback(cbArtSync);
  artRDM.setArtIPCallback(cbArtIpChanged);
  artRDM.setArtAddressCallback(cbArtAddressChanged);
  artRDM.setTODRequestCallback(cbArtTodRequest);
  artRDM.setTODFlushCallback(cbArtTodFlush);

  // Set NodeReport according reset info:
  switch (resetInfo.reason) {
    case REASON_DEFAULT_RST:  // normal start
    case REASON_EXT_SYS_RST:
    case REASON_SOFT_RESTART:
      artRDM.setNodeReport((char*)"OK: Device started", ARTNET_RC_POWER_OK);
      nextNodeReport = millis() + 4000;
      break;
    case REASON_WDT_RST:
      artRDM.setNodeReport((char*)"ERROR: (HWDT) Unexpected device restart", ARTNET_RC_POWER_FAIL);
      strcpy(nodeError, "Restart error: HWDT");
      nextNodeReport = millis() + 10000;
      nodeErrorTimeout = millis() + 30000;
      break;
    case REASON_EXCEPTION_RST:
      artRDM.setNodeReport((char*)"ERROR: (EXCP) Unexpected device restart", ARTNET_RC_POWER_FAIL);
      strcpy(nodeError, "Restart error: EXCP");
      nextNodeReport = millis() + 10000;
      nodeErrorTimeout = millis() + 30000;
      break;
    case REASON_SOFT_WDT_RST:
      artRDM.setNodeReport((char*)"ERROR: (SWDT) Unexpected device restart", ARTNET_RC_POWER_FAIL);
      strcpy(nodeError, "Error on Restart: SWDT");
      nextNodeReport = millis() + 10000;
      nodeErrorTimeout = millis() + 30000;
      break;
    case REASON_DEEP_SLEEP_AWAKE:
      // not used
      break;
  }
  
  // Start artnet
  artRDM.begin();

  yield();
}

void initPorts() {
  // Port A - DMX output:
  if (configActive.portA_mode == PORT_TYPE_DMX_OUT || configActive.portA_mode == PORT_TYPE_RDM_OUT) {
    statusStrip.SetPixelColor(ADDR_STATUS_LED_A, pink);
    // Start DMX output:
    dmxA.begin(PIN_DMX_DIR_A, artRDM.getDMX(portA[0], portA[1]));
    // Setup RDM:
    if (configActive.portA_mode == PORT_TYPE_RDM_OUT && !dmxA.rdmEnabled()) {
      dmxA.rdmEnable(CONF_ESTA_MAN, CONF_ESTA_DEV);
      dmxA.rdmSetCallBack(cbDmxRdmReceiveA);
      dmxA.todSetCallBack(cbDmxSendTodA);
    }
  // Port A - DMX input:
  } else if (configActive.portA_mode == PORT_TYPE_DMX_IN) {
    statusStrip.SetPixelColor(ADDR_STATUS_LED_A, yellow);
    // Start DMX input:
    dmxA.begin(PIN_DMX_DIR_A, artRDM.getDMX(portA[0], portA[1]));
    dmxA.dmxIn(true);
    dmxA.setInputCallback(cbDmxInputReceive);

    dataIn = (byte*) os_malloc(sizeof(byte) * 512);
    memset(dataIn, 0, 512);
  // Port A - WS2812 output:
  } else if (configActive.portA_mode == PORT_TYPE_WS2812) {
    statusStrip.SetPixelColor(ADDR_STATUS_LED_A, green);
    // Prepare output and configure strip:
    digitalWrite(PIN_DMX_DIR_A, HIGH);
    #ifdef SOLE_OUTPUT
      pixPortA = new NeoPixelBus<NeoGrbFeature, NeoEsp8266AsyncUart0Ws2812xMethod>(configActive.portA_pixel_count, PIN_PORT_A);
    #else
      pixPortA = new NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart0Ws2812xMethod>(configActive.portA_pixel_count, PIN_PORT_A);
    #endif
    pixPortA->Begin();
    if (configActive.portA_pixel_mode != PIXEL_FX_MODE_MAP) {
      pixFXA = new pixPatterns(pixPortA);
    }
  }
  
  #ifndef SOLE_OUTPUT
    // Port B - DMX output:
    if (configActive.portB_mode == PORT_TYPE_DMX_OUT || configActive.portB_mode == PORT_TYPE_RDM_OUT) {
      statusStrip.SetPixelColor(ADDR_STATUS_LED_B, pink);
      // Start DMX output:
      dmxB.begin(PIN_DMX_DIR_B, artRDM.getDMX(portB[0], portB[1]));
      // Setup RDM:
      if (configActive.portB_mode == PORT_TYPE_RDM_OUT && !dmxB.rdmEnabled()) {
        dmxB.rdmEnable(CONF_ESTA_MAN, CONF_ESTA_DEV);
        dmxB.rdmSetCallBack(cbDmxRdmReceiveB);
        dmxB.todSetCallBack(cbDmxSendTodB);
      }
    // Port B - WS2812 output:
    } else if (configActive.portB_mode == PORT_TYPE_WS2812) {
      statusStrip.SetPixelColor(ADDR_STATUS_LED_B, green);
      // Prepare output and configure strip:
      digitalWrite(PIN_DMX_DIR_B, HIGH);
      pixPortB = new NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart1Ws2812xMethod>(configActive.portB_pixel_count, PIN_PORT_B);
      pixPortB->Begin();
      if (configActive.portB_pixel_mode != PIXEL_FX_MODE_MAP) {
        pixFXB = new pixPatterns(pixPortB);
      }
    }
  #endif
}

#ifdef OLED
void initDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0);
  display.setTextWrap(false);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  // Clear display to prevent that Adafruit logo gets shown:
  display.clearDisplay();

  // Show boot logo on OLED display:
  display.drawBitmap(0, 0, dispLogo, DISP_LOGO_WIDTH, DISP_LOGO_HEIGHT, 1);
  display.display();
}
#endif

/* ############# Init functions end ############# */

void doNodeReport() {
  if (nextNodeReport > millis()) {
    // Time for next node report not reached
    return;
  }
  
  char c[ARTNET_NODE_REPORT_LENGTH];
  char pixelInfo[20];

  if (nodeErrorTimeout > millis()) {
    // No timeout -> Do next node report in 2s
    nextNodeReport = millis() + 2000;
  } else {
    // Timeout -> Do next node report in 5s
    nextNodeReport = millis() + 5000;
  }
  
  if (nodeError[0] != '\0' && !nodeErrorShowing && nodeErrorTimeout > millis()) {
    nodeErrorShowing = true;
    strcpy(c, nodeError);
  } else {
    nodeErrorShowing = false;
    strcpy(c, "OK: PortA:");
    switch (configActive.portA_mode) {
      case PORT_TYPE_DMX_OUT:
        strcat(c, " DMX Out");
        break;
      case PORT_TYPE_RDM_OUT:
        strcat(c, " RDM Out");
        break;
      case PORT_TYPE_DMX_IN:
        strcat(c, " DMX In");
        break;
      case PORT_TYPE_WS2812:
        if (configActive.portA_pixel_mode == PIXEL_FX_MODE_12) {
          strcat(c, " 12chan");
        }
        sprintf(pixelInfo, " WS2812 %ipixels", configActive.portA_pixel_count);
        strcat(c, pixelInfo);
        break;
    }
  
    #ifndef SOLE_OUTPUT
      strcat(c, ". PortB:");
      switch (configActive.portB_mode) {
        case PORT_TYPE_DMX_OUT:
          strcat(c, " DMX Out");
          break;
        case PORT_TYPE_RDM_OUT:
          strcat(c, " RDM Out");
          break;
        case PORT_TYPE_DMX_IN:
          /* Not possible -> Do nothing here */
          break;
        case PORT_TYPE_WS2812:
          if (configActive.portB_pixel_mode == PIXEL_FX_MODE_12) {
            strcat(c, " 12chan");
          }
          sprintf(pixelInfo, " WS2812 %ipixels", configActive.portB_pixel_count);
          strcat(c, pixelInfo);
          break;
      }
    #endif
  }
  
  artRDM.setNodeReport(c, ARTNET_RC_POWER_OK);
}

void pauseDmxOutputHandler(bool pause) {
  if (pause == true) {
    if ((dmxOutAPaused == false) && (configActive.portA_mode != PORT_TYPE_WS2812) && (configActive.portA_mode != PORT_TYPE_DMX_IN)) {
      dmxA.pause();
      dmxOutAPaused = true;
    }
    #ifndef SOLE_OUTPUT
      if ((dmxOutBPaused == false) && (configActive.portB_mode != PORT_TYPE_WS2812)) {
        dmxB.pause();
        dmxOutBPaused = true;
      }
    #endif
  } else {
    if (dmxOutAPaused == true) {
      dmxA.unPause();
      dmxOutAPaused = false;
    }
    #ifndef SOLE_OUTPUT
      if (dmxOutBPaused == true) {
        dmxB.unPause();
        dmxOutBPaused = false;
      }
    #endif
  }
}

/* ______________ Setup function ______________ */
void setup(void) {
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("DEBUG mode enabled");
  #endif

  // Make direction input to avoid boot garbage being sent out
  pinMode(PIN_DMX_DIR_A, OUTPUT);
  digitalWrite(PIN_DMX_DIR_A, LOW);
  #ifndef SOLE_OUTPUT
    pinMode(PIN_DMX_DIR_B, OUTPUT);
    digitalWrite(PIN_DMX_DIR_B, LOW);
  #endif

  // Initialize Status LEDs:
  statusStrip.Begin();
  statusStrip.SetPixelColor(ADDR_STATUS_LED_S, pink);
  statusStrip.SetPixelColor(ADDR_STATUS_LED_A, black);
  statusStrip.SetPixelColor(ADDR_STATUS_LED_B, black);
  statusStrip.Show();

  // Initialize OLED Display:
  #ifdef OLED
    initDisplay();
  #endif

  // Initialize Reset Config Button:
  bool resetDefaults = false;
  #ifdef PIN_RESET_CONFIG
    pinMode(PIN_RESET_CONFIG, INPUT);
    delay(5);
    // button pressed = low reading
    if (!digitalRead(PIN_RESET_CONFIG)) {
      delay(50);
      if (!digitalRead(PIN_RESET_CONFIG))
        resetDefaults = true;
    }
  #endif

  // Start LittleFS file system
  LittleFS.begin();
  // Load saved values or store defaults
  if (resetDefaults == false) {
    config_load();
  } else {
    config_init();
    config_save();
  }

  // Store our counters for resetting defaults
  if (resetInfo.reason != REASON_DEFAULT_RST && resetInfo.reason != REASON_EXT_SYS_RST && resetInfo.reason != REASON_SOFT_RESTART)
    config.wdtCounter++;
  else
    config.resetCounter++;
  // Store updated debug values
  config_save();

  // Set active config, used in loop() and callbacks
  configActive = config;

  // We only allow 1 DMX input - and RDM can't run alongside DMX in
  if (configActive.portA_mode == PORT_TYPE_DMX_IN && configActive.portB_mode == PORT_TYPE_RDM_OUT) {
    configActive.portB_mode = PORT_TYPE_DMX_OUT;
  }

  // Start wifi
  initWifi();

  // Setup Artnet Ports & Callbacks
  initArtnet();

  // Port Setup
  #ifndef DEBUG
    // Don't open any ports for a bit to let the ESP spill it's garbage to serial
    while (millis() < 3500)
      yield();

    initPorts();
  #endif

  // Start web server
  initWebServer();

  delay(10);

  #ifdef OLED
    // Show node name and ip address on OLED display
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(configActive.gen_nodeName);
    display.setCursor(0,16);
    if (configActive.net_ap_standalone == false && configActive.net_sta_ssid[0] != '\0') {
      display.print("STA - " + configActive.net_sta_ip.toString());
    } else {
      display.print("AP - " + configActive.net_ap_ip.toString());
    }
    display.display();
  #endif
}
/* ______________ Setup function end ______________ */

/* _________________ loop function ________________ */
void loop(void){
  // If the device lasts for 6 seconds, clear our reset timers
  if (config.resetCounter != 0 && millis() > TIMEOUT_CLEAR_COUNTERS) {
    config.resetCounter = 0;
    config.wdtCounter = 0;
    config_save();
    configActive.resetCounter = 0;
    configActive.wdtCounter = 0;
    ESPUI.updateNumber(numWdtCounter, configActive.wdtCounter);
    ESPUI.updateNumber(numRstCounter, configActive.resetCounter);
  }

  // Process mDNS for captive portal (only relevant for access point)
  if (accessPointStarted == true) {
    dnsServer.processNextRequest();
  }
  
  // Get the node details and handle Artnet
  doNodeReport();
  artRDM.handler();
  
  yield();

  #ifndef DEBUG
    // When no ArtNet data is received anymore, pause DMX output after timeout for better webserver performance:
    if ((artDmxReceiving == false) && (millis() > artDmxReceiveTimer)) {
      pauseDmxOutputHandler(true);
    } else {
      artDmxReceiveTimer = millis() + TIMEOUT_DMX;
      artDmxReceiving = false;
      pauseDmxOutputHandler(false);
    }
    // When someone is connected to webserver, pause DMX output (disarm UART interrupts) for better webserver performance:
    if (ESPUI.ws->count() > 0) {
      pauseDmxOutputHandler(true);
    } else if (ESPUI.ws->count() == 0) {
      pauseDmxOutputHandler(false);
    }

    // DMX handlers
    if (configActive.portA_mode != PORT_TYPE_WS2812) {
      dmxA.handler();
    }
    #ifndef SOLE_OUTPUT
      if (configActive.portB_mode != PORT_TYPE_WS2812) {
        dmxB.handler();
      }
    #endif

    if (ESPUI.ws->count() == 0) {
      // Do Pixel FX on port A
      if (configActive.portA_mode == PORT_TYPE_WS2812 && configActive.portA_pixel_mode != PIXEL_FX_MODE_MAP && pixFXA != NULL) {
        if (pixFXA->Update())
          pixDone = 0;
      }
      // Do Pixel FX on port B
      #ifndef SOLE_OUTPUT
        if (configActive.portB_mode == PORT_TYPE_WS2812 && configActive.portB_pixel_mode != PIXEL_FX_MODE_MAP && pixFXB != NULL) {
          if (pixFXB->Update())
            pixDone = 0;
        }
      #endif
    }
  #endif

  // Do pixel string output
  if (!pixDone)
  {
    if (pixPortA != NULL) {
      pixPortA->Show();
    }
    #ifndef SOLE_OUTPUT
      if (pixPortB != NULL) {
        pixPortB->Show();
      }
    #endif
  }

  // Handle received DMX
  if (newDmxIn) { 
    uint8_t g, p;
    newDmxIn = false;
    g = portA[0];
    p = portA[1];
    
    IPAddress bc = configActive.net_dmxIn_broadcast;
    artRDM.sendDMX(g, p, bc, dataIn, 512);

    statusStrip.SetPixelColor(ADDR_STATUS_LED_A, blue); //yellow
  }

  // Handle rebooting the system
  if (doReboot) {
    char c[ARTNET_NODE_REPORT_LENGTH] = "Device rebooting...";
    artRDM.setNodeReport(c, ARTNET_RC_POWER_OK);
    artRDM.artPollReply();

    ESP.restart();
  }

  // Output status to LEDs
  if ((statusTimer < millis()) && (Update.isRunning() == false)) {
    // Flash status LEDs
    if ((statusTimer % (2*INTERVAL_STATUS_LED)) > INTERVAL_STATUS_LED) {
      // Flash main status LED
      if (nodeError[0] != '\0') {
        statusStrip.SetPixelColor(ADDR_STATUS_LED_S, red);
      } else {
        statusStrip.SetPixelColor(ADDR_STATUS_LED_S, black);
      }
      // Reset port status LEDs in case of not receiving data anymore
      if (statusStrip.GetPixelColor(ADDR_STATUS_LED_A) == blue) {
        if (configActive.portA_mode == PORT_TYPE_DMX_OUT || configActive.portA_mode == PORT_TYPE_RDM_OUT) {
          statusStrip.SetPixelColor(ADDR_STATUS_LED_A, pink);
        } else if (configActive.portA_mode == PORT_TYPE_WS2812) {
          statusStrip.SetPixelColor(ADDR_STATUS_LED_A, green);
        } else if (configActive.portA_mode == PORT_TYPE_DMX_IN) {
          statusStrip.SetPixelColor(ADDR_STATUS_LED_A, yellow);
        }
      }
      if (statusStrip.GetPixelColor(ADDR_STATUS_LED_B) == blue) {
        if (configActive.portB_mode == PORT_TYPE_DMX_OUT || configActive.portB_mode == PORT_TYPE_RDM_OUT) {
          statusStrip.SetPixelColor(ADDR_STATUS_LED_B, pink);
        } else if (configActive.portB_mode == PORT_TYPE_WS2812) {
          statusStrip.SetPixelColor(ADDR_STATUS_LED_B, green);
        }
      }
    // Set main status LED according connection state
    } else if (((accessPointStarted == true) && (WiFi.softAPgetStationNum() > 0) && (ESPUI.ws->count() == 0)) ||
               ((accessPointStarted == false) && (WiFi.status() == WL_CONNECTED) && (ESPUI.ws->count() == 0))) {
      statusStrip.SetPixelColor(ADDR_STATUS_LED_S, green);
    } else if (ESPUI.ws->count() > 0) {
      statusStrip.SetPixelColor(ADDR_STATUS_LED_S, blue);
    } else {
      statusStrip.SetPixelColor(ADDR_STATUS_LED_S, white);
    }

    statusStrip.Show();
    statusTimer = millis() + INTERVAL_STATUS_LED;
  // Output update status to LEDs
  } else if ((statusTimer < millis()) && (Update.isRunning() == true)) {
    if ((statusTimer % (2*INTERVAL_STATUS_LED_UPDATE)) > INTERVAL_STATUS_LED_UPDATE) {
      statusStrip.SetPixelColor(ADDR_STATUS_LED_S, black);
      statusStrip.SetPixelColor(ADDR_STATUS_LED_A, black);
      statusStrip.SetPixelColor(ADDR_STATUS_LED_B, black);
    } else
    {
      if (((Update.progress()*100)/Update.size()) > 0) {
        statusStrip.SetPixelColor(ADDR_STATUS_LED_S, cyan);
      }
      if (((Update.progress()*100)/Update.size()) > 33) {
        statusStrip.SetPixelColor(ADDR_STATUS_LED_A, cyan);
      }
      if (((Update.progress()*100)/Update.size()) > 66) {
        statusStrip.SetPixelColor(ADDR_STATUS_LED_B, cyan);
      }
    }

    statusStrip.Show();
    statusTimer = millis() + INTERVAL_STATUS_LED_UPDATE;
  }
}
/* _______________ loop function end ______________ */
