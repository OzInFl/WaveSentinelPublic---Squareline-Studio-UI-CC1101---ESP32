// CLIPPER 1 - SUBGHZ - BT5 - WiFi Testing Tool
// Troy Dixon  
// Initial Source Published 6-19-2023 - GITHUB  (OzInFl)
//


#include "Arduino.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include "Audio.h"
#include "main.h"
#include <SD.h>
#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <RCSwitch.h>

//LVGL Libraries & Display Driver config
#include <lvgl.h>

//The Squareline Interface Code
#include "ui.h"
#include "ui_helpers.h"

// BLUETOOTH STUFF ///
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "SPIFFS.h"


//Global Paramaters

const char* ssid = "CLIPPER";
const char* password = "987654321";
const int wifi_channel = 12;

int OTAInProgress=0; //OTA Flag

//Audio I2S PIN Definitions
#define I2S_DOUT 37
#define I2S_BCLK 36
#define I2S_LRC 35

//LCD LVGL Stuff
#define LGFX_AUTODETECT // Autodetect board
#define LGFX_USE_V1     // set to use new version of library

///////////////////////////////////////////////////////Stuff From SubMarine

// SPIFFS
#include "SPIFFS.h"
#define SPIFFS_FILENAME_CAPTURED_SIGNAL "/captured.txt"
//#define SPIFFS_FILENAME_INCOMING_COMMAND "/incomingCommand.txt"
bool spiffsMounted = false;

// RECORDING SINGAL PARAMERS
#define MAX_EMPTY_RECORDING_CYCLES 32 // 32 RESET CYCLES
#define MINIMUM_RECORDED_TRANSITIONS 32
#define MINIMUM_RECORDTIME_MICROSECONDS 16000
#define MAX_LENGTH_RECORDED_SIGNAL 4096
#define MAX_TRANSITION_TIME_MICROSECONDS 32000
int recordedSignal[MAX_LENGTH_RECORDED_SIGNAL];
int recordedSamples = 0;
long lastRecordDuration = 0;

// SINGAL DETECTION
#define SIGNAL_DETECTION_FREQUENCIES_LENGTH 19
float signalDetectionFrequencies[SIGNAL_DETECTION_FREQUENCIES_LENGTH] = {300.00, 303.87, 304.25, 310.00, 315.00, 318.00, 390.00, 418.00, 433.07, 433.92, 434.42, 434.77, 438.90, 868.3, 868.35, 868.865, 868.95, 915.00, 925.00};
int detectedRssi = -100;
float detectedFrequency = 0.0;
int signalDetectionMinRssi = -65;
bool RECORDING_SIGNAL = false;

float CC1101_MHZ = 433.92;
bool CC1101_TX = false;
int CC1101_MODULATION = 2;
int CC1101_DRATE = 512;
float CC1101_RX_BW = 256;
int CC1101_PKT_FORMAT = 3;
float CC1101_LAST_AVG_LQI = 0;
float CC1101_LAST_AVG_RSSI = 0;
//float curfreq = 315.00;

///////////////////////////////////////////////////////////////End Submarine

// Create arrays to hold directory and file names
#define MAX_DIRS 50
#define MAX_FILES 50

char directoryNames[MAX_DIRS][20];
char fileNames[MAX_FILES][20];

// SD CARD - SPI PINS
#define SDSPI_HOST_ID SPI3_HOST
#define SD_MISO       GPIO_NUM_38 
#define SD_MOSI       GPIO_NUM_40
#define SD_SCLK       GPIO_NUM_39
#define SD_CS         GPIO_NUM_41

#define MICRO_SD_IO 41


String fileToTransmit = "";
String OpMode = "Idle";



//Web Service for Updates
WebServer server(80);

//Audio I2S Definitions
Audio audio;
bool playing;

// Initialize SD card file definitions
File root;
File recorded_signal_file;
File flipperFile;


// CC1101 Stuff
RCSwitch mySwitch = RCSwitch();
int ProtAnaRxEn=0;

// Create a buffer to store received data
byte buffer[64];


int txTestcode [512] = {60824, -66, 40307, -64, 19889, -100, 1757, -66, 1089, -66, 12747, -100, 11829, -68, 32647, -66, 7033, -17380, 67, -1260, 199, -132, 65, -198, 131, -100, 131, -132, 297, -134, 67, -630, 199, -66, 233, -132, 399, -134, 399, -130, 165, -396, 14485, -100, 4061, -66, 2219, -66, 23345, -66, 72679, -100, 33233, -66, 62597, -66, 44711, -66, 12647, -66, 11759, -98, 10387, -66, 3897, -68, 529, -98, 889, -66, 50761, -66, 229, -68, 17053, -64, 21881, -2366, 249, -808, 283, -804, 247, -828, 255, -824, 225, -840, 233, -830, 835, -240, 845, -240, 279, -800, 833, -276, 809, -276, 243, -804, 271, -796, 837, -276, 245, -802, 269, -798, 835, -276, 245, -808, 269, -796, 271, -802, 837, -276, 243, -806, 833, -240, 277, -804, 271, -10862, 267, -798, 269, -800, 271, -800, 271, -802, 269, -800, 271, -802, 835, -276, 813, -240, 279, -804, 833, -240, 845, -240, 279, -804, 271, -798, 833, -276, 245, -804, 269, -798, 835, -276, 245, -806, 269, -796, 271, -804, 835, -274, 245, -804, 835, -240, 279, -802, 269, -10846, 275, -806, 267, -800, 271, -804, 271, -800, 269, -800, 271, -802, 835, -276, 815, -240, 279, -800, 835, -240, 845, -274, 245, -804, 269, -798, 833, -276, 245, -802, 271, -800, 835, -276, 245, -804, 269, -798, 269, -806, 833, -276, 245, -804, 833, -276, 243, -806, 269, -10862, 269, -794, 271, -800, 269, -800, 271, -800, 271, -804, 269, -802, 835, -276, 813, -240, 279, -802, 833, -276, 811, -242, 279, -802, 269, -798, 835, -276, 245, -806, 269, -798, 837, -274, 245, -804, 271, -798, 269, -800, 835, -276, 245, -806, 831, -276, 245, -806, 269, -10856, 269, -798, 271, -800, 271, -802, 271, -802, 271, -798, 271, -804, 837, -276, 811, -274, 245, -802, 833, -276, 811, -276, 243, -802, 269, -798, 835, -276, 245, -808, 269, -796, 835, -276, 245, -806, 267, -798, 271, -806, 835, -276, 245, -802, 833, -276, 243, -806, 269, -10858, 267, -796, 269, -802, 269, -804, 271, -800, 271, -800, 271, -804, 833, -276, 811, -274, 245, -802, 833, -276, 811, -274, 243, -806, 269, -796, 835, -276, 243, -808, 269, -796, 835, -274, 247, -804, 269, -798, 269, -804, 837, -274, 245, -804, 833, -242, 277, -804, 267, -162736, 99, -1118, 363, -100, 195, -264, 229, -360, 97, -200, 755, -66, 6811, -100, 5675, -66, 6373, -98, 13705, -66, 14751, -68, 54237, -66, 32119, -66, 1531, -66, 5249, -68, 6397, -66, 44515, -100, 15541, -66, 10517, -68, 27293, -68, 20539, -66, 13969, -66, 657, -66, 3239, -98, 20847, -68, 14209, -66, 14423, -100, 42891, -100, 4619, -100, 24811, -66, 4341, -66, 10131, -66, 34427, -66, 13497, -2410, 231, -826, 269, -798, 269, -802, 269, -802, 271, -802, 271, -800, 835, -274, 811, -276, 243, -804, 837, -276, 777, -308, 243, -802, 271, -798, 837, -276, 243, -802, 269, -800, 833, -276, 245, -806, 269, -800, 271, -802, 833, -276, 243, -804, 837, -276, 243, -802, 271, -10832, 299, -796, 269, -802, 269, -800, 269, -802, 271, -804, 271, -800, 835, -276, 813, -240};
int txTeslaCode [512] = {60824, -66, 40307, -64, 19889, -100, 1757, -66, 1089, -66, 12747, -100, 11829, -68, 32647, -66, 7033, -17380, 67, -1260, 199, -132, 65, -198, 131, -100, 131, -132, 297, -134, 67, -630, 199, -66, 233, -132, 399, -134, 399, -130, 165, -396, 14485, -100, 4061, -66, 2219, -66, 23345, -66, 72679, -100, 33233, -66, 62597, -66, 44711, -66, 12647, -66, 11759, -98, 10387, -66, 3897, -68, 529, -98, 889, -66, 50761, -66, 229, -68, 17053, -64, 21881, -2366, 249, -808, 283, -804, 247, -828, 255, -824, 225, -840, 233, -830, 835, -240, 845, -240, 279, -800, 833, -276, 809, -276, 243, -804, 271, -796, 837, -276, 245, -802, 269, -798, 835, -276, 245, -808, 269, -796, 271, -802, 837, -276, 243, -806, 833, -240, 277, -804, 271, -10862, 267, -798, 269, -800, 271, -800, 271, -802, 269, -800, 271, -802, 835, -276, 813, -240, 279, -804, 833, -240, 845, -240, 279, -804, 271, -798, 833, -276, 245, -804, 269, -798, 835, -276, 245, -806, 269, -796, 271, -804, 835, -274, 245, -804, 835, -240, 279, -802, 269, -10846, 275, -806, 267, -800, 271, -804, 271, -800, 269, -800, 271, -802, 835, -276, 815, -240, 279, -800, 835, -240, 845, -274, 245, -804, 269, -798, 833, -276, 245, -802, 271, -800, 835, -276, 245, -804, 269, -798, 269, -806, 833, -276, 245, -804, 833, -276, 243, -806, 269, -10862, 269, -794, 271, -800, 269, -800, 271, -800, 271, -804, 269, -802, 835, -276, 813, -240, 279, -802, 833, -276, 811, -242, 279, -802, 269, -798, 835, -276, 245, -806, 269, -798, 837, -274, 245, -804, 271, -798, 269, -800, 835, -276, 245, -806, 831, -276, 245, -806, 269, -10856, 269, -798, 271, -800, 271, -802, 271, -802, 271, -798, 271, -804, 837, -276, 811, -274, 245, -802, 833, -276, 811, -276, 243, -802, 269, -798, 835, -276, 245, -808, 269, -796, 835, -276, 245, -806, 267, -798, 271, -806, 835, -276, 245, -802, 833, -276, 243, -806, 269, -10858, 267, -796, 269, -802, 269, -804, 271, -800, 271, -800, 271, -804, 833, -276, 811, -274, 245, -802, 833, -276, 811, -274, 243, -806, 269, -796, 835, -276, 243, -808, 269, -796, 835, -274, 247, -804, 269, -798, 269, -804, 837, -274, 245, -804, 833, -242, 277, -804, 267, -162736, 99, -1118, 363, -100, 195, -264, 229, -360, 97, -200, 755, -66, 6811, -100, 5675, -66, 6373, -98, 13705, -66, 14751, -68, 54237, -66, 32119, -66, 1531, -66, 5249, -68, 6397, -66, 44515, -100, 15541, -66, 10517, -68, 27293, -68, 20539, -66, 13969, -66, 657, -66, 3239, -98, 20847, -68, 14209, -66, 14423, -100, 42891, -100, 4619, -100, 24811, -66, 4341, -66, 10131, -66, 34427, -66, 13497, -2410, 231, -826, 269, -798, 269, -802, 269, -802, 271, -802, 271, -800, 835, -274, 811, -276, 243, -804, 837, -276, 777, -308, 243, -802, 271, -798, 837, -276, 243, -802, 269, -800, 833, -276, 245, -806, 269, -800, 271, -802, 833, -276, 243, -804, 837, -276, 243, -802, 271, -10832, 299, -796, 269, -802, 269, -800, 269, -802, 271, -804, 271, -800, 835, -276, 813, -240};
#define len_txTesla1 2589
int txTesla1 [2589] = {9175,-13444,131,-2386,1119,-32700,165,-294,97,-2928,297,-98,163,-12328,99,-794,99,-230,97,-132,297,-164,295,-100,491,-100,32700,-15934,361,-132,263,-296,559,-98,99,-98,5343,-12282,133,-268,199,-232,131,-132,397,-100,32700,-12816,197,-134,197,-332,165,-530,1885,-13078,197,-858,231,-824,27849,-614,347,-454,351,-426,383,-430,351,-434,351,-438,381,-408,381,-440,351,-442,349,-442,349,-442,381,-410,379,-412,379,-1202,381,-414,379,-414,807,-784,393,-416,791,-786,791,-822,361,-414,795,-818,759,-826,793,-792,795,-798,795,-794,401,-400,785,-400,379,-828,771,-404,379,-830,377,-394,791,-408,393,-398,393,-796,405,-396,397,-394,795,-406,395,-792,799,-404,395,-792,799,-800,401,-382,403,-416,357,-418,785,-418,383,-816,385,-412,775,-1182,409,-400,393,-398,817,-780,385,-412,779,-808,811,-782,391,-418,791,-784,787,-824,793,-794,793,-794,795,-794,401,-400,787,-402,395,-790,801,-402,397,-788,403,-396,791,-404,395,-394,395,-792,403,-380,403,-410,805,-378,411,-802,775,-406,379,-832,771,-800,407,-396,395,-394,397,-396,795,-406,395,-792,405,-396,789,-1198,395,-396,395,-398,785,-816,387,-410,777,-810,777,-812,389,-416,789,-784,789,-824,795,-794,795,-794,795,-794,401,-398,787,-402,397,-788,801,-402,395,-788,405,-396,791,-404,395,-396,393,-792,405,-380,401,-410,803,-378,411,-802,775,-410,393,-792,803,-802,403,-366,421,-378,403,-410,803,-378,409,-806,377,-412,409,-1176,415,-388,407,-388,407,-388,409,-356,437,-358,439,-356,437,-388,409,-388,407,-356,439,-358,437,-388,407,-358,437,-1176,415,-358,437,-358,833,-774,407,-396,791,-802,799,-796,403,-368,819,-796,797,-794,799,-794,795,-796,793,-798,401,-366,819,-404,397,-786,799,-404,397,-786,403,-398,787,-406,395,-394,395,-794,405,-398,395,-394,793,-406,395,-790,803,-402,377,-798,803,-802,401,-380,399,-410,407,-380,803,-412,377,-804,411,-378,803,-1202,379,-408,379,-410,803,-778,415,-386,815,-780,789,-790,395,-416,793,-784,823,-792,793,-796,795,-794,797,-794,399,-380,795,-404,379,-802,805,-408,393,-790,405,-396,793,-406,395,-394,395,-792,403,-380,401,-412,801,-380,409,-802,773,-410,411,-802,773,-802,405,-396,395,-396,395,-396,791,-406,393,-794,407,-396,791,-1198,395,-396,393,-398,819,-782,385,-410,781,-810,775,-812,389,-418,789,-784,787,-824,795,-794,795,-794,795,-796,401,-368,817,-404,395,-788,799,-404,397,-786,403,-398,789,-404,395,-394,395,-792,409,-396,393,-396,791,-404,379,-798,807,-404,377,-798,805,-802,405,-396,393,-396,397,-394,793,-406,393,-794,407,-396,395,-1184,409,-394,397,-394,399,-394,397,-392,401,-392,437,-356,437,-358,437,-356,437,-358,437,-358,437,-358,437,-358,439,-1178,415,-358,435,-358,833,-774,407,-398,789,-798,797,-796,399,-380,795,-806,797,-802,797,-798,795,-794,797,-796,399,-378,793,-410,393,-792,801,-406,397,-790,405,-396,787,-404,377,-402,409,-804,415,-358,433,-358,831,-378,409,-768,807,-404,379,-800,805,-804,403,-396,393,-398,393,-396,793,-408,395,-792,405,-398,787,-1198,395,-396,393,-400,817,-784,385,-412,777,-808,809,-778,391,-418,789,-784,819,-792,795,-794,793,-794,797,-794,399,-378,797,-406,409,-770,807,-406,395,-790,407,-396,789,-408,395,-394,393,-796,405,-398,393,-396,791,-408,393,-794,801,-400,377,-798,807,-802,399,-378,403,-392,433,-358,829,-378,393,-796,409,-394,791,-1202,393,-396,393,-398,817,-778,409,-384,809,-774,811,-784,393,-418,791,-782,823,-760,827,-796,795,-796,793,-794,401,-368,815,-402,377,-796,805,-404,409,-768,409,-378,833,-378,409,-372,409,-806,411,-378,407,-380,805,-414,375,-806,809,-378,409,-770,807,-802,401,-378,437,-358,435,-358,829,-380,393,-796,407,-398,393,-1184,409,-396,397,-394,397,-394,433,-360,433,-358,435,-358,435,-358,437,-358,437,-356,437,-358,437,-358,439,-356,439,-1178,413,-376,407,-380,805,-780,409,-378,833,-774,803,-800,405,-366,819,-800,795,-796,795,-798,795,-798,795,-796,401,-368,819,-402,397,-788,799,-402,397,-786,405,-366,821,-404,399,-390,397,-788,405,-396,395,-394,795,-408,395,-790,799,-404,377,-798,805,-802,403,-366,425,-396,395,-394,795,-406,397,-790,403,-378,797,-1198,409,-408,377,-408,801,-776,415,-388,817,-778,785,-792,431,-384,793,-786,791,-790,825,-794,795,-796,795,-794,399,-376,797,-408,393,-794,803,-404,397,-788,403,-380,795,-408,409,-374,411,-802,411,-378,407,-378,807,-412,377,-804,809,-376,409,-804,775,-800,403,-380,401,-410,407,-378,807,-412,377,-808,411,-358,831,-1172,395,-398,393,-398,817,-778,383,-416,811,-774,813,-780,391,-418,791,-784,821,-792,797,-794,793,-794,793,-796,401,-366,819,-404,397,-790,801,-400,377,-796,409,-394,791,-408,395,-396,393,-794,403,-380,435,-378,805,-378,409,-806,775,-406,379,-800,805,-800,405,-396,395,-394,397,-394,795,-406,395,-792,405,-378,435,-1166,415,-378,407,-380,411,-378,413,-410,381,-408,415,-376,415,-376,413,-380,413,-378,413,-378,413,-378,411,-412,379,-1206,407,-380,405,-384,803,-814,377,-412,811,-774,809,-780,391,-418,791,-782,821,-792,795,-796,793,-796,795,-794,403,-382,797,-392,405,-782,823,-398,369,-816,403,-368,819,-402,397,-390,379,-800,409,-396,395,-396,793,-406,379,-830,773,-404,379,-830,773,-802,405,-398,393,-396,397,-394,795,-406,395,-790,403,-380,799,-1202,391,-398,411,-406,769,-808,383,-414,805,-782,791,-790,431,-382,795,-786,789,-824,793,-796,795,-792,795,-796,401,-382,799,-390,409,-792,799,-404,397,-790,403,-398,789,-404,395,-394,393,-794,403,-380,401,-410,805,-378,409,-804,775,-406,379,-802,807,-802,403,-398,393,-396,393,-380,831,-376,411,-800,377,-412,803,-1170,411,-410,385,-408,777,-806,381,-416,781,-812,783,-786,397,-416,793,-784,789,-822,795,-798,795,-794,797,-794,401,-368,817,-398,379,-796,803,-404,379,-798,407,-412,801,-378,393,-398,391,-796,411,-394,397,-396,797,-406,395,-792,801,-404,397,-788,799,-798,403,-396,393,-398,395,-394,793,-404,379,-796,409,-380,435,-1168,415,-380,407,-380,411,-380,411,-412,381,-410,413,-378,413,-378,411,-380,411,-380,411,-380,413,-410,375,-426,369,-1216,397,-394,395,-396,785,-818,387,-410,779,-812,777,-814,385,-384,821,-782,785,-822,795,-794,795,-794,793,-794,399,-378,795,-410,393,-794,803,-404,397,-788,405,-396,789,-406,395,-394,395,-792,407,-396,395,-396,793,-404,379,-798,805,-402,381,-798,803,-804,405,-396,393,-398,395,-394,795,-406,395,-790,403,-380,797,-1200,411,-406,377,-410,803,-776,383,-412,807,-784,787,-826,397,-368,817,-796,795,-800,795,-794,795,-796,795,-794,399,-380,795,-406,379,-832,773,-404,379,-832,377,-396,791,-408,395,-396,391,-794,409,-394,397,-396,795,-406,395,-794,801,-402,397,-788,801,-798,403,-366,423,-398,393,-398,789,-404,395,-790,405,-396,791,-1202,395,-396,393,-398,817,-784,385,-410,777,-810,811,-778,389,-410,791,-800,797,-800,797,-796,795,-794,797,-796,401,-382,797,-390,407,-778,827,-400,367,-816,401,-366,821,-402,397,-394,395,-790,405,-396,395,-394,793,-408,393,-792,803,-402,377,-796,805,-804,403,-366,421,-380,401,-410,803,-378,409,-804,381,-394,399,-1210,415,-358,435,-358,437,-358,435,-358,437,-358,437,-358,437,-358,437,-358,437,-358,439,-356,439,-358,437,-356,439,-1174,417,-358,437,-356,833,-776,405,-396,791,-800,801,-798,401,-368,815,-798,799,-798,797,-794,793,-796,795,-796,403,-366,817,-404,397,-788,799,-404,395,-788,405,-396,791,-404,395,-394,395,-792,403,-380,433,-378,801,-380,409,-804,775,-410,393,-794,803,-800,399,-380,399,-410,405,-380,807,-380,409,-804,411,-378,803,-1170,411,-408,379,-410,801,-776,415,-382,805,-788,791,-794,431,-382,797,-784,789,-824,793,-794,799,-794,793,-794,401,-368,815,-402,379,-796,805,-404,379,-798,407,-380,831,-378,411,-406,379,-804,411,-378,407,-378,807,-412,377,-806,807,-376,411,-802,773,-804,403,-398,393,-396,397,-394,793,-406,393,-794,407,-398,789,-1198,397,-394,395,-396,817,-782,385,-412,777,-804,811,-780,391,-418,791,-784,791,-792,825,-796,795,-794,793,-796,399,-378,795,-410,393,-794,801,-404,397,-788,405,-396,789,-406,395,-394,393,-794,403,-380,433,-378,805,-378,409,-804,773,-412,393,-794,803,-796,405,-398,391,-398,395,-394,793,-406,395,-792,403,-378,435,-1168,413,-380,407,-380,411,-378,413,-410,379,-410,413,-378,413,-378,413,-378,413,-380,413,-378,413,-378,413,-410,385,-1212,387,-388,419,-384,811,-782,415,-358,835,-776,799,-800,399,-380,795,-804,799,-802,797,-798,793,-796,797,-794,397,-380,793,-408,379,-832,773,-406,395,-794,405,-396,793,-404,397,-394,393,-794,403,-380,401,-412,801,-378,411,-802,773,-408,411,-802,773,-802,403,-396,395,-396,397,-394,793,-406,395,-790,407,-396,791,-1200,395,-396,393,-398,817,-780,379,-414,811,-778,809,-782,391,-418,789,-784,791,-826,793,-794,795,-792,795,-796,397,-380,795,-406,379,-832,773,-404,379,-800,411,-394,795,-406,395,-396,393,-796,405,-378,403,-410,803,-410,379,-802,775,-410,395,-792,801,-800,401,-378,401,-410,405,-380,805,-412,377,-806,411,-378,803,-1206,357,-434,379,-406,805,-774,383,-414,807,-784,791,-822,397,-382,795,-786,791,-826,793,-796,795,-794,795,-792,403,-380,799,-390,417,-790,785,-392,409,-796,405,-396,787,-406,393,-396,395,-794,407,-396,395,-394,793,-408,393,-794,801,-400,379,-796,803,-800,403,-378,401,-412,405,-380,805,-410,379,-804,411,-378,407,-1202,381,-380,411,-412,379,-410,381,-412,413,-378,411,-378,413,-378,413,-380,413,-380,411,-410,381,-410,379,-412,377,-1206,415,-396,383,-390,807,-808,385,-388,805,-812,773,-806,403,-378,797,-804,799,-798,797,-798,797,-798,797,-794,401,-368,819,-400,377,-796,803,-404,395,-792,405,-380,795,-410,393,-398,391,-798,407,-382,401,-412,803,-410,377,-806,773,-408,395,-792,801,-798,403,-398,393,-396,397,-394,793,-406,395,-792,407,-396,791,-1194,381,-434,379,-408,771,-812,383,-414,805,-782,787,-820,399,-380,793,-788,789,-828,793,-794,795,-796,795,-794,401,-398,787,-404,365,-818,797,-404,395,-790,403,-398,791,-404,395,-394,395,-792,407,-396,395,-396,793,-406,395,-792,799,-400,379,-798,803,-800,401,-380,401,-412,405,-380,805,-410,379,-804,409,-380,801,-1206,379,-408,387,-410,779,-812,381,-418,779,-810,781,-788,393,-418,791,-786,789,-826,795,-794,793,-796,795,-796,399,-400,787,-400,379,-798,803,-406,395,-790,407,-396,789,-404,395,-394,395,-794,405,-396,395,-396,793,-404,381,-832,771,-404,379,-830,771,-802,403,-398,393,-396,397,-394,793,-406,395,-792,407,-396,395,-1182,409,-396,397,-394,399,-394,397,-394,401,-392,401,-392,401,-392,437,-356,437,-390,405,-358,437,-388,407,-388,407,-1178,415,-358,437,-388,801,-774,407,-396,791,-800,801,-798,397,-380,795,-802,801,-796,797,-798,797,-798,797,-794,397,-380,795,-406,379,-800,805,-408,393,-794,405,-396,791,-404,397,-392,395,-794,405,-396,395,-396,791,-404,381,-830,773,-406,379,-798,803,-802,401,-380,401,-410,407,-380,803,-412,379,-804,377,-412,803,-1208,355,-438,357,-436,781,-780,413,-418,777,-776,813,-786,393,-416,791,-786,789,-826,795,-796,793,-794,793,-796,399,-380,795,-406,395,-794,801,-404,397,-788,405,-396,791,-402,379,-400,409,-806,377,-412,405,-380,805,-410,379,-804,773,-412,391,-796,803,-800,403,-398,393,-396,395,-396,791,-408,395,-790,405,-378,797,-1204,381,-436,379,-406,773,-808,383,-418,783,-812,785,-788,393,-410,795,-802,797,-802,797,-794,795,-794,795,-794,399,-380,795,-408,395,-794,803,-404,395,-790,401,-380,797,-406,379,-436,379,-806,409,-380,405,-380,805,-412,377,-804,809,-376,411,-802,771,-804,403,-380,401,-392,435,-360,829,-378,391,-794,409,-394,397,-32700,1131,-232,32700,-12032,165,-3550,795,-264,195,-494,129,-12370,133,-730,131,-3166,461,-98,32700,-12384,163,-264,97,-298,99,-426,133,-990,463,-100,531,-98,32700,-10254,129,-362,133,-298,131,-628,99,-300,97,-526,32700,-12262,131,-530,165,-100,99,-100,295,-232,231,-694,2073,-11990,99,-922,163,-262,99,-262,97,-98,99,-164,97,-560,129,-132,32700,-12354,99,-364,165,-494,229,-532,131,-98,129,-984,97,-430,1249,-10580,99,-464,133,-432,231,-662,165,-432,133,-264,133,-794,32700,-12126,263,-1060,297,-830,97,-662,99,-132,163,-100,32700,-12128,133,-496,131,-234,131,-794,131,-366,363,-132,165,-166,597,-232,15917,-12030,265,-132,167,-430,99,-98,99,-100,131,-266,165,-1956,101,-332,263,-132,431,-200,397,-200,361,-98,32700,-12474,131,-298,265,-166,131,-860,133,-132,133,-132,827,-98,691};


byte bytesToSend[1036];
byte sendPacket[60];



// The signal to send for tesla charge jacks
const uint16_t pulseWidth = 400;
const uint16_t messageDistance = 23;
const uint8_t transmtesla = 5;
const uint8_t messageLength = 43;

const uint8_t sequence[messageLength] = { 
  0x02,0xAA,0xAA,0xAA,  // Preamble of 26 bits by repeating 1010
  0x2B,                 // Sync byte
  0x2C,0xCB,0x33,0x33,0x2D,0x34,0xB5,0x2B,0x4D,0x32,0xAD,0x2C,0x56,0x59,0x96,0x66,
  0x66,0x5A,0x69,0x6A,0x56,0x9A,0x65,0x5A,0x58,0xAC,0xB3,0x2C,0xCC,0xCC,0xB4,0xD2,
  0xD4,0xAD,0x34,0xCA,0xB4,0xA0};
//END

//Analyzer stuff
#define samplesize 2000
long transmit_push[2000];
int error_toleranz = 200;
const int minsample = 30;
unsigned long sample[samplesize];
unsigned long samplesmooth[samplesize];
int samplecount;


// CC1101 Modules - GDO Pins  (CSN are in the VSPI area)  //
#define CCGDO0A 13 //GPIO13 - TX Line Module 1
#define CCGDO2A 21 //GPIO21 - RX Line Module 1

#define CCGDO2B 2 //GPIO2  - RX Line Module 2 (NOT TESTED YET!)
#define CCGDO0B 1 //GPIO1  - TX Line Module 2 (NOT TESTED YET!)
// End CC1101 Pins //

volatile long last_RXmillis; //CC1101 Receive timer
volatile long last_micros;
String RXbuffer;//RX buffer

// Define ALTERNATE_PINS to use non-standard GPIO pins for VSPI bus - DUE TO USING THE ESP32-S3 BUILT INTO THE WT32-SC01-PLUS
#define ALTERNATE_PINS
#ifdef ALTERNATE_PINS
  #define VSPI_MISO   11
  #define VSPI_MOSI   10
  #define VSPI_SCLK   14
  #define VSPI_SSA    12 //MODULE 1 CSN
  #define VSPI_SSB    42 //MODULE 2 CSN
#else
  #define VSPI_MISO   MISO
  #define VSPI_MOSI   MOSI
  #define VSPI_SCLK   SCK
  #define VSPI_SSAA     SS
#endif

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#define ESPI HSPI
#endif

static const int spiClk = 1000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;

static LGFX lcd; // declare LCD display variable

//Bluetooth Server Stuff
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};


void initBT5(){
  // Create the BLE Device
  BLEDevice::init("CLIPPER UART");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  lv_label_set_text_static(ui_lblSplash,"Waiting");
  Serial.println("Waiting a client connection to notify...");

}


void doBlue(){
   if (deviceConnected) {
        pTxCharacteristic->setValue(&txValue, 1);
        pTxCharacteristic->notify();
        txValue++;
		delay(10); // bluetooth stack will go into congestion, if too many packets are sent
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        lv_label_set_text_static(ui_lblSplash,"start advertising");
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
//// END BLUETOOTH STUFF ///


/*** Header Function declarations ***/
void sendSamples(int samples[], int samplesLength);
void populateFileDropdown();
void display_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);


//CC1101 Settings and Stuff
void initCC1101(){
    ELECHOUSE_cc1101.setSpiPin(VSPI_SCLK,VSPI_MISO,VSPI_MOSI,VSPI_SSA);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setGDO(CCGDO0A, CCGDO2A);
    ELECHOUSE_cc1101.setMHZ(CC1101_MHZ);        // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    ELECHOUSE_cc1101.setPA(12);
    if(CC1101_TX){
    ELECHOUSE_cc1101.SetTx();                             // set Transmit on
    pinMode(CCGDO0A,OUTPUT);  
  } else {
    ELECHOUSE_cc1101.SetRx();                             // set Receive on
    pinMode(CCGDO2A,INPUT);  
  }               
  ELECHOUSE_cc1101.setModulation(CC1101_MODULATION);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setDRate(CC1101_DRATE);                // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
  ELECHOUSE_cc1101.setRxBW(CC1101_RX_BW);                 // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
  ELECHOUSE_cc1101.setPktFormat(CC1101_PKT_FORMAT);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 
                                                          // 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 
                                                          // 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 
                                                          // 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
  
    if(!ELECHOUSE_cc1101.getCC1101()){       // Check the CC1101 Spi connection.
      //Serial.println("CC1101 Connection Error");
      lv_label_set_text_static(ui_lblSplash,"1101 Init Fail");
    }else{lv_label_set_text_static(ui_lblSplash,"1101 Init Good");}
}

#include <fstream>
#include <sstream>
#include <string> 

void read_data_file(std::string filename, int& frequency, int& raw_data_size, int* raw_data) {
    std::ifstream file(filename);
    if (!file) {
        // Error opening file
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;
        if (keyword == "Frequency:") {
            iss >> frequency;
        } else if (keyword == "RAW_Data:") {
            int value;
            raw_data_size = 0;
            while (iss >> value) {
                raw_data[raw_data_size++] = value;
            }
        }
    }
}



void sendSamples(int samples[], int samplesLength) {
  //CC1101_MHZ=mhz;
  if(CC1101_TX == false){
    CC1101_TX = true;
    initCC1101();
  }

  int delay = 0;
  unsigned long time;
  byte n = 0;

  for (int i=0; i < samplesLength; i++) {
    // TRANSMIT
    n = 1;
    delay = samples[i];
    if(delay < 0){
      // DONT TRANSMIT
      delay = delay * -1;
      n = 0;
    }

    digitalWrite(CCGDO0A,n);
    
    delayMicroseconds(delay);
  }

  // STOP TRANSMITTING
  digitalWrite(CCGDO0A,0);

  //delay(5);
  lv_label_set_text_static(ui_lblMainStatus,"Transmission completed.");
  
  //digitalWrite(PIN_LED_TX, LOW);
}







void Rxloop(){
	if (millis() > (last_RXmillis + 5000)){
    int curRSSI = ELECHOUSE_cc1101.getRssi();
		ELECHOUSE_cc1101.goSleep();
    //lv_textarea_set_text(ui_txtMainRxBuffer,String(RXbuffer).c_str());
		Serial.println(RXbuffer);
		RXbuffer = "";
		ELECHOUSE_cc1101.SetRx();
		last_RXmillis = millis();
	}
}

void radioHandlerOnChange() {
	int delta_micros = micros() - last_micros;
	
	bool input = digitalRead(CCGDO2A);
	if(input == 1){
    lv_label_set_text_static(ui_lblProtAnaRXEn,"RX-SENSED");
		RXbuffer += "\n0 -> 1 after " + String(delta_micros);
	} else {
		RXbuffer += "\n1 -> 0 after " + String(delta_micros);
	}
	lv_textarea_set_text(ui_txtProtAnaResults,String(RXbuffer).c_str());
  lv_textarea_set_text(ui_txtProtAnaResults,"\r\n");
  RXbuffer="";
	last_micros = micros();
}

void handleFlipperCommandLine(String command, String value){
    if(command == "Frequency"){
      float frequency = value.toFloat() / 1000000;   
      lv_label_set_text_static(ui_lblPresetsStatus,String(frequency).c_str());
      ELECHOUSE_cc1101.setMHZ(frequency);
      CC1101_MHZ = frequency;
    }
}

void SDInit(){
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);
  SD.begin(SD_CS);
}


void transmitFlipperFile(String filename, bool transmit){
  if(transmit){
    // SETUP CC1101 AGAIN

    initCC1101();
  }
    
  
  SDInit();

  flipperFile = SD.open(filename);

  if (!flipperFile) {
    lv_label_set_text_static(ui_lblPresetsStatus,"NOT A FLIPPER FILE");
  } else {
    // PARSE CONTENT CHAR BY CHAR
    String command = "";
    String value = "";

    int data;
    char dataChar;
    bool appendCommand = true;
    bool breakLoop = false;
    int samples[512];
    int currentSample = 0;    

    while ((data = flipperFile.read()) >= 0 && breakLoop == false) {
        dataChar = data;

        switch (dataChar) {
          case ':':
              appendCommand = false;
              break;
          case '\n':
              // REMOVE SPACES IN FRONT OF VALUE
              while(value.startsWith(" ")){
                value = value.substring(1);
              }

              Serial.println("DUMP:");
              Serial.println(command + " | " + value);

              if(transmit == false){
                // SETUP CC1101 PARAMETERS
                handleFlipperCommandLine(command, value);
              } else {
                // TRANSMIT ON PREVIOUSLY SETUP CC1101
                handleFlipperCommandLine(command, value);
                if(command == "RAW_Data" && transmit){
                  sendSamples(samples, 512);  ///check thiis line
                }
              }

              // GET READY FOR THE NEXT ROW
              appendCommand = true;
              command = "";
              value = "";
              currentSample = 0;
              memset(samples, 0, sizeof(samples));       
              break;
          default:
              if(appendCommand){
                command += String(dataChar);
              } else {
                value += String(dataChar);

                if(command == "RAW_Data"){
                  if(dataChar == ' '){
                    // REPLACE SPACES IN CURRENT SAMPLE
                    value.replace(" ","");
                    if(value != ""){
                      samples[currentSample] = value.toInt();
                      currentSample++;
                      value = "";
                    }
                    
                  }
                } 
              }
              break;
        }
    }

    flipperFile.close();
       
  }
  
}

void ProtAnalyzerloop() {
   
    

    if (mySwitch.available()) {
    lv_textarea_set_text(ui_txtProtAnaResults,"");
    lv_textarea_set_text(ui_txtProtAnaProtocol,"");
    lv_textarea_set_text(ui_txtProtAnaDBPTD,"");
    lv_textarea_set_text(ui_txtProtAnaReceived,"");


    int value = mySwitch.getReceivedValue();
    
    if (value == 0) {
      lv_textarea_set_text(ui_txtProtAnaResults,"Unknown encoding");
    } else {
      int databuffer[64]; // get a copy of the received timings before they are overwritten
      int numberoftimings = 2 * mySwitch.getReceivedBitlength() + 2;
      if(numberoftimings > 64) numberoftimings = 64;
      for (int i = 0; i < numberoftimings; i++) {
       databuffer[i] = mySwitch.getReceivedRawdata()[i];
      }
      
      lv_textarea_add_text(ui_txtProtAnaResults,"-----RECEIVED PACKET INFO-----");
      //lv_textarea_add_text(ui_txtProtAnaResults,"\r\n");
      //lv_textarea_add_text(ui_txtProtAnaResults,"Received: ");
      lv_textarea_set_text(ui_txtProtAnaReceived, String(mySwitch.getReceivedValue()).c_str());
      //lv_textarea_add_text(ui_txtProtAnaResults,"\r\n");
      //lv_textarea_add_text(ui_txtProtAnaBitLength,"bit length: ");
      lv_textarea_set_text(ui_txtProtAnaBitLength, String(mySwitch.getReceivedBitlength()).c_str());
      //lv_textarea_add_text(ui_txtProtAnaResults,"\r\n");
      //lv_textarea_add_text(ui_txtProtAnaResults,"Protocol: ");
      lv_textarea_add_text(ui_txtProtAnaProtocol, String(mySwitch.getReceivedProtocol()).c_str());
      //lv_textarea_add_text(ui_txtProtAnaResults,"\r\n");
      //lv_textarea_add_text(ui_txtProtAnaResults,"Data bit Offset: ");
      unsigned int databitsoffset = abs( (int)mySwitch.getReceivedLevelInFirstTiming() - (int)mySwitch.getReceivedInverted());
      //lv_textarea_add_text(ui_txtProtAnaResults, std::to_string(mySwitch.getReceivedLevelInFirstTiming()).c_str());
      //lv_textarea_add_text(ui_txtProtAnaResults, std::to_string(mySwitch.getReceivedInverted()).c_str());
      //lv_textarea_add_text(ui_txtProtAnaResults, std::to_string(databitsoffset).c_str());
      unsigned long dataduration = 0;
      for (unsigned int i = 1 + databitsoffset; i < numberoftimings - 1 + databitsoffset; i++) {
        dataduration += databuffer[i];
      }
      //lv_textarea_add_text(ui_txtProtAnaResults,"data bits of pulse train duration: ");
      lv_textarea_set_text(ui_txtProtAnaDBPTD,String(dataduration).c_str());
      //lv_textarea_add_text(ui_txtProtAnaResults,"\r\n");
      unsigned int averagebitduration = (int)(0.5 + ((double)dataduration)/mySwitch.getReceivedBitlength());
      unsigned int protocolratio = (unsigned int)(0.5 + ((double)(averagebitduration - mySwitch.getReceivedDelay())) / (double)mySwitch.getReceivedDelay());
      lv_textarea_add_text(ui_txtProtAnaResults,"RX delay:  ");
      lv_textarea_add_text(ui_txtProtAnaResults,String(mySwitch.getReceivedDelay()).c_str());
    
      char buffer[10]; // assuming the integer will be less than 10 digits
int value = (databitsoffset==0) ? 
        (int) (0.5 + (double)databuffer[2*mySwitch.getReceivedBitlength()+1]/(double)mySwitch.getReceivedDelay())
        :
        (int) (0.5 + (double)databuffer[0]/(double)mySwitch.getReceivedDelay());
itoa(value, buffer, 10);
lv_textarea_add_text(ui_txtProtAnaResults, buffer);

      lv_textarea_add_text(ui_txtProtAnaResults,", ");

      char buffer2[10]; // assuming the integer will be less than 10 digits
int value2 = (databitsoffset==0) ? 
        (int) (0.5 + (double)databuffer[0]/(double)mySwitch.getReceivedDelay())
        :
        (int) (0.5 + (double)databuffer[1]/(double)mySwitch.getReceivedDelay());
itoa(value2, buffer2, 10);
lv_textarea_add_text(ui_txtProtAnaResults, buffer2);
      
      lv_textarea_add_text(ui_txtProtAnaResults," }, { ");
      lv_textarea_add_text(ui_txtProtAnaResults,"1");
      lv_textarea_add_text(ui_txtProtAnaResults,", ");
      lv_textarea_add_text(ui_txtProtAnaResults,String(protocolratio).c_str());
      lv_textarea_add_text(ui_txtProtAnaResults," }, { ");
      lv_textarea_add_text(ui_txtProtAnaResults,String(protocolratio).c_str());
      lv_textarea_add_text(ui_txtProtAnaResults,", ");
      lv_textarea_add_text(ui_txtProtAnaResults,"1");
      lv_textarea_add_text(ui_txtProtAnaResults," }, ");
      lv_textarea_add_text(ui_txtProtAnaResults,(mySwitch.getReceivedInverted()) ? "true" : "false" );
      lv_textarea_add_text(ui_txtProtAnaResults," }");

      // raw signal
      lv_textarea_add_text(ui_txtProtAnaResults,"==============");
      lv_textarea_add_text(ui_txtProtAnaResults,"first level ");
      lv_textarea_add_text(ui_txtProtAnaResults,(mySwitch.getReceivedLevelInFirstTiming() == 0) ? "down" : "up" );
      for (int i = 0; i < 2*mySwitch.getReceivedBitlength()+2 - 1 + databitsoffset; i++) {
        lv_textarea_add_text(ui_txtProtAnaResults,String(databuffer[i]).c_str());
        lv_textarea_add_text(ui_txtProtAnaResults," ");
        if((i - databitsoffset) % 16 == 0) lv_textarea_add_text(ui_txtProtAnaResults,"");
      }
      if ((2*mySwitch.getReceivedBitlength()+2 - 1 + databitsoffset - 1) % 16 != 0) lv_textarea_add_text(ui_txtProtAnaResults,"");
      if (databitsoffset != 1) lv_textarea_add_text(ui_txtProtAnaResults,String(databuffer[2*mySwitch.getReceivedBitlength()+1]).c_str());

      // plot signal in spreadsheet
      lv_textarea_add_text(ui_txtProtAnaResults,"==============");
     
    }

    mySwitch.resetAvailable();
  }
}

void callback(){
  //placeholder callback function
}




void initSpiffs(){
  if(SPIFFS.begin(true)){
    //Serial.println("SPIFFS mounted successfully");
    spiffsMounted = true;
  } else {
    //Serial.println("An Error has occurred while mounting SPIFFS");  
    spiffsMounted = false;  
  }
}



//  --------------------------------------AUDIO FUNCTIONS------------------------ //
void playAudio(){
SDInit();
// Set Volume
    audio.setVolume(15);
    
    // Open music file
    if(!audio.connecttoFS(SD,"/test.mp3")){
    lv_label_set_text_static(ui_lblMainStatus,"Shit Failed");
    }
    
    
//   -------------------------- END  AUDIO  FUNCITONS  --------------------------- //
}

void lvgl_loop(void *parameter)
{

  while (true)
  {
    lv_timer_handler();
    delay(5);
  }
  vTaskDelete(NULL);
}

void guiHandler()
{

  xTaskCreatePinnedToCore(
      // xTaskCreate(
      lvgl_loop,   // Function that should be called
      "LVGL Loop", // Name of the task (for debugging)
      100000,       // Stack size (bytes)
      NULL,        // Parameter to pass
      1,           // Task priority
      // NULL
      NULL, // Task handle
      1);
}

// MAIN SETUP FUNCTION --------------------------------------------------------------------------- MAIN SETUP FUNCTION //
void setup(void)
{
  
  //initialise instance of the SPIClass attached to VSPI 
  vspi = new SPIClass(VSPI);
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SSA); //SCLK, MISO, MOSI, SS
  pinMode(VSPI_SSA, OUTPUT); //VSPI SS
  pinMode(SD_CS,OUTPUT); //SD Card SS
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);
  SD.begin(SD_CS);


  //I2S Stuff
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);

/* this callback function will be invoked when starting */
  ArduinoOTA.onStart([]() {
    lv_label_set_text_static(ui_lblSettingsStatus,"UPDATE STARTED");
    //Serial.println("Start updating");
  });

  /* this callback function will be invoked when updating end */
  ArduinoOTA.onEnd([]() {
    OTAInProgress=0;
    lv_label_set_text_static(ui_lblSettingsStatus,"COMPLETE - RESTARTING");
    delay(5000);  
    ESP.restart();
  });

    /* this callback function will be invoked when updating error */
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) lv_label_set_text_static(ui_lblSettingsStatus,"Auth Failed");
    else if (error == OTA_BEGIN_ERROR) lv_label_set_text_static(ui_lblSettingsStatus,"Begin Failed");
    else if (error == OTA_CONNECT_ERROR) lv_label_set_text_static(ui_lblSettingsStatus,"Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) lv_label_set_text_static(ui_lblSettingsStatus,"Receive Failed");
    else if (error == OTA_END_ERROR) lv_label_set_text_static(ui_lblSettingsStatus,"End Failed");
  });
  /* this callback function will be invoked when a number of chunks of software was flashed
    so we can use it to calculate the progress of flashing */
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    String UpdProgress="Progress: ";
    UpdProgress+=String((progress / (total / 100))).c_str();
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    lv_label_set_text_static(ui_lblSettingsStatus,UpdProgress.c_str());
    lv_bar_set_value(ui_barProgress,progress / (total / 100),LV_ANIM_ON);
  });


  
   //Start The Serial Debug Port
  Serial.begin(115200); /* prepare for possible serial debug */
  
  // INIT SPIFFS - For Captured Content
  initSpiffs();

  //Initialize the LCD/tft Graphics obnjects
  lcd.init(); // Initialize LCD
  lcd.setRotation(2);
  lv_init();  // Initialize lvgl

  /* LVGL : Setting up buffer to use for display */
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

  /*** LVGL : Setup & Initialize the display device driver ***/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = display_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*** LVGL : Setup & Initialize the input device driver ***/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  lv_indev_drv_register(&indev_drv);

  /*** Initialize the Squareline Interface ***/
  ui_init(); // The Squareline interface
  guiHandler();
  lv_timer_handler();
  
  /***Initialize the CC1101 Radio and set the frequency ***/
  CC1101_MHZ=433.92;
  initCC1101();
  mySwitch.enableReceive(CCGDO2A);  // Receiver on
  mySwitch.enableTransmit(CCGDO0A); // Transmitter Enabler

  //initBT5();
  // Open a file to record the signal
  recorded_signal_file = SD.open("/recorded_signal.bin", FILE_WRITE);
  
  //lv_textarea_set_cursor_type(ui_txtProtAnaResults, LV_CURSOR_NONE);
}





// MAIN LOOP FUNCTION --------------------------------------------------------------------------- MAIN LOOP FUNCTION //
void loop()
{
  

  if (OTAInProgress==0){
  //Do Nothing Regarding OTA
  //lv_timer_handler(); /* let the GUI do its work */
  void doBlue();
  audio.loop();
  }

  if (OTAInProgress==1){
  ArduinoOTA.handle();
  server.handleClient();
  }

  if(ProtAnaRxEn==1){
    //CC1101_MHZ=433.92;
   //CC1101_TX = false;
   //initCC1101();
      ProtAnalyzerloop();
    }

  }


 /// AUX FUNCTIONS FOR LCD ----------------------------------------------------------------------- AUX FUNCTIONS FOR LCD //

  /*** Display callback to flush the buffer to screen ***/
  void display_flush(lv_disp_drv_t * disp, const lv_area_t *area, lv_color_t *color_p)
  {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    lcd.startWrite();
    lcd.setAddrWindow(area->x1, area->y1, w, h);
    lcd.pushColors((uint16_t *)&color_p->full, w * h, true);
    lcd.endWrite();

    lv_disp_flush_ready(disp);
  }

  /*** Touchpad callback to read the touchpad ***/
  void touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
  {
    uint16_t touchX, touchY;
    bool touched = lcd.getTouch(&touchX, &touchY);

    if (!touched)
    {
      data->state = LV_INDEV_STATE_REL;
    }
    else
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touchX;
      data->point.y = touchY;

      // Serial.printf("Touch (x,y): (%03d,%03d)\n",touchX,touchY );
    }
  }

// Get directory names from SD card and populate first dropdown
void populateDirectoryDropdown() {
  lv_dropdown_clear_options(ui_ddPresetsFolder);
  SDInit();
  root = SD.open("/");
  int i = 0;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      break;
    }
    if (entry.isDirectory()) {
      lv_dropdown_add_option(ui_ddPresetsFolder, entry.name(),LV_DROPDOWN_POS_LAST);
      strncpy(directoryNames[i], entry.name(), sizeof(directoryNames[i]));
      i++;
      if (i >= 20) {
        break;
      }
    }
    entry.close();
  }
  //root.rewindDirectory();
  
  
  populateFileDropdown();
}

// Get file names from selected directory and populate second dropdown
void populateFileDropdown() {

  lv_dropdown_clear_options(ui_ddPresetsFile);
  int i = 0;
  int dirIndex = lv_dropdown_get_selected(ui_ddPresetsFolder);
  String directoryPath = "/";
  directoryPath += directoryNames[dirIndex];
  File dir = SD.open(directoryPath.c_str());
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }
    if (!entry.isDirectory()) {
      lv_dropdown_add_option(ui_ddPresetsFile, entry.name(),LV_DROPDOWN_POS_LAST);
      strncpy(fileNames[i], entry.name(), sizeof(fileNames[i]));
      i++;
      if (i >= 20) {
        break;
      }
    }
    entry.close();
  }
  dir.close();
}




//TESLA STUFF



void sendByte(uint8_t dataByte) {
  for (int8_t bit=7; bit>=0; bit--) { // MSB
    digitalWrite(CCGDO0A, (dataByte & (1 << bit)) != 0 ? HIGH : LOW);
    delayMicroseconds(pulseWidth);
  }
}

void sendBits(int *buff, int length, int gdo0) {
  for (int i = 0; i < length; i++) {
    digitalWrite(gdo0, buff[i] < 0 ? LOW : HIGH);
    delayMicroseconds(abs(buff[i]));
  }
  digitalWrite(gdo0, LOW);
}

void sendTeslaSignal(float freq) {
  pinMode(CCGDO0A,OUTPUT);
  
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(2);
  ELECHOUSE_cc1101.setMHZ(freq);
  ELECHOUSE_cc1101.setDeviation(0);
  ELECHOUSE_cc1101.SetTx();
  for (uint8_t t=0; t < transmtesla; t++) {
    for (uint8_t i=0; i<messageLength; i++) sendByte(sequence[i]);
    digitalWrite(CCGDO0A, LOW);
    delay(messageDistance);
  }
  ELECHOUSE_cc1101.setSidle();
}






////OTA FUNCTIONS/////

void handleRoot() {
  String html = "<html><body>";
  html += "<h1>ESP32 Firmware Update</h1>";
  html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
  html += "<input type='file' name='firmware'>";
  html += "<input type='submit' value='Upload Firmware'>";
  html += "</form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleUpdate() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    lv_label_set_text_static(ui_lblSettingsStatus,"Starting firmware update");
    if (!Update.begin()) {
      lv_label_set_text_static(ui_lblSettingsStatus,"Update failed to begin.");
      server.send(500, "text/plain", "Update failed to begin.");
      return;
    }else{
    
    lv_label_set_text_static(ui_lblSettingsStatus,"Firmware update complete.");
    server.send(200, "text/plain", "Firmware update complete.");
  }
  }
}


///END OTA FUNCITONS///



// LCD Events - Should this be moved back to the ui_events ?? 

void setTxFlag(){
  lv_label_set_text_static(ui_lblMainStatus,"TRANSMITTING");
   //lv_img_set_src(ui_indMain,"TXInd.png");
  //delay(10);

}

void setRxFlag(){
lv_label_set_text_static(ui_lblMainStatus,"RECEIVING");
//lv_img_set_src(ui_indMain,"RXInd.png");
}

void setIdleFlag(){
  lv_label_set_text_static(ui_lblMainStatus,"IDLE");
  //lv_img_set_src(ui_indMain,"IdleInd.png");
}



void wifiScanAndUpdateUI(lv_obj_t* textarea, lv_obj_t* dropdown) {

  lv_textarea_set_text(ui_lblWifiScanNetsFound,"-Scanning-");
  // Scan for Wi-Fi networks
  int numNetworks = WiFi.scanNetworks();

  // Clear the textarea
  lv_textarea_set_text(textarea, "");

  // Clear the dropdown
  lv_dropdown_clear_options(dropdown);

  // Update the textarea and dropdown with the scanned networks
  for (int i = 0; i < numNetworks; i++) {
    // Get the SSID of the current network
    String ssid = WiFi.SSID(i);

    // Get the RSSI of the current network
    int32_t rssi = WiFi.RSSI(i);

    // Construct the line to be added to the textarea
    String line = ssid + " (RSSI: " + String(rssi) + " dBm)\n";

    // Update the textarea
    lv_textarea_add_text(textarea, line.c_str());

    // Update the dropdown
    lv_dropdown_add_option(dropdown, ssid.c_str(), LV_DROPDOWN_POS_LAST);
  }
   lv_textarea_set_text(ui_lblWifiScanNetsFound,"-Nets Found-");
}

void fcnTxTest(lv_event_t * e)
{
	// Your code here
  ELECHOUSE_cc1101.SetTx();
	lv_label_set_text_static(ui_lblMainStatus,"433 Test Complete");
  ELECHOUSE_cc1101.setMHZ(433.92);
  sendSamples(txTestcode,512); 
}




void fcnTeslaTx(lv_event_t * e)
{
	// Your code here
  lv_label_set_text_static(ui_lblMainStatus,"TX Tesla Begin");
  delay(100);
  sendTeslaSignal(315.00);
  lv_label_set_text_static(ui_lblMainStatus,"TX Tesla Complete");
  
}

void fcnMainReset(lv_event_t * e)
{
	// Your code here
  
  ESP.deepSleep(1);
}

void fcnSetPreset(lv_event_t * e)
{
	// Your code here
  lv_label_set_text_static(ui_lblMainStatus,"Preset Loaded");
}

void fcnPresetTx(lv_event_t * e)
{
char filebuffer[1024];
char folderbuffer[1024];
// Get the currently selected option
lv_dropdown_get_selected_str(ui_ddPresetsFolder, folderbuffer,1024);
lv_dropdown_get_selected_str(ui_ddPresetsFile, filebuffer,1024);
// Store the selected option in a C++ string variable
//String file2tx;
std::string selected_folder(folderbuffer);
std::string selected_file(filebuffer);
String fullfilename="/";
fullfilename+=folderbuffer;
fullfilename+="/";
fullfilename+=filebuffer;
//,String(fullfilename).c_str());
CC1101_TX=false;

ELECHOUSE_cc1101.setPA(12);
  transmitFlipperFile(String(fullfilename).c_str(), true);
  
}


void fcnProtAnaRxEn(lv_event_t * e)
{
    
       lv_label_set_text_static(ui_lblProtAnaRXEn,"RX ON");
       ELECHOUSE_cc1101.SetRx(CC1101_MHZ);
       CC1101_TX=false;
       ProtAnaRxEn=1;
}

void fcnProtAnaRxOff(lv_event_t * e)
{
	// Your code here
        lv_label_set_text_static(ui_lblProtAnaRXEn,"RX OFF");
        ELECHOUSE_cc1101.SetTx(CC1101_MHZ);
        CC1101_TX=true;
        ProtAnaRxEn=0;  
}

void fcnProtAnaCancel(lv_event_t * e)
{
	// Your code here
  ProtAnaRxEn=0;  
}

void fcnProtAnaClear(lv_event_t * e)
{
	// Your code here
  lv_textarea_set_text(ui_txtProtAnaResults,"");
  lv_textarea_set_text(ui_txtProtAnaBitLength,"-");
    lv_textarea_set_text(ui_txtProtAnaProtocol,"-");
    lv_textarea_set_text(ui_txtProtAnaDBPTD,"-");
    lv_textarea_set_text(ui_txtProtAnaReceived,"-");
}

void fcnPresetPopDir(lv_event_t * e)
{
	// Your code here
  //lv_dropdown_clear_options(ui_ddPresetsFolder);
  populateDirectoryDropdown();
}

void fcnPopulateFileDropdown(lv_event_t * e)
{
	// Your code here
  populateFileDropdown();
}


void fcnClearPresetsFilesDD(lv_event_t * e)
{
	// Your code here
  lv_dropdown_clear_options(ui_ddPresetsFile);
}

void fcnMainPreTX(lv_event_t * e)
{
	// Your code here
  playAudio();

}

void fcnGetCurfreq(lv_event_t * e)
{
  CC1101_MHZ = atof(lv_textarea_get_text(ui_txtMainFreq));
  lv_textarea_set_text(ui_txtKybdFreq,String(CC1101_MHZ).c_str());
}

void fcnSetFreq(lv_event_t * e)
{
  CC1101_MHZ = atof(lv_textarea_get_text(ui_txtKybdFreq));
  lv_textarea_set_text(ui_txtMainFreq,String(CC1101_MHZ).c_str());
}

void fcnReset(lv_event_t * e)
{
	// Your code here
  
  
}

void fcnSettingsOTA(lv_event_t * e)
{
	// Your code here
  OTAInProgress=1;
  WiFi.softAP(ssid, password,wifi_channel);
  
  lv_label_set_text_static(ui_lblSettingsStatus,"Connect to IP");
  
  lv_label_set_text_static(ui_lblSettingsStatus,"OTA READY");
  lv_label_set_text_static(ui_lblSettingsIPAddr,"192.168.4.1"); //Took Out String(WiFi.softAPIP()).c_str()
  // Start OTA
  ArduinoOTA.begin();
  
  // Set a callback function to reset the ESP32 after the update is completed  (DOESENT WORK - TODO)
  
  // Handle firmware update via web page
  server.on("/", HTTP_GET, handleRoot);
  server.on("/update", HTTP_POST, handleUpdate);
  server.onNotFound(handleRoot);
  server.begin();
  
}


void fcnMainConfig(lv_event_t * e)
{
	// Your code here
}



void fcnConfigTx(lv_event_t * e)
{
	// Your code here
}


void fcnKybdAlphaOk(lv_event_t * e)
{
	// Your code here
}

void fcnKybdAlphaCncl(lv_event_t * e)
{
	// Your code here
}

void fcnTCKybdTrans(lv_event_t * e)
{
	// Your code here
}

void fcnTCKybdFreq(lv_event_t * e)
{
	// Your code here
}

void fcnTCKybdDev(lv_event_t * e)
{
	// Your code here
}
void fcnScanWifi(lv_event_t * e)
{
	// Your code here
    wifiScanAndUpdateUI(ui_txtWifiScanNetsFound, ui_ddlWifiSSID);
  
}

void fcnRCSWTXOn(lv_event_t * e)
{
  lv_label_set_text_static(ui_lblRCSWStatus,"TX 'On' Command.");
  String FirstFive = lv_label_get_text(ui_lblBit0);
  FirstFive += lv_label_get_text(ui_lblBit1);
  FirstFive += lv_label_get_text(ui_lblBit2);
  FirstFive += lv_label_get_text(ui_lblBit3);
  FirstFive += lv_label_get_text(ui_lblBit4);

  String SecondFive = lv_label_get_text(ui_lblBit5);
  SecondFive += lv_label_get_text(ui_lblBit6);
  SecondFive += lv_label_get_text(ui_lblBit7);
  SecondFive += lv_label_get_text(ui_lblBit8);
  SecondFive += lv_label_get_text(ui_lblBit9);

  mySwitch.switchOn(String(FirstFive).c_str(),String(SecondFive).c_str());
  //delay(1000);
  String TxResult="TX ON: ";
  TxResult +=FirstFive;
  TxResult+=SecondFive;
  lv_label_set_text_static(ui_lblRCSWStatus,String(TxResult).c_str());
}
void fcnRCSWTXOff(lv_event_t * e)
{
lv_label_set_text_static(ui_lblRCSWStatus,"TX 'Off' Command.");
  String FirstFive = lv_label_get_text(ui_lblBit0);
  FirstFive += lv_label_get_text(ui_lblBit1);
  FirstFive += lv_label_get_text(ui_lblBit2);
  FirstFive += lv_label_get_text(ui_lblBit3);
  FirstFive += lv_label_get_text(ui_lblBit4);

  String SecondFive = lv_label_get_text(ui_lblBit5);
  SecondFive += lv_label_get_text(ui_lblBit6);
  SecondFive += lv_label_get_text(ui_lblBit7);
  SecondFive += lv_label_get_text(ui_lblBit8);
  SecondFive += lv_label_get_text(ui_lblBit9);

  mySwitch.switchOff(String(FirstFive).c_str(),String(SecondFive).c_str());
  //delay(1000);
  String TxResult="TX OFF: ";
  TxResult +=FirstFive;
  TxResult+=SecondFive;
  lv_label_set_text_static(ui_lblRCSWStatus,String(TxResult).c_str());
}
