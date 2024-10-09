
#define stid1 "updatehub"
#define stpw1 "12341234"

//#define stid1 "ASC"
//#define stpw1 "123456-a"

//#define stid2 "ASC"
//#define stpw2 "123456-a"

#define stid2 "Public WiFi Free"
#define stpw2 ""

#define testmode 0
#define debugs 0
#define nodedebug 0
#define GITdebug 0
#define relayDly 5

#define lcdoff 50
#define sd_cs 27
#define svled 12
#define txled 33
#define rxled 32
#define mcurx 16
#define mcutx 17
#define gpsrx 26
#define relay 15
#define BIS 0
#define oberx 4
#define sw1 5
#define OUT digitalWrite
#define PIN digitalRead

String firm;
float firmV, servernodeV, serverfirmV;
uint32_t timeout = 2000;

#include <ArduinoJson.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <NetworkClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
//#include <WiFiMulti.h>
//WiFiMulti wifiMulti;
//#include "mbedtls/sha256.h"

byte firmup, firm_timeout;
WiFiClient dclient;

#define SDC 2
#define MBI 3
#define BOX 4
#define FLS 5

#include <Ticker.h>
Ticker blinker;
byte timeok;

#include <FastBot2.h>
FastBot2 myBot;
byte botuse;
int64_t myid, groupid;
char bottoken[48];

WebServer server(80);
#include <WebSocketsServer.h>
WebSocketsServer webSocket = WebSocketsServer(81);

//원격업데이트
#include <HTTPClient.h>
HTTPClient http;
//#include <ESP32httpUpdate.h>

#include "main.h"
#include "login.h"
#include "xDRIV.h"
#include "Udate.h"

IPAddress apIP(8, 8, 8, 8);
IPAddress apsubnet(255, 255, 255, 0);
#include <EEPROM.h>

//captivePortal-------------------------------
#include <WiFiClient.h>
#include <DNSServer.h>
const byte DNS_PORT = 53;
DNSServer dnsServer;
//captivePortal------------------------------


#include <Wire.h>
//HardwareSerial LCDSerial(1);
#include <U8g2lib.h>
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#include "FS.h"
#include "LittleFS.h"
#include "SD.h"
File file;
File sdfile;
#include <ESPping.h>

#include "qrcoderm.h"
const int QRcode_Version = 6;//set the version (range 1->40)
const int QRcode_ECC = 0;


bool opened = false;
static byte updatecancel;
byte eepWrite, offlineupdate;
byte firmtumeout;
byte tic0, tic1, tic2, tic_1, tic_01, tic_001;
byte down, nodeupdate, upchk, writeErr, fileskip, nodeRxok, boot, nodetxok;
byte values[270] = {0xff, 0x00, 0xFF, 0xFF, 0x53, 0x46, 0x00, 0x00, 0x17, 0x00, 0x01, 0x00, 0x00};
byte delnode[32] = {0xFF, 0x00, 0x01, 0x00, 0x53, 0x45};
byte mon, days, hr24, minn, secn, virchk, servererr, VcheckOk, view_log;
uint16_t totalFilecnt, sendcnt, skipcnt, successcnt, yer, retrycnt;
float nodeVersion, freesize, totalsize;

byte SDreaderr, checkUpdate, autoupdate;
byte botsave, chatidsave, groupesave, bottest;
byte tgon, autotest, wrongfile, wrongfilecnt;
uint32_t filesize;
byte filesource, updatefinish ,mobileupdate;
byte lcdRotate, sdok, clientoff;
int8_t busdirection = -1;
uint16_t totalFrame;
uint32_t SMV;
uint16_t currentFrame, Pingtime;
int mcuVersion[4];
byte wifiConnected, patchup;
byte watchdog;
uint16_t writeerrcnt;
bool manuup, bisinComplete;

byte dir_cnt, dirno, dirscen;
byte folderupdate;
byte downerr, sdpup, qron, checkdp, mobilUpcnt;
uint16_t downErrcnt, sw0cnt, sw1cnt, lcdonTime, infilecnt;;
uint32_t httptotalsize;
byte lastConnected, bootup, versionerr;

String Fdir, Ffile;
String unreadfilelist;
String writeerrlist, macstr = "0000";
String apid, appw, logid, logpw;
String logtxt, skiplist, mainpath;
String busid, svr_add, svr_id, svr_pw;
String currentUdateFile, sdk;
String busnos = F("수신전");
String retrylist;
String noans = F("NA");
String downErrlist;
String strin = "";
String lastbus, SDtxt;
String sdlisdt, filepath, ssid;
String boxerr;
String dirlist;
byte downTarget, flashupdate, hotspoton;
String folderpath, upfilelist, keyRepo, downRepo;
String githubToken, githubID;
uint16_t fsTotalfile;

void setup() {
//Serial.begin(115200);
  Serial.begin(38400, SERIAL_8N1, 3, 1, 0);//RX,TX,0반전
  pinMode(sw1, INPUT_PULLUP);
  pinMode(svled, OUTPUT);
  pinMode(rxled, OUTPUT);
  pinMode(txled, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(BIS, OUTPUT);
  pinMode(14, OUTPUT);
  Serial.println(F("start"));
  OUT(relay, 0);
  OUT(BIS, 0);
  OUT(14, 0);
  OUT(svled, 1);
  OUT(rxled, 1);
  OUT(txled, 1);

  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_unifont_t_korean2);
  u8g2.setFontMode(0);
  u8g2.clearDisplay();
  Serial.println(F("\n\nstart"));
  String f = __FILE__;
  byte c, d;
  byte e = f.length();
  firm = f.substring(e - 9, e - 4);
  EEPROM.begin(4096);

  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 15);
    u8g2.print(F("MirSystem BDU"));

    u8g2.setCursor(0, 31);
    u8g2.print("V: " + firm);

    u8g2.setCursor(0, 47);
    u8g2.print("초기화중...");
  } while ( u8g2.nextPage());

  firmV = firm.toFloat();
  Serial.println(String(firmV));
  sdk = ESP.getSdkVersion();
  sdk = sdk.substring(1, 10);
  Serial.println(String(sdk));
  Serial2.begin(38400, SERIAL_8N1, mcurx, mcutx, 0);//0=반전

  if (LittleFS.begin(1)) {
    Serial.println(F("Mount LittleFS"));
  } else {
    Serial.println(F("LittleFS Fail"));
    return;
  }

  if (PIN(sw1) == 0) {
    delay(100);
    if (PIN(sw1) == 0) {
      factory_reset();
      ESP.restart();
    }
  }

  eep_read();
  if(lcdRotate == 0){
    u8g2.setDisplayRotation(U8G2_R0);
  }else{
    u8g2.setDisplayRotation(U8G2_R2);
  }

  sdinit();
  String dpversion = F("FV");
  dpversion += String(firmV) + F("   NV");
  dpversion += String(nodeVersion);

/*
  backup();
  Serial.println("****");
  restor();
*/    

  //esp_partition_erase_range(esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL), 0, ESP_PARTITION_SUBTYPE_DATA_NVS);
  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect();
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(stid1, stpw1);

  space_check();
  listNode(LittleFS, "/", 1);
  
  Serial.println(String(fsTotalfile) + F("파일"));
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String s1 = String(mac[4], HEX);
  String s2 = String(mac[5], HEX);
  if (mac[4] < 16)s1 == "0" + s1;
  if (mac[5] < 16)s2 == "0" + s2;
  s1 += s2;
  s1.toUpperCase();
  Serial.println(s1);
  macstr = s1;
  String t = String(freesize) + "/" + String(totalsize) + "MB";
  Serial.println(totalsize);
  Serial.println("");
  Serial.print(F("Total space:"));
  Serial.println(String(totalsize, 3) + F("MB"));
  Serial.print(F("Free space:"));
  Serial.println (String(freesize, 3) + F("MB"));
  String st = busid + " " + t;
  apstart();
  OUT(svled, 0);

  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 15);
    u8g2.print(F("MirSystem BDU"));

    u8g2.setCursor(0, 31);
    u8g2.print(dpversion);

    u8g2.setCursor(0, 47);
    u8g2.print("AP:" + apid);

    u8g2.setCursor(0, 62);
    u8g2.print(st);
  } while ( u8g2.nextPage());

  for(byte b=0; b<3; b++){
    mcuVersion[b] = -1;
    getmcu_ver(b);
  }
  virdp();

  byte ftest = 0;
  if(EEPROM.readByte(2049) != 1){
    EEPROM.writeByte(2049, 1);
    EEPROM.commit();
    ftest = 1;
  }
  serveron();
  space_check();
  OUT(svled, 0);
  OUT(rxled, 0);
  OUT(txled, 0);
  boot = 0;
  String ip = WiFi.localIP().toString();
  String s = busid;
  boot = 1;
  fileupload();
  OUT(relay, 0);
  OUT(BIS, 0);
  setPingTimeout(500);
  Pingtime = 999;

  sdinit();
  upfilelist = F("/autorun");
  nodeupdate = 1;
  totalFilecnt = 0;
  SDupdate();

  if (totalFilecnt > 0){
    skiplist = skiplist.substring(0, skiplist.length() - 1);
    sdok = 1;
    sdlisdt = F("warni[");
    sdlisdt += busid + F("] 오프라인 SD업데이트 완료\n전체파일: ");
    sdlisdt += String(totalFilecnt) + F("개\n성공: ");
    sdlisdt += String(successcnt) + F("개\n실패: ");
    sdlisdt += String(skipcnt) + F("개\n");
    sdlisdt += skiplist;
    Serial.println(sdlisdt);

    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 15);
      u8g2.print("성공:" + String(successcnt) + "개");
  
      u8g2.setCursor(0, 31);
      u8g2.print("실패:" + String(skipcnt) + "개");
  
      u8g2.setCursor(0, 47);
      u8g2.print("확인은 AP접속");
  
      u8g2.setCursor(0, 62);
      u8g2.print("스위치를누르세요");
    } while ( u8g2.nextPage());

    String hs = F("SD업데이트 ");
    hs += char(2);
    hs += F("성공:");
    hs += String(successcnt);
    hs += char(5);
    hs += " 실패:";
    hs += String(skipcnt);
    txtDp(3, hs);
    if(skipcnt > 0){
      delay(5000);
      hs = busid + char(5);
      hs += "실패" + String(skipcnt) + ":";
      hs += char(4);
      hs += skiplist.substring(0, 48);
      if (skipcnt > 0)txtDp(7, hs);
    }

    int cv = 0;
    while(PIN(sw1) == 1){
/*
      if(cv == 0){
        nodenosend("0035", 0);
      }
      else if(cv == 500){
        nodenosend("0035", 1);
      }
      else if(cv == 1000)cv = 0;
      delay(10);
      cv ++;
*/

      serverrun();
    }
  }
  qron = 3;
  nodeupdate = 0;
  Serial.println(F("BOOT"));
  //nodeVersion = 0.0;
  blinker.attach(0.001, onTimer);
}


byte statime, scantime, servercnt, wifioff;

void loop() {
  watchdog = 0;

  if(sdpup == 1){//SD개별
    sdPupdate();
    sdpup = 0;
  }
  else if(sdpup == 2){//SD폴더
    nodeupdate = 1;
    SDupdate();
    sdpup = 0;
  }

  if (nodeupdate == 2){
    updateInit();
    mcufirmupdate();
  }

//1초
  if (tic_1 == 1) {
    tic_1 = 0;
    //if(bootup < 30)bootup ++;

//핫스팟 자동
    if(hotspoton > 0){
      if(wifiConnected == 0)hotspoton = 0;
    }
    if(ssid == "updatehub"){
      if(statime == 0){
        Serial.println("statime == 0");
        if(hotspoton < 3){
          Serial.println("hotspoton < 3");
          chkping();
          gettime();
          hotspotupdate();
        }
        statime = 2;
      }
      scantime = 20;
    }else{
      if(wifiConnected != 3 && scantime < 40)scantime ++;
      if(scantime == 20){
        //scanwifi();
        //WiFi.disconnect();
        //WiFi.begin("Public WiFi Free", "");
        bootup = 0;
      }
      //Serial.println(String(wifiConnected) + "/" + String(hotspoton) + "/" + String(scantime) + "/" + String(bootup));
      if(scantime > 19){
        if(wifiConnected != 3){
          if(hotspoton < 3){
            if(scantime < 40){
              Pingtime = 999;
              wifioff ++;
              if(wifioff > 3){
                wifioff = 0;
                WiFi.begin(stid2, stpw2);
                Serial.print(F("WiFi.begin"));
              }
            }
          }
        }else{
          scantime = 40;
        }
      }
    }

    if (statime == 2) {
      if (ssid != "updatehub") {
        OUT(relay, 1);
        OUT(BIS, 1);
        delay(relayDly);
        txtDp(5, F("핫스팟종료"));
        MDNS.end();
        OUT(relay, 0);
        OUT(BIS, 0);
        statime = 3;
      }
    }


    if(mcuVersion[0] == -1 && millis() < 10000){
      for(byte b=0; b<3; b++){
        getmcu_ver(b);
      }
    }

    if(hotspoton == 0){
      if(wifiConnected == 3){
        if(Pingtime > 500)chkping();
        if(timeok == 0){
          gettime();
        }else{
          if(bootup == 0)bootup = 1;
        }
        if (bootup == 1) {
          String n0 = String(mcuVersion[0]);
          String n1 = String(mcuVersion[1]);
          String n2 = String(mcuVersion[2]);
          byte mcuerr = 0;
          if(mcuVersion[0] == -1){
            n0 = noans;
            mcuerr = 1;
          }
          if(mcuVersion[1] == -1){
            n1 = noans;
            mcuerr = 1;
          }
          if(mcuVersion[2] == -1){
            n2 = noans;
            mcuerr = 1;
          }
  /*
          logtxt = F("부팅완료 FV:");
          logtxt += String(firmV) + F(" SDK:");
          logtxt += sdk + F(" 플래시:");
          logtxt += String(freesize) + "/" + String(totalsize) + F("MB 서버핑:");
          logtxt += String(Pingtime) + F("\n노선:");
          logtxt += String(nodeVersion) + F(" 전:");
          logtxt += n0 + " 측:" + n1 + " 후:" + n2;
          if(mcuerr == 1)logtxt += F("\nDP보드 버전확인의 응답이 없습니다.");
          String st;
          int ttl = listNode(LittleFS, "/", 1);
          if(ttl == -1){
            st = F("\n내부플래시 읽기오류\n");
          }else{
            st = F("\n남은 노선파일: ");
            st += String(ttl) + F("개\n");
          }
          logtxt += st;
          logsave();
          tgTx();
  */
          bootup = 2;
        }

        myBot.tick();
        if(bootup == 2 && Pingtime < 500){
          versionerr = 0;
          if(ssid != "updatehub")version_check();
          bootup = 3;
          if(versionerr == 1){
            //logtxt = mainpath + F("/version경로에 cmd.txt파일이 없습니다.");
            //logsave();
            //tgTx();
            bootup = 4;
          }
        }
        if(bootup == 3 && upchk > 0 && autotest == 0 && nodeupdate == 0){
          if(checkUpdate >= upchk){
            checkUpdate = 0;
            versionerr = 0;
            if(ssid != "updatehub")version_check();
          }
        }
        if(bootup > 3){
          bootup ++;
          if(bootup == 184){
            bootup = 4;
            versionerr = 0;
            if(ssid != "updatehub")version_check();
            if(versionerr == 0)bootup = 3;
          }
        }

      }
    }else{
      if(bootup == 183){
        version_check();
        bootup = 3;
      }
      myBot.tick();
    }

    if(lcdonTime < lcdoff)lcdonTime ++;
    if(lcdonTime == lcdoff){
      lcdonTime ++;
      u8g2.clear();
    }

    if(Pingtime < 400 && bottest == 1 && nodeupdate == 0){
       myBot.attachUpdate(tgRx);
    }

    if (eepWrite == 1) {
      eepWrite = 0;
      EEPROM.commit();
    }

    //Serial.println(ESP.getFreeHeap());
    if (boot < 20) boot ++;
    if(checkdp > 0)checkdp ++;
    if(checkdp > 7)checkdp = 0;
    if (boot > 3) {
      if(qron == 0 && checkdp == 0 && firmup == 0 && lcdonTime < lcdoff){
        maindp();
      }
    }
  }



  if (tic_01 == 1) {//0.01초
    tic_01 = 0;
    if(flashupdate > 0){
      updateflash();
      flashupdate = 0;
    }
    if (offlineupdate == 0) {
      if (nodeupdate == 1){
        if(filesource == BOX){
          logsave();
          tgTx();
          logtxt = "";
          retrycnt = 0;
          while(1){
            skipcnt = 0;
            skiplist = "";
            UpdateFromList();
            upfilelist = skiplist;
            if(upfilelist == ""){
              nodeupdate = 0;
              break;
            }
            retrycnt ++;
            if(retrycnt > 2){
              nodeupdate = 0;
              break;
            }
          }
          UpdateReport();
        }
      }
    }
    if(mobileupdate == 3){
      Serial.println(F("Mobileupdate cancel"));
      server.stop();
      //WiFi.mode(WIFI_OFF);
      //WiFi.begin();
      apstart();
      serveron();
      mobilUpcnt = 0;
      updatecancel = 1;
      UpdateReport();
      mobileupdate = 0;
      socket_tx(F("uploadclr"));
      clientoff = 0;
    }
  }

  if (tic_001 == 1) {//0.001초
    servercnt ++;
    if(servercnt > 1){
      servercnt = 0;
      serverrun();
    }
    tic_001 = 0;
    if(manuup == 0)serialinstr();
    if (PIN(sw1) == 0) {
      sw1cnt ++;
      if(sw1cnt > 100 && sw1cnt < 120){
        sw1cnt = 130;
        if(lcdonTime < lcdoff)qron ++;
        if(qron > 2)qron = 0;
        lcdonTime = 0;
        if(qron == 0){
          maindp();
        }
        else if(qron == 1){
          qrdp();
        }
        else if(qron == 2){
          virdp();
        }
      }
    } else {
      sw1cnt = 0;
    }
  }
}
