#ifndef _MY_CONFIG_H_
#define _MY_CONFIG_H_

const char* ssidAP     = "ESP32_IOT";     // Tên wifi AP Mode
const char* passwordAP = "";          // Mật khẩu AP Mode

String  Essid   = "";                 // EEPROM tên wifi nhà bạn
String  Epass   = "";                 // EEPROM mật khẩu wifi nhà bạn
String  Etoken = "";                  // EEPROM mã token blynk
int  EtimeOpenDoor = 0;               // thời gian mở khóa
int  EnumberEnterWrong = 0;           // số lần nhập sai tối đa
int  EtimeLock = 0;                   // thời gian khóa khi nhập sai N lần
int  EenableChangePass = 0;
int  EpassDoor = 1111;



#endif