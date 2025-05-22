#define BLYNK_PRINT Serial
// You should get Auth Token in the Blynk App.
#define BLYNK_TEMPLATE_ID           
#define BLYNK_TEMPLATE_NAME         
char BLYNK_AUTH_TOKEN[32]   =   "";
// Import required libraries
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SimpleKalmanFilter.h>
#include "index_html.h"
#include "data_config.h"
#include <EEPROM.h>
#include <Arduino_JSON.h>
#include "icon.h"
#include <Firebase_ESP_Client.h>
#include <time.h>  // Thư viện NTP
#include <HTTPClient.h>


#define API_KEY "AIzaSyAcjTbbmLAiaKpAgeKsqPfZSgxT1wHR4sE"
#define DATABASE_URL "https://khoacua-c27bb-default-rtdb.firebaseio.com"
#define USER_EMAIL "thuynguyenntt.15@gmail.com"
#define USER_PASSWORD "nguyenthuy1512"

// ==== ĐỐI TƯỢNG FIREBASE ====
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool passwordNeedsSync = false;  // Biến đánh dấu việc mật khẩu cần được đồng bộ

// ==== NTP CONFIG ====
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7 * 3600;  // GMT+7
const int   daylightOffset_sec = 0;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

//----------------------- Khai báo 1 số biến Blynk -----------------------
bool blynkConnect = true;
BlynkTimer timer; 
// Một số Macro
#define ENABLE    1
#define DISABLE   0
// ---------------------- Khai báo cho OLED 1.3 --------------------------
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define EEPROM_LOG_START 300      // Ô bắt đầu lưu log
#define MAX_LOGS 5                // Lưu tối đa 5 log, mỗi log ~40 byte

// Lưu log đơn giản dạng: method|status|note
void saveLogToEEPROM(String method, String status, String note) {
  String logEntry = method + "|" + status + "|" + note;
  int logCount = EEPROM.read(EEPROM_LOG_START);
  if (logCount >= MAX_LOGS) return;

  int addr = EEPROM_LOG_START + 1 + logCount * 40;
  for (int i = 0; i < logEntry.length() && i < 39; i++) {
    EEPROM.write(addr + i, logEntry[i]);
  }
  EEPROM.write(addr + logEntry.length(), '\0');
  EEPROM.write(EEPROM_LOG_START, logCount + 1);
  EEPROM.commit();
  Serial.println("💾 Lưu log cục bộ vào EEPROM: " + logEntry);
}

void syncLocalLogsToFirebase() {
  int logCount = EEPROM.read(EEPROM_LOG_START);
  if (logCount == 0) return;

  Serial.println("🔁 Đồng bộ " + String(logCount) + " log từ EEPROM lên Firebase...");

  for (int i = 0; i < logCount; i++) {
    int addr = EEPROM_LOG_START + 1 + i * 40;
    char buffer[41];
    for (int j = 0; j < 40; j++) {
      buffer[j] = EEPROM.read(addr + j);
    }
    buffer[40] = '\0';
    String logStr = String(buffer);
    int sep1 = logStr.indexOf('|');
    int sep2 = logStr.lastIndexOf('|');

    if (sep1 > 0 && sep2 > sep1) {
      String method = logStr.substring(0, sep1);
      String status = logStr.substring(sep1 + 1, sep2);
      String note = logStr.substring(sep2 + 1);
      sendLog(method, status, note);
    }
  }

  EEPROM.write(EEPROM_LOG_START, 0); // Xóa sau khi sync
  EEPROM.commit();
  Serial.println("✅ Đồng bộ log hoàn tất và xóa khỏi EEPROM.");
}


#define i2c_Address 0x3C //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G oled = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define OLED_SDA      21
#define OLED_SCL      22

typedef enum {
  SCREEN0,
  SCREEN1,
  SCREEN2,
  SCREEN3,
  SCREEN4,
  SCREEN5,
  SCREEN6,
  SCREEN7,
  SCREEN8,
  SCREEN9,
  SCREEN10,
  SCREEN11,
  SCREEN12,
  SCREEN13
}SCREEN;
int screenOLED = SCREEN0;

bool enableShow1 = DISABLE;
bool enableShow2 = DISABLE;
bool enableShow3 = DISABLE;
bool autoWarning = DISABLE;

// Khai bao LED
#define LED           33
#define LED_ON        0
#define LED_OFF        1
// Khai báo RELAY
#define RELAY         25
// Khai báo BUZZER
#define BUZZER        2
uint32_t timeCountBuzzerWarning = 0;

String OLED_STRING1 = "Xin chao";
String OLED_STRING2 = "Hi my friend";

String OLED_STRING3 = "Canh bao";
// ----------------------------- Khai báo Keypad -------------------------------------------
#include "Adafruit_Keypad.h"
#define KEYPAD_ROW1   4
#define KEYPAD_ROW2   16
#define KEYPAD_ROW3   17
#define KEYPAD_ROW4   5
#define KEYPAD_COL1   12
#define KEYPAD_COL2   14
#define KEYPAD_COL3   27
const byte ROWS = 4; // rows
const byte COLS = 3; // columns
// define the symbols on the buttons of the keypads
char keys[ROWS][COLS] = {
    {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'}};
byte rowPins[ROWS] = {KEYPAD_ROW1, KEYPAD_ROW2, KEYPAD_ROW3, KEYPAD_ROW4};
byte colPins[COLS] = {KEYPAD_COL1, KEYPAD_COL2, KEYPAD_COL3}; 
Adafruit_Keypad myKeypad = Adafruit_Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);
keypadEvent e;
typedef enum {
   IDLE_MODE,
   WAIT_CLOSE,
   CHANGE_PASSWORD,
   LOCK_1MIN
}MODE;
uint8_t modeRun = IDLE_MODE;
uint8_t allowAccess = 0;
uint8_t countError = 0;
uint8_t countKey = 0;
char passwordEnter[5] = {0}; 
char password[5] = "1111";            // mật khẩu mặc định
char keyMode;
uint32_t timeMillisAUTO=0;
uint8_t notiOne = 0;        

typedef enum {
   ENTER_PASS_1,
   ENTER_PASS_2
}CHANGEPASS;
uint8_t changePassword = ENTER_PASS_1;
char passwordChange1[5] = {0};        // mode đổi pass,mảng lưu pass lần 1
char passwordChange2[5] = {0};        // mode đổi pass,mảng lưu pass lần 2
char starkey[8];
// -----------------------------------------------------------------------

//-------------------- Khai báo Button-----------------------
#include "mybutton.h"
#define BUTTON_DOWN_PIN   34
#define BUTTON_UP_PIN     35
#define BUTTON_SET_PIN    32

#define BUTTON1_ID  1
#define BUTTON2_ID  2
#define BUTTON3_ID  3
Button buttonSET;
Button buttonDOWN;
Button buttonUP;
void button_press_short_callback(uint8_t button_id);
void button_press_long_callback(uint8_t button_id);
//------------------------------------------------------------
TaskHandle_t TaskButton_handle      = NULL;
TaskHandle_t TaskOLEDDisplay_handle = NULL;

void testHttpConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("🌐 WiFi đã kết nối, kiểm tra truy cập Internet...");

    HTTPClient http;
    http.begin("http://example.com"); // Có thể thay bằng "http://google.com"
    int httpCode = http.GET();

    if (httpCode > 0) {
      Serial.print("✅ HTTP Response Code: ");
      Serial.println(httpCode);
      Serial.println("🌐 Internet hoạt động bình thường");
    } else {
      Serial.print("❌ Không thể truy cập Internet. Lỗi: ");
      Serial.println(httpCode);
    }

    http.end();
  } else {
    Serial.println("❌ Chưa kết nối WiFi, không thể kiểm tra Internet");
  }
}

void setupTime() {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("🌐 Syncing time with NTP...");

    int retry = 0;
    while (time(nullptr) < 1000000000 && retry < 20) { // Thời gian hợp lệ > 1B
        Serial.print(".");
        delay(500);
        retry++;
    }

    time_t now = time(nullptr);
    if (now < 1000000000) {
        Serial.println("\n❌ Không thể đồng bộ thời gian qua NTP.");
    } else {
        Serial.println("\n✅ Thời gian đã được đồng bộ: " + String(ctime(&now)));
        Serial.println("🕒 UNIX Time: " + String(now));
    }
}

// ==== Hàm xử lý key Firebase không hợp lệ (Đã sửa) ====
String sanitizeKey(String key) {
    key.replace("/", "_");
    key.replace(".", "_");
    key.replace("#", "_");
    key.replace("$", "_");
    key.replace("[", "_");
    key.replace("]", "_");
    key.replace(" ", "_");  // Xử lý khoảng trắng
    return key;
}

// ==== Hàm lấy thời gian ISO dạng an toàn (Đã sửa) ====
String getISOTime() {
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);

    if (timeinfo == nullptr) {
        Serial.println("⚠️ Lỗi lấy thời gian! timeinfo = nullptr");
        return "";  // Thời gian chưa sẵn sàng
    }

    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y%m%dT%H%M%S", timeinfo);  // ISO format yyyymmddThhmmss
    return sanitizeKey(String(buffer));
}

// ==== Hàm gửi log lên Firebase (Đã chỉnh sửa) ====
void sendLog(String method, String status, String note) {
    Serial.println("🟡 BẮT ĐẦU GỬI LOG...");
    Serial.println("→ method: " + method);
    Serial.println("→ status: " + status);
    Serial.println("→ note: " + note);

    // Kiểm tra Firebase
    if (!Firebase.ready()) {
        Serial.println("⚠️ Firebase chưa sẵn sàng → lưu log cục bộ.");
        saveLogToEEPROM(method, status, note);
        return;
    }

    // Kiểm tra thời gian thực
    String isoTime = getISOTime();
    Serial.println("⏰ ISO Time: " + isoTime);
    if (isoTime == "") {
        Serial.println("⚠️ Thời gian thực chưa sẵn sàng, bỏ qua ghi log!");
        return;
    }

    // Tạo khóa log duy nhất từ thời gian và millis
    String key = isoTime + String(millis() % 1000);
    String path = "/door_logs/" + key;

    // Chuẩn bị dữ liệu JSON
    FirebaseJson json;
    json.set("timestamp", isoTime);
    json.set("method", sanitizeKey(method)); // "keypad", "blynk", v.v.
    json.set("status", sanitizeKey(status)); // "success", "fail"
    json.set("note", sanitizeKey(note));     // Ghi chú thêm

    // Debug: In JSON ra Serial
    String debugJson;
    json.toString(debugJson, true);
    Serial.println("📦 JSON gửi đi:\n" + debugJson);

    // Gửi lên Firebase
    Serial.println("📍 Đang ghi log vào Firebase tại: " + path);
    if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
        Serial.println("✅ Ghi log Firebase (JSON) thành công.");
    } else {
        Serial.println("❌ Lỗi ghi Firebase (JSON): " + fbdo.errorReason());
    }
}


void setup() {
    Serial.begin(115200);
    delay(2000); // Đợi Serial ổn định
    Serial.println("🔧 Bắt đầu setup...");

    EEPROM.begin(512);
    readEEPROM();

    // Cấu hình GPIO
    pinMode(RELAY, OUTPUT);  digitalWrite(RELAY, DISABLE);
    pinMode(BUZZER, OUTPUT); digitalWrite(BUZZER, DISABLE);
    pinMode(LED, OUTPUT);    digitalWrite(LED, LED_OFF);

    // OLED và Keypad
    myKeypad.begin();
    oled.begin(i2c_Address, true);
    oled.setTextSize(2);
    oled.setTextColor(SH110X_WHITE);

    autoWarning = EEPROM.read(210);
    savePASStoBUFF();
    EpassDoor = convertPassToNumber();

    // Button
    pinMode(BUTTON_SET_PIN, INPUT_PULLUP);
    pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
    pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
    button_init(&buttonSET, BUTTON_SET_PIN, BUTTON1_ID);
    button_init(&buttonUP, BUTTON_UP_PIN, BUTTON2_ID);
    button_init(&buttonDOWN, BUTTON_DOWN_PIN, BUTTON3_ID);
    button_pressshort_set_callback((void *)button_press_short_callback);
    button_presslong_set_callback((void *)button_press_long_callback);

    // Task
    xTaskCreatePinnedToCore(TaskButton, "TaskButton", 1024 * 10, NULL, 20, &TaskButton_handle, 1);
    xTaskCreatePinnedToCore(TaskOLEDDisplay, "TaskOLEDDisplay", 1024 * 16, NULL, 20, &TaskOLEDDisplay_handle, 1);
    xTaskCreatePinnedToCore(TaskKeypad, "TaskKeypad", 1024 * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(TaskCleanupOldLogs, "TaskCleanup", 8192, NULL, 1, NULL, 1);

    // Kết nối WiFi và NTP
    connectSTA();      
    setupTime();       
    testHttpConnection();

    // Firebase config
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    Serial.println("⏳ Đang kiểm tra kết nối Firebase...");
    int retry = 0;
    while (!Firebase.ready() && retry < 10) {
        Serial.println("⏳ Đợi Firebase...");
        delay(1000);
        retry++;
    }

    if (Firebase.ready()) {
        Serial.println("✅ Firebase đã sẵn sàng");

        syncLocalLogsToFirebase(); // Gửi log từ EEPROM nếu có

        if (Firebase.RTDB.setString(&fbdo, "/test", "ESP32 connected")) {
            Serial.println("📤 Gửi dữ liệu test thành công");
            sendLog("debug", "success", "test sendLog from setup");
        } else {
            Serial.println("❌ Gửi dữ liệu test thất bại: " + fbdo.errorReason());
        }
    } else {
        Serial.println("❌ Firebase vẫn chưa sẵn sàng sau khi thử lại");
    }

    // Khởi động task kiểm tra Firebase định kỳ
    xTaskCreatePinnedToCore(TaskFirebase, "TaskFirebase", 4096, NULL, 1, NULL, 1);
}


// ==== HÀM TÍNH UNIX TIME ====
unsigned long getUnixTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return 0;
  return mktime(&timeinfo);
}

void loop() {
  vTaskDelete(NULL);
  delay(10000);
}
bool trySendLog(String method, String status, String note, int retryCount = 3) {
    for (int i = 0; i < retryCount; i++) {
        if (Firebase.ready()) {
            sendLog(method, status, note);
            return true;
        }
        Serial.println("⏳ Firebase chưa sẵn sàng, đợi 500ms...");
        delay(500);
    }
    return false;
}
void TaskKeypad(void *pvParameters) {
    while (1) {
        switch (modeRun) {
            case IDLE_MODE:
                allowAccess = 0;

                if (notiOne == 1) {
                    Blynk.virtualWrite(V0, DISABLE);
                    digitalWrite(LED, LED_OFF);
                    Serial.println("Khoa cua");
                    controlLock(DISABLE);
                    if (!trySendLog("keypad", "success", "door_closed")) {
                        saveLogToEEPROM("keypad", "success", "door_closed");
                    }
                    notiOne = 0;

                    screenOLED = SCREEN1;
                    enableShow1 = ENABLE; enableShow2 = DISABLE; enableShow3 = DISABLE;
                    OLED_STRING1 = "Xin chao";
                    OLED_STRING2 = "Hi my friend";
                    strcpy(starkey, "        ");
                }

                myKeypad.tick();
                while (myKeypad.available()) {
                    e = myKeypad.read();
                    if (e.bit.EVENT == KEY_JUST_RELEASED) {
                        buzzerBeep(1);
                        char keyChar = (char)e.bit.KEY;

                        if (keyChar == '#') {
                            Serial.println("cancel");
                            countKey = 0;
                            notiOne = 1;
                            strcpy(starkey, "        ");
                            modeRun = IDLE_MODE;
                        } else {
                            passwordEnter[countKey] = keyChar;
                            starkey[countKey * 2] = '*';
                            starkey[countKey * 2 + 1] = ' ';
                            countKey++;

                            screenOLED = SCREEN2;
                            enableShow1 = DISABLE; enableShow2 = ENABLE; enableShow3 = DISABLE;
                            OLED_STRING1 = "Nhap mat khau";
                            OLED_STRING2 = starkey;
                        }
                    }
                }

                // Khi đủ 4 ký tự
                if (countKey > 3) {
                    passwordEnter[4] = '\0';
                    delay(50);
                    Serial.print("🔑 Nhap vao: ");
                    Serial.println(passwordEnter);
                    timeMillisAUTO = millis();

                    if (strcmp(passwordEnter, password) == 0) {
                        buzzerBeep(3);
                        allowAccess = 1;
                        Serial.println("✅ Mat khau dung");
                        enableShow2 = ENABLE; enableShow1 = DISABLE; enableShow3 = DISABLE;
                        screenOLED = SCREEN2;
                        OLED_STRING1 = "Mat khau dung";
                        OLED_STRING2 = "Moi vao";

                        digitalWrite(LED, LED_ON);
                        Serial.println("🔓 Mo khoa");
                        controlLock(ENABLE);

                        if (Firebase.ready()) {
                            Serial.println("🟢 Gọi sendLog → mật khẩu đúng");
                            sendLog("keypad", "success", "door opened");
                        } else {
                            Serial.println("⚠️ Firebase chưa sẵn sàng, lưu log success vào EEPROM");
                            saveLogToEEPROM("keypad", "success", "door opened");
                        }

                        modeRun = WAIT_CLOSE;

                    } else {
                        // 🔴 Mật khẩu sai
                        Serial.println("❌ Mat khau sai → chuẩn bị gửi log 'fail'");
                        Serial.println("1");  // ✅ Dòng bạn yêu cầu debug

                        enableShow2 = ENABLE; enableShow1 = DISABLE; enableShow3 = DISABLE;
                        screenOLED = SCREEN2;
                        OLED_STRING1 = "Mat khau sai";
                        OLED_STRING2 = "Con " + String(EnumberEnterWrong - countError - 1) + " lan";
                        buzzerWarningBeep(1000);

                        if (Firebase.ready()) {
                            Serial.println("🔴 Gọi sendLog → mật khẩu sai");
                            sendLog("keypad", "fail", "wrong password");
                        } else {
                            Serial.println("⚠️ Firebase chưa sẵn sàng → lưu log cục bộ");
                            saveLogToEEPROM("keypad", "fail", "wrong password");
                        }

                        if (autoWarning == 1) {
                            Blynk.logEvent("auto_warning", "Canh bao dot nhap");
                        }

                        countError++;
                        notiOne = 1;
                        delay(1000);

                        if (countError >= EnumberEnterWrong) {
                            modeRun = LOCK_1MIN;
                        }
                    }

                    // Reset lại mật khẩu nhập
                    countKey = 0;
                    strcpy(starkey, "        ");
                }
                break;

            case WAIT_CLOSE:
                countError = 0;
                notiOne = 1;
                if (millis() - timeMillisAUTO > EtimeOpenDoor * 1000) {
                    timeMillisAUTO = millis();
                    countKey = 0;
                    delay(20);
                    modeRun = IDLE_MODE;
                }

                myKeypad.tick();
                while (myKeypad.available()) {
                    e = myKeypad.read();
                    if (e.bit.EVENT == KEY_JUST_RELEASED) {
                        buzzerBeep(3);
                        char key = (char)e.bit.KEY;
                        Serial.println(key);
                        timeMillisAUTO = millis();

                        if (key == '*') {
                            if (EenableChangePass == ENABLE) {
                                Serial.println("mode doi pass");
                                enableShow3 = ENABLE; enableShow1 = DISABLE; enableShow2 = DISABLE;
                                screenOLED = SCREEN3;
                                OLED_STRING1 = "Mat khau moi";
                                OLED_STRING2 = "";

                                controlLock(DISABLE);
                                blinkLED(3);
                                strcpy(starkey, "        ");
                                modeRun = CHANGE_PASSWORD;
                            } else {
                                Serial.println("Khong cho phep doi mat khau");
                                enableShow3 = ENABLE; enableShow1 = DISABLE; enableShow2 = DISABLE;
                                screenOLED = SCREEN3;
                                OLED_STRING1 = "Khong the";
                                OLED_STRING2 = "doi mat khau";
                                controlLock(DISABLE);
                                buzzerWarningBeep(1000);
                                strcpy(starkey, "        ");
                                delay(1000);
                                modeRun = IDLE_MODE;
                            }
                        } else {
                            modeRun = IDLE_MODE;
                        }
                    }
                }
                break;

            case CHANGE_PASSWORD:
                changePasswordFunc();
                break;

            case LOCK_1MIN:
                notiOne = 1;
                Serial.println("thu lai sau 1 phut");
                buzzerWarningBeep(2000);
                for (int i = EtimeLock; i >= 0; i--) {
                    enableShow1 = ENABLE; enableShow2 = DISABLE; enableShow3 = DISABLE;
                    screenOLED = SCREEN1;
                    OLED_STRING1 = "Thu lai sau";
                    OLED_STRING2 = String(i) + " giay";
                    Serial.print("Thu lai sau "); Serial.println(i);
                    delay(1000);
                }
                countError = 0;
                modeRun = IDLE_MODE;
                break;
        }

        delay(10);  // Giữ CPU cho các task khác
    }
}

//--------------------------- Hàm Control Lock --------------------------------
void controlLock(int state) {
    digitalWrite(RELAY, state);
    Blynk.virtualWrite(V0, state);  
}
//---------------------- Hàm đổi mật khẩu ----------------------------------------------
void changePasswordFunc() {
  switch (changePassword) {
    case ENTER_PASS_1:
      myKeypad.tick();
      while (myKeypad.available()) {
        e = myKeypad.read();
        timeMillisAUTO = millis();

        if (e.bit.EVENT == KEY_JUST_RELEASED) {
          buzzerBeep(1);
          if ((char)e.bit.KEY == '#') {
            countKey = 0;
            modeRun = IDLE_MODE;
            Serial.println("cancel");
            notiOne = 1;
          } else {
            passwordChange1[countKey] = (char)e.bit.KEY;
            Serial.println((char)e.bit.KEY);
            starkey[countKey * 2] = '*';
            starkey[countKey * 2 + 1] = ' ';
            countKey++;
            // Hiển thị mật khẩu nhập vào
            screenOLED = SCREEN3;
            OLED_STRING1 = "Mat khau moi";
            OLED_STRING2 = starkey;
          }
        }
      }

      if (countKey > 3) {
        delay(200);
        changePassword = ENTER_PASS_2;
        buzzerBeep(2);
        countKey = 0;
        screenOLED = SCREEN3;
        OLED_STRING1 = "Nhap lai";
        OLED_STRING2 = "";
        memset(starkey, 0, sizeof(starkey));
      }
      break;

    case ENTER_PASS_2:
      myKeypad.tick();
      while (myKeypad.available()) {
        e = myKeypad.read();
        timeMillisAUTO = millis();

        if (e.bit.EVENT == KEY_JUST_RELEASED) {
          buzzerBeep(1);
          if ((char)e.bit.KEY == '#') {
            countKey = 0;
            modeRun = IDLE_MODE;
            Serial.println("cancel");
            notiOne = 1;
          } else {
            passwordChange2[countKey] = (char)e.bit.KEY;
            starkey[countKey * 2] = '*';
            starkey[countKey * 2 + 1] = ' ';
            countKey++;
            // Hiển thị mật khẩu nhập lại
            screenOLED = SCREEN3;
            OLED_STRING1 = "Nhap lai";
            OLED_STRING2 = starkey;
          }
        }
      }

      if (countKey > 3) {
        memset(starkey, 0, sizeof(starkey));
        delay(50);

        if (strcmp(passwordChange1, passwordChange2) == 0) {
          // Mật khẩu trùng khớp
          screenOLED = SCREEN3;
          OLED_STRING1 = "Doi mat khau";
          OLED_STRING2 = "Thanh cong";
          buzzerBeep(5);
          blinkLED(3);

          passwordChange2[4] = '\0';
          memcpy(password, passwordChange1, 5);

          Serial.print("new pass: ");
          Serial.println(password);
          Blynk.logEvent("auto_warning", "Cảnh báo đổi mật khẩu : " + String(password));

          // Lưu vào EEPROM
          uint32_t valueCV = convertPassToNumber();
          EpassDoor = valueCV;
          savePWtoEEP(valueCV); // Lưu mật khẩu vào EEPROM
          savePASStoBUFF(); // Cập nhật buffer

          // Gửi mật khẩu mới lên Firebase nếu có kết nối WiFi
          if (WiFi.status() == WL_CONNECTED) {
            passwordNeedsSync = true;  // Đánh dấu cần đồng bộ mật khẩu lên Firebase
          } else {
            Serial.println("⚠️ Không có kết nối WiFi, mật khẩu không được đồng bộ lên Firebase.");
          }

          Serial.println("CHANGE SUCCESSFUL");
          delay(2000);
          changePassword = ENTER_PASS_1;
          modeRun = IDLE_MODE;
        } else {
          // Mật khẩu nhập lại không đúng
          blinkLED(1);
          screenOLED = SCREEN3;
          OLED_STRING1 = "Mat khau sai";
          OLED_STRING2 = "Nhap lai";
          Serial.println("NOT CORRECT");
          buzzerWarningBeep(1000);
          changePassword = ENTER_PASS_1;
          strcpy(starkey, "        ");
          OLED_STRING1 = "Mat khau moi";
          OLED_STRING2 = "";
        }

        countKey = 0;
      }
      break;
  }
}


void TaskCleanupOldLogs(void *pvParameters) {
  while (1) {
    if (Firebase.ready()) {
      Firebase.RTDB.getJSON(&fbdo, "/door_logs");

      if (fbdo.dataType() == "json") {
        FirebaseJson &json = fbdo.jsonObject();
        size_t count = json.iteratorBegin();
        String key, value;
        int type;

        for (size_t i = 0; i < count; i++) {
          json.iteratorGet(i, type, key, value);
          // Giả sử key có định dạng YYYYMMDDTHHMMSS
          struct tm t;
          if (strptime(key.c_str(), "%Y%m%dT%H%M%S", &t)) {
            time_t logTime = mktime(&t);
            if (time(nullptr) - logTime > 3 * 24 * 3600) { // lớn hơn 3 ngày
              String path = "/door_logs/" + key;
              Firebase.RTDB.deleteNode(&fbdo, path.c_str());
              Serial.println("🗑️ Đã xóa log cũ: " + key);
            }
          }
        }
        json.iteratorEnd();
      }
    }
    delay(86400000); // 24 giờ
  }
}

// Xóa 1 ô hình chữ nhật từ tọa độ (x1,y1) đến (x2,y2)
void clearRectangle(int x1, int y1, int x2, int y2) {
   for(int i = y1; i < y2; i++) {
     oled.drawLine(x1, i, x2, i, 0);
   }
}

void clearOLED(){
  oled.clearDisplay();
  oled.display();
}

int countSCREEN9 = 0;
// Task hiển thị OLED
void TaskOLEDDisplay(void *pvParameters) {
  while (1) {
      switch(screenOLED) {
        case SCREEN0: // Hiệu ứng khởi động
          for(int j = 0; j < 3; j++) {
            for(int i = 0; i < FRAME_COUNT_loadingOLED; i++) {
              oled.clearDisplay();
              oled.drawBitmap(32, 0, loadingOLED[i], FRAME_WIDTH_64, FRAME_HEIGHT_64, 1);
              oled.display();
              delay(FRAME_DELAY/4);
            }
          }
          screenOLED = SCREEN4;
          break;
        case SCREEN1:   // Man hinh chao
          for(int j = 0; j < 2 && enableShow1 == ENABLE; j++) {
            for(int i = 0; i < FRAME_COUNT_hiOLED && enableShow1 == ENABLE; i++) {
              oled.clearDisplay();
              oled.setTextSize(1);
              oled.setCursor(40, 22);
              oled.print(OLED_STRING1);
              oled.setTextSize(1);
              oled.setCursor(40, 42);
              oled.print(OLED_STRING2);
              clearRectangle(0, 0, 32, 64);
              oled.drawBitmap(0, 16, hiOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
              oled.display();
              delay(FRAME_DELAY);
            }
            oled.display();
            delay(100);
          }
          break;
        case SCREEN2:   // Man hinh nhap key
          for(int j = 0; j < 2 && enableShow2 == ENABLE; j++) {
            for(int i = 0; i < FRAME_COUNT_keyOLED && enableShow2 == ENABLE; i++) {
              oled.clearDisplay();
              oled.setTextSize(1);
              oled.setCursor(40, 22);
              oled.print(OLED_STRING1);
              oled.setTextSize(1);
              oled.setCursor(40, 42);
              oled.print(OLED_STRING2);
              clearRectangle(0, 0, 32, 64);
              oled.drawBitmap(0, 16, keyOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
              oled.display();
              delay(FRAME_DELAY);
            }
            oled.display();
            delay(100);
          }
          break;
        case SCREEN3:  // Man hinh doi mat khau
          for(int j = 0; j < 2 && enableShow3 == ENABLE; j++) {
            for(int i = 0; i < FRAME_COUNT_lockOpenOLED && enableShow3 == ENABLE; i++) {
              oled.clearDisplay();
              
              oled.setTextSize(1);
              oled.setCursor(40, 22);
              oled.print(OLED_STRING1);
              oled.setTextSize(1);
              oled.setCursor(40, 42);
              oled.print(OLED_STRING2);
              clearRectangle(0, 0, 32, 64);
              oled.drawBitmap(0, 16, lockOpenOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
              oled.display();
              delay(FRAME_DELAY);
            }
            oled.display();
            delay(100);
          }
          break; 
        case SCREEN4:    // Đang kết nối Wifi
          oled.clearDisplay();
          oled.setTextSize(1);
          oled.setCursor(40, 5);
          oled.print("WIFI");
          oled.setTextSize(1.5);
          oled.setCursor(40, 17);
          oled.print("Dang ket noi..");
      
          for(int i = 0; i < FRAME_COUNT_wifiOLED; i++) {
            clearRectangle(0, 0, 32, 32);
            oled.drawBitmap(0, 0, wifiOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
            oled.display();
            delay(FRAME_DELAY);
          }
          break;
        case SCREEN5:    // Kết nối wifi thất bại
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(40, 5);
            oled.print("WIFI");
            oled.setTextSize(1.5);
            oled.setCursor(40, 17);
            oled.print("Mat ket noi.");
            oled.drawBitmap(0, 0, wifiOLED[FRAME_COUNT_wifiOLED - 1 ], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
            oled.drawLine(31, 0 , 0, 31 , 1);
            oled.drawLine(32, 0 , 0, 32 , 1);
            oled.display();
            delay(2000);
            screenOLED = SCREEN9;
          break;
        case SCREEN6:   // Đã kết nối Wifi, đang kết nối Blynk
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(40, 5);
            oled.print("WIFI");
            oled.setTextSize(1.5);
            oled.setCursor(40, 17);
            oled.print("Da ket noi.");
            oled.drawBitmap(0, 0, wifiOLED[FRAME_COUNT_wifiOLED - 1 ], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);

            oled.setTextSize(1);
            oled.setCursor(40, 34);
            oled.print("BLYNK");
            oled.setTextSize(1.5);
            oled.setCursor(40, 51);
            oled.print("Dang ket noi..");
                        

            for(int i = 0; i < FRAME_COUNT_blynkOLED; i++) {
              clearRectangle(0, 32, 32, 64);
              oled.drawBitmap(0, 32, blynkOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
              oled.display();
              delay(FRAME_DELAY);
            }

          break;
        case SCREEN7:   // Đã kết nối Wifi, Đã kết nối Blynk
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(40, 5);
            oled.print("WIFI");
            oled.setTextSize(1.5);
            oled.setCursor(40, 17);
            oled.print("Da ket noi.");
            oled.drawBitmap(0, 0, wifiOLED[FRAME_COUNT_wifiOLED - 1 ], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);

            oled.setTextSize(1);
            oled.setCursor(40, 34);
            oled.print("BLYNK");
            oled.setTextSize(1.5);
            oled.setCursor(40, 51);
            oled.print("Da ket noi.");
            oled.drawBitmap(0, 32, blynkOLED[FRAME_COUNT_wifiOLED/2], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
            oled.display();
            delay(2000);
            screenOLED = SCREEN1;
          break;
        case SCREEN8:   // Đã kết nối Wifi, Mat kết nối Blynk
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(40, 5);
            oled.print("WIFI");
            oled.setTextSize(1.5);
            oled.setCursor(40, 17);
            oled.print("Da ket noi.");
            oled.drawBitmap(0, 0, wifiOLED[FRAME_COUNT_wifiOLED - 1 ], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);

            oled.setTextSize(1);
            oled.setCursor(40, 34);
            oled.print("BLYNK");
            oled.setTextSize(1.5);
            oled.setCursor(40, 51);
            oled.print("Mat ket noi.");
            oled.drawBitmap(0, 32, blynkOLED[FRAME_COUNT_wifiOLED/2], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
            oled.drawLine(31, 32 , 0, 63 , 1);
            oled.drawLine(32, 32 , 0, 64 , 1);
            oled.display();
            delay(2000);
            screenOLED = SCREEN9;
          break;
        case SCREEN9:   // Cai đặt 192.168.4.1
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(40, 5);
            oled.setTextSize(1);
            oled.print("Ket noi Wifi:");
            oled.setCursor(40, 17);
            oled.setTextSize(1);
            oled.print("ESP32_IOT");

            oled.setCursor(40, 38);
            oled.print("Dia chi IP:");
    
            oled.setCursor(40, 50);
            oled.print("192.168.4.1");

            for(int i = 0; i < FRAME_COUNT_settingOLED; i++) {
              clearRectangle(0, 0, 32, 64);
              oled.drawBitmap(0, 16, settingOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
              oled.display();
              delay(FRAME_DELAY*2);
            }
            countSCREEN9++;
            if(countSCREEN9 > 10) {
              countSCREEN9 = 0;
              screenOLED = SCREEN1;
              enableShow1 = ENABLE;
            }

            break;
          case SCREEN10:    // auto : on
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(40, 20);
            oled.print(OLED_STRING3);
            oled.setTextSize(2);
            oled.setCursor(40, 32);
            oled.print("DISABLE"); 
            for(int i = 0; i < FRAME_COUNT_autoOnOLED; i++) {
              clearRectangle(0, 0, 32, 64);
              oled.drawBitmap(0, 16, autoOnOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
              oled.display();
              delay(FRAME_DELAY);
            }
            clearRectangle(40, 32, 128, 64);
            oled.setCursor(40, 32);
            oled.print("ENABLE"); 
            oled.display();   
            delay(2000);
            screenOLED = SCREEN1;
            enableShow1 = ENABLE;
            break;
          case SCREEN11:     // auto : off
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(40, 20);
            oled.print(OLED_STRING3);
            oled.setTextSize(2);
            oled.setCursor(40, 32);
            oled.print("ENABLE");
            for(int i = 0; i < FRAME_COUNT_autoOffOLED; i++) {
              clearRectangle(0, 0, 32, 64);
              oled.drawBitmap(0, 16, autoOffOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
              oled.display();
              delay(FRAME_DELAY);
            }
            clearRectangle(40, 32, 128, 64);
            oled.setCursor(40, 32);
            oled.print("DISABLE"); 
            oled.display();    
            delay(2000);
            screenOLED = SCREEN1;  
            enableShow1 = ENABLE;
            break;
          case SCREEN12:  // gui du lieu len blynk
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(40, 20);
            oled.print("Gui du lieu");
            oled.setCursor(40, 32);
            oled.print("den BLYNK"); 
            for(int i = 0; i < FRAME_COUNT_sendDataOLED; i++) {
                clearRectangle(0, 0, 32, 64);
                oled.drawBitmap(0, 16, sendDataOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
                oled.display();
                delay(FRAME_DELAY);
            } 
            delay(1000);
            screenOLED = SCREEN1; 
            enableShow1 = ENABLE;
            break;
          case SCREEN13:   // khoi dong lai
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(0, 20);
            oled.print("Khoi dong lai");
            oled.setCursor(0, 32);
            oled.print("Vui long doi ..."); 
            oled.display();
            break;
          default : 
            delay(500);
            break;
      } 
      delay(10);
  }
}
// lưu pass từ EEPROM vào mảng
void savePASStoBUFF()
{
    uint32_t c,d;
    c = EEPROM.read(250);
    d = EEPROM.read(251);
    if(c == 255 && d == 255) {
       memcpy (password, "1111", 4);
    } else {
      password[3] =  (char) (d %10 + 48 ) ;
      password[2] =  (char) (d /10 + 48 ) ;
      password[1] =  (char) (c %10 + 48 ) ;
      password[0] =  (char) (c /10 + 48 ) ;  
    }
    Serial.println(password);

}
// chuyển mảng thành số
uint32_t convertPassToNumber()
{
     uint32_t valuee = ((uint32_t)password[3] - 48) + ((uint32_t)password[2]-48)*10 +
    ((uint32_t)password[1]-48)*100 + ((uint32_t)password[0]-48)*1000;
    Serial.println(valuee);
    return valuee;
}

void savePWtoEEP(uint32_t valuePASS)
{
    uint32_t c,d;
    d = valuePASS / 100;
    c = valuePASS % 100;

    EEPROM.write(250, d);EEPROM.commit();
    EEPROM.write(251, c);EEPROM.commit();
    Serial.println( d);
    Serial.println( c);
}



//-----------------Kết nối STA wifi, chuyển sang wifi AP nếu kết nối thất bại ----------------------- 
void connectSTA() {
      delay(5000);
      enableShow1 = DISABLE;
      if ( Essid.length() > 1 ) {  
      Serial.println(Essid);        //Print SSID
      Serial.println(Epass);        //Print Password
      Serial.println(Etoken);        //Print token
      Etoken = Etoken.c_str();
      WiFi.begin(Essid.c_str(), Epass.c_str());   //c_str()
      int countConnect = 0;
      while (WiFi.status() != WL_CONNECTED) {
          delay(500);   
          if(countConnect++  == 15) {
            Serial.println("Ket noi Wifi that bai");
            Serial.println("Kiem tra SSID & PASS");
            Serial.println("Ket noi Wifi: ESP32 de cau hinh");
            Serial.println("IP: 192.168.4.1");
            screenOLED = SCREEN5;
            digitalWrite(BUZZER, ENABLE);
            delay(2000);
            digitalWrite(BUZZER, DISABLE);
            delay(3000);
            break;
          }
          // MODE đang kết nối wifi
          screenOLED = SCREEN4;
          delay(2000);
      }
      Serial.println("");
      if(WiFi.status() == WL_CONNECTED) {
        Serial.println("Da ket noi Wifi: ");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP()); 
        Serial.println((char*)Essid.c_str());

       // MODE wifi đã kết nối, đang kết nối blynk
       screenOLED = SCREEN6;
       delay(2000);
        strcpy(BLYNK_AUTH_TOKEN,Etoken.c_str());
        
        Blynk.config(BLYNK_AUTH_TOKEN);
        blynkConnect = Blynk.connect();
        if(blynkConnect == false) {
            screenOLED = SCREEN8;
            delay(2000);
            connectAPMode(); 
        }
        else {
            Serial.println("Da ket noi BLYNK");
            enableShow1 = ENABLE;
            // MODE đã kết nối wifi, đã kết nối blynk
            screenOLED = SCREEN7;
            delay(2000);
            xTaskCreatePinnedToCore(TaskBlynk,            "TaskBlynk" ,           1024*16 ,  NULL,  20  ,  NULL ,  1); 
            xTaskCreatePinnedToCore(TaskFirebase, "TaskFirebase", 4096, NULL, 1, NULL, 1);
            buzzerBeep(5);  
            return; 
        }
      }
      else {
        digitalWrite(BUZZER, ENABLE);
        delay(2000);
        digitalWrite(BUZZER, DISABLE);
        // MODE truy cập vào 192.168.4.1
        screenOLED = SCREEN9;
        connectAPMode(); 
      }
        
    }
}


//--------------------------- switch AP Mode --------------------------- 
void connectAPMode() {

  // Khởi tạo Wifi AP Mode, vui lòng kết nối wifi ESP32, truy cập 192.168.4.1
  WiFi.softAP(ssidAP, passwordAP);  

  // Gửi trang HTML khi client truy cập 192.168.4.1
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Gửi data ban đầu đến clientgetDataFromClient
  server.on("/data_before", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = getJsonData();
    request->send(200, "application/json", json);
  });

  // Get data từ client
  server.on("/post_data", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "SUCCESS");
    enableShow1 = DISABLE;
    screenOLED = SCREEN13;
    delay(5000);
    ESP.restart();
  }, NULL, getDataFromClient);

  // Start server
  server.begin();
}

//------------------- Hàm đọc data từ client gửi từ HTTP_POST "/post_data" -------------------
void getDataFromClient(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  Serial.print("get data : ");
  Serial.println((char *)data);
  JSONVar myObject = JSON.parse((char *)data);
  if(myObject.hasOwnProperty("ssid"))
    Essid = (const char*) myObject["ssid"];
  if(myObject.hasOwnProperty("pass"))
    Epass = (const char*)myObject["pass"] ;
  if(myObject.hasOwnProperty("token"))
    Etoken = (const char*)myObject["token"];
  if(myObject.hasOwnProperty("passDoor"))
    EpassDoor = (int) myObject["passDoor"];
  if(myObject.hasOwnProperty("timeOpenDoor"))
    EtimeOpenDoor = (int) myObject["timeOpenDoor"];
  if(myObject.hasOwnProperty("numberEnterWrong")) 
    EnumberEnterWrong = (int) myObject["numberEnterWrong"];
  if(myObject.hasOwnProperty("timeLock")) 
    EtimeLock = (int) myObject["timeLock"];

  writeEEPROM();
  savePWtoEEP(EpassDoor);
  savePASStoBUFF();

}

// ------------ Hàm in các giá trị cài đặt ------------
void printValueSetup() {
    Serial.print("ssid = ");
    Serial.println(Essid);
    Serial.print("pass = ");
    Serial.println(Epass);
    Serial.print("token = ");
    Serial.println(Etoken);
    Serial.print("passDoor = ");
    Serial.println(EpassDoor);
    Serial.print("timeOpenDoor = ");
    Serial.println(EtimeOpenDoor);
    Serial.print("numberEnterWrong = ");
    Serial.println(EnumberEnterWrong);
    Serial.print("timeLock = ");
    Serial.println(EtimeLock);
    Serial.print("enableChangePass = ");
    Serial.println(EenableChangePass);
    
    Serial.print("autoWarning = ");
    Serial.println(autoWarning);
}

//-------- Hàm tạo biến JSON để gửi đi khi có request HTTP_GET "/" --------
String getJsonData() {
  JSONVar myObject;
  myObject["ssid"]  = Essid;
  myObject["pass"]  = Epass;
  myObject["token"] = Etoken;
  myObject["passDoor"] = EpassDoor;
  myObject["timeOpenDoor"] = EtimeOpenDoor;
  myObject["numberEnterWrong"] = EnumberEnterWrong;
  myObject["timeLock"] = EtimeLock;
  
  String jsonData = JSON.stringify(myObject);
  return jsonData;
}

//-------------------------------------------------------------------------------
//--------------------------------Task Blynk-------------------------------------

//------------------------- check autoWarning from BLYNK  -----------------------
BLYNK_WRITE(V3) {
  buzzerBeep(2);
  Blynk.logEvent("check_data","Mật khẩu: " + String(password));
}
BLYNK_WRITE(V2) {
    enableShow1 = DISABLE;
    EenableChangePass = param.asInt();
    buzzerBeep(2);
    EEPROM.write(204, EenableChangePass);  EEPROM.commit();
    Serial.print("EenableChangePass : "); Serial.println(EenableChangePass);
    OLED_STRING3 = "Doi mat khau";
    if(EenableChangePass == 0) screenOLED = SCREEN11;
    else screenOLED = SCREEN10;
}
BLYNK_WRITE(V1) {
    enableShow1 = DISABLE;
    autoWarning = param.asInt();
    buzzerBeep(1);
    EEPROM.write(210, autoWarning);  EEPROM.commit();
    OLED_STRING3 = "Canh bao";
    if(autoWarning == 0) screenOLED = SCREEN11;
    else screenOLED = SCREEN10;
}
//------------------------- check autoWarning from BLYNK  -----------------------
BLYNK_WRITE(V0) {
    int state = param.asInt();
    if(state == 1) {
        Serial.println("open door");
        controlLock(ENABLE);
        buzzerBeep(3);
        delay(EtimeOpenDoor*1000);
        controlLock(DISABLE);

        // Gửi log trong task (tránh treo)
        xTaskCreatePinnedToCore(TaskLogBlynk, "TaskLogBlynk", 4096, NULL, 1, NULL, 0);
    }    
}

void TaskLogBlynk(void *pvParameters) {
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Đợi 1s cho chắc

  Serial.println("📥 TaskLogBlynk đang gửi log...");
  if (Firebase.ready()) {
    sendLog("blynk", "success", "door_opened");  // <== THÊM DÒNG NÀY
  } else {
    Serial.println("⚠️ Firebase chưa sẵn sàng trong TaskLogBlynk");
    saveLogToEEPROM("blynk", "success", "door_opened");
  }

  vTaskDelete(NULL);
}



//---------------------------Task TaskSwitchAPtoSTA---------------------------
void TaskBlynk(void *pvParameters) {
    Blynk.virtualWrite(V1, autoWarning); 
    Blynk.virtualWrite(V2, EenableChangePass); 
    while(1) {
      Blynk.run();
      timer.run(); 
      delay(10);
    }
}
void TaskFirebase(void *pvParameters) {
  while (1) {
    // Kiểm tra trạng thái Firebase
    if (!Firebase.ready()) {
      Serial.println("⚠️ Firebase chưa sẵn sàng! Đang thử kết nối lại...");

      // Thử kết nối lại Firebase
      Firebase.begin(&config, &auth);
      Firebase.reconnectWiFi(true);

      delay(2000);  // Đợi Firebase phản hồi

      if (Firebase.ready()) {
        Serial.println("✅ Firebase đã kết nối lại thành công!");
        syncLocalLogsToFirebase();  // Đồng bộ các logs nếu cần
      } else {
        Serial.println("❌ Firebase vẫn chưa sẵn sàng: " + fbdo.errorReason());
      }
    }

    // Nếu cần đồng bộ mật khẩu lên Firebase, kiểm tra và thực hiện đồng bộ
    if (passwordNeedsSync) {
      Serial.println("🔄 Đồng bộ mật khẩu lên Firebase...");
      
      String passwordStr = String(password);  // Chuyển mật khẩu sang chuỗi

      // Cập nhật mật khẩu mới lên Firebase
      String path = "/password";  // Đường dẫn lưu mật khẩu trong Firebase
      FirebaseJson json;
      json.set("value", passwordStr);       // Mật khẩu mới
      json.set("updated_at", getISOTime()); // Thời gian cập nhật

      if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
        Serial.println("✅ Đồng bộ mật khẩu thành công.");
        passwordNeedsSync = false;  // Reset flag sau khi đồng bộ thành công
      } else {
        Serial.println("❌ Lỗi đồng bộ mật khẩu: " + fbdo.errorReason());
      }
    }

    // Kiểm tra lại trạng thái Firebase mỗi 5 giây
    delay(5000);
  }
}

/*
 * Các hàm liên quan đến lưu dữ liệu cài đặt vào EEPROM
*/
//--------------------------- Read Eeprom  --------------------------------
void readEEPROM() {
    for (int i = 0; i < 32; ++i)       //Reading SSID
        Essid += char(EEPROM.read(i)); 
    for (int i = 32; i < 64; ++i)      //Reading Password
        Epass += char(EEPROM.read(i)); 
    for (int i = 64; i < 96; ++i)      //Reading Password
        Etoken += char(EEPROM.read(i)); 
    if(Essid.length() == 0) Essid = "BLK";

    EtimeOpenDoor = EEPROM.read(200);
    EnumberEnterWrong = EEPROM.read(201);

    EtimeLock = EEPROM.read(202)*100 + EEPROM.read(203);
    EenableChangePass = EEPROM.read(204);

    autoWarning     = EEPROM.read(210);
    printValueSetup();
}

// ------------------------ Clear Eeprom ------------------------

void clearEeprom() {
    Serial.println("Clearing Eeprom");
    for (int i = 0; i < 300; ++i) 
      EEPROM.write(i, 0);
}

// -------------------- Hàm ghi data vào EEPROM ------------------
void writeEEPROM() {
    clearEeprom();
    for (int i = 0; i < Essid.length(); ++i)
          EEPROM.write(i, Essid[i]);  
    for (int i = 0; i < Epass.length(); ++i)
          EEPROM.write(32+i, Epass[i]);
    for (int i = 0; i < Etoken.length(); ++i)
          EEPROM.write(64+i, Etoken[i]);

    EEPROM.write(200, EtimeOpenDoor);              // lưu thời gian mở khóa
    EEPROM.write(201, EnumberEnterWrong);          // lưu số lần nhập sai tối đa

    EEPROM.write(202, EtimeLock / 100);      // lưu hàng nghìn + trăm thời gian khóa
    EEPROM.write(203, EtimeLock % 100);      // lưu hàng chục + đơn vị thời gian khóa
    EEPROM.write(204, EenableChangePass);    // lưu giá trị cho phép thay đổi mật khẩu
    EEPROM.commit();

    Serial.println("write eeprom");
    delay(500);
}


//-----------------------Task Task Button ----------
void TaskButton(void *pvParameters) {
    while(1) {
      handle_button(&buttonSET);
      handle_button(&buttonUP);
      handle_button(&buttonDOWN);
      delay(10);
    }
}
//-----------------Hàm xử lí nút nhấn nhả ----------------------
void button_press_short_callback(uint8_t button_id) {
    switch(button_id) {
      case BUTTON1_ID :  
        buzzerBeep(1);
        Serial.println("btSET press short");
        break;
      case BUTTON2_ID :
        buzzerBeep(1);
        Serial.println("btUP press short");
        break;
      case BUTTON3_ID :
        Serial.println("btDOWN press short");
        digitalWrite(RELAY, ENABLE);
        buzzerBeep(3);
        delay(EtimeOpenDoor*1000);
        digitalWrite(RELAY, DISABLE);
        
        break;  
    } 
} 
//-----------------Hàm xử lí nút nhấn giữ ----------------------
void button_press_long_callback(uint8_t button_id) {
  switch(button_id) {
    case BUTTON1_ID :
      buzzerBeep(2);  
      enableShow1 = DISABLE;
      Serial.println("btSET press long");
      screenOLED = SCREEN9;
      clearOLED();
      connectAPMode(); 
      break;
    case BUTTON2_ID :
      buzzerBeep(2);
      Serial.println("btUP press short");
      break;
    case BUTTON3_ID :
      buzzerBeep(2);
      Serial.println("btDOWN press short");
      OLED_STRING3 = "Canh bao";
      enableShow1 = DISABLE;
      autoWarning = 1 - autoWarning;
      EEPROM.write(210, autoWarning);  EEPROM.commit();
      Blynk.virtualWrite(V1, autoWarning); 
      if(autoWarning == 0) screenOLED = SCREEN11;
      else screenOLED = SCREEN10;
      break;  
  } 
} 
// ---------------------- Hàm điều khiển còi -----------------------------
void buzzerBeep(int numberBeep) {
  for(int i = 0; i < numberBeep; ++i) {
    digitalWrite(BUZZER, ENABLE);
    delay(100);
    digitalWrite(BUZZER, DISABLE);
    delay(100);
  }  
}
void buzzerWarningBeep(int time) {
    digitalWrite(BUZZER, ENABLE);
    delay(time);
    digitalWrite(BUZZER, DISABLE);
}
// ---------------------- Hàm điều khiển LED -----------------------------
void blinkLED(int numberBlink) {
  for(int i = 0; i < numberBlink; ++i) {
    digitalWrite(LED, DISABLE);
    delay(300);
    digitalWrite(LED, ENABLE);
    delay(300);
  } 
}

