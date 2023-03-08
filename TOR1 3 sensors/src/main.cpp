using namespace std;
/*********

*********/
#define ILLUMINATION
// #define HYDROGEN_SENSOR
// #define DEBUG1
// #define DEBUG3
// #define DEBUG5
// #define DEBUG485
// #define RXD0 2 //RX0 pin
// #define TXD0 13 //TX0 pin
#define RXD1 35          // 35         //RX1 pin
#define TXD1 32          // 32            //TX1 pin
#define RXD2 16          // RX2 pin
#define TXD2 17          // TX2 pin
#define RXD3 34          // RX3 pin
#define TXD3 13          // TX3 pin
#define MAX485_RE_NEG 14 // dir

#define LED1 27
#define LED3 12

#define RELAY 2            // выход включения рлее
#define BUZZER 12          // бузер
#define Slave_ID_Moist 243 // адресс датчика влажности
#define Slave_ID_H2 1      // адрес датчика водорода

#define BUZER_ON // использование бузера
// таймера в миллисекундах
#define TIMER15m 900000
#define TIMER30m 1800000
// Задержки ошибок
#define H2_485_DELAY 4    // количество попыток чтения датчика H2
#define EE364_485_DELAY 4 // количество попыток чтения датчика EE364 влажности

// Периоды выполнения задач
#define PERIOD_LCD pdMS_TO_TICKS(201)
#define PERIOD_WiFi pdMS_TO_TICKS(5000)
#define PERIOD_Web pdMS_TO_TICKS(1056)
#define PERIOD_RTC pdMS_TO_TICKS(1022)
#define PERIOD_RS485 pdMS_TO_TICKS(300)
#define PERIOD_PRINT pdMS_TO_TICKS(1010)
#define PERIOD_LEDquick pdMS_TO_TICKS(50)
#define PERIOD_LEDslow pdMS_TO_TICKS(50)
#define PERIOD_LEDfon pdMS_TO_TICKS(50)
#define PERIOD_Debug pdMS_TO_TICKS(5000)
#define PERIOD_DS18b20 pdMS_TO_TICKS(10101)

// настройки ЛЕД
#define LED_PIN 15
#define COLOR_ORDER GRB
#define CHIPSET WS2811
#define NUM_LEDS 22
#define BRIGHTNESS 120
#define FRAMES_PER_SECOND 60
// освещение
#define TEMP_OF_LED_ON 15
#define SCALE_FADE_OFF 30
#define Nacal CHSV(30, 255, 150) // цвет нити накала люм лампы
#define TIMER_BRITH 100
#define TIMER_CONTR 255
#define TIMER_PASS_TONE 100
#define TIMER_LESS_TONE 62
#define TIMER_PASS CHSV(TIMER_PASS_TONE, TIMER_CONTR, TIMER_BRITH)
#define TIMER_LESS CHSV(TIMER_LESS_TONE, TIMER_CONTR, TIMER_BRITH)

#define WHITE_BACKLIGHT CHSV(0, 0, 100)                                               // цвет белой подсветки
#define GREEN_BACKLIGHT CHSV(100, 255, 120)                                           // цвет зеленой подсветки
#define YELOW_BACKLIGHT_H 60                                                          // тон желтой подсветки
#define YELOW_BACKLIGHT_S 240                                                         // контраст желтой подсветки
#define YELOW_BACKLIGHT_V 150                                                         // яркость желтой подсветки
#define YELOW_BACKLIGHT CHSV(YELOW_BACKLIGHT_H, YELOW_BACKLIGHT_S, YELOW_BACKLIGHT_V) // цвет желтой подсветки
#define FLASH_BACKLIGHT_DELTA 100                                                     // диапазон мигания
#define YELOW_BACKLIGHT_SPEED 2                                                       // диапазон мигания
#define RED_BACKLIGHT_H 0                                                             // тон красной подсветки

#define EEPROM_SIZE 10 // определите количество байтов, к которым вы хотите получить доступ

#include <string>
#include <Arduino.h>
// #include "SoftwareSerial.h"
#include "FastLED.h"
// #include <stdio.h>
#include <EEPROM.h> //  библиотека для чтения и записи из/во флэш-память
#include <OneWire.h>
#include <ModbusMaster.h>
// #include <WiFi.h>
#include <SD.h>
#include "Nextion.h"
// #include "EE364.h"
#include "iarduino_RTC.h" // Подключаем библиотеку iarduino_RTC для работы с модулями реального времени.

#include <Adafruit_Thermal.h> //  Подключаем библиотеку для работы с принтером
#include <WiFi.h>             // Load Wi-Fi library
// page1 Main
NexNumber x0 = NexNumber(1, 1, "x0");  // значение активность.
NexNumber x1 = NexNumber(1, 2, "x1");  // значение влажность, ппм
NexNumber x2 = NexNumber(1, 3, "x2");  // значение температура
NexNumber x3 = NexNumber(1, 12, "x3"); // значение водород
NexNumber x4 = NexNumber(1, 17, "x4"); // максимум активність води
NexNumber x5 = NexNumber(1, 18, "x5"); // мінімум активність води
NexNumber x6 = NexNumber(1, 22, "x6"); // максимум вміст води
NexNumber x7 = NexNumber(1, 21, "x7"); // мінімум вміст води


NexButton b4 = NexButton(1, 25, "b4");       // КНОПКА сброс мін\макс
NexDSButton bt0 = NexDSButton(1, 15, "bt0"); // КОПКА переключатель печать после таймера
NexButton b1 = NexButton(1, 13, "b1");       // КНОПКА запуск таймера 15 минут
NexButton b3 = NexButton(1, 14, "b3");       // КНОПКА запуск таймера 30 минут
NexText data = NexText(1, 7, "data");        // текущая дата
NexText time1 = NexText(1, 8, "time1");      // время текущее
NexText time13 = NexText(1, 13, "t13");      // таймер
NexText st = NexText(1, 13, "st");           // индикация состояния датчика ВОДОРОДА
NexVariable va0 = NexVariable(1, 17, "va0"); // инициализация дисплея
NexText t3 = NexText(1, 16, "t3");           // индикатор МОДБАС

// page2 Report
NexButton b5 = NexButton(2, 8, "b5"); // запустить печать

// page 3 Setting
NexButton b11 = NexButton(3, 14, "b11");       // записать анстройки времени
NexDSButton bt35 = NexDSButton(3, 18, "bt35"); // кнопка включения/отключения подсветки
NexText t45 = NexText(3, 15, "t45");           // показания температуры платы
NexText t46 = NexText(3, 19, "t46");           // показания температуры ЦПУ
NexNumber n0 = NexNumber(3, 9, "n0");          // час
NexNumber n1 = NexNumber(3, 10, "n1");         // минута
NexNumber n2 = NexNumber(3, 11, "n2");         // день
NexNumber n3 = NexNumber(3, 12, "n3");         // месяц
NexNumber n4 = NexNumber(3, 13, "n4");         // год

// page 7 Service
NexText t90 = NexText(7, 1, "t90"); // вывод буфера рс485

// сторінки
NexPage page0 = NexPage(0, 0, "Start");   // Page added as a touch event
NexPage page1 = NexPage(1, 0, "Main");    // Page added as a touch event
NexPage page2 = NexPage(2, 0, "Report");  // Page added as a touch event
NexPage page3 = NexPage(3, 0, "Settyng"); // Page added as a touch event
NexPage page4 = NexPage(4, 0, "keybdB");  // Page added as a touch event
NexPage page5 = NexPage(5, 0, "page0");   // Page added as a touch event
NexPage page6 = NexPage(6, 0, "Pass");    // Page added as a touch event
NexPage page7 = NexPage(7, 0, "Service"); // Page added as a touch event
NexPage page8 = NexPage(8, 0, "Trend");   // Page added as a touch event
NexPage page9 = NexPage(9, 0, "Temp");    // Page added as a touch event

NexTouch *nex_listen_list[] =
    {
        &b1,
        &b3,
        &b4,
        &b5,
        &b11,
        &bt0,
        &bt35,
        &page0,
        &page1,
        &page2,
        &page3,
        &page5,
        &page4,
        NULL};

ModbusMaster modbus;

// EE364 moistSens;
float reg1;
int low_word, high_word;
void preTransmission() { digitalWrite(MAX485_RE_NEG, HIGH); }
void postTransmission()
{
  // delayMicroseconds(2000);
  digitalWrite(MAX485_RE_NEG, LOW);
}
//*******
Adafruit_Thermal printer(&Serial1); //  Подключаем принтер в порту

// iarduino_RTC watch(RTC_DS3231);                         // Объявляем объект watch для работы с RTC модулем на базе чипа DS3231, используется шина I2C.
iarduino_RTC watch(RTC_DS1302, 26, 33, 25); // Объявляем объект watch для работы с RTC модулем на базе чипа DS1302, указывая выводы Arduino подключённые к выводам модуля RST, CLK, DAT
// ThreeWire myWire(25,33,26); // IO, SCLK, CE
// RtcDS1302<ThreeWire> Rtc(myWire);

// WiFiServer server(80); // Set web server port number to 80
OneWire ds(4);
CRGB leds[NUM_LEDS];

const char *ssid = "TOR1s";
const char *password = "123456789";

String header; // Переменная для хранения HTTP-запроса

String Start_1 = "off"; // индикация кнопок (ВЕБ)
String Stop_1 = "off";  // индикация кнопок (ВЕБ)
String Page_1 = "Main"; // назване текущей (активной) станици (ВЕБ)
String Print_1 = "off"; // индикация кнопок (ВЕБ)

// String(DATA_) = "00/00/00"; // ДАТА
// String(TIME_) = "00:00";    // Время

char DATA_[10] = "00/00/00", DATA_p[10] = "00/00/00"; // ДАТА
char TIME_[10] = "00:00", TIME_p[10] = "00:00";       // Время
char t[10];

// String(TEXT_W)="Attention! No connection to sensors"; // текст состояния
String(TEXT_W) = ""; // текст состояния
String Dan = String((char)0x00, 20);
; // данные прочитаны с компорта (Экрана)+ ****

String Data1;  // строка передачи на єкран для даты (NEXTION)
String timer1; // строка передачи на єкран для часы (NEXTION)

String WA;  // строка передачи на єкран активность для воды (NEXTION)
String WC;  // строка передачи на єкран содержание для воды (NEXTION)
String TE1; // строка передачи на єкран для температура(NEXTION)
String H2;  // строка передачи на єкран для водорода(NEXTION)
// String t; //временная строка

float WaterA, WaterAmin, WaterAmax; // активность воды c (датчика)
float WaterC, WaterCmin, WaterCmax; // содержание воды c (датчика)
float te;                           // температура c (датчика)
float h2_PCB_T, h2_Oil_T;
float WaterA_m = 0.0, WaterC_m = 0.0, te_m = 0.0, h2_m = 0.0;
float WaterA_p = 0.0, WaterAmin_p = 0.0, WaterAmax_p = 0.0, WaterC_p = 0.0, WaterCmin_p = 0.0, WaterCmax_p = 0.0, te_p = 0.0, h2_p = 0.0; // переменные для печати
float PCB_Temper_TOR1, CPU_Temper_TOR1;                                                                                                   // температура с датчика DS18b20
uint32_t h2;                                                                                                                              // температура c (датчика)
uint16_t h2_status, h2_status_m;

uint32_t timer = 0;  // целевое время таймера
bool timer_run;      // таймер запущен
bool printing_timer; // печать после таймера
uint8_t result;
uint8_t D, M, Y, h, m, s, W, D_m = 0, M_m = 0, Y_m = 0, h_m = 0, m_m = 0, s_m = 0, W_m = 0; // Объявляем переменные для получения следующих значений: D-день, M-месяц, Y-год, h-часы, m-минуты, s-секунды, W-день недели.
uint8_t h1, m1, s1;                                                                         // для работы таймера ТЕСТА:  h1-часы, m1-минуты, s1-секунды.
uint8_t h1_t = 1, m1_t = 0, h2_Ready_delay = 0, h2_error_delay = 0, EE364_error_delay;      // уставка времени теста:  h1_t-часы, m1_t-минуты.

int pag = 1, pag_m = 0; // номер страници экрана (NEXTION)
int updat = 2;          // обновление экрана (NEXTION)
int up_i = 0;           // счетчик для обновления
int led_fon = 0;        // режим фоновой подсветки
int send, send1;
int updat_rs = 0; // период чтение RS485
int step_var = 0; // шаг чтение переменных
// int NI =0; // проверка скорости

// bool Start =false; // запуск/ стоп  теста (алгоритм)
// bool Stop =false; // остановка теста (алгоритм)
// bool Reset =false; // Сброс времени теста теста (алгоритм)
bool Print_on = false; // включение принтера (алгоритм)
bool WIFI_on = false;  // включение WIFI

bool upd_fors = false; // принудительное обновление экрана (NEXTION)

bool getResultMsg(ModbusMaster *node, uint8_t result);

bool h2_Ready = false, h2_ready_m; // готовность датчика водорода

bool dynamic_ilumination = 0; // динамическая илюминация

bool Over_Temp; // перегрев платы

TaskHandle_t Handle_LCD1, Handle_LCD2, Handle_after10seccondOneTime, Handle_WiFi, Handle_RS485, Handle_Print, Handle_Web, Handle_RTC, Handle_LED_quick, Handle_LED_slow, Handle_LED_fon, Handle_Debug, Handle_DS18b20;
QueueHandle_t q_effectLEDquick, q_effectLEDslow, q_Tmep;
void Task_LCD_update(void *pvParam);
void Task_LCD_listen(void *pvParam);
void Task_after10seccondOneTime(void *pvParam);
void Task_WiFi(void *pvParam);
void Task_RS485(void *pvParam);
void Task_Print(void *pvParam);
void Task_Web(void *pvParam);
void Task_RTC(void *pvParam);
void Task_LED_quick(void *pvParam);
void Task_LED_slow(void *pvParam);
void Task_LED_fon(void *pvParam);
void Task_Debug(void *pvParam);
void Task_DS18b20(void *pvParam);
void updateAll(void);
int8_t recvPageNumber(uint32_t timeout = 100);

/*EEPROM MAP
0 - WiFi on/off
1 -
2 -
3 -
4 - dynamic ilumination on\off
5 - MODBUS setting
6 -
7 -
8 -
9 -
*/

//***************************************************************************************************************************************************************************************************************************************

void setup()
{
  //  Serial.begin(9600, SERIAL_8N1, RXD0, TXD0); // для Экрана Nextion
  Serial.begin(115200, SERIAL_8N1);            // для теста
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1); // для принтера
  Serial2.begin(9600, SERIAL_8E1, RXD2, TXD2); // для чтения датчиков по RS485
  // SWserial.begin(115200, SWSERIAL_8N1, RXD3, TXD3);

  EEPROM.begin(EEPROM_SIZE); // инициализировать EEPROM с предопределенным размером

  WIFI_on = EEPROM.read(0); // прочитать последнее состояние светодиода из флэш-памяти
                            // digitalWrite(ledPin, ledState); // установить светодиод в последнее сохраненное состояние
  // WiFi.disconnect(true);
  dynamic_ilumination = EEPROM.readBool(4);

  pinMode(MAX485_RE_NEG, OUTPUT);
  digitalWrite(MAX485_RE_NEG, LOW);
  modbus.begin(Slave_ID_Moist, Serial2);
  modbus.preTransmission(preTransmission);
  modbus.postTransmission(postTransmission);

  printer.begin();
  printer.setSize('S'); //  Устанавливаем маленький размер шрифта 'S' (Small) - используется по умолчанию
  printer.setDefault(); //  Устанавливаем настройки принтера по умолчанию. Функцию удобно использовать если Вы желаете вывести текст, но хотите быть уверены что на него не повлияют ранее установленные размеры или начертания.
  // printer.offline();
  printer.setTimes(6000, 3000);
  delay(300);
  // ЧАСЫ
  //  Ждем готовности модуля отвечать на запросы. RTC
  watch.begin(); // Инициируем работу с модулем. RTC

  // WIFI
  //  WiFi.softAP(ssid, password);
  //  IPAddress IP = WiFi.softAPIP();
  //    WiFi.disconnect(true, false);
#ifdef DEBUG1
  SWserial.print("AP IP address: ");
  SWserial.println(IP);
#endif
  // server.begin();

  // тестовый пин
  pinMode(12, OUTPUT); // Initialize the output variables as outputs
  pinMode(14, OUTPUT); // Initialize the output variables as outputs
  pinMode(27, OUTPUT); // Initialize the output variables as outputs
  pinMode(2, OUTPUT);  // Initialize the output variables as outputs
  q_effectLEDquick = xQueueCreate(10, sizeof(int));
  q_effectLEDslow = xQueueCreate(10, sizeof(int));

  nexInit();

  xTaskCreatePinnedToCore(Task_LCD_update, "Task LCD update", 5000, NULL, 3, &Handle_LCD1, 0);
  xTaskCreatePinnedToCore(Task_LCD_listen, "Task LCD listen", 5000, NULL, 3, &Handle_LCD2, 1);
  xTaskCreatePinnedToCore(Task_after10seccondOneTime, "Task after 10 seccond 1 time", 5000, NULL, 1, &Handle_after10seccondOneTime, 0);
  // xTaskCreatePinnedToCore(Task_Web, "Task WEB", 50000, NULL, 2, &Handle_Web, 1);

  xTaskCreatePinnedToCore(Task_Print, "Task Print", 10000, NULL, 2, &Handle_Print, 0);
  xTaskCreatePinnedToCore(Task_RTC, "Task RTC", 5000, NULL, 2, &Handle_RTC, 0);
  xTaskCreatePinnedToCore(Task_RS485, "Task 485", 5000, NULL, 2, &Handle_RS485, 0);
#ifdef ILLUMINATION
  xTaskCreatePinnedToCore(Task_LED_quick, "Task LED quick", 5000, NULL, 2, &Handle_LED_quick, 0);
  xTaskCreatePinnedToCore(Task_LED_slow, "Task LED slow", 5000, NULL, 1, &Handle_LED_slow, 0);
  xTaskCreatePinnedToCore(Task_LED_fon, "Task LED fon", 5000, NULL, 1, &Handle_LED_fon, 0);
  int mode = 4;
  xQueueSendToBack(q_effectLEDquick, &mode, 0); // включение ЛЕД ленты
  mode = 9;
  xQueueSendToBack(q_effectLEDquick, &mode, 0); // включение ЛЕД ленты
  led_fon = 0;
#endif
  xTaskCreatePinnedToCore(Task_DS18b20, "Task DS18b20", 5000, NULL, 1, &Handle_DS18b20, 0);
  //xTaskCreatePinnedToCore(Task_Debug, "Task Debug", 5000, NULL, 2, &Handle_Debug, 0);
  /**/
}

// long lastMillis = 0;

void loop()
{
  vTaskDelete(NULL);
} // конец цикла

// bool getResultMsg(ModbusMaster *node, uint8_t result)
// {
//   String tmpstr2 = "\r\n";
//   switch (result)
//   {
//   case node->ku8MBSuccess:
//     return true;
//     break;
//   case node->ku8MBIllegalFunction:
//     tmpstr2 += "Illegal Function";
//     break;
//   case node->ku8MBIllegalDataAddress:
//     tmpstr2 += "Illegal Data Address";
//     break;
//   case node->ku8MBIllegalDataValue:
//     tmpstr2 += "Illegal Data Value";
//     break;
//   case node->ku8MBSlaveDeviceFailure:
//     tmpstr2 += "Slave Device Failure";
//     break;
//   case node->ku8MBInvalidSlaveID:
//     tmpstr2 += "Invalid Slave ID";
//     break;
//   case node->ku8MBInvalidFunction:
//     tmpstr2 += "Invalid Function";
//     break;
//   case node->ku8MBResponseTimedOut:
//     tmpstr2 += "Response Timed Out";
//     break;
//   case node->ku8MBInvalidCRC:
//     tmpstr2 += "Invalid CRC";
//     break;
//   default:
//     tmpstr2 += "Unknown error: " + String(result);
//     break;
//   }
//   // Serial.println(tmpstr2);
//   return false;
// }
void bt0pop(void *ptr)
{
  uint32_t p;
  bt0.getValue(&p);
  printing_timer = (bool)p;
}

void bt0push(void *ptr)
{
}

void b1push(void *ptr)
{
  timer = millis() + TIMER15m; // запустить таймер на 15 минут (900000 миллисекунд)
  timer_run = true;
  send = 5;
  xQueueReset(q_effectLEDslow);
  xQueueSendToFront(q_effectLEDslow, &send, 0);
  WaterAmax = WaterA;
  WaterAmin = WaterA;
  WaterCmax = WaterC;
  WaterCmin = WaterC;
}

void b3push(void *ptr)
{
  timer = millis() + TIMER30m; // запустить таймер на 30 минут (1800000 миллисекунд)
  timer_run = true;
  send1 = 6;
  xQueueReset(q_effectLEDslow);
  xQueueSendToFront(q_effectLEDslow, &send1, 0);
  WaterAmax = WaterA;
  WaterAmin = WaterA;
  WaterCmax = WaterC;
  WaterCmin = WaterC;
}

void b4push(void *ptr)
{
  WaterAmax = WaterA;
  WaterAmin = WaterA;
  WaterCmax = WaterC;
  WaterCmin = WaterC;
}

void b5push(void *ptr)
{
  Print_on = true;
}

void b11push(void *ptr)
{
  uint32_t min, hour, day, moun, year;
  n0.getValue(&hour);
  n1.getValue(&min);
  n2.getValue(&day);
  n3.getValue(&moun);
  n4.getValue(&year);

  watch.settime(-1, min, hour, day, moun, year);
  // Rtc.SetDateTime (RtcDateTime (year, moun, day, hour, min, 0));

  // watch.settime(0, 13, 14, 15, 16, 17);
}

void bt35pop(void *ptr)
{
  uint32_t ret;
  bt35.getValue(&ret);
  dynamic_ilumination = ret;
  if ((dynamic_ilumination) != (EEPROM.read(4)))
  {
    if (dynamic_ilumination)
    {
      EEPROM.write(4, 1);
      EEPROM.commit();
    }
    else
    {
      EEPROM.write(4, 0);
      EEPROM.commit();
    }
  }
}

void page0PushCallback(void *ptr) { pag = 0;}
void page1PushCallback(void *ptr) { pag = 1;}
void page2PushCallback(void *ptr) { pag = 2;}
void page3PushCallback(void *ptr) { pag = 3;}
void page4PushCallback(void *ptr) { pag = 4;}
void page5PushCallback(void *ptr) { pag = 5;}
void page6PushCallback(void *ptr) { pag = 6;}
void page7PushCallback(void *ptr) { pag = 7;}
void page8PushCallback(void *ptr) { pag = 8;}
void page9PushCallback(void *ptr) { pag = 9;}



void Task_LCD_listen(void *pvParam)
{
  bt0.attachPop(bt0pop, &bt0);
  bt0.attachPush(bt0push, &bt0);
  bt35.attachPop(bt35pop, &bt35);

  b1.attachPush(b1push, &b1);
  b3.attachPush(b3push, &b3);
  b4.attachPush(b4push, &b4);
  b5.attachPush(b5push, &b5);
  b11.attachPush(b11push, &b11);
  b1.attachPop(b1push, &b1);
  b3.attachPop(b3push, &b3);
  b5.attachPop(b5push, &b5);
  b11.attachPop(b11push, &b11);
  page0.attachPush(page0PushCallback);
  page1.attachPush(page1PushCallback);
  page2.attachPush(page2PushCallback);
  page3.attachPush(page3PushCallback);
  page4.attachPush(page4PushCallback);
  page5.attachPush(page5PushCallback);
  page6.attachPush(page6PushCallback);
  page7.attachPush(page7PushCallback);
  page8.attachPush(page8PushCallback);
  page9.attachPush(page9PushCallback);

  while (1)
  {
    delay(100);
    // vTaskSuspend (testLoop_h);
    // nexLoop(nex_listen_list);
    // vTaskResume (testLoop_h);
  }
}

void Task_after10seccondOneTime(void *pvParam)
{
  while (1)
  {
    delay(10000);
    WaterAmax = WaterA;
    WaterAmin = WaterA;
    WaterCmax = WaterC;
    WaterCmin = WaterC;
    vTaskDelete(NULL);
  }
}

void Task_LCD_update(void *pvParam)
{
  char buffer[20];
  uint32_t ret;
  uint32_t ans;

  if (EEPROM.read(4) == 1)
  {
    dynamic_ilumination = true;
  }
  else
  {
    dynamic_ilumination = false;
  }

  while (1)
  {

    vTaskDelay(PERIOD_LCD);
    nexLoop(nex_listen_list);
    digitalWrite(RELAY, printing_timer);
    /*ans = va0.getValue(&ret);
    if ((ret != 1) && (ans == 1))
    {
      // x0.setValue((uint32_t)(WaterA * 100));
      // x1.setValue((uint32_t)(WaterC * 100));
      // x2.setValue((uint32_t)(te * 100));
      // x3.setValue((uint32_t)(h2));
      (timer > millis()) ? sprintf(buffer, "%.2li:%.2li", (timer - millis()) / 60000, ((timer - millis()) % 60000) / 1000) : sprintf(buffer, "00:00");
      time13.setText(buffer);
      data.setText(DATA_);
      time1.setText(TIME_);
      n0.setValue(h);
      n1.setValue(m);
      n2.setValue(D);
      n3.setValue(M);
      n4.setValue(Y);
      bt35.setValue(dynamic_ilumination);
      bt0.setValue(printing_timer);
      va0.setValue(1);

      //обновляем переменные для вывода на печать
      WaterA_p = WaterA;
      WaterC_p = WaterC;
      te_p = te;
      h2_p = h2;
    }*/

    // sendCommand("sendme");
    // pag = recvPageNumber();
    switch (pag)
    {
    case 1: // Page 1

      x0.setValue((uint32_t)(WaterA * 1000 + 0.5));
      x1.setValue((uint32_t)(WaterC * 100 + 0.5));
      x2.setValue((uint32_t)(te * 100 + 0.5));

      if (WaterA > WaterAmax) {WaterAmax = WaterA;}
      if (WaterA < WaterAmin) {WaterAmin = WaterA;}
      if (WaterC > WaterCmax) {WaterCmax = WaterC;}
      if (WaterC < WaterCmin) {WaterCmin = WaterC;}

      x4.setValue((uint32_t)(WaterAmax * 1000 + 0.5));
      x5.setValue((uint32_t)(WaterAmin * 1000 + 0.5));
      x6.setValue((uint32_t)(WaterCmax * 100 + 0.5));
      x7.setValue((uint32_t)(WaterCmin * 100 + 0.5));

    

#ifdef HYDROGEN_SENSOR
      x3.setValue((uint32_t)(h2));
#endif
      (timer > millis()) ? sprintf(buffer, "%.2li:%.2li", (timer - millis()) / 60000, ((timer - millis()) % 60000) / 1000) : sprintf(buffer, "00:00");
      time13.setText(buffer);
      data.setText(DATA_);
      time1.setText(TIME_);

      // обновляем переменные для вывода на печать
      strcpy(DATA_p, DATA_);
      strcpy(TIME_p, TIME_);
      WaterA_p = WaterA;
      WaterAmax_p = WaterAmax;
      WaterAmin_p = WaterAmin;

      WaterC_p = WaterC;
      WaterCmax_p = WaterCmax;
      WaterCmin_p = WaterCmin;

      te_p = te;
      h2_p = h2;
#ifdef GYDROGEN
      // выводим готовность датчика водророда
      if (!h2_Ready)
      {
        st.setText("AutoCalibrat.");
        st.Set_background_color_bco(65520);
      }
      else
      {
        st.setText("Ready");
        st.Set_background_color_bco(28288);
      }
#endif
      // индикатор связи МОДБАС
      switch (result)
      {
      case modbus.ku8MBSuccess:
        t3.Set_background_color_bco(2016); // зеленый
        break;
      case modbus.ku8MBResponseTimedOut:
        t3.Set_background_color_bco(63488); // красный
        break;
      case modbus.ku8MBInvalidSlaveID:
        t3.Set_background_color_bco(65504); // желтый
        break;
      default:
        t3.Set_background_color_bco(63519); // фиолетовый
        break;
      }

      pag_m = pag;
      break;
    case 2: // Page 2 при входе на страницу Печать!
            // обновляются данные 1 раз.
      if (pag != pag_m)
      {
        WaterA_p = WaterA;
        WaterAmax_p = WaterAmax;
        WaterAmin_p = WaterAmin;

        WaterC_p = WaterC;
        WaterCmax_p = WaterCmax;
        WaterCmin_p = WaterCmin;
        te_p = te;
        h2_p = h2;
        strcpy(DATA_p, DATA_);
        strcpy(TIME_p, TIME_);

        // обновляем переменные для вывода на печать
        x0.setValue((uint32_t)(WaterA_p * 1000 + 0.5));
        x1.setValue((uint32_t)(WaterC_p * 100 + 0.5));
        x2.setValue((uint32_t)(te_p * 100 + 0.5));

        x4.setValue((uint32_t)(WaterAmax_p * 1000 + 0.5));
        x5.setValue((uint32_t)(WaterAmin_p * 1000 + 0.5));
        x6.setValue((uint32_t)(WaterCmax_p * 100 + 0.5));
        x7.setValue((uint32_t)(WaterCmin_p * 100 + 0.5));
#ifdef HYDROGEN_SENSOR
        x3.setValue((uint32_t)(h2_p));
#endif
        data.setText(DATA_p);
        time1.setText(TIME_p);
      }
      pag_m = pag;
      break;
    case 3: // Page 3     //обновляются данные 1 раз. НАСТРОЙКИ
      data.setText(DATA_);
      time1.setText(TIME_);
      if ((pag_m != 3) && (pag_m != 4) && (pag != pag_m))
      {
        n0.setValue(h);
        n1.setValue(m);
        n2.setValue(D);
        n3.setValue(M);
        n4.setValue(Y);
        bt35.setValue(dynamic_ilumination);
      }

      ;

      // if ((ret) != (EEPROM.read(4)))
      // {
      //   if (dynamic_ilumination)
      //   {
      //     EEPROM.write(4, 1);
      //     EEPROM.commit();
      //   }
      //   else
      //   {
      //     EEPROM.write(4, 0);
      //     EEPROM.commit();
      //   }
      // }

      // обновляем переменные для вывода на печать
      strcpy(DATA_p, DATA_);
      strcpy(TIME_p, TIME_);
      WaterA_p = WaterA;
      WaterAmax_p = WaterAmax;
      WaterAmin_p = WaterAmin;

      WaterC_p = WaterC;
      WaterCmax_p = WaterCmax;
      WaterCmin_p = WaterCmin;

      te_p = te;
      h2_p = h2;
      pag_m = pag;
      break;

    case 7: // сервисное меню
      char buf[150];
      sprintf(buf, "Moist=%.3f, ppm=%.3f, temp=%.3f, res=%i, H2_PCB_T=%.3f, H2_Oil_Temp=%.3f \n", WaterA, WaterC, te, result, h2_PCB_T, h2_Oil_T);
      t90.setText(buf);
      pag_m = pag;
      break;

    case -1: // ошибка
      break;
    default:
      // обновляем переменные для вывода на печать
      //  strcpy(DATA_p, DATA_);
      //  strcpy(TIME_p, TIME_);
      //  WaterA_p = WaterA;
      //  WaterC_p = WaterC;
      //  te_p = te;
      //  h2_p = h2;
      break;
    }
  }
}

void Task_Web(void *pvParam)
{
  // // TickType_t xLastWakeTime;
  // // xLastWakeTime = xTaskGetTickCount();
  // while (1)
  // {
  //   // vTaskDelayUntil(&xLastWakeTime, periodWeb);
  //   vTaskDelay(PERIOD_Web);
  //   //*************************************************************************************************************************************************************************
  //   // ВЕБ СЕРВЕР
  //   //*************************************************************************************************************************************************************************
  //   // WiFiClient client = server.available(); // Listen for incoming clients

  //   if (client)
  //   {                          // If a new client connects,
  //                              // Serial.println("New Client.");          // print a message out in the serial port
  //     String currentLine = ""; // make a String to hold incoming data from the client
  //     while (client.connected())
  //     { // loop while the client's connected
  //       if (client.available())
  //       {                         // if there's bytes to read from the client,
  //         char c = client.read(); // read a byte, then
  //                                 //     Serial.write(c);                    // print it out the serial monitor
  //         header += c;
  //         if (c == '\n')
  //         { // if the byte is a newline character
  //           // if the current line is blank, you got two newline characters in a row.
  //           // that's the end of the client HTTP request, so send a response:
  //           if (currentLine.length() == 0)
  //           {

  //             // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  //             // and a content-type so the client knows what's coming, then a blank line:
  //             client.println("HTTP/1.1 200 OK");
  //             client.println("Content-type:text/html");
  //             client.println("Connection: close");
  //             client.println();

  //             ///////////

  //             // Команда REPORT
  //             if (header.indexOf("GET /REPORT") >= 0)
  //             {
  //               Page_1 = "Report"; //
  //             }

  //             // Команда PRINT
  //             if (header.indexOf("GET /PRINT/on") >= 0)
  //             {
  //               Print_1 = "off";
  //               Print_on = true; //
  //             }
  //             // Команда PRINT
  //             if (header.indexOf("GET /PRINT/off") >= 0)
  //             {
  //               Print_1 = "on";
  //             }

  //             // Команда Return
  //             if (header.indexOf("GET /Return") >= 0)
  //             {
  //               Page_1 = "Main"; //
  //             }

  //             // Команда SETTYNG
  //             if (header.indexOf("GET /SETTYNG") >= 0)
  //             {
  //               Page_1 = "Setting"; //
  //             }

  //             /////**** Команда настройка RTC
  //             // watch.settime(s,m,h,D,M,Y,W); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             // uint8_t D, M, Y, h, m, s, W; // Объявляем переменные для получения следующих значений: D-день, M-месяц, Y-год, h-часы, m-минуты, s-секунды, W-день недели.
  //             //  настройка ДЕНЬ+
  //             if (header.indexOf("GET /Day+") >= 0)
  //             {
  //               D = D + 1;
  //               watch.settime(-1, -1, -1, D, -1, -1, -1); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             }
  //             // настройка ДЕНЬ-
  //             if (header.indexOf("GET /Day-") >= 0)
  //             {
  //               D = D - 1;
  //               watch.settime(-1, -1, -1, D, -1, -1, -1); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             }
  //             // настройка месяц+
  //             if (header.indexOf("GET /Month+") >= 0)
  //             {
  //               M = M + 1;
  //               watch.settime(-1, -1, -1, -1, M, -1, -1); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             }
  //             // настройка месяц-
  //             if (header.indexOf("GET /Month-") >= 0)
  //             {
  //               M = M - 1;
  //               watch.settime(-1, -1, -1, -1, M, -1, -1); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             }
  //             // настройка год+
  //             if (header.indexOf("GET /Year+") >= 0)
  //             {
  //               Y = Y + 1;
  //               watch.settime(-1, -1, -1, -1, -1, Y, -1); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             }
  //             // настройка год-
  //             if (header.indexOf("GET /Year-") >= 0)
  //             {
  //               Y = Y - 1;
  //               watch.settime(-1, -1, -1, -1, -1, Y, -1); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             }
  //             // настройка час+
  //             if (header.indexOf("GET /hour+") >= 0)
  //             {
  //               h = h + 1;
  //               watch.settime(-1, -1, h, -1, -1, -1, -1); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             }
  //             // настройка час-
  //             if (header.indexOf("GET /hour-") >= 0)
  //             {
  //               h = h - 1;
  //               watch.settime(-1, -1, h, -1, -1, -1, -1); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             }
  //             // настройка минуты+
  //             if (header.indexOf("GET /minute+") >= 0)
  //             {
  //               m = m + 1;
  //               watch.settime(-1, m, -1, -1, -1, -1, -1); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             }
  //             // настройка минуты-
  //             if (header.indexOf("GET /minute-") >= 0)
  //             {
  //               m = m - 1;
  //               watch.settime(-1, m, -1, -1, -1, -1, -1); // Записываем время в модуль: 0 сек, 51 мин, 21 час, 27, октября, 2015 года, вторник.
  //             }

  //             /// вывод ВЕБ страници
  //             // Display the HTML web page
  //             client.println("<!DOCTYPE html><html>");
  //             if (Page_1 == "Main")
  //             {
  //               client.println("<head><meta http-equiv=\"Refresh\" content=\"5\" /> </head>"); //обновление страници каждые 10 сек
  //             }
  //             client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  //             client.println("<link rel=\"icon\" href=\"data:,\">");
  //             // CSS to style the on/off buttons
  //             // Feel free to change the background-color and font-size attributes to fit your preferences
  //             client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: left;}");
  //             client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 10px;");
  //             client.println("text-decoration: none; font-size: 16px; margin: 2px; cursor: pointer;}");
  //             client.println(".button2 {background-color: #555555;}</style></head>");

  //             //********************************

  //             if (Page_1 == "Main")
  //             {
  //               // Web Page Heading
  //               client.println("<h3 style=\"color: #5e9ca0;\"><span style=\"color: #0000ff;\">Date");
  //               client.println(DATA_);
  //               client.println("&nbsp; &nbsp; &nbsp;Actual time ");
  //               client.println(TIME_);
  //               client.println("</span></h3>");
  //               client.println("<h3><span style=\"background-color: #ffffff; color: #ff0000;\">");
  //               client.println(TEXT_W);
  //               client.println("</span></h3><div>");
  //               client.println("<h3><span style=\"background-color: #ffffff; color: #000000;\">&nbsp;</span><span style=\"background-color: #ffffff; color: #000000;\">Water activity </span><span style=\"background-color: #ffff00; color: #0000ff;\">");
  //               client.println(String(WaterA));
  //               client.println("&nbsp; &nbsp;</span>&nbsp; AW</h3></div><div>");
  //               client.println("<h3><span style=\"background-color: #ffffff; color: #000000;\">Water content&nbsp; </span><span style=\"background-color: #00ff00; color: #000000;\">");
  //               client.println(String(WaterC));
  //               client.println(" </span>&nbsp; ppm</h3></div><div>");
  //               client.println("<h3><span style=\"background-color: #ffffff; color: #000000;\">&nbsp; Temperature&nbsp; </span><span style=\"background-color: #ff99cc; color: #0000ff;\">");
  //               client.println(String(te));
  //               client.println("</span>&nbsp; C</h3> <div>");

  //               //кнопки
  //               client.println("</div>");
  //               client.println("<a href=\"/REPORT\"><button class=\"button\">REPORT</button></a>");
  //             }

  //             //********************************
  //             //Экран отчет
  //             if (Page_1 == "Report")
  //             {
  //               // Web Page Heading

  //               client.println("<h3><em> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;GlobeCore</em></h3>");
  //               client.println("<h3>=================================</h3>");
  //               client.println("<h3>Date ");
  //               client.println(DATA_);
  //               client.println(" &nbsp; &nbsp;Actual time ");
  //               client.println(TIME_);
  //               client.println("<h3>");

  //               client.println("<h3>=================================</h3><div>");
  //               client.println("<h3>Water activity, AW &nbsp; &nbsp; &nbsp; &nbsp;");
  //               client.println(String(WaterA));
  //               client.println("</h3><h3>Water content, &nbsp;ppm &nbsp; &nbsp; ");
  //               client.println(String(WaterC));
  //               client.println("&nbsp;</h3></div><h3>Temperature, C &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;");
  //               client.println(String(te));
  //               client.println("</h3></div><div><h3>=================================</h3>");

  //               //кнопки
  //               if (Print_1 == "on")
  //               {
  //                 client.println("<a href=\"/PRINT/on\"><button class=\"button \">PRINT</button></a>");
  //               }
  //               else
  //               {
  //                 client.println("<a href=\"/PRINT/off\"><button class=\"button button2\">PRINT</button></a>");
  //               }

  //               client.println("<a href=\"/SETTYNG\"><button class=\"button\">SETTYNG</button></a>");
  //               client.println("<a href=\"/Return\"><button class=\"button\">Return</button></a>");
  //             }

  //             //********************************

  //             //Экран Настройки
  //             if (Page_1 == "Setting")
  //             {
  //               // Web Page Heading

  //               client.println("<h3>Date ");
  //               client.println(DATA_);
  //               client.println("&nbsp;Actual time ");
  //               client.println(TIME_);
  //               client.println("<h3>");

  //               //кнопки
  //               // uint8_t D, M, Y, h, m, s, W; // Объявляем переменные для получения следующих значений: D-день, M-месяц, Y-год, h-часы, m-минуты, s-секунды, W-день недели.
  //               client.println("<p><a href=\"/Day+\"><button class=\"button\">...Day+..</button></a>");
  //               client.println("<a href=\"/Day-\"><button class=\"button\">...Day-..</button></a></p>");
  //               client.println("<p><a href=\"/Month+\"><button class=\"button\">Month+</button></a>");
  //               client.println("<a href=\"/Month-\"><button class=\"button\">Month -</button></a></p>");
  //               client.println("<p><a href=\"/Year+\"><button class=\"button\">..Year+..</button></a>");
  //               client.println("<a href=\"/Year-\"><button class=\"button\">..Year-..</button></a></p>");
  //               client.println("<p><a href=\"/hour+\"><button class=\"button\">..hour+..</button></a>");
  //               client.println("<a href=\"/hour-\"><button class=\"button\">..hour-..</button></a></p>");
  //               client.println("<p><a href=\"/minute+\"><button class=\"button\">minute+</button></a>");
  //               client.println("<a href=\"/minute-\"><button class=\"button\">minute-</button></a></p>");
  //               client.println("<p><a href=\"/update\"><button class=\"button\">.........UPDATE..........</button></a></p>");
  //               client.println("<p><a href=\"/Return\"><button class=\"button\">..........Return...........</button></a></p>");
  //             }

  //             client.println("</body></html>");
  //             // The HTTP response ends with another blank line
  //             client.println();
  //             // Break out of the while loop
  //             break;
  //           }
  //           else
  //           { // if you got a newline, then clear currentLine
  //             currentLine = "";
  //           }
  //         }
  //         else if (c != '\r')
  //         {                   // if you got anything else but a carriage return character,
  //           currentLine += c; // add it to the end of the currentLine
  //         }
  //       }
  //     }
  //     // Clear the header variable
  //     header = "";
  //     // Close the connection
  //     client.stop();
  //     //  Serial.println("Client disconnected.");
  //     //  Serial.println("");
  //   }
  // }
}

void Task_Print(void *pvParam)
{
  // TickType_t xLastWakeTime;
  // xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // vTaskDelayUntil(&xLastWakeTime, periodPrint);
    vTaskDelay(PERIOD_PRINT);
    // vTaskSuspendAll();
    //*************************************************************************************************************************************************************************
    // ПЕЧАТЬ
    //*************************************************************************************************************************************************************************
    if (Print_on)
    {
      // // digitalWrite(LED_BUILTIN, HIGH);
      // // SWserial.println("PRINT") ;
      // printer.wake();
      // delay(50);
      printer.begin();
      delay(100);

      printer.setSize('S'); //  Устанавливаем маленький размер шрифта 'S' (Small) - используется по умолчанию
      printer.setDefault(); //  Устанавливаем настройки принтера по умолчанию. Функцию удобно использовать если Вы желаете вывести текст, но хотите быть уверены что на него не повлияют ранее установленные размеры или начертания.
      // printer.offline();
      printer.setTimes(6000, 3000);

      // printer.feed(2);
      printer.flush();

      printer.justify('C');                               //  Устанавливаем выравнивание текста по центру 'C' (Center)
      printer.println("=GlobeCore=");                     //  Выводим текст
      printer.justify('L');                               //  Устанавливаем выравнивание текста по левому краю 'L' (Left) - используется по умолчанию
      printer.println("==============================="); //  Выводим текст
      printer.print("Data ");                             //  Выводим текст
      printer.print(DATA_p);                              //  Выводим текст
      printer.print("  time ");                           //  Выводим текст
      printer.println(TIME_p);                            //  Выводим текст
      printer.println("==============================="); //  Выводим текст
      printer.print("Water activiti, AW  ");
      printer.println(WaterA_p, 3);
      printer.print("           MAX, AW  ");
      printer.println(WaterAmax_p, 3);
      printer.print("           MIN, AW  ");
      printer.println(WaterAmin_p, 3);
      printer.println();
      printer.print("Water content, ppm  ");
      printer.println(WaterC_p, 2);
      printer.print("          MAX, ppm  ");
      printer.println(WaterCmax_p, 2);
      printer.print("          MIN, ppm  ");
      printer.println(WaterCmin_p, 2);
      printer.println();

      printer.print("Temperature, C      ");
      printer.println(te_p, 2);
#ifdef HYDROGEN_SENSOR
      printer.print("Hydrogen, ppm       ");
      printer.print(h2_p, 0);
      if (!h2_Ready)
        printer.println("*");
      else
        printer.println();
#endif
      printer.println("==============================="); //  Выводим текст
      printer.println("Notes:_________________________"); //  Выводим текст
      printer.println("_______________________________"); //  Выводим текст
      printer.feed(3);

      // printer.offline();
      Print_on = false; // сбросить команду печать
      // digitalWrite(LED_BUILTIN, LOW);
    }

    digitalWrite(27, !digitalRead(27)); // мигаем светодиодом
    // xTaskResumeAll();
    //*************************************************************************************************************************************************************************
  }
}

void Task_RTC(void *pvParam)
{
  //*************************************************************************************************************************************************************************
  // Работа RTC (модулем часов)
  //*************************************************************************************************************************************************************************

  // TickType_t xLastWakeTime;
  // xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    // vTaskDelayUntil(&xLastWakeTime, PERIOD_RTC);
    vTaskDelay(PERIOD_RTC);

    sprintf(DATA_, "%.2i/%.2i/%.2i", D, M, Y);
    sprintf(TIME_, "%.2i:%.2i", h, m);

    // vTaskDelayUntil(&xLastWakeTime, periodRTC);

    watch.gettime(); // Считываем текущее время из модуля в буфер библиотеки.
    // Rtc.GetDateTime ();

    D = watch.day;     // Получаем из буфера библиотеки текущий день месяца 1-31.
    M = watch.month;   // Получаем из буфера библиотеки текущий месяц       1-12.
    Y = watch.year;    // Получаем из буфера библиотеки текущий год         0-99.
    h = watch.Hours;   // Получаем из буфера библиотеки текущие часы        0-23.
    m = watch.minutes; // Получаем из буфера библиотеки текущие минуты      0-59.
    s = watch.seconds; // Получаем из буфера библиотеки текущие секунды     0-59.
    W = watch.weekday; // Получаем из буфера библиотеки текущий день недели 0-6.

    // DATA_ = (String(D) + "/" + String(M) + "/" + String(Y));
    // TIME_ = (String(h) + ":" + String(m));

    sprintf(DATA_, "%.2i/%.2i/%.2i", D, M, Y);
    sprintf(TIME_, "%.2i:%.2i", h, m);

    if (timer_run)
    {
      if (timer < millis())
      {
        timer_run = false;

        uint32_t p; // оновлення стану кнопки
        bt0.getValue(&p);
        printing_timer = (bool)p;

        if (printing_timer)
        {
          Print_on = true;
        }
#ifdef BUZER_ON
        digitalWrite(BUZZER, true);
        vTaskDelay(1000);
        digitalWrite(BUZZER, false);
        vTaskDelay(300);
        digitalWrite(BUZZER, true);
        vTaskDelay(100);
        digitalWrite(BUZZER, false);
        vTaskDelay(200);
        digitalWrite(BUZZER, true);
        vTaskDelay(100);
        digitalWrite(BUZZER, false);
#endif
      }
    }
  }
}

void Task_RS485(void *pvParam)
{
  //*************************************************************************************************************************************************************************
  // Данные  (чтение датчика)
  //*************************************************************************************************************************************************************************
  // uint16_t bfdata[2];
  // TickType_t xLastWakeTime;
  // xLastWakeTime = xTaskGetTickCount();

  union
  {
    uint32_t x;
    float f;
  } wa;
  while (1)
  {
    // vTaskDelayUntil(&xLastWakeTime, periodRS485);
    vTaskDelay(PERIOD_RS485);
    ////////////// Чтение активности!

    modbus.begin(Slave_ID_Moist, Serial2);
    modbus.clearResponseBuffer();
    result = modbus.readHoldingRegisters(0x33, 2);
    if (result == modbus.ku8MBSuccess)
    {
      wa.x = (((unsigned long)modbus.getResponseBuffer(0x01) << 16) | modbus.getResponseBuffer(0x00));
      WaterA = wa.f;
      EE364_error_delay = 0;
    }
    else
    {
      EE364_error_delay++;
      if (EE364_error_delay > EE364_485_DELAY)
      {
        WaterA = 0.0;
      }
    }

    ////////////// Чтение влажности!
    vTaskDelay(PERIOD_RS485);
    modbus.clearResponseBuffer();
    result = modbus.readHoldingRegisters(0x35, 2);
    if (result == modbus.ku8MBSuccess)
    {
      wa.x = (((unsigned long)modbus.getResponseBuffer(0x01) << 16) | modbus.getResponseBuffer(0x00));
      WaterC = wa.f;
      EE364_error_delay = 0;
    }
    else
    {
      EE364_error_delay++;
      if (EE364_error_delay > EE364_485_DELAY)
      {
        WaterC = 0.0;
      }
    }
    // vTaskDelayUntil(&xLastWakeTime, periodRS485);
    vTaskDelay(PERIOD_RS485);

    ////////////// Чтение температуры!
    modbus.clearResponseBuffer();
    result = modbus.readHoldingRegisters(0x19, 2);
    if (result == modbus.ku8MBSuccess)
    {
      wa.x = (((unsigned long)modbus.getResponseBuffer(0x01) << 16) | modbus.getResponseBuffer(0x00));
      te = wa.f;
      EE364_error_delay = 0;
    }
    else
    {
      EE364_error_delay++;
      if (EE364_error_delay > EE364_485_DELAY)
      {
        te = 0.0;
      }
    }

////////////// Чтение водорода!
// vTaskDelayUntil(&xLastWakeTime, periodRS485);
#ifdef HYDROGEN_SENSOR
    vTaskDelay(PERIOD_RS485);

    modbus.begin(Slave_ID_H2, Serial2);
    modbus.clearResponseBuffer();

    result = modbus.readHoldingRegisters(0x0, 2);
    if (result == modbus.ku8MBSuccess)
    {
      // содержание водорода, ппм
      wa.x = (((unsigned long)modbus.getResponseBuffer(0x00) << 16) | modbus.getResponseBuffer(0x01));
      modbus.clearResponseBuffer();
      h2 = wa.x;
      h2_error_delay = 0;
    }
    else
    {
      h2_error_delay++;
      if (h2_error_delay > H2_485_DELAY)
      {
        h2 = 0;
      }
    }

    ////////////////// чтение температуры H2
    vTaskDelay(PERIOD_RS485);
    modbus.clearResponseBuffer();
    result = modbus.readHoldingRegisters(0x7, 2);

    if (result == modbus.ku8MBSuccess)
    {
      // содержание водорода, ппм
      h2_PCB_T = (((float)modbus.getResponseBuffer(0x00) / 100.0) - 100.0);
      h2_Oil_T = (((float)modbus.getResponseBuffer(0x01) / 100.0) - 100.0);
      modbus.clearResponseBuffer();
      h2_error_delay = 0;
    }
    else
    {
      h2_error_delay++;
      if (h2_error_delay > H2_485_DELAY)
      {
        h2_PCB_T = 0;
        h2_Oil_T = 0;
      }
    }

    ///////////   Чтение статуса H2
    //*
    vTaskDelay(PERIOD_RS485);

    modbus.clearResponseBuffer();
    result = modbus.readHoldingRegisters(111, 1);

    if (result == modbus.ku8MBSuccess)
    {
      h2_status = modbus.getResponseBuffer(0x00);
      modbus.clearResponseBuffer();
      h2_error_delay = 0;
    }
    else
    {
      h2_error_delay++;
      if (h2_error_delay > H2_485_DELAY)
      {
        h2_status = 0;
      }
    }
    if (((h2_status & 0b1000000000000000) == 0b1000000000000000) and ((h2_status & 0b0000000000111000) == 0b0000000000001000))
    {
      h2_Ready = true;
      led_fon = 1;
    }
    else
    {
      h2_Ready = false;
      led_fon = 2;
    }
    /**/

    /// Если изменился статус готовности датчика водорода - просигнализировать.
#ifdef ILLUMINATION
    if (h2_Ready != h2_ready_m)
    {
      // h2_Ready_delay++;
      // if (h2_Ready_delay > 2)
      // {
      if (h2_Ready)
      {
        int send = 7;
        int send1 = 8;
        xQueueSendToBack(q_effectLEDquick, &send, 0);
        xQueueSendToBack(q_effectLEDquick, &send1, 0);
        xQueueSendToBack(q_effectLEDquick, &send, 0);
        xQueueSendToBack(q_effectLEDquick, &send1, 0);
        xQueueSendToBack(q_effectLEDquick, &send, 0);
        xQueueSendToBack(q_effectLEDquick, &send1, 0);
      }
      else
      {
        int send = 6;
        int send1 = 2;
        xQueueSendToBack(q_effectLEDquick, &send, 0);
        xQueueSendToBack(q_effectLEDquick, &send1, 0);
        xQueueSendToBack(q_effectLEDquick, &send, 0);
        xQueueSendToBack(q_effectLEDquick, &send1, 0);
        xQueueSendToBack(q_effectLEDquick, &send, 0);
        xQueueSendToBack(q_effectLEDquick, &send1, 0);
      }
      h2_ready_m = h2_Ready;
      h2_Ready_delay = 0;
      // }
    }
    else
    {
      h2_Ready_delay = 0;
    }
    if (h2_status == 0)
    {
      led_fon = 3;
    }
    else
    {
    }
#endif
#endif
  }
}

void Task_LED_quick(void *pvParam)
{
  int mode = 0;
  uint8_t i2 = 0, j;
  pinMode(15, OUTPUT);
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  while (1)
  {
    vTaskDelay(PERIOD_LEDquick);
    if (!Over_Temp)
    {
#ifdef DEBUG3
      Serial.printf("set=%i, в очереди свободно - %i пунктов\r\n", mode, uxQueueSpacesAvailable(q_effectLEDquick));
#endif
      if (mode > 0)
      {
        vTaskSuspend(Handle_LED_slow);
        vTaskSuspend(Handle_LED_fon);
      }
      switch (mode)
      {
      case 0:
        i2 = 0;
        xQueueReceive(q_effectLEDquick, &mode, 0);
        // xQueuePeek(q_effectLEDquick, &mode, 0);
        dynamic_ilumination == false ? mode = 0 : mode; // Если динамическая подсветка выключена - ничего не делать
        if (mode == 0)
        {
          vTaskResume(Handle_LED_slow);
        }
        break;

      case 1:
        // плавное включение
        fill_solid(leds, NUM_LEDS, CHSV(100, 0, i2));
        i2 = qadd8(i2, TEMP_OF_LED_ON);
        if (i2 >= 255)
        {
          mode = 0;
          i2 = 0;
        }
        break;

      case 2:
        // плавное выключение
        fill_solid(leds, NUM_LEDS, leds->fadeToBlackBy(SCALE_FADE_OFF));
        if (leds[0].getAverageLight() == 0)
        {
          fill_solid(leds, NUM_LEDS, 0);
          // xQueueReceive(q_effectLEDquick, &mode, 0);
          mode = 0;
        }
        break;

      case 3:
        // включение.. мигание люм лампы
        fill_solid(leds, NUM_LEDS, Tungsten100W);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(100));
        fill_solid(leds, NUM_LEDS, 0);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(500));
        fill_solid(leds, NUM_LEDS, Tungsten100W);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(50));
        fill_solid(leds, NUM_LEDS, 0);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(50));
        fill_solid(leds, NUM_LEDS, Tungsten100W);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(50));
        fill_solid(leds, NUM_LEDS, 0);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(50));
        fill_solid(leds, NUM_LEDS, Tungsten100W);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(50));
        fill_solid(leds, NUM_LEDS, 0);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(200));
        // xQueueReceive(q_effectLEDquick, &mode, 0);
        mode = 0;
        break;

      case 4:
        // включение.. мигание люм лампы с накалом
        fill_solid(leds, NUM_LEDS, 0);
        leds[0] = Nacal;
        leds[NUM_LEDS - 1] = Nacal;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(3000));
        fill_solid(leds, NUM_LEDS, 0xFFFFFF);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(50));
        fill_solid(leds, NUM_LEDS, 0);
        leds[0] = Nacal;
        leds[NUM_LEDS - 1] = Nacal;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(300));
        fill_solid(leds, NUM_LEDS, 0xFFFFFF);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(20));
        fill_solid(leds, NUM_LEDS, 0);
        leds[0] = Nacal;
        leds[NUM_LEDS - 1] = Nacal;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(1000));
        fill_solid(leds, NUM_LEDS, CHSV(0, 50, 150));
        // xQueueReceive(q_effectLEDquick, &mode, 0);
        mode = 0;
        break;

      case 6: // плавно включить желтый
        vTaskSuspend(Handle_LED_slow);
        fill_solid(leds, NUM_LEDS, CHSV(60, 255, i2));
        if (i2 >= 255)
        {

          mode = 0;
          i2 = 0;
        }
        i2 = qadd8(i2, TEMP_OF_LED_ON);
        break;

      case 7: // включить плавно зеленый
        vTaskSuspend(Handle_LED_slow);
        fill_solid(leds, NUM_LEDS, CHSV(TIMER_PASS_TONE, TIMER_CONTR, i2));
        i2 = qadd8(i2, TEMP_OF_LED_ON);
        if (i2 > TIMER_BRITH)
        {

          mode = 0;
          i2 = 0;
        }
        break;

      case 8: // выключить плавно
        vTaskSuspend(Handle_LED_slow);
        fill_solid(leds, NUM_LEDS, CHSV(TIMER_PASS_TONE, TIMER_CONTR, TIMER_BRITH - i2));
        i2 = qadd8(i2, TEMP_OF_LED_ON);
        if (i2 > TIMER_BRITH)
        {
          mode = 0;
          i2 = 0;
        }

        break;

      case 9: // включение хайтек
        vTaskSuspend(Handle_LED_slow);
        fill_solid(leds, NUM_LEDS, 0);
        j = NUM_LEDS / 2 - 1;
        fill_solid(&leds[j - 1], 4, CHSV(20, 30, 150));
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(300));
        for (int i = 1; i < ((NUM_LEDS / 2) - 1); i++)
        {
          fill_solid(&leds[j - 1 - i], (4 + (i * 2)), CHSV(20, 30, 150));
          FastLED.show();
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        mode = 0;

        break;

      default:
        // xQueueReceive(q_effectLEDquick, &mode, 0);
        mode = 0;
        break;
      }
    }
    else
    {
      xQueueReceive(q_effectLEDslow, &mode, 0);
      mode = 0;
      fill_solid(leds, NUM_LEDS, 0);
    }
    FastLED.show();

#ifdef DEBUG3
    // Serial.print (i);   Serial.print (" ");  Serial.println (s);

#endif
  }
}

void Task_LED_slow(void *pvParam)
{
  int mode, time_of_led, send, send1;
  uint8_t led;
  while (1)
  {
    vTaskDelay(PERIOD_LEDslow);
#ifdef DEBUG3
    Serial.printf("set=%i, в очереди свободно - %i пунктов\r\n", mode, uxQueueSpacesAvailable(q_effectLEDquick));
#endif
    if (!Over_Temp)
    {
      xQueuePeek(q_effectLEDslow, &mode, 0);
      dynamic_ilumination == false ? mode = 0 : mode;
      if (mode > 0)
      {
        vTaskSuspend(Handle_LED_fon);
      }
      else
      {
        vTaskResume(Handle_LED_fon);
      }
      switch (mode)
      {
      case 0:
        // leds[1] == CRGB(0) ? leds[1] = CHSV(200, 100, 100) : leds[1] = CRGB(0, 0, 0);

        break;
      case 5:
        // таймер 15 минут, 30 минут
        if (timer_run)
        {
          fill_solid(leds, NUM_LEDS, TIMER_LESS);
          led = (TIMER15m - (timer - millis())) / (TIMER15m / NUM_LEDS);
          fill_solid(leds, led, TIMER_PASS);
          time_of_led = (TIMER15m - (timer - millis())) - ((int)led * (TIMER15m / NUM_LEDS)); // время светодиода
          leds[led] = CHSV(lerp8by8(TIMER_LESS_TONE, TIMER_PASS_TONE, (time_of_led * 255) / (TIMER15m / NUM_LEDS)), TIMER_CONTR, TIMER_BRITH);
          // if (i1 == 0)
          // {
          //   leds[i1].maximizeBrightness(255);
          // }
          // else if (i1 == NUM_LEDS)
          // {
          //   leds[i1 - 1].maximizeBrightness(TIMER_BRITH);
          // }
          // else if (i1 > NUM_LEDS)
          // {
          //   ;
          // }
          // else if ((i1 > 0) & (i1 < NUM_LEDS))
          // {
          //   leds[i1].maximizeBrightness(255);
          //   leds[i1 - 1].maximizeBrightness(TIMER_BRITH);
          // }
          // i1++;
        }
        else
        {
          fill_solid(leds, NUM_LEDS, TIMER_PASS);
          xQueueReceive(q_effectLEDslow, &mode, 0);
          mode = 0;
          send = 7;
          send1 = 2;
          xQueueSendToBack(q_effectLEDquick, &send, 0);
          xQueueSendToBack(q_effectLEDquick, &send1, 0);
          xQueueSendToBack(q_effectLEDquick, &send, 0);
          xQueueSendToBack(q_effectLEDquick, &send1, 0);
          xQueueSendToBack(q_effectLEDquick, &send, 0);
          // send = 4;
          // xQueueSendToBack(q_effectLEDquick, &send, 0);
        }
        break;

      case 6:
        // таймер 15 минут, 30 минут
        if (timer_run)
        {
          fill_solid(leds, NUM_LEDS, TIMER_LESS);
          led = (TIMER30m - (timer - millis())) / (TIMER30m / NUM_LEDS);
          fill_solid(leds, led, TIMER_PASS);
          time_of_led = (TIMER30m - (timer - millis())) - ((int)led * (TIMER30m / NUM_LEDS)); // время светодиода
          leds[led] = CHSV(lerp8by8(TIMER_LESS_TONE, TIMER_PASS_TONE, (time_of_led * 255) / (TIMER30m / NUM_LEDS)), TIMER_CONTR, TIMER_BRITH);
          // if (i1 == 0)
          // {
          //   leds[i1].maximizeBrightness(255);
          // }
          // else if (i1 == NUM_LEDS)
          // {
          //   leds[i1 - 1].maximizeBrightness(TIMER_BRITH);
          // }
          // else if (i1 > NUM_LEDS)
          // {
          //   ;
          // }
          // else if ((i1 > 0) & (i1 < NUM_LEDS))
          // {
          //   leds[i1].maximizeBrightness(255);
          //   leds[i1 - 1].maximizeBrightness(TIMER_BRITH);
          // }
          // i1++;
        }
        else
        {
          fill_solid(leds, NUM_LEDS, TIMER_PASS);
          xQueueReceive(q_effectLEDslow, &mode, 0);
          mode = 0;
          send = 7;
          send1 = 2;
          xQueueSendToBack(q_effectLEDquick, &send, 0);
          xQueueSendToBack(q_effectLEDquick, &send1, 0);
          xQueueSendToBack(q_effectLEDquick, &send, 0);
          xQueueSendToBack(q_effectLEDquick, &send1, 0);
          xQueueSendToBack(q_effectLEDquick, &send, 0);
          // send = 4;
          // xQueueSendToBack(q_effectLEDquick, &send, 0);
        }
        break;

      default:
        // xQueueReceive(q_effectLEDslow, &mode, 0);
        mode = 0;
        break;
      }
      FastLED.show();
    }
    else
    {
      xQueueReceive(q_effectLEDslow, &mode, 0);
      mode = 0;
      fill_solid(leds, NUM_LEDS, 0);
      FastLED.show();
    }

#ifdef DEBUG3
    // Serial.print (i);   Serial.print (" ");  Serial.println (s);

#endif
  }
}

void Task_LED_fon(void *pvParam)
{
  int8_t i;
  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(PERIOD_LEDfon));
    if (!Over_Temp)
    {
      if (dynamic_ilumination)
      {
        switch (led_fon)
        {
        case 0: // белая подсветка
          fill_solid(leds, NUM_LEDS, WHITE_BACKLIGHT);
          FastLED.show();
          vTaskDelay(pdMS_TO_TICKS(1000));
          break;

        case 1: // зеленая подсветка
          fill_solid(leds, NUM_LEDS, GREEN_BACKLIGHT);
          FastLED.show();
          vTaskDelay(pdMS_TO_TICKS(1000));
          break;

        case 2: // желтая мигающая подсветка
          if (!((-(FLASH_BACKLIGHT_DELTA) <= i) & (i <= (FLASH_BACKLIGHT_DELTA))))
          {
            i = (-(FLASH_BACKLIGHT_DELTA));
          }
          fill_solid(leds, NUM_LEDS, CHSV(YELOW_BACKLIGHT_H, YELOW_BACKLIGHT_S, qadd8(YELOW_BACKLIGHT_V, abs8(i))));
          FastLED.show();
          i = i + YELOW_BACKLIGHT_SPEED;
          break;

        case 3: // красная мигающая подсветка
          if (!((-(FLASH_BACKLIGHT_DELTA) <= i) & (i <= (FLASH_BACKLIGHT_DELTA))))
          {
            i = (-(FLASH_BACKLIGHT_DELTA));
          }
          fill_solid(leds, NUM_LEDS, CHSV(RED_BACKLIGHT_H, YELOW_BACKLIGHT_S, qadd8(YELOW_BACKLIGHT_V, abs8(i))));
          FastLED.show();
          i = i + YELOW_BACKLIGHT_SPEED;
          break;

        default:
          break;
        }
      }
      else
      {
        fill_solid(leds, NUM_LEDS, WHITE_BACKLIGHT);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
    else
    {
      fill_solid(leds, NUM_LEDS, 0);
      FastLED.show();
    }
  }
}

void Task_Debug(void *pvParam)
{
  while (1)
  {
    vTaskDelay(PERIOD_Debug);
    char buf[50];
    sprintf(buf, "Moist=%.3f, ppm=%.3f, temp=%.3f, result = %i, H2_PCB_Temp=%.3f, H2_Oil_Temp=%.3f \n", WaterA, WaterC, te, result, h2_PCB_T, h2_Oil_T);
    t90.setText(buf);
    //   uint8_t set, r, g, b;
    //   int s;
    //   while (1)
    //   {
    //     vTaskDelay(periodDebug);
    //     switch (led_fon)
    //     {
    //     case 0:
    //     led_fon = 1;
    //       break;
    //         case 1:
    //     led_fon = 2;
    //       break;

    //               case 2:
    //     led_fon = 0;
    //       break;
    //     default:
    //       break;
    //     }

    //     // if (Serial.readBytes((&r, &g, &b), 3) > 0)
    //     // {
    //     //   s = set - 0x30;
    //     //   if (q_effectLEDquick != 0)
    //     //   {
    //     //     xQueueSendToFront(q_effectLEDquick, &s, 0);
    //     //     set = 0;
    //     //   }
    //     //   // if (s == 6)
    //     //   // {
    //     //   //   timer = millis() + TIMER30m; // запустить таймер на 30 минут (900000 миллисекунд)
    //     //   //   timer_run = true;
    //     //   // }
    //     //   if (s == 5)
    //     //   {
    //     //     timer = millis() + TIMER15m; // запустить таймер на 30 минут (900000 миллисекунд)
    //     //     timer_run = true;
    //     //   }
    //     //   // Serial.printf("set=%i\r\n", set);
    //     // }
    //   }
  }
}

void Task_DS18b20(void *pvParam)
{
  ds.reset();     // Начинаем взаимодействие со сброса всех предыдущих команд и параметров
  ds.write(0xCC); // Даем датчику DS18b20 команду пропустить поиск по адресу. В нашем случае только одно устрйоство
  ds.write(0x44);

  while (1)
  {
    vTaskDelay(PERIOD_DS18b20);

    ds.reset(); // Начинаем взаимодействие со сброса всех предыдущих команд и параметров
    ds.write(0xCC);
    ds.write(0xBE);

    PCB_Temper_TOR1 = ((ds.read() | (ds.read() << 8))) * 0.0625;

    ds.reset();     // Начинаем взаимодействие со сброса всех предыдущих команд и параметров
    ds.write(0xCC); // Даем датчику DS18b20 команду пропустить поиск по адресу. В нашем случае только одно устрйоство
    ds.write(0x44);

    CPU_Temper_TOR1 = temperatureRead();

    // if (CPU_Temper_TOR1 > 100.0 | PCB_Temper_TOR1 > 100.0) {
    //   Over_Temp = true;
    // }else {
    //   Over_Temp = false;
    // }
    // if (!ds.search(addr))
    // {
    //   Serial.println("No more addresses.");
    //   Serial.println();
    //   ds.reset_search();
    //   delay(250);
    // }

    // Serial.print("ROM =");
    // for (i = 0; i < 8; i++)
    // {
    //   Serial.write(' ');
    //   Serial.print(addr[i], HEX);
    // }

    // if (OneWire::crc8(addr, 7) != addr[7])
    // {
    //   Serial.println("CRC is not valid!");
    // }
    // Serial.println();

    // // the first ROM byte indicates which chip
    // switch (addr[0])
    // {
    // case 0x10:
    //   Serial.println("  Chip = DS18S20"); // or old DS1820
    //   type_s = 1;
    //   break;
    // case 0x28:
    //   Serial.println("  Chip = DS18B20");
    //   type_s = 0;
    //   break;
    // case 0x22:
    //   Serial.println("  Chip = DS1822");
    //   type_s = 0;
    //   break;
    // default:
    //   Serial.println("Device is not a DS18x20 family device.");
    //   break;
    // }

    // ds.reset();
    // ds.select(addr);
    // ds.write(0x44, 1); // start conversion, with parasite power on at the end

    // delay(1000); // maybe 750ms is enough, maybe not
    // // we might do a ds.depower() here, but the reset will take care of it.

    // present = ds.reset();
    // ds.select(addr);
    // ds.write(0xBE); // Read Scratchpad

    // Serial.print("  Data = ");
    // Serial.print(present, HEX);
    // Serial.print(" ");
    // for (i = 0; i < 9; i++)
    // { // we need 9 bytes
    //   data[i] = ds.read();
    //   Serial.print(data[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.print(" CRC=");
    // Serial.print(OneWire::crc8(data, 8), HEX);
    // Serial.println();

    // // Convert the data to actual temperature
    // // because the result is a 16 bit signed integer, it should
    // // be stored to an "int16_t" type, which is always 16 bits
    // // even when compiled on a 32 bit processor.
    // int16_t raw = (data[1] << 8) | data[0];
    // if (type_s)
    // {
    //   raw = raw << 3; // 9 bit resolution default
    //   if (data[7] == 0x10)
    //   {
    //     // "count remain" gives full 12 bit resolution
    //     raw = (raw & 0xFFF0) + 12 - data[6];
    //   }
    // }
    // else
    // {
    //   byte cfg = (data[4] & 0x60);
    //   // at lower res, the low bits are undefined, so let's zero them
    //   if (cfg == 0x00)
    //     raw = raw & ~7; // 9 bit resolution, 93.75 ms
    //   else if (cfg == 0x20)
    //     raw = raw & ~3; // 10 bit res, 187.5 ms
    //   else if (cfg == 0x40)
    //     raw = raw & ~1; // 11 bit res, 375 ms
    //   //// default is 12 bit resolution, 750 ms conversion time
    // }
    // PCB_Temper_TOR1 = (float)raw / 16.0;
    // Serial.print("  Temperature = ");
    // Serial.print(PCB_Temper_TOR1);
    // Serial.print(" Celsius, ");
  }
}

void updateAll(void)
{
#ifdef DEBUG5
  SWserial.println("Update after SLEEP " + Dan);
#endif
  Serial.print("data.txt=\"" + String(DATA_) + "\""); // строка даты
  // Serial.print(0xFF,HEX);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
  D_m = D;
  M_m = M;
  Y_m = Y;

  Serial.print("time1.txt=\"" + String(TIME_) + "\""); // строка часы
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
  m_m = m;
  h_m = h;

  Serial.print("x0.val=");         //
                                   //      Serial.print((s*100),0);  //
  Serial.print((WaterA * 100), 0); //
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
  WaterA_m = WaterA;
  Serial.print("x1.val=");         //
                                   //       Serial.print((s*100),0);  //
  Serial.print((WaterC * 100), 0); //
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
  WaterC_m = WaterC;

  Serial.print("x2.val=");     //
  Serial.print((te * 100), 0); //
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
  te_m = te;

  Serial.print("x3.val="); //
  Serial.print(h2);        //
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
  h2_m = h2;

  if (!h2_Ready)
  {
    Serial.print("st.txt=\"AutoCalibrat.\"");
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.print("st.bco=65520"); // лимонный
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
  }
  //          else if (((h2_status & 0b1000000000000000) == 0b1000000000000000) and ((h2_status & 0b0000000000111000) == 0b0000000000001000))
  else if (h2_Ready)
  {
    Serial.print("st.txt=\"Ready\"");
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.print("st.bco=28288"); // зеленый
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
  }
  h2_status_m = h2_status;
}

int8_t recvPageNumber(uint32_t timeout)
{
  sendCommand("sendme");

  int8_t ret = 1;
  uint8_t temp[5] = {0};

  nexSerial.setTimeout(timeout);
  if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
  {
    ret = -1;
    return ret;
  }

  if (temp[0] == 0x66 && temp[2] == 0xFF && temp[3] == 0xFF && temp[4] == 0xFF)
  {
    ret = temp[1];
  }

  return ret;
}
