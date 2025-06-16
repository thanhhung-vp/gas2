//#define ERA_DEBUG
//#define ERA_SERIAL Serial
#define ERA_LOCATION_VN
//#define ERA_AUTH_TOKEN "0ac0db74-dca7-4b37-b525-8800eccdeb10"
#define ERA_AUTH_TOKEN "6f6e7773-291b-4aae-88d6-f08e8aa010ce"
#include <Arduino.h>
#include <ERa.hpp>
#define BUTTON 5
char buffer[64];
#include <Automation/ERaSmart.hpp>
#include <Time/ERaEspTime.hpp>
#include <HardwareSerial.h>
HardwareSerial SerialPort(2);//UART2 ESP32----STM32
#define LED_PIN  2
#define MQ2 34
float ppm;
uint8_t gas_active=0;
//const char ssid[] = "Duong192004";
//const char pass[] = "88888888";
const char ssid[] = "Duong192004";
const char pass[] = "88888888";
ERaEspTime syncTime;
TimeElement_t ntpTime;
ERaSmart smart(ERa, syncTime);

/* This function is called every time the Virtual Pin 0 state change */
ERA_WRITE(V2) {
    /* Get value from Virtual Pin 0 and write LED */
    uint8_t value = param.getInt();
    digitalWrite(LED_PIN, value ? HIGH : LOW);

    // Send the LED status back
    ERa.virtualWrite(V2, digitalRead(LED_PIN));
}
/* This function will execute each time from the Text Box to your chip */
ERA_WRITE(V1) {
    /* Get value from Virtual Pin 1 */
    ERaString estr = param.getString();

    // If you type "on", turn on LED
    // If you type "off", turn off LED
    if (estr == "on") {
        digitalWrite(LED_PIN, HIGH);
    }
    else if (estr == "off") {
        digitalWrite(LED_PIN, LOW);
    }

    // Send it back
    ERa.virtualWrite(V1, estr);
    // Send the LED status back
    //ERa.virtualWrite(V0, digitalRead(LED_PIN));
}
ERA_WRITE_SMS() {
    ERA_LOG("ERa", "Write SMS to %s: %s", to, message);
    return true;
}

/* This function will run every time ERa is connected */
ERA_CONNECTED() {
    ERA_LOG("ERa", "ERa connected!");
}

/* This function will run every time ERa is disconnected */
ERA_DISCONNECTED() {
    ERA_LOG("ERa", "ERa disconnected!");
}

/* This function send uptime every second to Virtual Pin 1 */
void timerEvent() {
    if(ppm>1)
      {
        ERa.virtualWrite(V0, ppm);
        Serial.print("Converted PPM: ");
        Serial.println(ppm); // Debug giá trị float
      }
}

void setup() {
/*#if defined(ERA_DEBUG)
    Serial.begin(115200);
#endif*/
    syncTime.setTimeZone(7L);
    syncTime.begin();
    Serial.begin(115200);
    SerialPort.begin(115200, SERIAL_8N1, 16, 17);//UART2 ESP32 --- STM32
    pinMode(LED_PIN, OUTPUT);
    pinMode(MQ2,INPUT);
    pinMode(BUTTON,INPUT_PULLUP);
    ERa.setScanWiFi(true);
    ERa.begin(ssid, pass);
    ERa.addInterval(500L, timerEvent);
}

void loop() {
    ERa.run();
    if(digitalRead(BUTTON)==0)
      {
        while(digitalRead(BUTTON)==0);
        SerialPort.print("REQ_LOGS\r\n");
        Serial.println("Sent log request to STM32: REQ_LOGS");
      }
      //Nhận data từ STM32 UART
  if (SerialPort.available()) {
    String data = SerialPort.readStringUntil('\n');
    if (data.startsWith("PPM:")) 
      {
        String ppmStr = data.substring(4); // Trích xuất giá trị PPM
        ppm = ppmStr.toFloat(); // Chuyển chuỗi nhận được sang kiểu float
        if(ppm>500 && gas_active==0)
          {
             gas_active=1;
             syncTime.getTime(ntpTime);
             snprintf(buffer, sizeof(buffer), "TIME:%04d-%02d-%02d %02d:%02d:%02d, PPM:%.2f\r\n",
             ntpTime.year + 1970, ntpTime.month, ntpTime.day,
             ntpTime.hour, ntpTime.minute, ntpTime.second,ppm);
             Serial.println("Sent time and float ppm to STM32: " + String(buffer));
             SerialPort.print(buffer);// gui TIME và PPM cho stm32 khi ppm>500
          }
          else if(ppm<500 && gas_active==1){
            gas_active=0;
          }
      }
      //Nhận log TIME +PPM  lưu FLASH TỪ STM32 SAU ĐÓ in lên Serial0 UART0 ESP32
    /*else if(data.startsWith("LOG_DATA:")) 
    {
        String logStr = data.substring(9); // Trích xuất dữ liệu log
        Serial.println("Log: " + logStr); // Hiển thị log trên Serial Monitor
    }
    else if (data.startsWith("NO_LOGS")) 
    {
        Serial.println("No logs available"); // Hiển thị thông báo không có log
    }*/
    //Serial.print("Converted PPM: ");
    //Serial.println(ppm); // Debug giá trị float
  }
    delay(500);
}
