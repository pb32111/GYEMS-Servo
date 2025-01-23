#include <SoftwareSerial.h>
#include "GYEMS.h"

// กำหนดพินสำหรับ RS485 EN
#define RS485_EN 2

// กำหนดพอร์ตซอฟต์แวร์ซีเรียล
SoftwareSerial RS485Serial(10, 11); // RX, TX

GYEMS myServo(1); // กำหนด ID ของเซอร์โวมอเตอร์เป็น 1

void setup() {
    pinMode(RS485_EN, OUTPUT);
    RS485Serial.begin(115200); // เริ่มการสื่อสารที่ baudrate 115200
    digitalWrite(RS485_EN, LOW); // ตั้งค่าเริ่มต้นเป็นโหมดรับข้อมูล
    
    Serial.begin(115200); // เริ่มการสื่อสารซีเรียลกับคอมพิวเตอร์สำหรับการดีบัก
}

void loop() {
    // ตัวอย่างการใช้ GYEMS library
    myServo.SetZero(); // ตั้งค่าตำแหน่งปัจจุบันเป็นตำแหน่งศูนย์
    delay(1000);
    
    myServo.SpeedControl(360); // ควบคุมความเร็วที่ 360 degree per second (dps)
    delay(2000);
    
    myServo.MotorStop(); // หยุดมอเตอร์
    delay(1000);

    float currentPosition = myServo.GetCurrentDEG(); // อ่านตำแหน่งปัจจุบัน
    Serial.print("Current Position: ");
    Serial.println(currentPosition);
    
    delay(1000);
}
