#ifndef GYEMS_H
#define GYEMS_H

#include "Arduino.h"
#include <SoftwareSerial.h>  // รวมไลบรารี SoftwareSerial

#define RS485_EN 2
#define MAXDPS 30000

class GYEMS
{
public:
    GYEMS(int ID, int rxPin, int txPin, int baudRate);
    // คอนสตรักเตอร์เพื่อกำหนดค่า pinMode, baudrate และ SoftwareSerial

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// ฟังก์ชันการแปลงข้อมูล ///////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void Int16ToByteData(unsigned int Data, unsigned char StoreByte[2]);
    void Int32ToByteData(unsigned long Data, unsigned char StoreByte[4]);
    void Int64ToByteData(unsigned long long Data, unsigned char StoreByte[8]);
    unsigned int Make12BitData(unsigned char loByte, unsigned char hiByte);
    unsigned int Make14BitData(unsigned char loByte, unsigned char hiByte);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// ฟังก์ชันการควบคุมเซอร์โว /////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    float GetCurrentDEG();
    float EstimateDPS();
    float GetAverageSpeed();
    float ReadReply();
    void MotorOff();
    void MotorStop();
    void MotorRun();
    void SetZero();
    void TorqueControl(unsigned int Torque);
    void SpeedControl(float DPS);
    void PositionControlMode1(unsigned long long Deg);
    void PositionControlMode2(unsigned long long Deg, unsigned long DPS);
    void PositionControlMode3(unsigned long Deg, byte Direction);
    void PositionControlMode4(unsigned long Deg, unsigned long DPS, byte Direction);

private:
    int _currentPosition;
    byte Header = 0x3E;
    byte _ID;
    SoftwareSerial mySerial;  // ตัวแปรสมาชิก SoftwareSerial

    unsigned char ReadByte = 0;
    float OutputDeg;
    float CurrentDeg;

    float t2;
    float t1 = 0.0;
    float period;
    float theta1 = 0.0;
    float theta2;
    float CurrentDPS;
    float LastDPS;
};

GYEMS::GYEMS(int ID, int rxPin, int txPin, int baudRate) : mySerial(rxPin, txPin) {
    _ID = ID;
    pinMode(RS485_EN, OUTPUT);
    mySerial.begin(baudRate); // กำหนดค่า SoftwareSerial ด้วย baud rate ที่ระบุ
}

// กำหนดฟังก์ชันอื่น ๆ...

#endif  // GYEMS_H
