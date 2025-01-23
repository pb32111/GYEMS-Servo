/***************************************************************
* Arduino GYEMS RMD-L-70 Servo Motor Library - Version 1.0
* by Rasheed Kittinanthapanya
* a simple RS485 communication between GYEMS servo and Arduino
***************************************************************/

#include "Arduino.h"
#include "GYEMS.h"
#include <SoftwareSerial.h>

// กำหนด TX = Pin 10, RX = Pin 11
SoftwareSerial mySerial(10, 11);

GYEMS::GYEMS(int baudRate)
{
    mySerial.begin(baudRate);  // ใช้ SoftwareSerial แทน Serial1
    _ID = (byte)baudRate;  // Convert int to byte
    pinMode(RS485_EN, OUTPUT);  // Set enable pin for RS485 module as pin 2 of Arduino
    digitalWrite(RS485_EN, LOW);
}

void GYEMS::Int16ToByteData(unsigned int Data, unsigned char StoreByte[2])
{
    StoreByte[0] = (Data & 0xFF00) >> 8;  // High byte
    StoreByte[1] = Data & 0x00FF;  // Low byte
}

void GYEMS::Int32ToByteData(unsigned long Data, unsigned char StoreByte[4])
{
    StoreByte[0] = (Data & 0xFF000000) >> 24;  // Highest byte
    StoreByte[1] = (Data & 0x00FF0000) >> 16;
    StoreByte[2] = (Data & 0x0000FF00) >> 8;
    StoreByte[3] = Data & 0x000000FF;  // Lowest byte
}

void GYEMS::Int64ToByteData(unsigned long long Data, unsigned char StoreByte[8])
{
    StoreByte[0] = (Data & 0xFF00000000000000) >> 56;  // Highest byte
    StoreByte[1] = (Data & 0x00FF000000000000) >> 48;
    StoreByte[2] = (Data & 0x0000FF0000000000) >> 40;
    StoreByte[3] = (Data & 0x000000FF00000000) >> 32;
    StoreByte[4] = (Data & 0x00000000FF000000) >> 24;
    StoreByte[5] = (Data & 0x0000000000FF0000) >> 16;
    StoreByte[6] = (Data & 0x000000000000FF00) >> 8;
    StoreByte[7] = Data & 0x00000000000000FF;  // Lowest byte
}

unsigned int GYEMS::Make12BitData(unsigned char loByte, unsigned char hiByte)
{
    unsigned int word = (loByte & 0xFF) | ((hiByte & 0xFF) << 8);  // Construct 16-bit int
    return map(word, 0, 16383, 0, 4095);  // Convert 16-bit to 12-bit data
}

unsigned int GYEMS::Make14BitData(unsigned char loByte, unsigned char hiByte)
{
    return (loByte & 0xFF) | ((hiByte & 0xFF) << 8);  // Construct 16-bit int (encoder counts 14 bits)
}

float GYEMS::ReadReply()
{
    int i = 0;
    bool ReadOK = false;
    unsigned char EncoderReply[8] = {0};
    unsigned int EncoderData = 0;
    float CurrentDeg = 0;

    while (mySerial.available() > 0 && i < 8)  // Wait for incoming data on Rx
    {
        EncoderReply[i++] = mySerial.read();  // Read incoming byte
        ReadOK = true;
    }

    if (ReadOK)
    {
        EncoderData = Make14BitData(EncoderReply[5], EncoderReply[6]);  // Construct encoder data
        delayMicroseconds(2000);  // Delay to avoid mixing up data

        CurrentDeg = map((float)EncoderData, 0.0, 16383.0, 0.0, 360.0);  // Convert to degrees
    }

    return CurrentDeg;
}

float GYEMS::GetCurrentDEG()
{
    byte EncoderCommand = 0x90;
    byte DataLength = 0x00;
    byte DataCheckByte = Header + EncoderCommand + _ID + DataLength;

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(EncoderCommand);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
    delayMicroseconds(800);  // Delay to avoid mixing up data

    return ReadReply();  // Read reply and return degrees
}

float GYEMS::EstimateDPS()
{
    byte EncoderCommand = 0x90;
    byte DataLength = 0x00;
    byte DataCheckByte = Header + EncoderCommand + _ID + DataLength;

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(EncoderCommand);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving

    float theta2 = ReadReply();  // Current position
    unsigned long t2 = micros();  // Current time
    float period = (t2 - t1) * 0.000001;  // Time difference
    float CurrentDPS = (theta2 - theta1) / period;  // Velocity

    if (abs(CurrentDPS) > MAXDPS)
    {
        CurrentDPS = LastDPS;  // Reset to previous value if there's a spike
    }

    t1 = t2;  // Update previous time
    theta1 = theta2;  // Update previous angle
    LastDPS = CurrentDPS;  // Update previous velocity

    return CurrentDPS;
}

float GYEMS::GetAverageSpeed()
{
    float AccumSpeed = 0;
    int sampler = 250;  // Total sampling data for averaging

    for (int i = 0; i < sampler; i++)
    {
        AccumSpeed += EstimateDPS();  // Accumulate speed
        delayMicroseconds(1000);  // Delay to stabilize loop
    }

    return AccumSpeed / sampler;  // Return average speed
}

void GYEMS::MotorOff()
{
    byte OffCommand = 0x80;
    byte DataLength = 0x00;
    byte DataCheckByte = Header + OffCommand + _ID + DataLength;

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(OffCommand);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
    delayMicroseconds(800);  // Delay to avoid mixing up data
}

void GYEMS::MotorStop()
{
    byte StopCommand = 0x81;
    byte DataLength = 0x00;
    byte DataCheckByte = Header + StopCommand + _ID + DataLength;

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(StopCommand);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
    delayMicroseconds(800);  // Delay to avoid mixing up data
}

void GYEMS::MotorRun()
{
    byte RunCommand = 0x88;
    byte DataLength = 0x00;
    byte DataCheckByte = Header + RunCommand + _ID + DataLength;

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(RunCommand);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
    delayMicroseconds(800);  // Delay to avoid mixing up data
}

void GYEMS::Setzero()
{
    byte SetZeroCommand = 0x19;
    byte DataLength = 0x00;
    byte DataCheckByte = Header + SetZeroCommand + _ID + DataLength;

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(SetZeroCommand);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
    delayMicroseconds(800);  // Delay to avoid mixing up data
}

void GYEMS::TorqueControl(unsigned int Torque)
{
    byte TorqueCommand = 0xA1;
    byte DataLength = 0x02;
    byte FrameCheckSum = Header + TorqueCommand + DataLength + _ID;
    unsigned char TorqueByte[2];
    Int16ToByteData(Torque, TorqueByte);
    byte DataCheckByte = TorqueByte[1] + TorqueByte[0];

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(TorqueCommand);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(FrameCheckSum);
    mySerial.write(TorqueByte[1]);
    mySerial.write(TorqueByte[0]);
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
}

void GYEMS::SpeedControl(float DPS)
{
    unsigned long SpeedLSB = DPS * 100;
    byte SpeedCommand = 0xA2;
    byte DataLength = 0x04;
    byte FrameCheckSum = Header + SpeedCommand + DataLength + _ID;
    unsigned char SpeedByte[4];
    Int32ToByteData(SpeedLSB, SpeedByte);
    byte DataCheckByte = SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(SpeedCommand);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(FrameCheckSum);
    mySerial.write(SpeedByte[3]);
    mySerial.write(SpeedByte[2]);
    mySerial.write(SpeedByte[1]);
    mySerial.write(SpeedByte[0]);
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
}

void GYEMS::PositionControlMode1(unsigned long long Deg)
{
    unsigned long long DegLSB = Deg * 100;
    byte PositionCommand1 = 0xA3;
    byte DataLength = 0x08;
    byte FrameCheckSum = Header + PositionCommand1 + _ID + DataLength;
    unsigned char PositionByte[8];
    Int64ToByteData(DegLSB, PositionByte);
    byte DataCheckByte = 0;
    for (int i = 0; i < 8; i++) {
        DataCheckByte += PositionByte[i];
    }

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(PositionCommand1);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(FrameCheckSum);
    for (int i = 0; i < 8; i++) {
        mySerial.write(PositionByte[i]);
    }
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
}

void GYEMS::PositionControlMode2(unsigned long long Deg, unsigned long DPS)
{
    unsigned long long DegLSB = Deg * 100;
    unsigned long SpeedLSB = DPS * 100;
    byte PositionCommand2 = 0xA4;
    byte DataLength = 0x0C;
    byte FrameCheckSum = Header + PositionCommand2 + _ID + DataLength;
    unsigned char PositionByte[8];
    unsigned char SpeedByte[4];
    Int64ToByteData(DegLSB, PositionByte);
    Int32ToByteData(SpeedLSB, SpeedByte);
    byte DataCheckByte = 0;
    for (int i = 0; i < 8; i++) {
        DataCheckByte += PositionByte[i];
    }
    for (int i = 0; i < 4; i++) {
        DataCheckByte += SpeedByte[i];
    }

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(PositionCommand2);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(FrameCheckSum);
    for (int i = 0; i < 8; i++) {
        mySerial.write(PositionByte[i]);
    }
    for (int i = 0; i < 4; i++) {
        mySerial.write(SpeedByte[i]);
    }
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
}

void GYEMS::PositionControlMode3(unsigned long Deg, byte Direction)
{
    unsigned long DegLSB = Deg * 100;
    byte PositionCommand3 = 0xA5;
    byte DataLength = 0x04;
    byte FrameCheckSum = Header + PositionCommand3 + _ID + DataLength;
    unsigned char PositionByte[4];
    Int32ToByteData(DegLSB, PositionByte);
    byte DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1];

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(PositionCommand3);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(FrameCheckSum);
    mySerial.write(Direction);
    for (int i = 0; i < 4; i++) {
        mySerial.write(PositionByte[i]);
    }
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
}

void GYEMS::PositionControlMode4(unsigned long Deg, unsigned long DPS, byte Direction)
{
    unsigned long DegLSB = Deg * 100;
    unsigned long SpeedLSB = DPS * 100;
    byte PositionCommand4 = 0xA6;
    byte DataLength = 0x08;
    byte FrameCheckSum = Header + PositionCommand4 + _ID + DataLength;
    unsigned char PositionByte[4];
    unsigned char SpeedByte[4];
    Int32ToByteData(DegLSB, PositionByte);
    Int32ToByteData(SpeedLSB, SpeedByte);
    byte DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

    digitalWrite(RS485_EN, HIGH);  // Enable RS485 for sending
    delayMicroseconds(50);  // Delay before sending

    mySerial.write(Header);
    mySerial.write(PositionCommand4);
    mySerial.write(_ID);
    mySerial.write(DataLength);
    mySerial.write(FrameCheckSum);
    mySerial.write(Direction);
    for (int i = 0; i < 4; i++) {
        mySerial.write(PositionByte[i]);
    }
    for (int i = 0; i < 4; i++) {
        mySerial.write(SpeedByte[i]);
    }
    mySerial.write(DataCheckByte);
    mySerial.flush();

    digitalWrite(RS485_EN, LOW);  // Enable RS485 for receiving
}
