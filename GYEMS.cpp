#include <SoftwareSerial.h>
#include "GYEMS.h"

#define RX_PIN 10  // ขาที่ใช้เป็น RX
#define TX_PIN 11  // ขาที่ใช้เป็น TX
#define RS485_EN 2 // ขาที่ใช้เป็น enable สำหรับ RS485

SoftwareSerial mySerial(RX_PIN, TX_PIN);

GYEMS::GYEMS(int ID) {
  _ID = (byte)ID;
  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);
  mySerial.begin(115200); // ใช้ mySerial แทน Serial1 สำหรับ Arduino Uno
}

void GYEMS::Int16ToByteData(unsigned int Data, unsigned char StoreByte[2]) {
  StoreByte[0] = (Data & 0xFF00) >> 8;
  StoreByte[1] = (Data & 0x00FF);
}

void GYEMS::Int32ToByteData(unsigned long Data, unsigned char StoreByte[4]) {
  StoreByte[0] = (Data & 0xFF000000) >> 24;
  StoreByte[1] = (Data & 0x00FF0000) >> 16;
  StoreByte[2] = (Data & 0x0000FF00) >> 8;
  StoreByte[3] = (Data & 0x000000FF);
}

void GYEMS::Int64ToByteData(unsigned long long Data, unsigned char StoreByte[8]) {
  StoreByte[0] = (Data & 0xFF00000000000000) >> 56;
  StoreByte[1] = (Data & 0x00FF000000000000) >> 48;
  StoreByte[2] = (Data & 0x0000FF0000000000) >> 40;
  StoreByte[3] = (Data & 0x000000FF00000000) >> 32;
  StoreByte[4] = (Data & 0x00000000FF000000) >> 24;
  StoreByte[5] = (Data & 0x0000000000FF0000) >> 16;
  StoreByte[6] = (Data & 0x000000000000FF00) >> 8;
  StoreByte[7] = (Data & 0x00000000000000FF);
}

unsigned int GYEMS::Make12BitData(unsigned char loByte, unsigned char hiByte) {
  unsigned int word;
  unsigned int Ang12Bit;
  word = (loByte & 0xFF) | ((hiByte & 0xFF) << 8);
  Ang12Bit = map(word, 0, 16383, 0, 4095);
  return Ang12Bit;
}

unsigned int GYEMS::Make14BitData(unsigned char loByte, unsigned char hiByte) {
  unsigned int word;
  unsigned int Ang14Bit;
  Ang14Bit = (loByte & 0xFF) | ((hiByte & 0xFF) << 8);
  return Ang14Bit;
}

float GYEMS::ReadReply() {
  int i = 0;
  bool ReadOK = false;
  unsigned char EncoderReply[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  unsigned int EncoderData = 0;
  while ((mySerial.available() > 0)) {
    ReadByte = mySerial.read();
    EncoderReply[i] = ReadByte;
    i++;
    ReadOK = true;
  }
  if (ReadOK == true) {
    EncoderData = Make14BitData(EncoderReply[5], EncoderReply[6]);
    delayMicroseconds(2000);
    CurrentDeg = map((float)EncoderData, 0.0, 16383.0, 0.0, 360.0);
  }
  return CurrentDeg;
}

float GYEMS::GetCurrentDEG() {
  byte EncoderCommand = 0x90;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + EncoderCommand + _ID + DataLength;
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(EncoderCommand);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
  delayMicroseconds(800);
  OutputDeg = ReadReply();
  delayMicroseconds(800);
  return OutputDeg;
}

float GYEMS::EstimateDPS() {
  byte EncoderCommand = 0x90;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + EncoderCommand + _ID + DataLength;
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(EncoderCommand);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
  theta2 = ReadReply();
  t2 = micros();
  period = (t2 - t1) * 0.000001;
  CurrentDPS = (theta2 - theta1) / period;
  if (abs(CurrentDPS) > MAXDPS) {
    CurrentDPS = LastDPS;
  }
  t1 = t2;
  theta1 = theta2;
  LastDPS = CurrentDPS;
  return CurrentDPS;
}

float GYEMS::GetAverageSpeed() {
  float Speed;
  float AccumSpeed;
  int sampler = 250;
  float aveDPS;
  int i = 0;
  for (i = 0; i < sampler; i++) {
    Speed = EstimateDPS();
    if (i == 0) {
      AccumSpeed = Speed;
    } else {
      AccumSpeed = AccumSpeed + Speed;
    }
    delayMicroseconds(1000);
  }
  aveDPS = AccumSpeed / sampler;
  return aveDPS;
}

void GYEMS::MotorOff() {
  byte OffCommand = 0x80;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + OffCommand + _ID + DataLength;
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(OffCommand);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
  delayMicroseconds(800);
}

void GYEMS::MotorStop() {
  byte StopCommand = 0x81;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + StopCommand + _ID + DataLength;
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(StopCommand);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
  delayMicroseconds(800);
}

void GYEMS::MotorRun() {
  byte RunCommand = 0x88;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + RunCommand + _ID + DataLength;
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(RunCommand);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
  delayMicroseconds(800);
}

void GYEMS::Setzero() {
  byte SetZeroCommand = 0x19;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + SetZero + _ID + DataLength;
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(SetZero);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
  delayMicroseconds(800);
}

void GYEMS::TorqueControl(unsigned int Torque) {
  byte TorqueCommand = 0xA1;
  byte DataLength = 0x02;
  byte FrameCheckSum = Header + TorqueCommand + DataLength + _ID;
  unsigned char TorqueByte[2];
  Int16ToByteData(Torque, TorqueByte);
  byte DataCheckByte = TorqueByte[1] + TorqueByte[0];
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(TorqueCommand);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(FrameCheckSum);
  mySerial.write(TorqueByte[1]);
  mySerial.write(TorqueByte[0]);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
}

void GYEMS::SpeedControl(float DPS) {
  float SpeedLSB = DPS * 100;
  byte SpeedCommand = 0xA2;
  byte DataLength = 0x04;
  byte FrameCheckSum = Header + SpeedCommand + DataLength + _ID;
  unsigned char SpeedByte[4];
  Int32ToByteData((int)SpeedLSB, SpeedByte);
  byte DataCheckByte = SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
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
  digitalWrite(RS485_EN, LOW);
}

void GYEMS::PositionControlMode1(unsigned long long Deg) {
  float CurrentDeg;
  unsigned long long DegLSB = Deg * 100;
  byte PositionCommand1 = 0xA3;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + PositionCommand1 + _ID + DataLength;
  unsigned char PositionByte[8];
  Int64ToByteData(DegLSB, PositionByte);
  byte DataCheckByte = PositionByte[7] + PositionByte[6] + PositionByte[5] + PositionByte[4] + PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0];
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(PositionCommand1);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(FrameCheckSum);
  mySerial.write(PositionByte[7]);
  mySerial.write(PositionByte[6]);
  mySerial.write(PositionByte[5]);
  mySerial.write(PositionByte[4]);
  mySerial.write(PositionByte[3]);
  mySerial.write(PositionByte[2]);
  mySerial.write(PositionByte[1]);
  mySerial.write(PositionByte[0]);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
}

void GYEMS::PositionControlMode2(unsigned long long Deg, unsigned long DPS) {
  unsigned long long DegLSB = Deg * 100;
  unsigned long SpeedLSB = DPS * 100;
  byte PositionCommand2 = 0xA4;
  byte DataLength = 0x0C;
  byte FrameCheckSum = Header + PositionCommand2 + _ID + DataLength;
  unsigned char PositionByte[8];
  unsigned char SpeedByte[4];
  Int32ToByteData(SpeedLSB, SpeedByte);
  Int64ToByteData(DegLSB, PositionByte);
  byte DataCheckByte = PositionByte[7] + PositionByte[6] + PositionByte[5] + PositionByte[4] + PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(PositionCommand2);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(FrameCheckSum);
  mySerial.write(PositionByte[7]);
  mySerial.write(PositionByte[6]);
  mySerial.write(PositionByte[5]);
  mySerial.write(PositionByte[4]);
  mySerial.write(PositionByte[3]);
  mySerial.write(PositionByte[2]);
  mySerial.write(PositionByte[1]);
  mySerial.write(PositionByte[0]);
  mySerial.write(SpeedByte[3]);
  mySerial.write(SpeedByte[2]);
  mySerial.write(SpeedByte[1]);
  mySerial.write(SpeedByte[0]);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
}

void GYEMS::PositionControlMode3(unsigned long Deg, byte Direction) {
  unsigned long DegLSB = Deg * 100;
  byte PositionCommand3 = 0xA5;
  byte DataLength = 0x04;
  byte FrameCheckSum = Header + PositionCommand3 + _ID + DataLength;
  unsigned char PositionByte[4];
  Int32ToByteData(DegLSB, PositionByte);
  byte DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1];
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(PositionCommand3);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(FrameCheckSum);
  mySerial.write(Direction);
  mySerial.write(PositionByte[3]);
  mySerial.write(PositionByte[2]);
  mySerial.write(PositionByte[1]);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
}

void GYEMS::PositionControlMode4(unsigned long Deg, unsigned long DPS, byte Direction) {
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
  digitalWrite(RS485_EN, HIGH);
  delayMicroseconds(50);
  mySerial.write(Header);
  mySerial.write(PositionCommand4);
  mySerial.write(_ID);
  mySerial.write(DataLength);
  mySerial.write(FrameCheckSum);
  mySerial.write(Direction);
  mySerial.write(PositionByte[3]);
  mySerial.write(PositionByte[2]);
  mySerial.write(PositionByte[1]);
  mySerial.write(SpeedByte[3]);
  mySerial.write(SpeedByte[2]);
  mySerial.write(SpeedByte[1]);
  mySerial.write(SpeedByte[0]);
  mySerial.write(DataCheckByte);
  mySerial.flush();
  digitalWrite(RS485_EN, LOW);
}
