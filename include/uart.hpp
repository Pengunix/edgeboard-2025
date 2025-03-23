#pragma once
#include "common.hpp"

#define UART_FRAME_HEAD 0x34
#define UART_FRAME_TAIL 0x43
#define UART_TX_SIZE 12
#define UART_RX_SIZE 24

union UartTranFrame {
  uint8_t data[UART_TX_SIZE];
  struct {
    uint8_t head;
    int8_t buzzer;
    uint16_t servo;
    float speed;
    uint8_t _reserved;
    uint8_t dis; // x100
    uint8_t xorCheck;
    uint8_t tail;
  };
};

union UartRecvFrame {
  uint8_t data[UART_RX_SIZE];
  struct {
    uint8_t head;
    uint8_t keys;
    uint8_t dis_achieved;
    uint8_t _reserved;
    float speed;
    float roll;
    float pitch;
    float yaw;
    uint8_t _reserved1[2];
    uint8_t xorCheck;
    uint8_t tail;
  };
};

enum Buzzer {
  BUZZER_SLIENT = -1,
  BUZZER_OK = 0,   // 确认
  BUZZER_WARNNING, // 报警
  BUZZER_FINISH,   // 完成
  BUZZER_DING,     // 提示
  BUZZER_START,    // 开机
};

class Uart {
private:
  std::unique_ptr<std::thread> threadRec;
  std::shared_ptr<LibSerial::SerialPort> serialPort = nullptr;
  std::string portName;
 

  int receiveBytes(unsigned char &charBuffer, size_t msTimeout = 0) {
    try {
      serialPort->ReadByte(charBuffer, msTimeout);

    } catch (const LibSerial::ReadTimeout &) {
      spdlog::critical("Serail Receive Timeout");
      return -2;
    }
    return 0;
  };

public:
  Uart(const std::string &port) : portName(port){};
  ~Uart() { close(); };

  bool keypress = false;
  bool send = false;
  bool isOpen = false;
  Buzzer buzzer = BUZZER_SLIENT;

  int open(void) {
    serialPort = std::make_shared<LibSerial::SerialPort>();
    if (serialPort == nullptr) {
      spdlog::critical("Serial Pointer is NULL!");
      return -1;
    }
    try {
      serialPort->Open(portName);
      // 设置波特率
      serialPort->SetBaudRate(LibSerial::BaudRate::BAUD_500000);
      // 8位数据位
      serialPort->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      // 无流控
      serialPort->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
      // 无校验
      serialPort->SetParity(LibSerial::Parity::PARITY_NONE);
      // 1个停止位
      serialPort->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    } catch (const LibSerial::OpenFailed &) {
      spdlog::critical("Serial Open Failed!");
      isOpen = false;
      return -2;

    } catch (const LibSerial::AlreadyOpen &) {
      spdlog::critical("Serial Open Failed!");
      isOpen = false;
      return -3;

    } catch (...) {
      spdlog::critical("There are some ERROR in Serial!");
      isOpen = false;
      return -4;
    }

    isOpen = true;
    return 0;
  }

  void startReceive() {
    if (!isOpen) // 串口是否正常打开
      return;
    // 启动串口接收子线程
    threadRec = std::make_unique<std::thread>([this]() {
      while (1) {
        receiveCheck(); // 串口接收校验
      }
    });
  }

  void close(void) {
    spdlog::warn("Uart Exit!");
    carControl(0, PWMSERVOMID);

    // threadRec->join();
    if (serialPort != nullptr) { // 释放串口资源
      serialPort->Close();
      serialPort = nullptr;
    }
    isOpen = false;
  }

 void receiveCheck() {
//     if (!isOpen) {
//       return;
//     }

//     uint8_t resByte = 0;
//     int ret = receiveBytes(resByte, 0);
//     if (ret == 0) {
//       if (resByte == UART_FRAME_HEAD) {

//         serialStr.start = true;
//         serialStr.buffRead[0] = resByte;
//         serialStr.buffRead[2] = UART_FRAME_LENMIN;
//         serialStr.index = 1;
//       } else if (serialStr.index == 2) {

//         serialStr.buffRead[serialStr.index] = resByte;
//         serialStr.index++;
//         if (resByte > UART_FRAME_LENMAX || resByte < UART_FRAME_LENMIN) {
//           serialStr.buffRead[2] = UART_FRAME_LENMIN;
//           serialStr.index = 0;
//           serialStr.start = false;
//         }

//       } else if (serialStr.start && serialStr.index < UART_FRAME_LENMAX) {
//         serialStr.buffRead[serialStr.index] = resByte;
//         serialStr.index++;
//       }

//       // 帧长接收完毕
//       if ((serialStr.index >= UART_FRAME_LENMAX ||
//            serialStr.index >= serialStr.buffRead[2]) &&
//           serialStr.index > UART_FRAME_LENMIN) {

//         // 求校验和
//         uint8_t check = 0;
//         uint8_t length = UART_FRAME_LENMIN;
//         length = serialStr.buffRead[2];
//         for (int i = 0; i < length - 1; i++)
//           check += serialStr.buffRead[i];

//         if (check == serialStr.buffRead[length - 1]) {
//           memcpy(serialStr.buffFinish, serialStr.buffRead, UART_FRAME_LENMAX);
//           dataTransform();
//         }

//         serialStr.index = 0;
//         serialStr.start = false;
//       }
//     }
  }

  void carControl(float speed, uint16_t servo) {
    if (!isOpen) {
      return;
    }
    uint8_t check = 0;
    UartTranFrame buff = {0};

    buff.head = UART_FRAME_HEAD;
    buff.tail = UART_FRAME_TAIL;
    buff.speed = speed;
    buff.servo = servo;
    buff.buzzer = this->buzzer;

    for (int i = 0; i < UART_TX_SIZE; i++) {
      check ^= buff.data[i];
    }
    buff.xorCheck = check;
    LibSerial::DataBuffer lbuff;
    for (uint8_t &i: buff.data) {
      lbuff.emplace_back(i);
    }
    serialPort->Write(lbuff);
    this->buzzer = BUZZER_SLIENT;

  }
};
