#include "uart.hpp"
#include "unistd.h"

int main() {
  auto uart = std::make_shared<Uart>("/dev/ttyUSB0");
  uart->open();

  while(1) {
    uart->carControl(1.5, 1700);
    usleep(10000);
  }
}