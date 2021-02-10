#pragma once
#ifndef SIMPLEDRIVER_MODULE_H
#define SIMPLEDRIVER_MODULE_H

#include <stdint.h>
//using namespace std;
//#include "Module.h"
//using namespace std;
typedef struct {
  uint8_t drvtype;
  bool res;
  bool sw1;
  bool sw2;
  bool sw3;
  uint8_t chip;
  uint8_t uart_pin;
  uint32_t gconf;
  uint32_t ihold_irun;
  uint32_t chopconf;
  uint32_t pwmconf;
  uint32_t tpowerdown;
  uint32_t drvconf;
  uint32_t tcoolthrs;
  uint32_t thigh;
  uint32_t tpwmthrs;
  uint32_t coolconf;
  uint16_t microsteps;
  bool interpolation;
  uint8_t ihold;
  uint8_t irun;
  float rsense;
  uint16_t hold_ma;
  uint16_t run_ma;
} switchdriver;


class SimpleDriver //: public Module
{
    public:
        SimpleDriver();
        ~SimpleDriver();
         void on_module_loaded();
         uint8_t DriverSet(uint8_t drvnum,bool sw1,bool sw2,bool sw3,bool res);	
         bool PrepareUART(uint8_t drvnum, bool sw1, bool sw2, bool receivemode=false);
         uint8_t UARTDriverSet(uint8_t drvnum,bool res);
         uint32_t readuart(uint8_t drvnum);
         uint32_t send2208(uint8_t drvnum, uint8_t reg, uint32_t datagram);
         void softUARTwrite(uint8_t drvnum, uint8_t b);
         uint8_t setbit(uint8_t drvnum,bool bit);

         uint8_t PrepareSPI(uint8_t drvnum);
         uint8_t SPIsetbit(uint8_t drvnum, uint8_t type,bool bit);
         bool SPI_read_MISO(uint8_t drvnum);
         uint8_t SPItransfer(uint8_t drvnum, uint8_t val);
         uint32_t send516x(uint8_t drvnum, uint8_t address, uint32_t datagram=0);

         uint8_t dev70_1;
         uint8_t dev70_2;
         uint8_t dev71_2;
         uint8_t dev72_1;
         uint8_t dev72_2;

         uint8_t dev70io_1;
         uint8_t dev70io_2;
         uint8_t dev71io_2;
         uint8_t dev72io_1;
         uint8_t dev72io_2;

         uint8_t uart_pin[8];

     private:
         bool showdebug;
};

//SimpleDriver simple_drv;

#endif
