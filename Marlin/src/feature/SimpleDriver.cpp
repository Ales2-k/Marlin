
#define ZEROS_DEFAULT_DATA             0x00000000
#define GCONF_DEFAULT_DATA             0x00000101
#define TPOWERDOWN_DEFAULT_DATA        0x00000014
#define CHOPCONF_DEFAULT_DATA          0x10000053
#define PWMCONF_DEFAULT_DATA           0xC10D0024

//driver access request (read MSB bit is 0 and write MSB bit is 1)
#define READ_  0x00
#define WRITE_ 0x80

//sync and address of the target driver
#define SYNC        0x05
#define SLAVEADDR   0x00

//TMC516x reristers
#define DRVCONF_REGISTER              0x0A
#define TCOOLTHRS_REGISTER            0x14
#define THIGH_REGISTER                0x15
#define TSTEP_REGISTER                0x12
#define COOLCONF_REGISTER             0x6D

#define MSTEP_REG_SELECT_bm	0b10000000ul

#include "SimpleDriver_params.h"
#include "SimpleDriver.h"
#include "../inc/MarlinConfig.h"
#include <math.h>
#include <Wire.h>


//calculates CRC checksum and stores in last byte of message
void calc_crc(uint8_t *buf, int cnt)
{
    uint8_t *crc = buf + cnt -1; // CRC located in last byte of message
    uint8_t currentByte;

    *crc = 0;
    for (int i = 0; i < cnt-1; i++) {  // Execute for all bytes of a message
        currentByte = buf[i];          // Retrieve a byte to be sent from Array
        for (int j = 0; j < 8; j++) {
            if ((*crc >> 7) ^ (currentByte & 0x01)) {   // update CRC based result of XOR operation
                *crc = (*crc << 1) ^ 0x07;
            } else {
                *crc = (*crc << 1);
            }
            //crc &= 0xff;
            currentByte = currentByte >> 1;
        }   // for CRC bit
    }       // for message byte
}


SimpleDriver::SimpleDriver()
{
         dev70io_1=255;
         dev70io_2=255;
         dev71io_2=255;
         dev72io_1=255;
         dev72io_2=255;
}

SimpleDriver::~SimpleDriver()
{
}

// Load module
void SimpleDriver::on_module_loaded()
{   Wire.setClock(400000);
    Wire.begin();
    dev70_1=0;dev70_2=0;
    dev71_2=0;
    dev72_1=0;dev72_2=0;

#ifdef SIMPLE_DRIVER_SHOW_DEBUG
 this->showdebug=SIMPLE_DRIVER_SHOW_DEBUG; 
#else 
 this->showdebug=0;
#endif
uint8_t array_size;
#ifdef SIMPLE_DRIVER_ARRAY_SIZE
 array_size=SIMPLE_DRIVER_ARRAY_SIZE; 
#else 
 array_size=5;
#endif
if(this->showdebug) SERIAL_ECHO_MSG(" Simple drivers: ", array_size);

 switchdriver swdrv[5];

/////////////////////////////////////////////// drv start // pechal
#ifdef SIMPLE_DRIVER1_TYPE
 swdrv[0].drvtype=SIMPLE_DRIVER1_TYPE; 
#else 
 swdrv[0].drvtype=0;
#endif
#ifdef SIMPLE_DRIVER1_RESET
 swdrv[0].res=SIMPLE_DRIVER1_RESET; 
#else 
 swdrv[0].res=1;
#endif
#ifdef SIMPLE_DRIVER1_SW1 
 swdrv[0].sw1=SIMPLE_DRIVER1_SW1; 
#else 
 swdrv[0].sw1=0;
#endif
#ifdef SIMPLE_DRIVER1_SW2 
 swdrv[0].sw2=SIMPLE_DRIVER1_SW2; 
#else 
 swdrv[0].sw2=0;
#endif
#ifdef SIMPLE_DRIVER1_SW3 
 swdrv[0].sw3=SIMPLE_DRIVER1_SW3; 
#else 
 swdrv[0].sw3=0;
#endif
#ifdef SIMPLE_DRIVER1_CHIP 
 swdrv[0].chip=SIMPLE_DRIVER1_CHIP; 
#else 
 swdrv[0].chip=0;
#endif
#ifdef SIMPLE_DRIVER1_UART_PIN 
 swdrv[0].uart_pin=SIMPLE_DRIVER1_UART_PIN; 
#else
 swdrv[0].uart_pin=0;
#endif
#ifdef SIMPLE_DRIVER1_GCONF 
 swdrv[0].gconf=SIMPLE_DRIVER1_GCONF; 
#else
 swdrv[0].gconf=0;
#endif
#ifdef SIMPLE_DRIVER1_IHOLD_IRUN 
 swdrv[0].ihold_irun=SIMPLE_DRIVER1_IHOLD_IRUN; 
#else
 swdrv[0].ihold_irun=0;
#endif
#ifdef SIMPLE_DRIVER1_CHOPCONF 
 swdrv[0].chopconf=SIMPLE_DRIVER1_CHOPCONF; 
#else
 swdrv[0].chopconf=0;
#endif
#ifdef SIMPLE_DRIVER1_PWMCONF 
 swdrv[0].pwmconf=SIMPLE_DRIVER1_PWMCONF; 
#else
 swdrv[0].pwmconf=0;
#endif
#ifdef SIMPLE_DRIVER1_TPOWERDOWN 
 swdrv[0].tpowerdown=SIMPLE_DRIVER1_TPOWERDOWN; 
#else
 swdrv[0].tpowerdown=0;
#endif

#ifdef SIMPLE_DRIVER1_DRVCONF 
 swdrv[0].drvconf=SIMPLE_DRIVER1_DRVCONF; 
#else
 swdrv[0].drvconf=0;
#endif
#ifdef SIMPLE_DRIVER1_TCOOLTHRS 
 swdrv[0].tcoolthrs=SIMPLE_DRIVER1_TCOOLTHRS; 
#else
 swdrv[0].tcoolthrs=0;
#endif
#ifdef SIMPLE_DRIVER1_THIGH
 swdrv[0].thigh=SIMPLE_DRIVER1_THIGH; 
#else
 swdrv[0].thigh=0;
#endif
#ifdef SIMPLE_DRIVER1_TPWMTHRS 
 swdrv[0].tpwmthrs=SIMPLE_DRIVER1_TPWMTHRS; 
#else
 swdrv[0].tpwmthrs=0;
#endif
#ifdef SIMPLE_DRIVER1_COOLCONF
 swdrv[0].coolconf=SIMPLE_DRIVER1_COOLCONF; 
#else
 swdrv[0].coolconf=0;
#endif

#ifdef SIMPLE_DRIVER1_MICROSTEPS 
 swdrv[0].microsteps=SIMPLE_DRIVER1_MICROSTEPS; 
#else
 swdrv[0].microsteps=0;
#endif
#ifdef SIMPLE_DRIVER1_INTERPOLATION 
 swdrv[0].interpolation=SIMPLE_DRIVER1_INTERPOLATION; 
#else
 swdrv[0].interpolation=true;
#endif
#ifdef SIMPLE_DRIVER1_IHOLD 
 swdrv[0].ihold=SIMPLE_DRIVER1_IHOLD; 
#else
 swdrv[0].ihold=0;
#endif
#ifdef SIMPLE_DRIVER1_IRUN 
 swdrv[0].irun=SIMPLE_DRIVER1_IRUN; 
#else
 swdrv[0].irun=0;
#endif
#ifdef SIMPLE_DRIVER1_RSENSE 
 swdrv[0].rsense=SIMPLE_DRIVER1_RSENSE; 
#else
 swdrv[0].rsense=0;
#endif
#ifdef SIMPLE_DRIVER1_HOLD_MA 
 swdrv[0].hold_ma=SIMPLE_DRIVER1_HOLD_MA; 
#else
 swdrv[0].hold_ma=0;
#endif
#ifdef SIMPLE_DRIVER1_RUN_MA 
 swdrv[0].run_ma=SIMPLE_DRIVER1_RUN_MA; 
#else
 swdrv[0].run_ma=0;
#endif
/////////////////////////////////////////////// drv2
#ifdef SIMPLE_DRIVER2_TYPE
 swdrv[1].drvtype=SIMPLE_DRIVER2_TYPE; 
#else 
 swdrv[1].drvtype=0;
#endif
#ifdef SIMPLE_DRIVER2_RESET
 swdrv[1].res=SIMPLE_DRIVER2_RESET; 
#else 
 swdrv[1].res=1;
#endif
#ifdef SIMPLE_DRIVER2_SW1 
 swdrv[1].sw1=SIMPLE_DRIVER2_SW1; 
#else 
 swdrv[1].sw1=0;
#endif
#ifdef SIMPLE_DRIVER2_SW2 
 swdrv[1].sw2=SIMPLE_DRIVER2_SW2; 
#else 
 swdrv[1].sw2=0;
#endif
#ifdef SIMPLE_DRIVER2_SW3 
 swdrv[1].sw3=SIMPLE_DRIVER2_SW3; 
#else 
 swdrv[1].sw3=0;
#endif
#ifdef SIMPLE_DRIVER2_CHIP 
 swdrv[1].chip=SIMPLE_DRIVER2_CHIP; 
#else 
 swdrv[1].chip=0;
#endif
#ifdef SIMPLE_DRIVER2_UART_PIN 
 swdrv[1].uart_pin=SIMPLE_DRIVER2_UART_PIN; 
#else
 swdrv[1].uart_pin=0;
#endif
#ifdef SIMPLE_DRIVER2_GCONF 
 swdrv[1].gconf=SIMPLE_DRIVER2_GCONF; 
#else
 swdrv[1].gconf=0;
#endif
#ifdef SIMPLE_DRIVER2_IHOLD_IRUN 
 swdrv[1].ihold_irun=SIMPLE_DRIVER2_IHOLD_IRUN; 
#else
 swdrv[1].ihold_irun=0;
#endif
#ifdef SIMPLE_DRIVER2_CHOPCONF 
 swdrv[1].chopconf=SIMPLE_DRIVER2_CHOPCONF; 
#else
 swdrv[1].chopconf=0;
#endif
#ifdef SIMPLE_DRIVER2_PWMCONF 
 swdrv[1].pwmconf=SIMPLE_DRIVER2_PWMCONF; 
#else
 swdrv[1].pwmconf=0;
#endif
#ifdef SIMPLE_DRIVER2_TPOWERDOWN 
 swdrv[1].tpowerdown=SIMPLE_DRIVER2_TPOWERDOWN; 
#else
 swdrv[1].tpowerdown=0;
#endif
#ifdef SIMPLE_DRIVER2_DRVCONF 
 swdrv[1].drvconf=SIMPLE_DRIVER2_DRVCONF; 
#else
 swdrv[1].drvconf=0;
#endif
#ifdef SIMPLE_DRIVER2_TCOOLTHRS 
 swdrv[1].tcoolthrs=SIMPLE_DRIVER2_TCOOLTHRS; 
#else
 swdrv[1].tcoolthrs=0;
#endif
#ifdef SIMPLE_DRIVER2_THIGH
 swdrv[1].thigh=SIMPLE_DRIVER2_THIGH; 
#else
 swdrv[1].thigh=0;
#endif
#ifdef SIMPLE_DRIVER2_TPWMTHRS 
 swdrv[1].tpwmthrs=SIMPLE_DRIVER2_TPWMTHRS; 
#else
 swdrv[1].tpwmthrs=0;
#endif
#ifdef SIMPLE_DRIVER2_COOLCONF
 swdrv[1].coolconf=SIMPLE_DRIVER2_COOLCONF; 
#else
 swdrv[1].coolconf=0;
#endif
#ifdef SIMPLE_DRIVER2_MICROSTEPS 
 swdrv[1].microsteps=SIMPLE_DRIVER2_MICROSTEPS; 
#else
 swdrv[1].microsteps=0;
#endif
#ifdef SIMPLE_DRIVER2_INTERPOLATION 
 swdrv[1].interpolation=SIMPLE_DRIVER2_INTERPOLATION; 
#else
 swdrv[1].interpolation=true;
#endif
#ifdef SIMPLE_DRIVER2_IHOLD 
 swdrv[1].ihold=SIMPLE_DRIVER2_IHOLD; 
#else
 swdrv[1].ihold=0;
#endif
#ifdef SIMPLE_DRIVER2_IRUN 
 swdrv[1].irun=SIMPLE_DRIVER2_IRUN; 
#else
 swdrv[1].irun=0;
#endif
#ifdef SIMPLE_DRIVER2_RSENSE 
 swdrv[1].rsense=SIMPLE_DRIVER2_RSENSE; 
#else
 swdrv[1].rsense=0;
#endif
#ifdef SIMPLE_DRIVER2_HOLD_MA 
 swdrv[1].hold_ma=SIMPLE_DRIVER2_HOLD_MA; 
#else
 swdrv[1].hold_ma=0;
#endif
#ifdef SIMPLE_DRIVER2_RUN_MA 
 swdrv[1].run_ma=SIMPLE_DRIVER2_RUN_MA; 
#else
 swdrv[1].run_ma=0;
#endif
/////////////////////////////////////////////////////// drv3
#ifdef SIMPLE_DRIVER3_TYPE
 swdrv[2].drvtype=SIMPLE_DRIVER3_TYPE; 
#else 
 swdrv[2].drvtype=0;
#endif
#ifdef SIMPLE_DRIVER3_RESET
 swdrv[2].res=SIMPLE_DRIVER3_RESET; 
#else 
 swdrv[2].res=1;
#endif
#ifdef SIMPLE_DRIVER3_SW1 
 swdrv[2].sw1=SIMPLE_DRIVER3_SW1; 
#else 
 swdrv[2].sw1=0;
#endif
#ifdef SIMPLE_DRIVER3_SW2 
 swdrv[2].sw2=SIMPLE_DRIVER3_SW2; 
#else 
 swdrv[2].sw2=0;
#endif
#ifdef SIMPLE_DRIVER3_SW3 
 swdrv[2].sw3=SIMPLE_DRIVER3_SW3; 
#else 
 swdrv[2].sw3=0;
#endif
#ifdef SIMPLE_DRIVER3_CHIP 
 swdrv[2].chip=SIMPLE_DRIVER3_CHIP; 
#else 
 swdrv[2].chip=0;
#endif
#ifdef SIMPLE_DRIVER3_UART_PIN 
 swdrv[2].uart_pin=SIMPLE_DRIVER3_UART_PIN; 
#else
 swdrv[2].uart_pin=0;
#endif
#ifdef SIMPLE_DRIVER3_GCONF 
 swdrv[2].gconf=SIMPLE_DRIVER3_GCONF; 
#else
 swdrv[2].gconf=0;
#endif
#ifdef SIMPLE_DRIVER3_IHOLD_IRUN 
 swdrv[2].ihold_irun=SIMPLE_DRIVER3_IHOLD_IRUN; 
#else
 swdrv[2].ihold_irun=0;
#endif
#ifdef SIMPLE_DRIVER3_CHOPCONF 
 swdrv[2].chopconf=SIMPLE_DRIVER3_CHOPCONF; 
#else
 swdrv[2].chopconf=0;
#endif
#ifdef SIMPLE_DRIVER3_PWMCONF 
 swdrv[2].pwmconf=SIMPLE_DRIVER3_PWMCONF; 
#else
 swdrv[2].pwmconf=0;
#endif
#ifdef SIMPLE_DRIVER3_TPOWERDOWN 
 swdrv[2].tpowerdown=SIMPLE_DRIVER3_TPOWERDOWN; 
#else
 swdrv[2].tpowerdown=0;
#endif

#ifdef SIMPLE_DRIVER3_DRVCONF 
 swdrv[2].drvconf=SIMPLE_DRIVER3_DRVCONF; 
#else
 swdrv[2].drvconf=0;
#endif
#ifdef SIMPLE_DRIVER3_TCOOLTHRS 
 swdrv[2].tcoolthrs=SIMPLE_DRIVER3_TCOOLTHRS; 
#else
 swdrv[2].tcoolthrs=0;
#endif
#ifdef SIMPLE_DRIVER3_THIGH
 swdrv[2].thigh=SIMPLE_DRIVER3_THIGH; 
#else
 swdrv[2].thigh=0;
#endif
#ifdef SIMPLE_DRIVER3_TPWMTHRS 
 swdrv[2].tpwmthrs=SIMPLE_DRIVER3_TPWMTHRS; 
#else
 swdrv[2].tpwmthrs=0;
#endif
#ifdef SIMPLE_DRIVER3_COOLCONF
 swdrv[2].coolconf=SIMPLE_DRIVER3_COOLCONF; 
#else
 swdrv[2].coolconf=0;
#endif

#ifdef SIMPLE_DRIVER3_MICROSTEPS 
 swdrv[2].microsteps=SIMPLE_DRIVER3_MICROSTEPS; 
#else
 swdrv[2].microsteps=0;
#endif
#ifdef SIMPLE_DRIVER3_INTERPOLATION 
 swdrv[2].interpolation=SIMPLE_DRIVER3_INTERPOLATION; 
#else
 swdrv[2].interpolation=true;
#endif
#ifdef SIMPLE_DRIVER3_IHOLD 
 swdrv[2].ihold=SIMPLE_DRIVER3_IHOLD; 
#else
 swdrv[2].ihold=0;
#endif
#ifdef SIMPLE_DRIVER3_IRUN 
 swdrv[2].irun=SIMPLE_DRIVER3_IRUN; 
#else
 swdrv[2].irun=0;
#endif
#ifdef SIMPLE_DRIVER3_RSENSE 
 swdrv[2].rsense=SIMPLE_DRIVER3_RSENSE; 
#else
 swdrv[2].rsense=0;
#endif
#ifdef SIMPLE_DRIVER3_HOLD_MA 
 swdrv[2].hold_ma=SIMPLE_DRIVER3_HOLD_MA; 
#else
 swdrv[2].hold_ma=0;
#endif
#ifdef SIMPLE_DRIVER3_RUN_MA 
 swdrv[2].run_ma=SIMPLE_DRIVER3_RUN_MA; 
#else
 swdrv[2].run_ma=0;
#endif
/////////////////////////////////////////////// drv4
#ifdef SIMPLE_DRIVER4_TYPE
 swdrv[3].drvtype=SIMPLE_DRIVER4_TYPE; 
#else 
 swdrv[3].drvtype=0;
#endif
#ifdef SIMPLE_DRIVER4_RESET
 swdrv[3].res=SIMPLE_DRIVER4_RESET; 
#else 
 swdrv[3].res=1;
#endif
#ifdef SIMPLE_DRIVER4_SW1 
 swdrv[3].sw1=SIMPLE_DRIVER4_SW1; 
#else 
 swdrv[3].sw1=0;
#endif
#ifdef SIMPLE_DRIVER4_SW2 
 swdrv[3].sw2=SIMPLE_DRIVER4_SW2; 
#else 
 swdrv[3].sw2=0;
#endif
#ifdef SIMPLE_DRIVER4_SW3 
 swdrv[3].sw3=SIMPLE_DRIVER4_SW3; 
#else 
 swdrv[3].sw3=0;
#endif
#ifdef SIMPLE_DRIVER4_CHIP 
 swdrv[3].chip=SIMPLE_DRIVER4_CHIP; 
#else 
 swdrv[3].chip=0;
#endif
#ifdef SIMPLE_DRIVER4_UART_PIN 
 swdrv[3].uart_pin=SIMPLE_DRIVER4_UART_PIN; 
#else
 swdrv[3].uart_pin=0;
#endif
#ifdef SIMPLE_DRIVER4_GCONF 
 swdrv[3].gconf=SIMPLE_DRIVER4_GCONF; 
#else
 swdrv[3].gconf=0;
#endif
#ifdef SIMPLE_DRIVER4_IHOLD_IRUN 
 swdrv[3].ihold_irun=SIMPLE_DRIVER4_IHOLD_IRUN; 
#else
 swdrv[3].ihold_irun=0;
#endif
#ifdef SIMPLE_DRIVER4_CHOPCONF 
 swdrv[3].chopconf=SIMPLE_DRIVER4_CHOPCONF; 
#else
 swdrv[3].chopconf=0;
#endif
#ifdef SIMPLE_DRIVER4_PWMCONF 
 swdrv[3].pwmconf=SIMPLE_DRIVER4_PWMCONF; 
#else
 swdrv[3].pwmconf=0;
#endif
#ifdef SIMPLE_DRIVER4_TPOWERDOWN 
 swdrv[3].tpowerdown=SIMPLE_DRIVER4_TPOWERDOWN; 
#else
 swdrv[3].tpowerdown=0;
#endif

#ifdef SIMPLE_DRIVER4_DRVCONF 
 swdrv[3].drvconf=SIMPLE_DRIVER4_DRVCONF; 
#else
 swdrv[3].drvconf=0;
#endif
#ifdef SIMPLE_DRIVER4_TCOOLTHRS 
 swdrv[3].tcoolthrs=SIMPLE_DRIVER4_TCOOLTHRS; 
#else
 swdrv[3].tcoolthrs=0;
#endif
#ifdef SIMPLE_DRIVER4_THIGH
 swdrv[3].thigh=SIMPLE_DRIVER4_THIGH; 
#else
 swdrv[3].thigh=0;
#endif
#ifdef SIMPLE_DRIVER4_TPWMTHRS 
 swdrv[3].tpwmthrs=SIMPLE_DRIVER4_TPWMTHRS; 
#else
 swdrv[3].tpwmthrs=0;
#endif
#ifdef SIMPLE_DRIVER4_COOLCONF
 swdrv[3].coolconf=SIMPLE_DRIVER4_COOLCONF; 
#else
 swdrv[3].coolconf=0;
#endif

#ifdef SIMPLE_DRIVER4_MICROSTEPS 
 swdrv[3].microsteps=SIMPLE_DRIVER4_MICROSTEPS; 
#else
 swdrv[3].microsteps=0;
#endif
#ifdef SIMPLE_DRIVER4_INTERPOLATION 
 swdrv[3].interpolation=SIMPLE_DRIVER4_INTERPOLATION; 
#else
 swdrv[3].interpolation=true;
#endif
#ifdef SIMPLE_DRIVER4_IHOLD 
 swdrv[3].ihold=SIMPLE_DRIVER4_IHOLD; 
#else
 swdrv[3].ihold=0;
#endif
#ifdef SIMPLE_DRIVER4_IRUN 
 swdrv[3].irun=SIMPLE_DRIVER4_IRUN; 
#else
 swdrv[3].irun=0;
#endif
#ifdef SIMPLE_DRIVER4_RSENSE 
 swdrv[3].rsense=SIMPLE_DRIVER4_RSENSE; 
#else
 swdrv[3].rsense=0;
#endif
#ifdef SIMPLE_DRIVER4_HOLD_MA 
 swdrv[3].hold_ma=SIMPLE_DRIVER4_HOLD_MA; 
#else
 swdrv[3].hold_ma=0;
#endif
#ifdef SIMPLE_DRIVER4_RUN_MA 
 swdrv[3].run_ma=SIMPLE_DRIVER4_RUN_MA; 
#else
 swdrv[3].run_ma=0;
#endif
/////////////////////////////////////////////// drv5
#ifdef SIMPLE_DRIVER5_TYPE
 swdrv[4].drvtype=SIMPLE_DRIVER5_TYPE; 
#else 
 swdrv[4].drvtype=0;
#endif
#ifdef SIMPLE_DRIVER5_RESET
 swdrv[4].res=SIMPLE_DRIVER5_RESET; 
#else 
 swdrv[4].res=1;
#endif
#ifdef SIMPLE_DRIVER5_SW1 
 swdrv[4].sw1=SIMPLE_DRIVER5_SW1; 
#else 
 swdrv[4].sw1=0;
#endif
#ifdef SIMPLE_DRIVER5_SW2 
 swdrv[4].sw2=SIMPLE_DRIVER5_SW2; 
#else 
 swdrv[4].sw2=0;
#endif
#ifdef SIMPLE_DRIVER5_SW3 
 swdrv[4].sw3=SIMPLE_DRIVER5_SW3; 
#else 
 swdrv[4].sw3=0;
#endif
#ifdef SIMPLE_DRIVER5_CHIP 
 swdrv[4].chip=SIMPLE_DRIVER5_CHIP; 
#else 
 swdrv[4].chip=0;
#endif
#ifdef SIMPLE_DRIVER5_UART_PIN 
 swdrv[4].uart_pin=SIMPLE_DRIVER5_UART_PIN; 
#else
 swdrv[4].uart_pin=0;
#endif
#ifdef SIMPLE_DRIVER5_GCONF 
 swdrv[4].gconf=SIMPLE_DRIVER5_GCONF; 
#else
 swdrv[4].gconf=0;
#endif
#ifdef SIMPLE_DRIVER5_IHOLD_IRUN 
 swdrv[4].ihold_irun=SIMPLE_DRIVER5_IHOLD_IRUN; 
#else
 swdrv[4].ihold_irun=0;
#endif
#ifdef SIMPLE_DRIVER5_CHOPCONF 
 swdrv[4].chopconf=SIMPLE_DRIVER5_CHOPCONF; 
#else
 swdrv[4].chopconf=0;
#endif
#ifdef SIMPLE_DRIVER5_PWMCONF 
 swdrv[4].pwmconf=SIMPLE_DRIVER5_PWMCONF; 
#else
 swdrv[4].pwmconf=0;
#endif
#ifdef SIMPLE_DRIVER5_TPOWERDOWN 
 swdrv[4].tpowerdown=SIMPLE_DRIVER5_TPOWERDOWN; 
#else
 swdrv[4].tpowerdown=0;
#endif

#ifdef SIMPLE_DRIVER5_DRVCONF 
 swdrv[4].drvconf=SIMPLE_DRIVER5_DRVCONF; 
#else
 swdrv[4].drvconf=0;
#endif
#ifdef SIMPLE_DRIVER5_TCOOLTHRS 
 swdrv[4].tcoolthrs=SIMPLE_DRIVER5_TCOOLTHRS; 
#else
 swdrv[4].tcoolthrs=0;
#endif
#ifdef SIMPLE_DRIVER5_THIGH
 swdrv[4].thigh=SIMPLE_DRIVER5_THIGH; 
#else
 swdrv[4].thigh=0;
#endif
#ifdef SIMPLE_DRIVER5_TPWMTHRS 
 swdrv[4].tpwmthrs=SIMPLE_DRIVER5_TPWMTHRS; 
#else
 swdrv[4].tpwmthrs=0;
#endif
#ifdef SIMPLE_DRIVER5_COOLCONF
 swdrv[4].coolconf=SIMPLE_DRIVER5_COOLCONF; 
#else
 swdrv[4].coolconf=0;
#endif

#ifdef SIMPLE_DRIVER5_MICROSTEPS 
 swdrv[4].microsteps=SIMPLE_DRIVER5_MICROSTEPS; 
#else
 swdrv[4].microsteps=0;
#endif
#ifdef SIMPLE_DRIVER5_INTERPOLATION 
 swdrv[4].interpolation=SIMPLE_DRIVER5_INTERPOLATION; 
#else
 swdrv[4].interpolation=true;
#endif
#ifdef SIMPLE_DRIVER5_IHOLD 
 swdrv[4].ihold=SIMPLE_DRIVER5_IHOLD; 
#else
 swdrv[4].ihold=0;
#endif
#ifdef SIMPLE_DRIVER5_IRUN 
 swdrv[4].irun=SIMPLE_DRIVER5_IRUN; 
#else
 swdrv[4].irun=0;
#endif
#ifdef SIMPLE_DRIVER5_RSENSE 
 swdrv[4].rsense=SIMPLE_DRIVER5_RSENSE; 
#else
 swdrv[4].rsense=0;
#endif
#ifdef SIMPLE_DRIVER5_HOLD_MA 
 swdrv[4].hold_ma=SIMPLE_DRIVER5_HOLD_MA; 
#else
 swdrv[4].hold_ma=0;
#endif
#ifdef SIMPLE_DRIVER5_RUN_MA 
 swdrv[4].run_ma=SIMPLE_DRIVER5_RUN_MA; 
#else
 swdrv[4].run_ma=0;
#endif
/////////////////////////////////////////////// drv end

    for(uint8_t dr=0;dr<5;dr++)
     {
      uint8_t drvnum=dr+1;
//      uint8_t drvtypei=sd_drvtype[dr];
      SERIAL_ECHO_MSG(" drvtype: ", swdrv[dr].drvtype);
      if (swdrv[dr].drvtype==1) { //"switch"
//                         switchdriver * swdrv=(switchdriver *)&drvsetup[0];
                         if(showdebug){
                          SERIAL_ECHO_MSG(" drvnum: ", drvnum);
                          SERIAL_ECHO_MSG(" sw1: ", swdrv[dr].sw1);
                          SERIAL_ECHO_MSG(" sw2: ", swdrv[dr].sw2);
                          SERIAL_ECHO_MSG(" sw3: ", swdrv[dr].sw3);
                          SERIAL_ECHO_MSG(" reset: ",swdrv[dr].res);
                         }
                         DriverSet(drvnum,swdrv[dr].sw1,swdrv[dr].sw2,swdrv[dr].sw3,swdrv[dr].res);
     }else if(swdrv[dr].drvtype==2) { //"uart"
                if (swdrv[dr].chip==0) swdrv[dr].chip=1; //default 2208
                if(swdrv[dr].uart_pin==0) this->uart_pin[drvnum]=4; else this->uart_pin[drvnum]=swdrv[dr].uart_pin;
                if(uart_pin[drvnum]<=4) uart_pin[drvnum]=2; //4� ���� ��� 2� ���
                if(uart_pin[drvnum]>=5) uart_pin[drvnum]=3; //5� ���� ��� 3� ���
                PrepareUART(drvnum,swdrv[dr].sw1,swdrv[dr].sw2);
                delayMicroseconds(10000);

                if (swdrv[dr].gconf==0) swdrv[dr].gconf=0x000001C8;
                if (swdrv[dr].ihold_irun==0) swdrv[dr].ihold_irun=0x00041810;
                if (swdrv[dr].chopconf==0) swdrv[dr].chopconf=0x10000053;
                if (swdrv[dr].pwmconf==0) swdrv[dr].pwmconf=0xC10D0024;
                if (swdrv[dr].tpowerdown==0) swdrv[dr].tpowerdown=0x00000014;
                if (swdrv[dr].microsteps==0) swdrv[dr].microsteps=256;        
		if (swdrv[dr].microsteps!=256){ uint8_t mres=0;
					switch(swdrv[dr].microsteps) {
						case 256: mres=0; break;
						case 128: mres=1; break;
						case  64: mres=2; break;
						case  32: mres=3; break;
						case  16: mres=4; break;
						case   8: mres=5; break;
						case   4: mres=6; break;
						case   2: mres=7; break;
						case   0: mres=8; break;
						default: break;
						}
				    swdrv[dr].chopconf &= ~(CHOPCONF_MRES);    //delete the old value
				    swdrv[dr].chopconf |= (mres << CHOPCONF_MRES_SHIFT);  //set the new value
				    }


               if(!swdrv[dr].interpolation){         //default 1
               swdrv[dr].chopconf &= ~(CHOPCONF_INTPOL); //write 0
               } //else chopconf |= (CHOPCONF_INTPOL); //write 1

                if (swdrv[dr].ihold==0) swdrv[dr].ihold=16;        
		if (swdrv[dr].ihold!=16){
		    //delete the old value
		    swdrv[dr].ihold_irun &= ~(IHOLD_IRUN_IHOLD);
		    //set the new current scaling
		    swdrv[dr].ihold_irun |= swdrv[dr].ihold << IHOLD_IRUN_IHOLD_SHIFT;
                }
                if (swdrv[dr].irun==0) swdrv[dr].irun=24;        
		if (swdrv[dr].irun!=24){
		    //delete the old value
		    swdrv[dr].ihold_irun &= ~(IHOLD_IRUN_IRUN);
		    //set the new current scaling
		    swdrv[dr].ihold_irun  |= swdrv[dr].irun << IHOLD_IRUN_IRUN_SHIFT;
 	        }

                float irms=0;
                float ipeak=0;
                if (swdrv[dr].chip==1)// "tmc2208"
                 {       
                 if (swdrv[dr].rsense==0) swdrv[dr].rsense=0.11;        
                 irms=0.325*0.7071/(swdrv[dr].rsense+0.02);
                 ipeak=irms*1.42222;
                 }

                if (swdrv[dr].chip==2)// "tmc2209"
                 {       
                 if (swdrv[dr].rsense==0) swdrv[dr].rsense=0.11;        
                 irms=0.325*0.7071/(swdrv[dr].rsense+0.02);
                 ipeak=irms*1.42222;
                 }

                if (swdrv[dr].hold_ma==0) swdrv[dr].hold_ma=65535;        
		if (swdrv[dr].hold_ma!=65535){
                    if(swdrv[dr].hold_ma>irms*1000) swdrv[dr].ihold=31; 
			else { if((round(32.0*swdrv[dr].hold_ma/(1000.0*irms))-1)>0) swdrv[dr].ihold=round(32.0*swdrv[dr].hold_ma/(1000.0*irms))-1; else swdrv[dr].ihold=0;}
		    //delete the old value
		    swdrv[dr].ihold_irun &= ~(IHOLD_IRUN_IHOLD);
		    //set the new current scaling
		    swdrv[dr].ihold_irun |= swdrv[dr].ihold << IHOLD_IRUN_IHOLD_SHIFT;
                }
                if (swdrv[dr].run_ma==0) swdrv[dr].run_ma=65535;        
		if (swdrv[dr].run_ma!=65535){
                    if(swdrv[dr].run_ma>irms*1000) swdrv[dr].irun=31;
			else { if((round(32.0*swdrv[dr].run_ma/(1000.0*irms))-1)>0) swdrv[dr].irun=round(32.0*swdrv[dr].run_ma/(1000.0*irms))-1; else swdrv[dr].irun=0;}
		    //delete the old value
		    swdrv[dr].ihold_irun &= ~(IHOLD_IRUN_IRUN);
		    //set the new current scaling
		    swdrv[dr].ihold_irun  |= swdrv[dr].irun << IHOLD_IRUN_IRUN_SHIFT;
 	        }
             SERIAL_ECHO_MSG(" Driver N: ", drvnum);
             SERIAL_ECHO_MSG(" type: ", swdrv[dr].drvtype);
             SERIAL_ECHO_MSG(" chip: ", swdrv[dr].chip);
             if(this->showdebug){
               SERIAL_ECHO_MSG(" (Debug Irms:) ", irms);
               SERIAL_ECHO_MSG(" (Debug Ipeak:) ", ipeak);
                                 SERIAL_ECHOPGM(" Write GCONF: 0x");SERIAL_PRINTLN(swdrv[dr].gconf, HEX);
                                 SERIAL_ECHOPGM(" Write IHOLD_IRUN: 0x");SERIAL_PRINTLN(swdrv[dr].ihold_irun, HEX);
                                 SERIAL_ECHOPGM(" Write CHOPCONF: 0x");SERIAL_PRINTLN(swdrv[dr].chopconf, HEX);
                                 SERIAL_ECHOPGM(" Write PWMCONF: 0x");SERIAL_PRINTLN(swdrv[dr].pwmconf, HEX);
                                 SERIAL_ECHOPGM(" Write TPOWERDOWN: 0x");SERIAL_PRINTLN(swdrv[dr].tpowerdown, HEX);
                                }
		send2208(drvnum,GCONF_REGISTER|WRITE_,swdrv[dr].gconf);
		send2208(drvnum,IHOLD_IRUN_REGISTER|WRITE_,swdrv[dr].ihold_irun);
		send2208(drvnum,CHOPCONF_REGISTER|WRITE_,swdrv[dr].chopconf);
		send2208(drvnum,PWMCONF_REGISTER|WRITE_,swdrv[dr].pwmconf);
		send2208(drvnum,TPOWERDOWN_REGISTER|WRITE_,swdrv[dr].tpowerdown);
//             if(this->showdebug){ // not working on marlin Wire is BAD
//                SERIAL_ECHO_MSG(" Read: ", swdrv[dr].gconf);
//                send2208(drvnum,GCONF_REGISTER|READ_,swdrv[dr].gconf);
//                send2208(drvnum,CHOPCONF_REGISTER|READ_,swdrv[dr].chopconf);
//                send2208(drvnum,PWMCONF_REGISTER|READ_,swdrv[dr].pwmconf);
//	     }
       
       
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       }else if(swdrv[dr].drvtype==3) { //"spi"
           if (swdrv[dr].chip==0) swdrv[dr].chip=5; //default 2130
           PrepareSPI(drvnum);
           if (swdrv[dr].gconf==0) swdrv[dr].gconf=0x00000004;
           if (swdrv[dr].drvconf==0) swdrv[dr].drvconf=0x00020002;
           if (swdrv[dr].tcoolthrs==0) swdrv[dr].tcoolthrs=0x00000096;
           if (swdrv[dr].thigh==0) swdrv[dr].thigh=0x00000000;
           if (swdrv[dr].tpwmthrs==0) swdrv[dr].tpwmthrs=0x000007D0;
           if (swdrv[dr].ihold_irun==0) swdrv[dr].ihold_irun=0x00060303;
           if (swdrv[dr].chopconf==0) swdrv[dr].chopconf=0x14410153;
           if (swdrv[dr].pwmconf==0) swdrv[dr].pwmconf=0xC70DDA1E;
           if (swdrv[dr].coolconf==0) swdrv[dr].coolconf=0xFFFFFFFF;
           if (swdrv[dr].tpowerdown==0) swdrv[dr].tpowerdown=0x00000080;

           if (swdrv[dr].microsteps==0) swdrv[dr].microsteps=256;        
// SERIAL_ECHO_MSG(" Microsteps: ", swdrv[dr].microsteps);
// SERIAL_ECHOPGM(" CHOPCONF pre : 0x");SERIAL_PRINTLN(swdrv[dr].chopconf, HEX);      
             if (swdrv[dr].microsteps!=256){ uint8_t mres=0;
					switch(swdrv[dr].microsteps) {
						case 256: mres=0; break;
						case 128: mres=1; break;
						case  64: mres=2; break;
						case  32: mres=3; break;
						case  16: mres=4; break;
						case   8: mres=5; break;
						case   4: mres=6; break;
						case   2: mres=7; break;
						case   0: mres=8; break;
						default: break;
						}
				    swdrv[dr].chopconf &= ~(CHOPCONF_MRES);    //delete the old value
				    swdrv[dr].chopconf |= (mres << CHOPCONF_MRES_SHIFT);  //set the new value
				    }
// SERIAL_ECHOPGM(" CHOPCONF aft : 0x");SERIAL_PRINTLN(swdrv[dr].chopconf, HEX);      
               if(!swdrv[dr].interpolation){         //default 1
               swdrv[dr].chopconf &= ~(CHOPCONF_INTPOL); //write 0
               } //else chopconf |= (CHOPCONF_INTPOL); //write 1

                if (swdrv[dr].ihold==0) swdrv[dr].ihold=3;        
		if (swdrv[dr].ihold!=3){
		    //delete the old value
		    swdrv[dr].ihold_irun &= ~(IHOLD_IRUN_IHOLD);
		    //set the new current scaling
		    swdrv[dr].ihold_irun |= swdrv[dr].ihold << IHOLD_IRUN_IHOLD_SHIFT;
                }
                if (swdrv[dr].irun==0) swdrv[dr].irun=3;        
		if (swdrv[dr].irun!=3){
		    //delete the old value
		    swdrv[dr].ihold_irun &= ~(IHOLD_IRUN_IRUN);
		    //set the new current scaling
		    swdrv[dr].ihold_irun  |= swdrv[dr].irun << IHOLD_IRUN_IRUN_SHIFT;
 	        }
                float irms=0;
                float ipeak=0;
                if (swdrv[dr].chip==3) //"tmc5161"
                 {       
                 if (swdrv[dr].rsense==0) swdrv[dr].rsense=0.06;        
		 if (swdrv[dr].rsense>=0.22) {irms=1.1*0.22/swdrv[dr].rsense; ipeak=1.5*0.22/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.15) {irms=1.6*0.15/swdrv[dr].rsense; ipeak=2.2*0.15/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.12) {irms=2*0.12/swdrv[dr].rsense; ipeak=2.8*0.12/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.1) {irms=2.3*0.1/swdrv[dr].rsense; ipeak=3.3*0.1/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.075) {irms=3.1*0.075/swdrv[dr].rsense; ipeak=4.4*0.075/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.066) {irms=3.5*0.066/swdrv[dr].rsense; ipeak=5*0.066/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.06) {irms=3.8*0.06/swdrv[dr].rsense; ipeak=5.4*0.06/swdrv[dr].rsense;}
                 else {irms=3.8; ipeak=5.4;}
                 }

                if (swdrv[dr].chip==4) //"tmc5160"
                 {
                 if (swdrv[dr].rsense==0) swdrv[dr].rsense=0.075;        
                 if (swdrv[dr].rsense>=0.22) {irms=1.1*0.22/swdrv[dr].rsense; ipeak=1.5*0.22/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.15) {irms=1.6*0.15/swdrv[dr].rsense; ipeak=2.2*0.15/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.12) {irms=2*0.12/swdrv[dr].rsense; ipeak=2.8*0.12/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.1) {irms=2.3*0.1/swdrv[dr].rsense; ipeak=3.3*0.1/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.075) {irms=3.1*0.075/swdrv[dr].rsense; ipeak=4.4*0.075/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.066) {irms=3.5*0.066/swdrv[dr].rsense; ipeak=5*0.066/swdrv[dr].rsense;}
		 else if (swdrv[dr].rsense>=0.05) {irms=4.7*0.06/swdrv[dr].rsense; ipeak=6.6*0.06/swdrv[dr].rsense;}
                 else if (swdrv[dr].rsense>=0.033) {irms=7.1*0.06/swdrv[dr].rsense; ipeak=10*0.06/swdrv[dr].rsense;}
                 else if (swdrv[dr].rsense>=0.022) {irms=10.6*0.06/swdrv[dr].rsense; ipeak=15*0.06/swdrv[dr].rsense;}
                 else {irms=10.6; ipeak=15;}
                 }                         
                if (swdrv[dr].chip==5) //"tmc2130"
                 {       
                 if (swdrv[dr].rsense==0) swdrv[dr].rsense=0.11;        
                 irms=0.32*0.7071/(swdrv[dr].rsense+0.02);
                 ipeak=irms*1.42222;
                 }
                 
                if (swdrv[dr].hold_ma==0) swdrv[dr].hold_ma=65535;        
		if (swdrv[dr].hold_ma!=65535){
                    if(swdrv[dr].hold_ma>irms*1000) swdrv[dr].ihold=31; 
			else { if((round(32.0*swdrv[dr].hold_ma/(1000.0*irms))-1)>0) swdrv[dr].ihold=round(32.0*swdrv[dr].hold_ma/(1000.0*irms))-1; else swdrv[dr].ihold=0;}
		    //delete the old value
		    swdrv[dr].ihold_irun &= ~(IHOLD_IRUN_IHOLD);
		    //set the new current scaling
		    swdrv[dr].ihold_irun |= swdrv[dr].ihold << IHOLD_IRUN_IHOLD_SHIFT;
                }
                if (swdrv[dr].run_ma==0) swdrv[dr].run_ma=65535;        
		if (swdrv[dr].run_ma!=65535){
                    if(swdrv[dr].run_ma>irms*1000) swdrv[dr].irun=31;
			else { if((round(32.0*swdrv[dr].run_ma/(1000.0*irms))-1)>0) swdrv[dr].irun=round(32.0*swdrv[dr].run_ma/(1000.0*irms))-1; else swdrv[dr].irun=0;}
		    //delete the old value
		    swdrv[dr].ihold_irun &= ~(IHOLD_IRUN_IRUN);
		    //set the new current scaling
		    swdrv[dr].ihold_irun  |= swdrv[dr].irun << IHOLD_IRUN_IRUN_SHIFT;
 	        }
           send516x(drvnum,GCONF_REGISTER|WRITE_,swdrv[dr].gconf);      // GCONF
           if(swdrv[dr].chip==3||swdrv[dr].chip==4) send516x(drvnum,DRVCONF_REGISTER|WRITE_,swdrv[dr].drvconf);      // DRV_CONF //516x
           send516x(drvnum,TCOOLTHRS_REGISTER|WRITE_,swdrv[dr].tcoolthrs);      // TCOOLTHRS -> TSTEP based threshold
           send516x(drvnum,TPWMTHRS_REGISTER|WRITE_,swdrv[dr].tpwmthrs);      // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
           send516x(drvnum,IHOLD_IRUN_REGISTER|WRITE_,swdrv[dr].ihold_irun);      // IHOLD_IRUN IHOLD=3 IRUN=3 IHOLDDELAY=6               
           send516x(drvnum,CHOPCONF_REGISTER|WRITE_,swdrv[dr].chopconf);     /// CHOPCONF toff=3 hstrt=5 hend=2 tbl=2 tpfd=4 MRES=4(16usteps) intpol=1 0x14410153               
           send516x(drvnum,PWMCONF_REGISTER|WRITE_,swdrv[dr].pwmconf);      // PWMCONF // ��� ������!!!!!               
           send516x(drvnum,TPOWERDOWN_REGISTER|WRITE_,swdrv[dr].tpowerdown);      // TPOWERDOWN                                   
           if (swdrv[dr].thigh>0) send516x(drvnum,THIGH_REGISTER|WRITE_,swdrv[dr].thigh);      // THIGH fullstep switch
           if (swdrv[dr].coolconf!=0xFFFFFFFF) send516x(drvnum,COOLCONF_REGISTER|WRITE_,swdrv[dr].coolconf);
             SERIAL_ECHO_MSG(" Driver N: ", drvnum);
             SERIAL_ECHO_MSG(" type: ", swdrv[dr].drvtype);
             SERIAL_ECHO_MSG(" chip: ", swdrv[dr].chip);
           if(this->showdebug){
               SERIAL_ECHO_MSG(" (Debug Irms): ", irms);
               SERIAL_ECHO_MSG(" (Debug Ipeak): ", ipeak);
                              }
           send516x(drvnum,GCONF_REGISTER);      // GCONF         
           send516x(drvnum,CHOPCONF_REGISTER);     // CHOPCONF         
           send516x(drvnum,DRV_STATUS_REGISTER);      // PWMCONF                    
           }
        }
}

uint8_t SimpleDriver::PrepareSPI(uint8_t drvnum)
{
//extern I2C_HandleTypeDef hi2c1;        
uint8_t addres=0x70;
uint8_t data[2]={1,0}; 
uint8_t dataio[2]={3,0}; //Command byte 1-read 0-write Output Port register 2-write Polarity Inversion register 3-write Configuration register // 255 0b11111111 all ports input
 switch(drvnum) {
     case 1: 
      addres=0x70;   //MOSI SCK CS MISO
      dev70_1=0+(0<<1)+(1<<2)+(0<<3);
      data[1]=dev70_1+(dev70_2<<4);
      dev70io_1=0+(0<<1)+(0<<2)+(1<<3);
      dataio[1]=dev70io_1+(dev70io_2<<4);
     break;
     case 2:
      addres=0x70;
      dev70_2=0+(0<<1)+(1<<2)+(0<<3);
      data[1]=dev70_1+(dev70_2<<4);
      dev70io_2=0+(0<<1)+(0<<2)+(1<<3);
      dataio[1]=dev70io_1+(dev70io_2<<4);
     break;
     case 3:
      addres=0x71;
      dev71_2=0+(0<<1)+(1<<2)+(0<<3);
      data[1]=dev71_2<<4;
      dev71io_2=0+(0<<1)+(0<<2)+(1<<3);
      dataio[1]=15+(dev71io_2<<4);
     break;
     case 4:
      addres=0x72;
      dev72_1=0+(0<<1)+(1<<2)+(0<<3);
      data[1]=dev72_1+(dev72_2<<4);
      dev72io_1=0+(0<<1)+(0<<2)+(1<<3);
      dataio[1]=dev72io_1+(dev72io_2<<4);
     break;
     case 5:
      addres=0x72;
      dev72_2=0+(0<<1)+(1<<2)+(0<<3);
      data[1]=dev72_1+(dev72_2<<4);
      dev72io_2=0+(0<<1)+(0<<2)+(1<<3);
      dataio[1]=dev72io_1+(dev72io_2<<4);
     break;
           }
/*
if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) addres, (uint8_t*) dataio, (uint16_t) 2, (uint32_t)100)!=HAL_OK) return 1; // ��������� ������
		while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READ_Y){}
if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) addres, (uint8_t*) data, (uint16_t) 2, (uint32_t)100)!=HAL_OK) return 1;
		while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READ_Y){}*/
 Wire.beginTransmission((uint8_t) addres); Wire.write(dataio, 2); Wire.endTransmission(); 
 delayMicroseconds(1);
 Wire.beginTransmission((uint8_t) addres); Wire.write(data, 2); Wire.endTransmission();
 delayMicroseconds(1);
return HAL_OK;
}


uint8_t SimpleDriver::setbit(uint8_t drvnum,bool bit)
{ uint8_t data[2]={1,0}; //Command byte 0-read 1-write Output Port register 2-write Polarity Inversion register 3-write Configuration register // 255 0b11111111 all ports input
//  extern I2C_HandleTypeDef hi2c1;      
  uint8_t addres=0x70;
  switch(drvnum) {
     case 1: 
      addres=0x70;
      if(bit) dev70_1|=1<<uart_pin[drvnum]; 
       else dev70_1&=~(1<<uart_pin[drvnum]);
      data[1]=dev70_1+(dev70_2<<4);
     break;
     case 2:
      addres=0x70;
      if(bit) dev70_2|=1<<uart_pin[drvnum]; 
       else dev70_2&=~(1<<uart_pin[drvnum]);
      data[1]=dev70_1+(dev70_2<<4);
     break;
     case 3:
      addres=0x71;
      if(bit) dev71_2|=1<<uart_pin[drvnum]; 
       else dev71_2&=~(1<<uart_pin[drvnum]);
      data[1]=dev71_2<<4;
     break;
     case 4:
      addres=0x72;
      if(bit) dev72_1|=1<<uart_pin[drvnum]; 
       else dev72_1&=~(1<<uart_pin[drvnum]);
      data[1]=dev72_1+(dev72_2<<4);
     break;
     case 5:
      addres=0x72;
      if(bit) dev72_2|=1<<uart_pin[drvnum]; 
       else dev72_2&=~(1<<uart_pin[drvnum]);
      data[1]=dev72_1+(dev72_2<<4);
     break;
           }
//while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READ_Y){}
//return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) addres, (uint8_t*) data, (uint16_t) 2, (uint32_t)100);
 Wire.beginTransmission((uint8_t) addres); Wire.write(data,2); Wire.endTransmission(); 
 delayMicroseconds(1);
return HAL_OK;
}


uint8_t SimpleDriver::SPIsetbit(uint8_t drvnum, uint8_t type,bool bit) //type 0 MOSI 1 SCK 2 CS
{ uint8_t data[2]={1,0}; //Command byte 0-read 1-write Output Port register 2-write Polarity Inversion register 3-write Configuration register // 255 0b11111111 all ports input
  uint8_t addres=0x70;
  switch(drvnum) {
     case 1: 
      addres=0x70;
      if(bit) dev70_1|=1<<type; 
       else dev70_1&=~(1<<type);
      data[1]=dev70_1+(dev70_2<<4);
     break;
     case 2:
      addres=0x70;
      if(bit) dev70_2|=1<<type; 
       else dev70_2&=~(1<<type);
      data[1]=dev70_1+(dev70_2<<4);
     break;
     case 3:
      addres=0x71;
      if(bit) dev71_2|=1<<type; 
       else dev71_2&=~(1<<type);
      data[1]=dev71_2<<4;
     break;
     case 4:
      addres=0x72;
      if(bit) dev72_1|=1<<type; 
       else dev72_1&=~(1<<type);
      data[1]=dev72_1+(dev72_2<<4);
     break;
     case 5:
      addres=0x72;
      if(bit) dev72_2|=1<<type; 
       else dev72_2&=~(1<<type);
      data[1]=dev72_1+(dev72_2<<4);
     break;
           }
//while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READ_Y){}
//return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) addres, (uint8_t*) data, (uint16_t) 2, (uint32_t)100);
 Wire.beginTransmission((uint8_t) addres); Wire.write(data,2); Wire.endTransmission(); 
 delayMicroseconds(1);
return HAL_OK;
}

bool SimpleDriver::SPI_read_MISO(uint8_t drvnum)
{
uint8_t data[2]={0,0}; //Command byte 0-read 1-write Output Port register 2-write Polarity Inversion register 3-write Configuration register // 255 0b11111111 all ports input
//extern I2C_HandleTypeDef hi2c1;     
uint8_t addres=0;
uint8_t bit=0;
uint8_t rdata=0;
 switch(drvnum) {
     case 1: addres=0x70; bit=3; break; 
     case 2: addres=0x70; bit=7; break; 
     case 3: addres=0x71; bit=7; break;
     case 4: addres=0x72; bit=3; break;
     case 5: addres=0x72; bit=7; break;
           }
 Wire.beginTransmission((uint8_t) addres);
 Wire.write(data, 2);
 Wire.endTransmission(); 
 Wire.requestFrom((uint8_t) addres,(uint8_t)1);
 rdata=Wire.read();
return bool(rdata & (1 << bit));
}

uint8_t SimpleDriver::SPItransfer(uint8_t drvnum, uint8_t val) 
{
uint8_t del = 0;
uint8_t out = 0;
uint8_t bval = 0;
        uint8_t v2 = //order == MSBFIRST
            ((val & 0x01) << 7) |
            ((val & 0x02) << 5) |
            ((val & 0x04) << 3) |
            ((val & 0x08) << 1) |
            ((val & 0x10) >> 1) |
            ((val & 0x20) >> 3) |
            ((val & 0x40) >> 5) |
            ((val & 0x80) >> 7);
        val = v2;

//     * CPOL := 0, CPHA := 0 => INIT = 0, PRE = Z|0, MID = 1, POST =  0
//     * CPOL := 1, CPHA := 0 => INIT = 1, PRE = Z|1, MID = 0, POST =  1
//     * CPOL := 0, CPHA := 1 => INIT = 0, PRE =  1 , MID = 0, POST = Z|0
//     * CPOL := 1, CPHA := 1 => INIT = 1, PRE =  0 , MID = 1, POST = Z|1
     
    int sck = 0; // SPI_MODE0: _ckp = 0; _cke = 0;
    for (uint8_t bit = 0u; bit < 8u; bit++)
    {
//        ... Write bit 
        SPIsetbit(drvnum, 0,(val & (1<<bit)));      // 0 MOSI
        delayMicroseconds(del);
        sck ^= 1u; 
        SPIsetbit(drvnum, 1,1);      // 1 SCK=1
//         ... Read bit 
        {  bval = SPI_read_MISO(drvnum);
                out <<= 1; //MSBFIRST
                out |= bval;      }
        delayMicroseconds(del);
        sck ^= 1u;
        SPIsetbit(drvnum, 1,0);      // 0 SCK=0
    }
    return out;
}

uint32_t SimpleDriver::send516x(uint8_t drvnum, uint8_t address, uint32_t datagram)
{
  //TMC5130 takes 40 bit data: 8 address and 32 data
  unsigned long i_datagram = 0;
  uint8_t status=0;
  if(address & WRITE_) {
   SPIsetbit(drvnum, 2,0); //CS 
   delayMicroseconds(10);
   status=SPItransfer(drvnum, address); 
   i_datagram |= SPItransfer(drvnum,(datagram >> 24) & 0xff);
   i_datagram <<= 8;
   i_datagram |= SPItransfer(drvnum,(datagram >> 16) & 0xff);
   i_datagram <<= 8;
   i_datagram |= SPItransfer(drvnum,(datagram >> 8) & 0xff);
   i_datagram <<= 8;
   i_datagram |= SPItransfer(drvnum,(datagram) & 0xff);
  SPIsetbit(drvnum, 2,1); //CS 
//  if(this->showdebug)THEKERNEL->streams->printf("SPI sent: %02X, %02X, %02X, %02X, to %02X, status %02X\r\n", (uint8_t)(datagram >> 24), (uint8_t)(datagram >> 16), (uint8_t)(datagram >> 8), (uint8_t)(datagram >> 0),address,status);
        if(this->showdebug){
      SERIAL_ECHOPGM(" Send : 0x"); SERIAL_PRINTLN(datagram, HEX);      
       }
  }else{
   SPIsetbit(drvnum, 2,0); //CS 
   delayMicroseconds(10);
   status=SPItransfer(drvnum, address); 
   i_datagram |= SPItransfer(drvnum,(datagram >> 24) & 0xff);
   i_datagram <<= 8;
   i_datagram |= SPItransfer(drvnum,(datagram >> 16) & 0xff);
   i_datagram <<= 8;
   i_datagram |= SPItransfer(drvnum,(datagram >> 8) & 0xff);
   i_datagram <<= 8;
   i_datagram |= SPItransfer(drvnum,(datagram) & 0xff);
   delayMicroseconds(10);       
   SPIsetbit(drvnum, 2,1); //CS
   delayMicroseconds(100);
   SPIsetbit(drvnum, 2,0); //CS          
   delayMicroseconds(10);       
   status=SPItransfer(drvnum, address); 
   i_datagram |= SPItransfer(drvnum,(datagram >> 24) & 0xff);
   i_datagram <<= 8;
   i_datagram |= SPItransfer(drvnum,(datagram >> 16) & 0xff);
   i_datagram <<= 8;
   i_datagram |= SPItransfer(drvnum,(datagram >> 8) & 0xff);
   i_datagram <<= 8;
   i_datagram |= SPItransfer(drvnum,(datagram) & 0xff);
   delayMicroseconds(10);
  SPIsetbit(drvnum, 2,1); //CS 
//  if(this->showdebug)THEKERNEL->streams->printf("SPI receive: %02X, %02X, %02X, %02X, from %02X, status %02X\r\n", (uint8_t)(i_datagram >> 24), (uint8_t)(i_datagram >> 16), (uint8_t)(i_datagram >> 8), (uint8_t)(i_datagram >> 0),address,status);
        if(this->showdebug){
      SERIAL_ECHOPGM(" Receive : 0x"); SERIAL_PRINTLN(i_datagram, HEX);
      SERIAL_ECHOPGM(" From: 0x"); SERIAL_PRINTLN(address, HEX);
      SERIAL_ECHOPGM(" Status: 0x"); SERIAL_PRINTLN(status, HEX);
      SERIAL_ECHO_MSG(" ");
        }
  }
  delayMicroseconds(100);
  return i_datagram;
}

void SimpleDriver::softUARTwrite(uint8_t drvnum, uint8_t b)
{ //__disable_irq();	
  // Write the start bit 0
  setbit(drvnum,0);
  //9600 kbps microsec = 1000000*1/9600 =104
  delayMicroseconds(35);  
  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1) setbit(drvnum,1);// choose bit
    else setbit(drvnum,0);
    delayMicroseconds(35);  
    b >>= 1;
  }
  // Write the stop bit 1
  setbit(drvnum,1);
 //__enable_irq();	
  delayMicroseconds(35);  
}


uint32_t SimpleDriver::send2208(uint8_t drvnum, uint8_t reg, uint32_t datagram)
{
    uint32_t i_datagram = 0;
    if(reg & WRITE_) {
        uint8_t buf[] {(uint8_t)(SYNC), (uint8_t)(SLAVEADDR), (uint8_t)(reg), (uint8_t)(datagram >> 24), (uint8_t)(datagram >> 16), (uint8_t)(datagram >> 8), (uint8_t)(datagram >> 0), (uint8_t)(0x00)};
        //calculate checksum
        calc_crc(buf, 8);
        //write/read the values
        for(uint8_t i=0; i<8 ; i++)
        {
        softUARTwrite(drvnum,buf[i]);
        }
        if(this->showdebug){  
             SERIAL_ECHOPGM(" Send : 0x"); 
             SERIAL_PRINT(buf[0], HEX);      
             SERIAL_PRINT(buf[1], HEX);      
             SERIAL_PRINT(buf[2], HEX);      
             SERIAL_PRINT(buf[3], HEX);      
             SERIAL_PRINT(buf[4], HEX);      
             SERIAL_PRINT(buf[5], HEX);      
             SERIAL_PRINT(buf[6], HEX);      
             SERIAL_PRINTLN(buf[7], HEX);      
        }
//      THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X \r\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    } else {
        uint8_t buf[] {(uint8_t)(SYNC), (uint8_t)(SLAVEADDR), (uint8_t)(reg), (uint8_t)(0x00)};
        //calculate checksum
        calc_crc(buf, 4);
        //write/read the values
        for(uint8_t i=0; i<4; i++)
        {
        softUARTwrite(drvnum,buf[i]);
        }
        PrepareUART(drvnum,0,0,1);
        i_datagram = readuart(drvnum);
        PrepareUART(drvnum,0,0,0);
        //reply
        if(this->showdebug){
         SERIAL_ECHOPGM(" Send : 0x"); 
         SERIAL_PRINT(buf[0], HEX);      
         SERIAL_PRINT(buf[1], HEX);      
         SERIAL_PRINT(buf[2], HEX);      
         SERIAL_PRINT(buf[3], HEX);      
         SERIAL_ECHOPGM(" Receive : 0x"); SERIAL_PRINTLN(i_datagram, HEX);
//     THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X received: %02X, %02X, %02X, %02X\r\n", buf[0], buf[1], buf[2], buf[3], (uint8_t)(i_datagram >> 24), (uint8_t)(i_datagram >> 16), (uint8_t)(i_datagram >> 8), (uint8_t)(i_datagram >> 0));
       }
    }
    return i_datagram;
}

bool getbit(uint8_t nb, uint16_t k, uint8_t * dt, bool * d)
{
 uint16_t nbt=0;       
 uint8_t sdv=0;       
 do{         
  if (sdv+*(dt+nbt)>=nb) return bool(*(d+nbt));
  else {sdv=sdv+*(dt+nbt);nbt++;}       
 }while(nbt<k);        
return NULL;
}


uint32_t SimpleDriver::readuart(uint8_t drvnum)
{
uint8_t datas[2]={0,0};
bool data[40]={0,0}; //Command byte 0-read 1-write Output Port register 2-write Polarity Inversion register 3-write Configuration register // 255 0b11111111 all ports input
uint8_t datat[40];
uint32_t datagramm=0;
bool prevdata=1;
uint8_t rdata;
//extern I2C_HandleTypeDef hi2c1;     

uint8_t addres=0;
uint8_t bit=0;
 switch(drvnum) {
     case 1: addres=0x70; bit=uart_pin[drvnum]; break; //2
     case 2: addres=0x70; bit=4+uart_pin[drvnum]; break; //6
     case 3: addres=0x71; bit=4+uart_pin[drvnum]; break;
     case 4: addres=0x72; bit=uart_pin[drvnum]; break;
     case 5: addres=0x72; bit=4+uart_pin[drvnum]; break;
           }

uint8_t k=0;
//while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READ_Y){}
//HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) addres, (uint8_t*) data, (uint16_t) 2, (uint32_t)100);

uint32_t starttime=millis();
for(uint16_t i=0;i<300;i++)
 {
// while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READ_Y){}
// HAL_I2C_Master_Receive(&hi2c1, (uint16_t) addres, (uint8_t*) &rdata, (uint16_t) 1, (uint32_t)100);        
 Wire.beginTransmission((uint8_t) addres);
 Wire.write(datas, 2);
 Wire.endTransmission(); 
 Wire.requestFrom((uint8_t) addres,(uint8_t)1);
 rdata=Wire.read();
 delayMicroseconds(1);

 if((bool(rdata & (1 << bit)))!=prevdata) { 
//        datat[k]=floor((TIM5->CNT-starttime)/100.0); //100us 9600 //offset by i2c timeout
        datat[k]=floor((millis()-starttime)/100.0); //100us 9600 //offset by i2c timeout
        if (datat[k]<1) datat[k]=1;
//         starttime=TIM5->CNT;         
         starttime=millis();
        data[k]=!bool(rdata & (1 << bit));
        prevdata=bool(rdata & (1 << bit));
        k++; 
        }         
  }
 // analize  // find 01010000  //revers=10 
 //for(uint8_t i=0;i<k;i++) THEKERNEL->streams->printf("receive:%u %02X\r\n", datat[i], data[i]);
 
 uint16_t kb=0;
 static uint8_t pattern=0;
 uint8_t startb=0;
 do{
 for(uint8_t i=0;i<8;i++) pattern|=(getbit(i+kb,k,datat,data))<<i;//pattern=(bool(data[i+kb]))<<i;
 if (pattern!=10) {
                        kb++;
                        pattern=0; 
                       }
 else {startb=kb+30; break;}        
 }while(kb<k-8);
 if (startb!=0) {
 uint8_t sdvig=0;        
 for(uint8_t l=0;l<4;l++){
  for(uint8_t i=0;i<10;i++) {
   //THEKERNEL->streams->printf("bit:%u %u\r\n", l*10+i+startb, bool(getbit(l*10+i+startb,k,datat,data)));  
   if(i==0&&getbit(l*10+i+startb,k,datat,data)){startb=0;return 0;}
   if(i==9&&(!getbit(l*10+i+startb,k,datat,data))){startb=0;return 0;}
     if(i!=0&&i!=9) 
                {
                 datagramm|=(bool(getbit(l*10+i+startb,k,datat,data)))<<((3-l)*8+sdvig); 
                 sdvig++; 
                 if(sdvig==8) sdvig=0;       
                }          
   }
  }
 }
//THEKERNEL->streams->printf("datagram: %08X\r\n", datagramm); 
return datagramm; 
}

bool SimpleDriver::PrepareUART(uint8_t drvnum, bool sw1, bool sw2, bool receivemode)
{
//extern I2C_HandleTypeDef hi2c1;        
uint8_t addres=0;
uint8_t data[2]={1,0}; 
uint8_t dataio[2]={3,0}; //Command byte 1-read 0-write Output Port register 2-write Polarity Inversion register 3-write Configuration register // 255 0b11111111 all ports input
 switch(drvnum) {
     case 1: 
      addres=0x70;
      dev70_1=sw1+(sw2<<1)+(1<<2)+(1<<3);
//      dev70_1 &= ~(1 << uart_pin[drvnum]);
      data[1]=dev70_1+(dev70_2<<4);
//      dev70io_1=0+(0<<1)+(receivemode<<2)+(1<<3);
      dev70io_1=0+(0<<1)+(1<<2)+(1<<3);
      if (receivemode==false) dev70io_1 &= ~(1 << uart_pin[drvnum]); //0
         else dev70io_1 |= (1 << uart_pin[drvnum]); //1
      dataio[1]=dev70io_1+(dev70io_2<<4);
     break;
     case 2:
      addres=0x70;
      dev70_2=sw1+(sw2<<1)+(1<<2)+(1<<3);
      data[1]=dev70_1+(dev70_2<<4);
//      dev70io_2=0+(0<<1)+(receivemode<<2)+(1<<3);
      dev70io_2=0+(0<<1)+(1<<2)+(1<<3);
      if (receivemode==false) dev70io_2 &= ~(1 << uart_pin[drvnum]); //0
         else dev70io_2 |= (1 << uart_pin[drvnum]); //1
      dataio[1]=dev70io_1+(dev70io_2<<4);
     break;
     case 3:
      addres=0x71;
      dev71_2=sw1+(sw2<<1)+(1<<2)+(1<<3);
      data[1]=dev71_2<<4;
//      dev71io_2=0+(0<<1)+(receivemode<<2)+(1<<3);
      dev71io_2=0+(0<<1)+(1<<2)+(1<<3);
      if (receivemode==false) dev71io_2 &= ~(1 << uart_pin[drvnum]); //0
         else dev71io_2 |= (1 << uart_pin[drvnum]); //1
      dataio[1]=15+(dev71io_2<<4);
     break;
     case 4:
      addres=0x72;
      dev72_1=sw1+(sw2<<1)+(1<<2)+(1<<3);
      data[1]=dev72_1+(dev72_2<<4);
//      dev72io_1=0+(0<<1)+(receivemode<<2)+(1<<3);
      dev72io_1=0+(0<<1)+(1<<2)+(1<<3);
      if (receivemode==false) dev72io_1 &= ~(1 << uart_pin[drvnum]); //0
         else dev72io_1 |= (1 << uart_pin[drvnum]); //1
      dataio[1]=dev72io_1+(dev72io_2<<4);
     break;
     case 5:
      addres=0x72;
      dev72_2=sw1+(sw2<<1)+(1<<2)+(1<<3);
      data[1]=dev72_1+(dev72_2<<4);
//      dev72io_2=0+(0<<1)+(receivemode<<2)+(1<<3);
      dev72io_2=0+(0<<1)+(1<<2)+(1<<3);
      if (receivemode==false) dev72io_2 &= ~(1 << uart_pin[drvnum]); //0
         else dev72io_2 |= (1 << uart_pin[drvnum]); //1
      dataio[1]=dev72io_1+(dev72io_2<<4);
     break;
           }
/*
if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) addres, (uint8_t*) dataio, (uint16_t) 2, (uint32_t)100)!=HAL_OK) return 1; // ��������� ������
		while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READ_Y){}
if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) addres, (uint8_t*) data, (uint16_t) 2, (uint32_t)100)!=HAL_OK) return 1;
		while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READ_Y){}
*/
 Wire.beginTransmission((uint8_t) addres); Wire.write(dataio, 2); Wire.endTransmission(); 
 delayMicroseconds(1);
 Wire.beginTransmission((uint8_t) addres); Wire.write(data, 2); Wire.endTransmission();
 delayMicroseconds(1);

return HAL_OK;
}


uint8_t SimpleDriver::DriverSet(uint8_t drvnum,bool sw1,bool sw2,bool sw3,bool res)
{
//extern I2C_HandleTypeDef hi2c1;        
uint8_t addres=0;
uint8_t data[2]={1,0}; //Command byte 0-read 1-write Output Port register 2-write Polarity Inversion register 3-write Configuration register // 255 0b11111111 all ports output
uint8_t dataio[2]={3,0}; //Command byte 0-read 1-write Output Port register 2-write Polarity Inversion register 3-write Configuration register // 255 0b11111111 all ports output
 switch(drvnum) {
     case 1: 
//      addres=0x70<<1;
      addres=0x70;
      dev70_1=sw1+(sw2<<1)+(sw3<<2)+(res<<3);
      data[1]=dev70_1+(dev70_2<<4);
      dev70io_1=0;
      dataio[1]=dev70io_1+(dev70io_2<<4);
     break;
     case 2:
      addres=0x70;
      dev70_2=sw1+(sw2<<1)+(sw3<<2)+(res<<3);
      data[1]=dev70_1+(dev70_2<<4);
      dev70io_2=0;
      dataio[1]=dev70io_1+(dev70io_2<<4);
     break;
     case 3:
      addres=0x71;
      dev71_2=sw1+(sw2<<1)+(sw3<<2)+(res<<3);
      data[1]=dev71_2<<4;
      dev71io_2=0;
      dataio[1]=15+(dev71io_2<<4);
     break;
     case 4:
      addres=0x72;
      dev72_1=sw1+(sw2<<1)+(sw3<<2)+(res<<3);
      data[1]=dev72_1+(dev72_2<<4);
      dev72io_1=0;
      dataio[1]=dev72io_1+(dev72io_2<<4);
     break;
     case 5:
      addres=0x72;
      dev72_2=sw1+(sw2<<1)+(sw3<<2)+(res<<3);
      data[1]=dev72_1+(dev72_2<<4);
      dev72io_2=0;
      dataio[1]=dev72io_1+(dev72io_2<<4);
     break;
           }

 Wire.beginTransmission((uint8_t) addres);
 Wire.write(dataio, 2);
 Wire.endTransmission();
 delayMicroseconds(1);
 Wire.beginTransmission((uint8_t) addres);
 Wire.write(data, 2);
 Wire.endTransmission();
 delayMicroseconds(1);
/*
if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) addres, (uint8_t*) dataio, (uint16_t) 2, (uint32_t)100)!=HAL_OK) return 1; // ��������� ������
		while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READ_Y){}
if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) addres, (uint8_t*) data, (uint16_t) 2, (uint32_t)100)!=HAL_OK) return 1;
		while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READ_Y){}
*/
return HAL_OK;
}
