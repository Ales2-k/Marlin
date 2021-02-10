/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#define BOARD_INFO_NAME "SELENA CHERRY 1.0"

#if EITHER(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2KB
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2KB
#endif

//
// Limit Switches
//
#define X_STOP_PIN         PC14
#define Y_STOP_PIN         PC13
#define Z_STOP_PIN         PC15   

//
// Z Probe must be this pin
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  PC6
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN   PB7
#endif

//
// Power Loss Detection
//
#ifndef POWER_LOSS_PIN
//  #define POWER_LOSS_PIN   PB4
#endif

//
// Steppers
//
#define X_STEP_PIN         PD13
#define X_DIR_PIN          PD14
#define X_ENABLE_PIN       PD15

#define Y_STEP_PIN         PE8
#define Y_DIR_PIN          PE7
#define Y_ENABLE_PIN       PE10

#define Z_STEP_PIN         PD3
#define Z_DIR_PIN          PD7
#define Z_ENABLE_PIN       PD4

#define E0_STEP_PIN        PC2
#define E0_DIR_PIN         PC1
#define E0_ENABLE_PIN      PC0

#define E1_STEP_PIN        PC5
#define E1_DIR_PIN         PB0
#define E1_ENABLE_PIN      PB1

#define SDA     PB9
#define SCL     PB8

//
// Temperature Sensors
//
#define TEMP_0_PIN         PA1   // T0 <-> E0
#define TEMP_1_PIN         PA2   // T1 <-> E1
#define TEMP_BED_PIN       PA0   // T2 <-> Bed
//#define TEMP_PROBE_PIN     PC3   // Shares J4 connector with PD1

//
// Heaters / Fans
//
#define HEATER_0_PIN       PE14  // Heater0
#define HEATER_BED_PIN     PC8   // Hotbed
#define FAN_PIN            PE11  // Fan1
#define FAN1_PIN           PB4   // Fan0
#define FAN2_PIN           PE9   // Fan1

//
// Misc. Functions
//
#define SDSS               PB12
#define LED_PIN            PE12         //Alive

/**
 * -------------------------------------CHERRY V1.0-----------------------------------------------
 *               _____                                             _____                          |
 *          NC  | · · | GND                                    5V | · · | GND                     |
 *       NRESET | · · | PE6(SD_DET)           MISO  (LCD_D7)  PA6 | · · | PE3   (LCD_D6)          |
 *   (MOSI)PB15 | · · | PE1(BTN_EN2)                (LCD_D5)  PE2 | · · | PA5   (LCD_D4) RST SCK  |
 *  (SD_SS)PB12 | · · | PE0(BTN_EN1)          CS A0 (LCD_RS)  PA4 | · · | PA7   (LCD_EN) CS  MOSI |
 *    (SCK)PB13 | · · | PB14(MISO)                  (BTN_ENC) PE4 | · · | PE5   (BEEPER)          |
 *               ￣￣                                               ￣￣                              |
 *               EXP2                                              EXP1                           |
 * ---------------------------------------------------------------------------------------------
 */
//
// LCDs and Controllers
//
  #define SS_PIN           PB12   // PC11
  #define SCK_PIN          PB13   // PC12 // PC1
  #define MOSI_PIN         PB15   // PD2  // PD2
  #define MISO_PIN         PB14   // PC8
  #define SD_DETECT_PIN    PE6

#if HAS_WIRED_LCD

 #define BEEPER_PIN         PE5
 #define BTN_ENC            PE4

 #define BTN_EN1            PE0
 #define BTN_EN2            PE1

 #define LCD_PINS_RS        PA4
 #define LCD_PINS_ENABLE    PA7
 #define LCD_PINS_D4        PA5
 #define LCD_PINS_D5        PE2
 #define LCD_PINS_D6        PE3
 #define LCD_PINS_D7        PA6


  #if ENABLED(MKS_MINI_12864)
    #define DOGLCD_A0                       LCD_PINS_D6
    #define DOGLCD_CS                       LCD_PINS_D5
  
  #elif ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS                     LCD_PINS_ENABLE
      #define DOGLCD_A0                     LCD_PINS_RS

      #define LCD_RESET_PIN                 LCD_PINS_D4   // Must be high for LCD to operate.
      #if EITHER(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
        #ifndef RGB_LED_R_PIN
          #define RGB_LED_R_PIN             LCD_PINS_D5
        #endif
        #ifndef RGB_LED_G_PIN
          #define RGB_LED_G_PIN             LCD_PINS_D6
        #endif
        #ifndef RGB_LED_B_PIN
          #define RGB_LED_B_PIN             LCD_PINS_D7
        #endif
      #elif ENABLED(FYSETC_MINI_12864_2_1)
        #define NEOPIXEL_PIN                LCD_PINS_D5
      #endif
   #else
     #define DOGLCD_A0          LCD_PINS_D6
     #define DOGLCD_CS          LCD_PINS_D5
   #endif // !FYSETC_MINI_12864

  // Alter timing for graphical display
  #if HAS_MARLINUI_U8GLIB
    #ifndef BOARD_ST7920_DELAY_1
      #define BOARD_ST7920_DELAY_1 DELAY_NS(96)
    #endif
    #ifndef BOARD_ST7920_DELAY_2
      #define BOARD_ST7920_DELAY_2 DELAY_NS(48)
    #endif
    #ifndef BOARD_ST7920_DELAY_3
      #define BOARD_ST7920_DELAY_3 DELAY_NS(600)
    #endif
  #endif

  // Alter timing for graphical display
  #if HAS_GRAPHICAL_LCD
    #define BOARD_ST7920_DELAY_1 DELAY_NS(96)
    #define BOARD_ST7920_DELAY_2 DELAY_NS(48)
    #define BOARD_ST7920_DELAY_3 DELAY_NS(600)
  #endif

#endif // HAS_WIRED_LCD

#define ESP_WIFI_MODULE_COM                    1  // Must also set either SERIAL_PORT or SERIAL_PORT_2 to this
#define ESP_WIFI_MODULE_BAUDRATE        BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT & SERIAL_PORT_2

