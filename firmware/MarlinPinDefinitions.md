### ini/stm32f4.ini


>   MKS Robin Nano V3 <br/>

[env:mks_robin_nano_v3]<br/>
extends                     = stm32_variant<br/>
board                       = marlin_STM32F407VGT6_CCM<br/>
board_build.variant         = MARLIN_F4x7Vx // ALL NANO SETUPS EXTEND ON THIS \buildroot\share\PlatformIO\variants\MARLIN_F4x7Vx<br/>
board_build.offset          = 0xC000<br/>
board_upload.offset_address = 0x0800C000<br/>
board_build.rename          = Robin_nano_v3.bin<br/>
build_flags                 = $ \{stm32_variant.build_flags\} $\{stm32f4_I2C1.build_flags\} <br/>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                               -DHAL_PCD_MODULE_ENABLED<br/>
debug_tool                  = jlink<br/>
upload_protocol             = jlink<br/>
  
  > MKS Robin Nano V3 with USB Flash Drive Support<br/>
  
  [env:mks_robin_nano_v3_usb_flash_drive]<br/>
  extends           = env:mks_robin_nano_v3<br/>
  platform_packages = $\{stm_flash_drive.platform_packages}<br/>
  build_flags       = $\{stm_flash_drive.build_flags} $\{stm32f4_I2C1.build_flags}<br/>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                     -DUSE_USBHOST_HS<br/>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                      -DUSBD_IRQ_PRIO=5<br/>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                      -DUSBD_IRQ_SUBPRIO=6<br/>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                      -DUSE_USB_HS_IN_FS<br/>
  

> MKS Robin Nano V3 with USB Flash Drive Support and Shared Media
 
[env:mks_robin_nano_v3_usb_flash_drive_msc]<br/>
extends           = env:mks_robin_nano_v3_usb_flash_drive<br/>
build_flags       = $\{env:mks_robin_nano_v3_usb_flash_drive.build_flags}<br/>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                      -DUSBD_USE_CDC_MSC<br/>
build_unflags     = $\{env:mks_robin_nano_v3_usb_flash_drive.build_unflags}<br/>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;                      -DUSBD_USE_CDC<br/>

> MKS Robin Nano V3_1

[env:mks_robin_nano_v3_1]<br/>
extends           = env:mks_robin_nano_v3<br/>
board             = marlin_STM32F407VET6_CCM<br/>
  
 > MKS Robin Nano V3.1 with USB Flash Drive Support
 
[env:mks_robin_nano_v3_1_usb_flash_drive]<br/>
extends           = env:mks_robin_nano_v3_usb_flash_drive<br/>
board             = marlin_STM32F407VET6_CCM

> MKS Robin Nano V3.1 with USB Flash Drive Support and Shared Media

[env:mks_robin_nano_v3_1_usb_flash_drive_msc]<br/>
extends           = env:mks_robin_nano_v3_usb_flash_drive_msc<br/>
board             = marlin_STM32F407VET6_CCM<br/>

### buildroot\share\PlatformIO\variants\MARLIN_F4x7Vx\PeripheralPins.c

    // *** USB ***
    #ifdef HAL_PCD_MODULE_ENABLED
       WEAK const PinMap PinMap_USB_OTG_FS[] = {
         {PA_11, USB_OTG_FS, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_OTG_FS)}, // USB_OTG_FS_DM linked to USB
         {PA_12, USB_OTG_FS, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF10_OTG_FS)}, // USB_OTG_FS_DP linked to USB
         {NC,    NP,    0}
    };
    
       WEAK const PinMap PinMap_USB_OTG_HS[] = {
          #ifdef USE_USB_HS_IN_FS
             {PB_14, USB_OTG_HS, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_OTG_HS_FS)}, // USB_OTG_HS_DM linked to USB DISK
             {PB_15, USB_OTG_HS, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_OTG_HS_FS)}, // USB_OTG_HS_DP linked to USB DISK
          #endif /* USE_USB_HS_IN_FS */
          {NC,    NP,    0}
    };
    #endif


### Marlin/src/HAL/STM32/HardwareSerial.cpp
	// USART/UART PIN MAPPING FOR STM32F0/F1/F2/F4/F7
	// USART1
	#ifndef PIN_SERIAL1_TX
	 #define PIN_SERIAL1_TX  PA9 // USART1 Connected to TX1 in WiFi module
	#endif
	#ifndef PIN_SERIAL1_RX
	 #define PIN_SERIAL1_RX  PA10 // USART1 Connected to RX1 in WiFi module
	#endif
	// USART2 not used/overridden
	#ifndef PIN_SERIAL2_TX
	  #define PIN_SERIAL2_TX  PA2 // overriden in pins_MKS_ROBIN_NANO_V3_common.h as PIN FOR HOTEND 2 SENSOR
	#endif
	#ifndef PIN_SERIAL2_RX
	 #define PIN_SERIAL2_RX  PA3 // overriden in pins_MKS_ROBIN_NANO_V3_common.h as PIN FOR E1 ENABLE
	#endif
	// USART 3
	// exposed in J2
	// if MKS_WIFI_MODULE is not ENABLED then USART3 is assigned to WIFI_SERIAL_PORT
	#ifndef PIN_SERIAL3_TX
	 #define PIN_SERIAL3_TX  PB10
	#endif
	#ifndef PIN_SERIAL3_RX
	  #define PIN_SERIAL3_RX  PB11
	#endif
	// USART4 not used/overridden
	#ifndef PIN_SERIAL4_TX
	  #define PIN_SERIAL4_TX  PC10 // overriden in pins_MKS_ROBIN_NANO_V3_common.h as SD_SCK_PIN IF SD_CONNECTION_IS(ONBOARD)
	#endif
	#ifndef PIN_SERIAL4_RX
	  #define PIN_SERIAL4_RX  PC11 // overriden in pins_MKS_ROBIN_NANO_V3_common.h as SD_MISO_PIN IF SD_CONNECTION_IS(ONBOARD)
	#endif
	// USART5 not used/overridden
	#ifndef PIN_SERIAL5_TX
	  #define PIN_SERIAL5_TX  PC12 // overriden in pins_MKS_ROBIN_NANO_V3_common.h as SD_MOSI_PIN IF SD_CONNECTION_IS(ONBOARD)
	#endif
	#ifndef PIN_SERIAL5_RX
	  #define PIN_SERIAL5_RX  PD2 // overriden in pins_MKS_ROBIN_NANO_V3_common.h as Y_STOP_PIN
	#endif
	// USART6
	#ifndef PIN_SERIAL6_TX
	  #define PIN_SERIAL6_TX  PC6 // overriden in pins_MKS_ROBIN_NANO_V3_common.h LCD_RS
	#endif
	#ifndef PIN_SERIAL6_RX
	  #define PIN_SERIAL6_RX  PC7 // overriden in pins_MKS_ROBIN_NANO_V3_common.h WIFI_IO1_PIN
	#endif


### Marlin/src/pins/stm32f4/pins_MKS_ROBIN_NANO_V3_common.h


    // I2C pins for AT24C32D I2C-Compatible (2-Wire) Serial EEPROM 32-Kbit (4096 x 8)
    #if ANY(NO_EEPROM_SELECTED, I2C_EEPROM)
      #define I2C_EEPROM
      #define MARLIN_EEPROM_SIZE              0x1000  // 4K
      #define I2C_SCL_PIN                       PB6
      #define I2C_SDA_PIN                       PB7 
    #endif


  SERVO0_PIN PA8 // ENABLE FOR BLTOUCH SERVO

  X_DIAG_PIN  PA15 // ALSO ASSIGNED TO X_STOP_PIN
  Y_DIAG_PIN  PD2  // ALSO ASSIGNED TO Y_STOP_PIN
  Z_DIAG_PIN  PC8  // ALSO ASSIGNED TO Z_MIN_PIN
  E0_DIAG_PIN PC4 // ALSO ASSIGNED TO Z_MAX_PIN
  E1_DIAG_PIN PE7

  X_ENABLE_PIN   PE4  //            /--> ENABLE PIN
  X_STEP_PIN     PE3  // X STEPPER  |--> STEP PIN
  X_DIR_PIN      PE2  //            \--> DIRECTION PIN
  Y_ENABLE_PIN   PE1  //            /--> ENABLE PIN
  Y_STEP_PIN     PE0  // Y STEPPER  |--> STEP PIN
  Y_DIR_PIN      PB9  //            \--> DIRECTION PIN
  Z_ENABLE_PIN   PB8  //            /--> ENABLE PIN
  Z_STEP_PIN     PB5  // Z STEPPER  |--> STEP PIN
  Z_DIR_PIN      PB4  //            \--> DIRECTION PIN
  E0_ENABLE_PIN  PB3  //            /--> ENABLE PIN
  E0_STEP_PIN    PD6  // E0 STEPPER |--> STEP PIN
  E0_DIR_PIN     PD3  //            \--> DIRECTION PIN
  E1_ENABLE_PIN  PA3  //            /--> ENABLE PIN
  E1_STEP_PIN    PD15 // E1 STEPPER |--> STEP PIN
  E1_DIR_PIN     PA1  //            \--> DIRECTION PIN

  X_SERIAL_TX_PIN  PD5 // TX = RX PIN FOR X STEPPER IF HAS_TMC_UART  ** FOR MARLIN 2.1.3 UART FOR 2208 AND 2209 --> Marlin/src/core/drivers.h AXIS_HAS_UART(A) ( AXIS_DRIVER_TYPE(A,TMC2208) || AXIS_DRIVER_TYPE(A,TMC2209) )
  Y_SERIAL_TX_PIN  PD7 // TX = RX PIN FOR Y STEPPER IF HAS_TMC_UART  ** FOR MARLIN 2.1.3 UART FOR 2208 AND 2209 --> Marlin/src/core/drivers.h AXIS_HAS_UART(A) ( AXIS_DRIVER_TYPE(A,TMC2208) || AXIS_DRIVER_TYPE(A,TMC2209) )
  Z_SERIAL_TX_PIN  PD4 // TX = RX PIN FOR Z STEPPER IF HAS_TMC_UART  ** FOR MARLIN 2.1.3 UART FOR 2208 AND 2209 --> Marlin/src/core/drivers.h AXIS_HAS_UART(A) ( AXIS_DRIVER_TYPE(A,TMC2208) || AXIS_DRIVER_TYPE(A,TMC2209) )
  E0_SERIAL_TX_PIN PD9 // TX = RX PIN FOR E0 STEPPER IF HAS_TMC_UART ** FOR MARLIN 2.1.3 UART FOR 2208 AND 2209 --> Marlin/src/core/drivers.h AXIS_HAS_UART(A) ( AXIS_DRIVER_TYPE(A,TMC2208) || AXIS_DRIVER_TYPE(A,TMC2209) )
  E1_SERIAL_TX_PIN PD8 // TX = RX PIN FOR E1 STEPPER IF HAS_TMC_UART ** FOR MARLIN 2.1.3 UART FOR 2208 AND 2209 --> Marlin/src/core/drivers.h AXIS_HAS_UART(A) ( AXIS_DRIVER_TYPE(A,TMC2208) || AXIS_DRIVER_TYPE(A,TMC2209) )

  TEMP_0_PIN   PC1   // PIN FOR HOTEND 1 SENSOR 
  TEMP_1_PIN   PA2   // PIN FOR HOTEND 2 SENSOR; CAN BE ASSIGNED TO TEMP_PROBE_PIN OR TEMP_CHAMBER_PIN IF HOTENDS == 1 && !REDUNDANT_TEMP_MATCH(SOURCE, E1)
  TEMP_BED_PIN PC0   // TB1

  HEATER_0_PIN   PE5   // HEATER1
  HEATER_1_PIN   PB0   // HEATER2
  HEATER_BED_PIN PA0   // HOT BED

  FAN0_PIN PC14  // FAN
  FAN1_PIN PB1   // FAN1

  FIL_RUNOUT_PIN  PA4   // MT_DET_1 IF HAS_TFT_LVGL_UI
  FIL_RUNOUT2_PIN PE6   // MT_DET_2 IF HAS_TFT_LVGL_UI

  POWER_LOSS_PIN PA13  // PW_DET  

  WIFI_IO0_PIN   PC13 // IF ENABLED(MKS_WIFI_MODULE) ASSIGN MKS_WIFI_MODULE_SERIAL 1  // USART1 AND MKS_WIFI_MODULE_SPI 2  // SPI2 ELSE WIFI_SERIAL_PORT 3  // USART3
  WIFI_IO1_PIN   PC7  // IF ENABLED(MKS_WIFI_MODULE) ASSIGN MKS_WIFI_MODULE_SERIAL 1  // USART1 AND MKS_WIFI_MODULE_SPI 2  // SPI2 ELSE WIFI_SERIAL_PORT 3  // USART3
  WIFI_RESET_PIN PE9  // IF ENABLED(MKS_WIFI_MODULE) ASSIGN MKS_WIFI_MODULE_SERIAL 1  // USART1 AND MKS_WIFI_MODULE_SPI 2  // SPI2 ELSE WIFI_SERIAL_PORT 3  // USART3

  SDSS          PC9  // SPI3 CS FOR SD IF SD_CONNECTION_IS(ONBOARD)
  SD_SCK_PIN    PC10 // SPI3 SCK FOR SD IF SD_CONNECTION_IS(ONBOARD)
  SD_MISO_PIN   PC11 // SPI3 MISO FOR SD IF SD_CONNECTION_IS(ONBOARD)
  SD_MOSI_PIN   PC12 // SPI3 MOSI FOR SD IF SD_CONNECTION_IS(ONBOARD)
  SD_DETECT_PIN PD12 // DETECT PIN FOR SD IF SD_CONNECTION_IS(ONBOARD)

  SPI_FLASH_CS_PIN   PB12 // SPI2 CS IF ENABLED(SPI_FLASH) Winbond NOR 64Mbit Quad-SPI Flash Memory 8-Pin SOIC, W25Q64JVSSIQ
  SPI_FLASH_SCK_PIN  PB13 // SPI2 SCK IF ENABLED(SPI_FLASH) Winbond NOR 64Mbit Quad-SPI Flash Memory 8-Pin SOIC, W25Q64JVSSIQ
  SPI_FLASH_MISO_PIN PC2  // SPI2 MISO IF ENABLED(SPI_FLASH) Winbond NOR 64Mbit Quad-SPI Flash Memory 8-Pin SOIC, W25Q64JVSSIQ
  SPI_FLASH_MOSI_PIN PC3  // SPI2 MOSI IF ENABLED(SPI_FLASH) Winbond NOR 64Mbit Quad-SPI Flash Memory 8-Pin SOIC, W25Q64JVSSIQ

 EXP1_01_PIN PC5  // BEEPER
 EXP1_02_PIN PE13 // BTN_ENC
 EXP1_03_PIN PD13 // LCD_EN
 EXP1_04_PIN PC6  // LCD_RS
 EXP1_05_PIN PE14 // LCD_D4
 EXP1_06_PIN PE15 // LCD_D5
 EXP1_07_PIN PD11 // LCD_D6
 EXP1_08_PIN PD10 // LCD_D7
 EXP2_01_PIN PA6  // SPI1 MISO
 EXP2_02_PIN PA5  // SPI1 SCK
 EXP2_03_PIN PE8  // BTN_EN1
 EXP2_04_PIN PE10 // SPI1 CS
 EXP2_05_PIN PE11 // BTN_EN2
 EXP2_06_PIN PA7  // SPI1 MOSI
 EXP2_07_PIN PE12 // SPI1_RESET

### Marlin/src/pins/stm32f4/pins_MKS_ROBIN_NANO_V3.h
    // SPI connection for TMC2130, TMC2160, TMC5130, TMC5160, TMC2660
    #ifndef X_CS_PIN
       #define X_CS_PIN                         PD5
    #endif
    #ifndef Y_CS_PIN
      #define Y_CS_PIN                          PD7
    #endif
    #ifndef Z_CS_PIN
      #define Z_CS_PIN                          PD4
    #endif
    #ifndef E0_CS_PIN
      #define E0_CS_PIN                         PD9
    #endif
    #ifndef E1_CS_PIN                                  
       #define E1_CS_PIN                        PD8   
    #endif

    // This board only supports SW SPI for stepper drivers
    #if HAS_TMC_SPI
       #define TMC_USE_SW_SPI
    #endif
    #if !defined(TMC_SPI_MOSI) || TMC_SPI_MOSI == -1
       #undef TMC_SPI_MOSI
       #define TMC_SPI_MOSI                      PD14
    #endif
    #if !defined(TMC_SPI_MISO) || TMC_SPI_MISO == -1
       #undef TMC_SPI_MISO
       #define TMC_SPI_MISO                      PD1
    #endif
    #if !defined(TMC_SPI_SCK) || TMC_SPI_SCK == -1
      #undef TMC_SPI_SCK
      #define TMC_SPI_SCK                       PD0
    #endif