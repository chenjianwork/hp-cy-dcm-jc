####################################################################################################
DEFINES += -DSTM32F40_41xxx
DEFINES += -DUSE_STDPERIPH_DRIVER
DEFINES += -DHSE_VALUE=12000000

####################################################################################################
LDSCRIPT = chip/stm32f407_flash.lds

####################################################################################################
INCLUDE += include
INCLUDE += chip
INCLUDE += chip/CMSIS/Include
INCLUDE += chip/CMSIS/Device/ST/STM32F4xx/Include
INCLUDE += chip/STM32F4xx_StdPeriph_Driver/inc/

####################################################################################################
SOURCES += main.c

####################################################################################################
# 驱动管理器
####################################################################################################
SOURCES += drvmanager/drvmanager.c
SOURCES += drvmanager/drvmanager-cpu.c
SOURCES += drvmanager/drvmanager-led.c
#SOURCES += drvmanager/drvmanager-boot.c
SOURCES += drvmanager/drvmanager-spi.c
SOURCES += drvmanager/drvmanager-timer.c
SOURCES += drvmanager/drvmanager-uart.c
SOURCES += drvmanager/drvmanager-can.c
SOURCES += drvmanager/drvmanager-dio.c
SOURCES += drvmanager/drvmanager-adc.c
SOURCES += drvmanager/drvmanager-dac.c
SOURCES += drvmanager/drvmanager-flash.c

####################################################################################################
# 系统管理器
####################################################################################################
SOURCES += sysmanager/sysmanager.c
SOURCES += sysmanager/sysmanager-idle.c
SOURCES += sysmanager/sysmanager-running.c
SOURCES += sysmanager/sysmanager-para.c

####################################################################################################
# 故障管理器
############################################################################
#######################
SOURCES += errmanager/errmanager.c

####################################################################################################
# 通信管理器
####################################################################################################
SOURCES += commanager/commanager.c
SOURCES += commanager/commanager-debug.c
SOURCES += commanager/commanager-can.c

####################################################################################################
# 设备管理器
####################################################################################################
SOURCES += devmanager/devmanager.c
SOURCES += devmanager/devmanager-pt207.c
SOURCES += devmanager/devmanager-pt206.c
SOURCES += devmanager/devmanager-flow.c
SOURCES += devmanager/devmanager-firealarm.c
SOURCES += devmanager/devmanager-vfd.c

####################################################################################################
SOURCES += lib/algo/pid.c
SOURCES += lib/crypto/crc.c
SOURCES += lib/crypto/crcdata.c
SOURCES += lib/bitconverter.c
SOURCES += lib/endian.c

####################################################################################################
SOURCES += chip/startup_stm32f40_41xxx.S
SOURCES += chip/system_stm32f4xx.c

SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/misc.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c
SOURCES += chip/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c
