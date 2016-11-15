cmake_minimum_required(VERSION 2.8.4)

SET(CMAKE_SYSTEM_NAME Generic)

SET(CMAKE_SYSTEM_PROCESSOR arm)
SET(CMAKE_CROSSCOMPILING 1)
project(NewProject1)

# set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
# set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

set(CMAKE_VERBOSE_MAKEFILE ON)
# SET(CMAKE_C_COMPILER   arm-none-eabi-gcc)
# SET(CMAKE_CXX_COMPILER   arm-none-eabi-g++)
# SET(CMAKE_C_COMPILER    C:\\Program Files\ (x86)\\EmBitz\\0.42\\share\\em_armgcc\\bin\\arm-none-eabi-gcc)
# SET(CMAKE_CXX_COMPILER  C:\\Program Files\ (x86)\\EmBitz\\0.42\\share\\em_armgcc\\bin\\arm-none-eabi-g++)
INCLUDE(CMakeForceCompiler)
# set(CMAKE_C_COMPILER   /usr/bin/arm-none-eabi-gcc )
# set(CMAKE_CXX_COMPILER   /usr/bin/arm-none-eabi-g++ )
# set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11")

# specify the cross compiler
SET(CMAKE_C_COMPILER   arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
# set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
# set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

include_directories(NewProject1/inc NewProject1/freertos/inc NewProject1/SPL/inc NewProject1/cmsis )
set(CROSS_COMPILE     arm-none-eabi- )


ADD_DEFINITIONS(-DARM_MATH_CM4
        -D__FPU_USED
        -DSTM32F401RE
        -fno-strict-aliasing
        -DSTM32F4XX
        -DUSE_STDPERIPH_DRIVER
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -mthumb-interwork
        -mcpu=cortex-m4
        -mthumb
        -mlittle-endian )

set(SOURCE_FILES
    NewProject1/freertos/src/croutine.c
    NewProject1/freertos/src/event_groups.c
    NewProject1/freertos/src/heap_4.c
    NewProject1/freertos/src/list.c
    NewProject1/freertos/src/port.c
    NewProject1/freertos/src/queue.c
    NewProject1/freertos/src/tasks.c
    NewProject1/freertos/src/timers.c
    NewProject1/SPL/src/misc.c
    NewProject1/SPL/src/stm32f4xx_adc.c
    NewProject1/SPL/src/stm32f4xx_can.c
    NewProject1/SPL/src/stm32f4xx_crc.c
    NewProject1/SPL/src/stm32f4xx_cryp.c
    NewProject1/SPL/src/stm32f4xx_cryp_aes.c
    NewProject1/SPL/src/stm32f4xx_cryp_des.c
    NewProject1/SPL/src/stm32f4xx_cryp_tdes.c
    NewProject1/SPL/src/stm32f4xx_dac.c
    NewProject1/SPL/src/stm32f4xx_dbgmcu.c
    NewProject1/SPL/src/stm32f4xx_dcmi.c
    NewProject1/SPL/src/stm32f4xx_dma.c
    NewProject1/SPL/src/stm32f4xx_dma2d.c
    NewProject1/SPL/src/stm32f4xx_exti.c
    NewProject1/SPL/src/stm32f4xx_flash.c
  #  NewProject1/SPL/src/stm32f4xx_fmc.c
  #  NewProject1/SPL/src/stm32f4xx_fsmc.c
    NewProject1/SPL/src/stm32f4xx_gpio.c
    NewProject1/SPL/src/stm32f4xx_hash.c
    NewProject1/SPL/src/stm32f4xx_hash_md5.c
    NewProject1/SPL/src/stm32f4xx_hash_sha1.c
    NewProject1/SPL/src/stm32f4xx_i2c.c
    NewProject1/SPL/src/stm32f4xx_iwdg.c
    NewProject1/SPL/src/stm32f4xx_ltdc.c
    NewProject1/SPL/src/stm32f4xx_pwr.c
    NewProject1/SPL/src/stm32f4xx_rcc.c
    NewProject1/SPL/src/stm32f4xx_rng.c
    NewProject1/SPL/src/stm32f4xx_rtc.c
    NewProject1/SPL/src/stm32f4xx_sai.c
    NewProject1/SPL/src/stm32f4xx_sdio.c
    NewProject1/SPL/src/stm32f4xx_spi.c
    NewProject1/SPL/src/stm32f4xx_syscfg.c
    NewProject1/SPL/src/stm32f4xx_tim.c
    NewProject1/SPL/src/stm32f4xx_usart.c
    NewProject1/SPL/src/stm32f4xx_wwdg.c
    NewProject1/src/analog_input.c
    NewProject1/src/comm.c
    NewProject1/src/data_blobs.c
    NewProject1/src/dhcp.c
    NewProject1/src/dns.c
    NewProject1/src/LCDshield.c
    NewProject1/src/main.c
    NewProject1/src/my_helpers.c
    NewProject1/src/ringbuffer.c
    NewProject1/src/socket.c
    NewProject1/src/spliturl.c
    NewProject1/src/system_stm32f4xx.c
    NewProject1/src/term_io.c
    NewProject1/src/vs10xx.c
    NewProject1/src/w5500.c
    NewProject1/src/wizchip_conf.c NewProject1/inc/kanalenlijst.h)

add_executable(NewProject1 ${SOURCE_FILES})
