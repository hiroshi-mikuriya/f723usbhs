cmake_minimum_required(VERSION 3.6)

set(CMAKE_SYSTEM_NAME Generic)

project(NewFolder)

# product
set(ELF_FILE ${PROJECT_NAME}.elf)
set(HEX_FILE ${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_NAME}.bin)
set(MAP_FILE ${PROJECT_NAME}.map)

# compiler
set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_EXE_LINKER   arm-none-eabi-g++)
set(OBJCOPY            arm-none-eabi-objcopy)
set(SIZE               arm-none-eabi-size)

# compiler options
set(CPU       cortex-m7)
set(FPU       -mfpu=fpv5-sp-d16)
set(FLOAT_ABI -mfloat-abi=hard)

add_compile_options(-mcpu=${CPU})
add_compile_options(-mthumb)
add_compile_options(${FPU})
add_compile_options(${FLOAT_ABI})
add_compile_options(-O2)
add_compile_options(-Wall)
add_compile_options(-fdata-sections)
add_compile_options(-ffunction-sections)
add_compile_options(-MMD)
add_compile_options(-MP)

set(CMAKE_C_FLAGS -std=gnu99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS -fno-use-cxa-atexit)

add_link_options(-mcpu=${CPU})
add_link_options(${FPU})
add_link_options(${FLOAT_ABI})
add_link_options(--specs=nosys.specs)
add_link_options(-T${CMAKE_CURRENT_SOURCE_DIR}/STM32F723ZETx_FLASH.ld)
add_link_options(-lc)
add_link_options(-lm)
add_link_options(-lnosys)
add_link_options(-Wl,-Map=${MAP_FILE},--cref)
add_link_options(-Wl,--gc-sections)

enable_language(ASM)

# define
add_definitions(-DUSE_FULL_LL_DRIVER)
add_definitions(-DPREFETCH_ENABLE=1)
add_definitions(-DINSTRUCTION_CACHE_ENABLE=1)
add_definitions(-DDATA_CACHE_ENABLE=1)
add_definitions(-DSTM32F723xx)

# includes
include_directories(
    USB_DEVICE/App
    USB_DEVICE/Target
    Core/Inc
    Drivers/STM32F7xx_HAL_Driver/Inc
    Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    Drivers/CMSIS/Device/ST/STM32F7xx/Include
    Drivers/CMSIS/Include
    Drivers/CMSIS/Include
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
)

# sources
file(GLOB SRCS
    Core/Src/main.c
    Core/Src/stm32f7xx_it.c
    Core/Src/stm32f7xx_hal_msp.c
    Core/Src/stm32f7xx_hal_timebase_tim.c
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/Target/usbd_conf.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_gpio.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_rcc.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_utils.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_exti.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_dma.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_spi.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c
    Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c
    Core/Src/system_stm32f7xx.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    Core/Src/freertos.c
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c
	startup_stm32f723xx.s
)

# executables
add_executable(${ELF_FILE} ${SRCS})

add_custom_command(TARGET ${ELF_FILE} POST_BUILD
	COMMAND ${OBJCOPY} -Obinary ${ELF_FILE} ${BIN_FILE}
	COMMAND ${OBJCOPY} -Oihex   ${ELF_FILE} ${HEX_FILE}
	COMMAND ${SIZE} --format=berkeley ${ELF_FILE} ${HEX_FILE}
	COMMENT "Invoking: Cross ARM GNU Print Size"
)