# HAL编译项清理记录

## 背景

当前工程来源接近 HAL 全量模板，`Project/Fire-F407.uvprojx` 中编译了大量当前业务未使用的 HAL 外设源码。链接器会剔除很多未调用函数，所以最终 ROM 体积不一定明显变化，但这些编译项会增加重编译时间、输出文件数量和 map 文件噪声，也容易让后续维护时误以为这些外设正在使用。

当前业务代码主要使用：

- RCC/FLASH/PWR：系统时钟和 Flash 配置
- GPIO：LED、按键、RS485 方向、电机 IO
- TIM：步进电机脉冲输出比较
- UART：调试串口、夹爪串口、RS485 通信
- DMA：HAL TIM/UART 头文件中的句柄结构依赖 `DMA_HandleTypeDef`，即使业务当前不使用 DMA 传输，也需要保留 DMA 类型定义
- CORTEX：NVIC 中断配置

源码文件暂不删除，只从 Keil 工程编译列表中移除不需要的文件，并同步关闭 `User/stm32f4xx_hal_conf.h` 中对应宏。

## 清理顺序

### 第一批：显示和图像相关

这批与摄像头、LCD/图像显示相关，当前项目没有业务代码调用，优先清理，便于先编译和真机验证。

移除 Keil 编译项：

```text
Libraries/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dcmi.c
Libraries/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dcmi_ex.c
Libraries/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c
Libraries/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dsi.c
Libraries/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc.c
Libraries/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc_ex.c
```

关闭宏：

```c
/* #define HAL_DCMI_MODULE_ENABLED */
/* #define HAL_DMA2D_MODULE_ENABLED */
/* #define HAL_LTDC_MODULE_ENABLED */
```

说明：`stm32f4xx_hal_dsi.c` 当前在 Keil 工程中被编译，但 `stm32f4xx_hal_conf.h` 中没有启用 DSI 宏，因此只需要移除编译项。

### 第二批：外部存储、USB、SD卡等大块外设

第一批验证通过后，继续移除：

```text
stm32f4xx_hal_hcd.c
stm32f4xx_hal_pcd.c
stm32f4xx_hal_pcd_ex.c
stm32f4xx_ll_usb.c
stm32f4xx_hal_sd.c
stm32f4xx_hal_mmc.c
stm32f4xx_ll_sdmmc.c
stm32f4xx_hal_nand.c
stm32f4xx_hal_nor.c
stm32f4xx_hal_pccard.c
stm32f4xx_hal_sdram.c
stm32f4xx_hal_sram.c
stm32f4xx_ll_fsmc.c
```

对应关闭宏：

```c
/* #define HAL_HCD_MODULE_ENABLED */
/* #define HAL_PCD_MODULE_ENABLED */
/* #define HAL_SD_MODULE_ENABLED */
/* #define HAL_NAND_MODULE_ENABLED */
/* #define HAL_NOR_MODULE_ENABLED */
/* #define HAL_PCCARD_MODULE_ENABLED */
/* #define HAL_SDRAM_MODULE_ENABLED */
/* #define HAL_SRAM_MODULE_ENABLED */
```

### 第三批：当前业务未使用的小外设/专用外设

第二批验证通过后，继续移除：

```text
stm32f4xx_hal_adc.c
stm32f4xx_hal_adc_ex.c
stm32f4xx_hal_can.c
stm32f4xx_hal_cec.c
stm32f4xx_hal_crc.c
stm32f4xx_hal_cryp.c
stm32f4xx_hal_cryp_ex.c
stm32f4xx_hal_dac.c
stm32f4xx_hal_dac_ex.c
stm32f4xx_hal_dfsdm.c
stm32f4xx_hal_eth.c
stm32f4xx_hal_fmpi2c.c
stm32f4xx_hal_fmpi2c_ex.c
stm32f4xx_hal_hash.c
stm32f4xx_hal_hash_ex.c
stm32f4xx_hal_i2c.c
stm32f4xx_hal_i2c_ex.c
stm32f4xx_hal_i2s.c
stm32f4xx_hal_i2s_ex.c
stm32f4xx_hal_irda.c
stm32f4xx_hal_iwdg.c
stm32f4xx_hal_lptim.c
stm32f4xx_hal_qspi.c
stm32f4xx_hal_rng.c
stm32f4xx_hal_rtc.c
stm32f4xx_hal_rtc_ex.c
stm32f4xx_hal_sai.c
stm32f4xx_hal_sai_ex.c
stm32f4xx_hal_smartcard.c
stm32f4xx_hal_spdifrx.c
stm32f4xx_hal_spi.c
stm32f4xx_hal_usart.c
stm32f4xx_hal_wwdg.c
```

对应关闭宏：

```c
/* #define HAL_ADC_MODULE_ENABLED */
/* #define HAL_CAN_MODULE_ENABLED */
/* #define HAL_CRC_MODULE_ENABLED */
/* #define HAL_CRYP_MODULE_ENABLED */
/* #define HAL_DAC_MODULE_ENABLED */
/* #define HAL_HASH_MODULE_ENABLED */
/* #define HAL_I2C_MODULE_ENABLED */
/* #define HAL_I2S_MODULE_ENABLED */
/* #define HAL_IWDG_MODULE_ENABLED */
/* #define HAL_RNG_MODULE_ENABLED */
/* #define HAL_RTC_MODULE_ENABLED */
/* #define HAL_SAI_MODULE_ENABLED */
/* #define HAL_SPI_MODULE_ENABLED */
/* #define HAL_USART_MODULE_ENABLED */
/* #define HAL_IRDA_MODULE_ENABLED */
/* #define HAL_SMARTCARD_MODULE_ENABLED */
/* #define HAL_WWDG_MODULE_ENABLED */
```

## 保留项

当前阶段保留以下 HAL 编译项和宏：

```text
stm32f4xx_hal.c
stm32f4xx_hal_cortex.c
stm32f4xx_hal_dma.c
stm32f4xx_hal_dma_ex.c
stm32f4xx_hal_flash.c
stm32f4xx_hal_flash_ex.c
stm32f4xx_hal_flash_ramfunc.c
stm32f4xx_hal_gpio.c
stm32f4xx_hal_pwr.c
stm32f4xx_hal_pwr_ex.c
stm32f4xx_hal_rcc.c
stm32f4xx_hal_rcc_ex.c
stm32f4xx_hal_tim.c
stm32f4xx_hal_tim_ex.c
stm32f4xx_hal_uart.c
```

```c
#define HAL_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
```

## 验证方式

每一批清理后执行 Keil 编译，确认：

- 编译 `0 Error(s)`
- 串口命令、步进电机、夹爪、吸盘基础功能正常
- 如需评估收益，对比 `Listings/YH-F407.map` 中 `Total ROM Size` 和 `Output` 中生成对象数量
