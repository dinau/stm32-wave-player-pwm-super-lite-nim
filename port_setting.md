<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->

- [Port definition](#port-definition)
  - [Push Button SW (on board)](#push-button-sw-on-board)
  - [LED indicator](#led-indicator)
  - [USART2](#usart2)
  - [PWM With TIM3](#pwm-with-tim3)
  - [SPI1](#spi1)
  - [TEST_PORT](#test_port)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

### Port definition 

#### Push Button SW (on board)

| Port               | Model                                                |
|--------------------|------------------------------------------------------|
| PA_0 (Hi active)   | STM32F0Discovery, STM32F3Discovery, STM32F4Discovery |
| PC_13 (Low active) | Others                                               |

####  LED indicator

| Port                           | Model            |
|--------------------------------|------------------|
| PE_11 (on board LED green,LD7) | STM32F3Discovery |
| PD_12 (on board LED green,LD4) | STM32F4Discovery |
| PB_10, D6                      | Others           |

#### USART2

| Port     | Model     |
|----------|-----------|
| PA_2, D1 | USART2_TX |
| PA_3, D0 | USART2_RX |

#### PWM With TIM3

- Use PWM3-1, PWM3-2 on TIM3  
   Higher 8bit

   | Port                                     | Model |
   |------------------------------------------|-------|
   | PB_5, D4  PWM_PORT_LEFT_HI    (TIM3_CH2) | All   |
   | PB_4, D5  PWM_PORT_RIGHT_HI   (TIM3_CH1) | All   |

- Use PWM3-3,PWM3-4 with TIM3  
   Lower 8bit

   | Port                                     | Model |
   |------------------------------------------|-------|
   | PC_9, -  PWM_PORT_LEFT_LOW    (TIM3_CH4) | All   |
   | PC_8, -  PWM_PORT_RIGHT_LOW   (TIM3_CH3) | All   |

#### SPI1

- Chip select port (CS) 

   | Port      | Model                              |
   |-----------|------------------------------------|
   | PB_8      | STM32F3Discovery, STM32F4Discovery |
   | PB_6, D10 | Others                             |

- Other

   | Function | Port      | Model |
   |----------|-----------|-------|
   | SCK      | PA_5, D13 | All   |
   | MISO     | PA_6, D12 | All   |
   | MOSI     | PA_7, D11 | All   |

####  TEST_PORT

| Port     | Model |
|----------|-------|
| PA_8, D7 | All   |

