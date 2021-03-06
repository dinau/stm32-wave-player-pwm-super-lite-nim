<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->

- [PCM Wave Player Super Lite with Nim language on STM32 MCUs](#pcm-wave-player-super-lite-with-nim-language-on-stm32-mcus)
  - [Supported Board/MCU](#supported-boardmcu)
    - [NULCEO-F030R8](#nulceo-f030r8)
    - [NUCLEO-L152RE](#nucleo-l152re)
    - [NULCEO-F401RE](#nulceo-f401re)
    - [NULCEO-F411RE](#nulceo-f411re)
    - [STM32F0Discovery](#stm32f0discovery)
    - [STM32F3Discovery](#stm32f3discovery)
    - [STM32F4Discovery](#stm32f4discovery)
  - [Supported SD card](#supported-sd-card)
  - [Supported PCM Format](#supported-pcm-format)
  - [Defalut action](#defalut-action)
  - [Hardware setting/Schematic: (16bit sound resolution)](#hardware-settingschematic-16bit-sound-resolution)
  - [SD Card connection](#sd-card-connection)
  - [PWM output port](#pwm-output-port)
  - [LED indicator](#led-indicator)
  - [Nim compiler](#nim-compiler)
  - [C compiler: arm-none-eabi-gcc](#c-compiler-arm-none-eabi-gcc)
  - [Compiling source code](#compiling-source-code)
  - [Output music filename  through UART port](#output-music-filename--through-uart-port)
  - [Simple less bits mode](#simple-less-bits-mode)
  - [Pursue small code size](#pursue-small-code-size)
  - [Other links](#other-links)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

### PCM Wave Player Super Lite with Nim language on STM32 MCUs

16bit resolution PWM wave player with SD card, super lite version.

#### Supported Board/MCU

#####  [NULCEO-F030R8](https://os.mbed.com/platforms/ST-Nucleo-F030R8/)  

#####  [NUCLEO-L152RE](https://os.mbed.com/platforms/ST-Nucleo-L152RE/)  

#####  [NULCEO-F401RE](https://os.mbed.com/platforms/ST-Nucleo-F401RE/)  

#####  [NULCEO-F411RE](https://os.mbed.com/platforms/ST-Nucleo-F411RE/)  

#####  [STM32F0Discovery](https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-discovery-kits/stm32f0discovery.html)  

#####  [STM32F3Discovery](https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-discovery-kits/stm32f3discovery.html)  

#####  [STM32F4Discovery](https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-discovery-kits/stm32f4discovery.html)  

#### Supported SD card

* SDSC/SDHC card, FAT16 and FAT32.  
    1. At first, format SD card with [SD Card Formatter](https://www.sdcard.org/downloads/formatter_4/index.html)
    1. Copy PCM wav files to the SD card on the root directory.  

#### Supported PCM Format

- PCM wav file that have file extension ".wav" on root directory.  
- 16bit/8bit, fs(sampling rate)=32kHz,44.1kHz,48kHz.  
- Stereo/Mono.

#### Defalut action

- Afer power on, loop playback automatically and infinitly.
- **USER_BUTTON on board operation**: (PC_13 or PA0)  
  - _Next song_: One click in Play mode.  
  - _Pause_ : Push long time .  
  - _Play_ : One click from Pause.  

#### Hardware setting/Schematic: (16bit sound resolution)

- Refer to the file, [port_setting.md](https://github.com/dinau/stm32-wave-player-pwm-super-lite-nim/blob/main/port_setting.md).
- See folder doc\/*  
![](http://mpu.up.seesaa.net/image/16bit-wave-player-output-schema.png)  

#### SD Card connection

```console
Pin side
 --------------.
         9     = \    DAT2/NC
             1 ===|   CS/DAT3    [CS]
             2 ===|   CMD/DI     [DI]
             3 ===|   VSS1
 Bottom      4 ===|   VDD
 View        5 ===|   CLK        [CLK]
             6 ===|   VSS2
             7 ===|   DO/DAT0    [DO]
         8       =|   DAT1/IRQ
 -----------------'

                                         Arduino      NUCLEO-F411       NUCLEO-F030R8
 Logo side
 -----------------.
         8       =|   DAT1/IRQ
             7 ===|   DO/DAT0    [DO]     D12           D12/PA_6           D12/PA_6
             6 ===|   VSS2
 Top         5 ===|   CLK        [CLK]    D13           D13/PA_5           D13/PA_5
 View        4 ===|   VDD
             3 ===|   VSS1
             2 ===|   CMD/DI     [DI]     D11           D11/PA_7           D11/PA_7
             1 ===|   CS/DAT3    [CS]     D8            D10/PB_6           D10/PB_6
         9     = /    DAT2/NC
 ---------------'
```

#### PWM output port

- Left  upper(Hi)  PWM 8bit out: PB_5 (TM3_CH2)  
- Right upper(Hi)  PWM 8bit out: PB4  (TM3_CH1)  
- Left  lower(Low) PWM 8bit out: PC_9 (TM3_CH4)  
- Right lower(Low) PWM 8bit out: PC_8 (TM3_CH3)  

#### LED indicator

- See [port_setting.md](https://github.com/dinau/stm32-wave-player-pwm-super-lite-nim/blob/main/port_setting.md).
- If set up LED device to "LED indicator port", it will be dimmer during play mode (regular speed) and pause mode (fast speed).

#### Nim compiler

* Recomended nim compiler version is nim-1.6.4 at this time.

#### C compiler: arm-none-eabi-gcc

- v4.8.3 or later.  
- Recomend [GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)  

#### Compiling source code

* Example: (NUCLEO-F411RE board)

   ```sh
   $ cd src/F4/f401xe/nucleo-f411re
   $ make
   .........................................................
   CC: xprintf
   CC: startup_stm32f411xe
   CC: stdlib_system.nim
   CC: ../stm32f401xe.nim
   CC: ../../../lib/reginfo.nim
   CC: ../uart.nim
   CC: ../pwm.nim
   CC: board.nim
   CC: ../spi.nim
   CC: ../../../lib/systick.nim
   CC: ../../../lib/sd_card.nim
   CC: ../../../lib/fat_lib.nim
   CC: ../../../wave_player_main.nim
   CC: ../../../main.nim
   CC: start.nim
   Hint:  [Link]
   Hint: gc: arc; opt: speed; options: -d:danger
   90709 lines; 11.377s; 96.109MiB peakmem; proj: .\start; out: ...
      text    data     bss     dec     hex filename
      4148      16      88    4252    109c BINHEX\nucleo_f411re.elf
   ```

   Generated files are

     ```
     BINHEX/nucleo_f411re.bin  
     BINHEX/nucleo_f411re.hex  
     ...
     ```

You can just copy the **bin** file to mbed drive,
or can flash the **hex** file with ST-Link utility.  
  
If you don't like to compile from source code, you can immediately upload [doc/hex/*.hex or *.bin files](https://github.com/dinau/stm32-wave-player-pwm-super-lite-nim/tree/main/doc/hex) to flash.

#### Output music filename  through UART port

By default music filename is send through default UART(USB-CDC(Nucleo boards)) port.
Baudrate is **115200bps**.  
![](http://mpu.up.seesaa.net/image/filename-to-uart-port.png)  
If you don't need UART output feature, set **UART_INFO\* = false** in [src/conf_sys.nim](https://github.com/dinau/stm32-wave-player-pwm-super-lite-nim/blob/main/src/conf_sys.nim) and  
recompile the project.

#### Simple less bits mode

If set **PWM16BIT\* = false** in src/conf_sys.nim and recompile the project, **simple less bits mode** is enabled.  
In spite of less parts, wirings and PCM bit length, it has fairly sound quality.
![](http://mpu.up.seesaa.net/image/less-bits-wave-player-output-schema.png)

**Table 1.** Sound resolution in **simple less bits mode**  
| Board            | Sound resolution | TIM_PWM_BASE_CLOCK |
|:-----------------|-----------------:|-------------------:|
| nulceo_f030r8    |            10bit |              48MHz |
| nucleo_l152re    |             9bit |              32MHz |
| nulceo_f401re    |            10bit |              84MHz |
| nulceo_f411re    |        **11bit** |              96MHz |
| STM32F0Discovery |            10bit |              48MHz |
| STM32F3Discovery |            10bit |              64MHz |
| STM32F4Discovery |            10bit |              84MHz |

#### Pursue small code size

In `src/conf_sys.nim`,by eliminating some functionalities,  
the code size can be further reduced. For instance, set as follows:  

| Flags                              | Descriptions                                        |
|------------------------------------|-----------------------------------------------------|
| UART_INFO\*                = false | Show information through UART                       |
| PWM16BIT\*                 = false | Set simple less bits mode                           |
| DATA_8BIT_SUPPORT\*        = false | Only support PCM16bit format,neglect PCM8bit format |
| HAVE_LED_IND_PWM\*         = false | Eliminate LED indicator function                    |
| FS_48KHZ_QUP\*             = false | Only supoort less than fs=48KHz                     |

will result in about:  

```sh
text    data     bss     dec     hex filename
3480      16      80    3576     df8 BINHEX\nucleo_f411re.elf
```

Now it can do auto playback after power on,  
can operate play,pause and next song with push button,  
be able to play fairly quality sound according to Table 1,
can recognize PCM 16bit wav file(but output quality is reduced accoding to Table 1. and supports stereo/mono, fs=32KHz,44.1KHz.

#### Other links

* Wave player projectm, Super lite series
  - Nim language
    - [Arduino Wave Player PWM Super Lite Nim / Nim](https://github.com/dinau/arduino-wave-player-pwm-super-lite-nim) Completed.
  - Mbed2 C/C++ language
    - [Wave Player Super Lite / STM32(F0,L1,F4) / Mbed2 / C++](https://os.mbed.com/users/mimi3/code/wave_player_super_lite) Completed.
  - Jal language
    - [Pwm Wave Player Jalv2 / PIC16F1xxx / Jal](https://github.com/dinau/16f-wave-player-pwm-super-lite-jalv2)

