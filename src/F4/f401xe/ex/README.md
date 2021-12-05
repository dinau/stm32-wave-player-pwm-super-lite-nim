## STM32 on Nim test program

### Nim compiler:
Recomend that nim compiler version is from 0.19.0 to 0.19.9(nightly) at this moment.(2019/03)

### arm-none-eabi-gcc:
v4.8.3 or later  
Recomend Launchpad arm gcc.

### Supported Board/MCU:
nulceo_f030r8  
nucleo_l152re  
nulceo_f401re  
nulceo_f411re  
STM32F0Discovery  
STM32F3Discovery  
STM32F4Discovery  

### Compilation:
Example:
```console
$ cd nucleo_l152re
$ make
```

Generated files are,  
    BUILD/nucleo_l152re.**bin**  
    BUILD/nucleo_l152re.**hex**  
    etc.  
You can just copy bin file to mbed drive,  
or can flash hex file using st-link utility.  

See simple test program:  
    src/**main.nim**  




