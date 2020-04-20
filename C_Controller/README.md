# How to build simulator

To build the C_Controller you need
``` sh
$ git submodule update --init # Initialize the submodule (FreeRTOS)
$ cmake -DTARGET=native # prepare the build of the simulator
$ make # build the simulator
$ ./C_Controller # run the simulator without stdout output
```
# How to run with ihm
## Using fifo to emulate Serial (Unix Only)
``` sh
$ git clone https://github.com/Recovid/ihm.git
$ cd ihm
$ mkfifo in; mkfifo out
$ python main.py -m
$ cd C_Controller
$ ./C_Controller -f ../../ihm/out ../../ihm/in
```

## Using a virtual serial port (Unix Only)
``` sh
$ socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

``` sh
ihm $ python -s /dev/pts/2 # Replace with output of socat
```

``` sh
C_Controller $ ./C_Controller -s /dev/pts/3 # Replace with output of socat
```



# How to build real firmware

Both scenarios need a proper install of [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).

If you need to use UART4 instead of UART2 (real case scenario and not debug using USB port on Nucleo), you should pass an extra options to Cmake: "-DRASPI_CONNECTION=1"

## Using windows

You need an additionary tool for flashing your device "STM32CubeProgrammer".

````bash
set "STM32CubePath=C:\ST\STM32CubeIDE_1.3.0\STM32CubeIDE"
rem Add cross toolchain to your path
set "PATH=%PATH%;%STM32CubePath%\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.7-2018-q2-update.win32_1.0.0.201904181610\tools\bin;"
# Add unix build tools (make, ...)
set "PATH=%PATH%;%STM32CubePath%\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_1.1.0.201910081157\tools\bin;
#Run cmake to generate the makefile
cmake  -DTARGET=stm32f303 -DCMAKE_BUILD_TYPE=Debug -G"Unix Makefiles"
# make the binblob to flash
make -j8
make -j8 C_Controller.stm32f303.elf.binary

#Flash IT
"C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -hardRst
"C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -rdu
"C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -w TON_BINAIRE C_Controller.stm32f303.elf.binary --start

````


## Using linux


````bash
#Add cross toolchain to your path
PATH=$PATH:/opt/st/stm32cubeide_1.3.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.7-2018-q2-update.linux64_1.0.0.201904181610/tools/bin
#Add openocd to your path for flashing
PATH=$PATH:/opt/st/stm32cubeide_1.3.0/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_1.3.0.202002181050/tools/bin/
#Run cmake to generate the makefile
cmake  -DTARGET=stm32f303 -DCMAKE_BUIDL_TYPE=Debug -G"Unix Makefiles"
# make the bin to flash
make -j16 C_Controller.stm32f303.elf
# and flash it using OpenOCD
make -j16 C_Controller.stm32f303.elf.flash

````
