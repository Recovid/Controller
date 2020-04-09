# How to build

To build the C_Controller you need
``` sh
$ git submodule update --init # Initialize the submodule (FreeRTOS)
$ cmake -DTARGET=native # prepare the build of the simulator
$ make # build the simulator
$ ./simulateur # run the simulator without stdout output
```
# How to run with ihm
## Using fifo to emulate Serial (Unix Only)
``` sh
$ git clone https://github.com/Recovid/ihm.git
$ cd ihm
$ mkfifo in; mkfifo out
$ python main.py -m
$ cd C_Controller
$ ./simulateur -f ../../ihm/out ../../ihm/in
```

## Using a virtual serial port (Unix Only)
``` sh
$ socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

``` sh
ihm $ python -s /dev/pts/2 # Replace with output of socat
```

``` sh
C_Controller $ ./simulateur -s /dev/pts/3 # Replace with output of socat
```
