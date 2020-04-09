# How to build

To build the C_Controller you need

- `git submodule update --init` : Initialize the submodule (FreeRTOS)
- `cmake -DTARGET=native` : prepare the build of the simulator
- `make` : build the simulator
- `./simulateur` : run the simulator without stdout output

# How to run with ihm

- `git clone https://github.com/Recovid/ihm.git`
- `cd ihm`
- `mkfifo in; mkfifo out`
- `python main.py -m`
- `cd C_Controller`
- `./simulateur -f ../../ihm/out ../../ihm/in`

