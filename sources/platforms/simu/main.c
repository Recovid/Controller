#include "platform.h"



int main(int argc, char **argv)
{
  // TODO setup serial port according to provided arguments


  // Start the controller
  controller_main();

  // we should never get here
  return -1;
}


