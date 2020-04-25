#include "recovid.h"
#include "configuration.h"
#include "controller.h"
#include "breathing.h"
#include "lowlevel.h"



bool breathing_init() {
  return true;
};

void breathing_run(void *args) {
  UNUSED(args);
  printf("breathing started\n");
  while(true) {
    wait_ms(200);
    printf(".\n");
  }}