#include "recovid_revB.h"
#include "platform.h"




bool valve_init() {
  return true;
}

//! \returns false in case of hardware failure
bool valve_is_ok() {
  return true;
}

//! Positions electrovalve to connect patient with PEP to allow him to exhale
bool valve_exhale() {
  HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_EXHALE);
  return true;
}

//! Positions electrovalve to connect patient with BAVU to insufflate him or keep its airway pressure higher than PEP
bool valve_inhale() {
  HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_INHALE);
  return true;
}
