#include "recovid_revB.h"
#include "platform.h"
#include"../../../src/adrien/const_calibs.h"



bool init_valve() {
  return true;
}

//! \returns false in case of hardware failure
bool is_valve_ok() {
  return true;
}

//! Positions electrovalve to connect patient with PEP to allow him to exhale
bool valve_exhale() {
	
	#if	ENABLE_CORRECTION_FABRICE_GERMAIN		==		1 &&		MODEL_ERR_PAR_PHASES		==		1
		GLOB_PHASE___modele_erreur = EXPIRATION_MODEL_ERR; /// nouvelle implem FABRICE : 3 phases 
		ACCUMULATION_err_max_EXPI = 0.0f;
	#endif
			
  HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_EXHALE);
  return true;
}

//! Positions electrovalve to connect patient with BAVU to insufflate him or keep its airway pressure higher than PEP
bool valve_inhale() {
  HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_INHALE);
  return true;
}
