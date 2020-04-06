#ifndef CONTROLLER_SETTINGS_H
#define CONTROLLER_SETTINGS_H

// ------------------------------------------------------------------------------------------------
//! Public interface to the controller settings

//! Global settings use types that can be atomically read/write in a threadsafe way on STM32
//! \warning Only communication.c and controller.c should ever include this file

extern int FR_pm     ;
extern int VT_mL     ;
extern int PEP_cmH2O ;
extern int Vmax_Lpm  ;
extern long Tplat_ms  ;

extern int Pmax_cmH2O;
extern int Pmin_cmH2O;
extern int VTmin_mL  ;
extern int FRmin_pm  ;
extern int VMmin_Lm  ;

extern long Tpins_ms  ;
extern long Tpexp_ms  ;
extern long Tpbip_ms  ;

extern char init_str[80];

#endif // CONTROLLER_SETTINGS_H
