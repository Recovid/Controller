/*
 * types.h
*
 *  Created on: 11 sept. 2016
 *  Last Modification: 27 fev. 2017
 *      Author: Florian Bianco (florian.bianco@univ-lyon1.fr)
 *              Romain Delpoux (romain.delpoux@insa-lyon.fr)
 */
 
#ifndef TYPES_H_
#define TYPES_H_

typedef signed char s8;
typedef signed short s16;
typedef signed long s32;

enum {
  STATE_ERROR = -1,
  STATE_INIT  = 0,
  STATE_OK,
  STATE_RUN,
  STATE_STOP,
};

#endif /* TYPES_H_ */
