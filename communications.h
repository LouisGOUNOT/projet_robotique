/*
 * 	communications.h
 *
 *  Created on: 15 may 2021
 *  Author: Clément Albert & Louis Gounot
 *
 *
 */

#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H


void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, float* data, uint16_t size);


#endif /* COMMUNICATIONS_H */
