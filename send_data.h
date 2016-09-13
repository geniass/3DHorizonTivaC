/*
 * send_data.h
 *
 *  Created on: 01 Sep 2016
 *      Author: Ari Croock
 */

#ifndef SEND_DATA_H_
#define SEND_DATA_H_

//
// Fractional part multiplier = 10^{E}
// where E is the num of desired decimal places
//
#define FLOAT_FRAC_MULTIPLIER	1000.f

void sendIMUData(void);
void float_to_ints(int32_t *integer, uint32_t *fractional, float f);


#endif /* SEND_DATA_H_ */
