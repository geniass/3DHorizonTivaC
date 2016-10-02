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
#define FLOAT_FRAC_MULTIPLIER	10000

void sendIMUData(void);


#endif /* SEND_DATA_H_ */
