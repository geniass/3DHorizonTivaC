#ifndef DECIMATEFILTER_H_
#define DECIMATEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 1000 Hz

* 0 Hz - 25 Hz
  gain = 1
  desired ripple = 1 dB
  actual ripple = 0.22371862902938705 dB

* 100 Hz - 500 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -50.525736377847124 dB

*/

#define DECIMATEFILTER_TAP_NUM 31

typedef struct {
  float history[DECIMATEFILTER_TAP_NUM];
  unsigned int last_index;
} DecimateFilter;

void DecimateFilter_init(DecimateFilter* f);
void DecimateFilter_put(DecimateFilter* f, float input);
float DecimateFilter_get(DecimateFilter* f);

#endif
