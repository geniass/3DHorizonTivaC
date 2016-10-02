#ifndef HEAVEFILTER_H_
#define HEAVEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 50 Hz

* 0 Hz - 2 Hz
  gain = 1
  desired ripple = 0.5 dB
  actual ripple = 0.30777919778307977 dB

* 5 Hz - 25 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -41.64236602152817 dB

*/

#define HEAVEFILTER_TAP_NUM 35

typedef struct {
  float history[HEAVEFILTER_TAP_NUM];
  unsigned int last_index;
} HeaveFilter;

void HeaveFilter_init(HeaveFilter* f);
void HeaveFilter_put(HeaveFilter* f, float input);
float HeaveFilter_get(HeaveFilter* f);

#endif
