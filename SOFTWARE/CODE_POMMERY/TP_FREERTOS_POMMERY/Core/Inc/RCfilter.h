#ifndef RCFILTER_H
#define RCFILTER_H

#include <stdint.h>


#define SAMPLING_FREQ 48000
typedef struct {
    uint32_t coeff_A;
    uint32_t coeff_B;
    uint32_t coeff_D;
    uint16_t out_prev;
} h_RC_filter_t;


void RC_filter_init(h_RC_filter_t *h_RC_filter,uint16_t cutoff_frequency,uint16_t sampling_frequency);

uint16_t RC_filter_update(h_RC_filter_t *h_RC_filter, uint16_t input);

#endif
