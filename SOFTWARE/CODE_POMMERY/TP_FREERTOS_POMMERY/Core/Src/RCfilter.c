#include "RCFilter.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SCALE 1024u

void RC_filter_init(h_RC_filter_t *h,uint16_t cutoff_frequency,uint16_t sampling_frequency)
{
    if (!h) return;

    if (cutoff_frequency == 0) cutoff_frequency = 1;
    if (sampling_frequency == 0) sampling_frequency = 1;

    float fs = (float)sampling_frequency;
    float fc = (float)cutoff_frequency;
    float K  = fs / (2.0f * (float)M_PI * fc);

    float A = 1.0f;
    float B = K;
    float D = 1.0f + K;

    h->coeff_A = (uint32_t)(A * (float)SCALE + 0.5f);
    h->coeff_B = (uint32_t)(B * (float)SCALE + 0.5f);
    h->coeff_D = (uint32_t)(D * (float)SCALE + 0.5f);

}

uint16_t RC_filter_update(h_RC_filter_t *h, uint16_t input)
{
    if (!h) return input;

    uint64_t num = 0;
    num += (uint64_t)h->coeff_A * (uint64_t)input;
    num += (uint64_t)h->coeff_B * (uint64_t)h->out_prev;

    uint32_t den = h->coeff_D;
    if (den == 0u) return input;

    uint32_t y = (uint32_t)((num + (uint64_t)(den / 2u)) / (uint64_t)den);

    if (y > 0xFFFFu) y = 0xFFFFu;//on clamp au cas ou
    h->out_prev = (uint16_t)y;//on update l'etat precedent
    return (uint16_t)y;
}
