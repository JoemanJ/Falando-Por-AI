#include "filters.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>  // for memset

#define M_PI 3.14159265358979323846f

enum FILTER CURRENT_FILTER = DARTH_VADER;

// =================================================================================================================================
//                                                          Basic Filter Section
// =================================================================================================================================

// ---------------------------
// Filter init functions
// ---------------------------

// Initialize the low-pass filter
void lpf_init(LPF *f, float cutoff_freq, float sample_rate) {
	float dt = 1.0f / sample_rate;                  // Time step
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);  // RC time constant
    f->alpha = dt / (RC + dt);                      // Alpha coefficient
    f->prev = 0.0f;                                 // Previous output sample
}

void hpf_init(HPF *f, float cutoff_freq, float sample_rate) {
    float dt = 1.0f / sample_rate;                  // Time step
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);  // RC time constant
    f->alpha = RC / (RC + dt);                      // Alpha coefficient
    f->prev_x = 0.0f;                               // Previous input sample
    f->prev_y = 0.0f;                               // Previous output sample
}

// Initialize the echo effect
// This function sets up the echo effect with a delay time and decay factor.
void echo_init(Echo* e, float delay_ms, float decay, float sample_rate) {
    e->delay_samples = (int)(sample_rate * delay_ms / 1000.0f);                     // Convert delay time to samples
    if (e->delay_samples > MAX_DELAY_SAMPLES) e->delay_samples = MAX_DELAY_SAMPLES; // Clamp to max size
    e->size = e->delay_samples;                                                     // Set size
    e->index = 0;                                                                   // Reset index          
    e->decay = decay;                                                               // Set decay factor
    memset(e->buffer, 0, sizeof(e->buffer));                                        // Clear buffer
}

// Initialize the reverb effect
void reverb_init(Reverb* r, float delay_ms, float feedback, float mix, float sample_rate) {
    int delay_samples = (int)(sample_rate * delay_ms / 1000.0f);                // Convert delay time to samples
    if (delay_samples > MAX_DELAY_SAMPLES) delay_samples = MAX_DELAY_SAMPLES;   // Clamp to max size
    r->size = delay_samples;                                                    // Set size
    r->index = 0;                                                               // Reset index
    r->feedback = feedback;                                                     // Set feedback amount                                 
    r->mix = mix;                                                               // Set mix amount                                   
    memset(r->buffer, 0, sizeof(r->buffer));                                    // Clear buffer
}

// Initialize the pitch shifter
// pitch_factor: e.g. 0.7 for ~7 semitones down
void pitchshifter_init(PitchShifter* ps, float pitch_factor, float sample_rate) {
    memset(ps->buffer, 0, sizeof(ps->buffer));  // Clear buffer
    ps->write_index = 0;                        // Reset write index
    ps->read_index = 0.0f;                      // Reset read index
    ps->pitch_factor = pitch_factor;            // Set pitch factor
    ps->size = MAX_DELAY_SAMPLES;               // Set size to max delay samples
}

// ---------------------------
// Filter apply functions
// ---------------------------

// Apply low-pass filter
float apply_lpf(LPF *f, float x) {
    float y = f->alpha * x + (1.0f - f->alpha) * f->prev; // Apply low-pass filter formula
    f->prev = y;                                          // Update previous output sample
    return y;
}

float apply_hpf(HPF *f, float x) {
    float y = f->alpha * (f->prev_y + x - f->prev_x);   // Apply high-pass filter formula
    f->prev_x = x;                                      // Update previous input sample
    f->prev_y = y;                                      // Update previous output sample
    return y;
}

float apply_distortion(float x, float threshold) {
    if (x > threshold) return threshold;    // Clamp to threshold
    if (x < -threshold) return -threshold;  // Clamp to negative threshold
    return x;
}

float apply_echo(Echo* e, float x) {
    float delayed = e->buffer[e->index];    // Get delayed sample
    float y = x + delayed * e->decay;       // Apply decay to delayed sample

    e->buffer[e->index] = y;                // Store new sample in buffer
    e->index = (e->index + 1) % e->size;    // Increment index circularly

    return y;
}

float apply_reverb(Reverb* r, float x) {
    float delayed = r->buffer[r->index];                // Get delayed sample
    float y = x * (1.0f - r->mix) + delayed * r->mix;   // Mix input with delayed sample

    r->buffer[r->index] = x + delayed * r->feedback;    // Store new sample in buffer
    r->index = (r->index + 1) % r->size;                // Increment index circularly

    return y;
}

// Linear interpolation helper
static float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

// Process one sample with pitch shifting down
float apply_pitchshifter(PitchShifter* ps, float input) {
    ps->buffer[ps->write_index] = input;                    // Store input sample in buffer 

    // Calculate read index
    float output = 0.0f;                                    // Read sample at slower rate for pitch down
    int idx1 = (int)ps->read_index;                         // Get integer part of read index
    int idx2 = (idx1 + 1) % ps->size;                       // Get next index circularly
    float frac = ps->read_index - idx1;                     // Fractional part for interpolation

    // Linear interpolate between two samples
    output = lerp(ps->buffer[idx1], ps->buffer[idx2], frac);

    // Increment write index
    ps->write_index = (ps->write_index + 1) % ps->size;

    // Increment read index slower for pitch down
    ps->read_index += ps->pitch_factor;
    if (ps->read_index >= ps->size) {
        ps->read_index -= ps->size;
    }

    return output;
}

// =================================================================================================================================
//                                                          Custom Filters Section
// =================================================================================================================================

// ---------------------------
// Custom Filters Initialization
// ---------------------------

void equalizer_init(Equalizer* eq,
                    float low_gain, float mid_gain, float high_gain,
                    float low_cutoff, float high_cutoff,
                    float sample_rate) {
    eq->low_gain = low_gain;    // Set gains for each band
    eq->mid_gain = mid_gain;    // Mid band gain
    eq->high_gain = high_gain;  // High band gain

    // Initialize filters
    lpf_init(&eq->lpf, low_cutoff, sample_rate);        // Low band
    hpf_init(&eq->hpf, high_cutoff, sample_rate);       // High band

    hpf_init(&eq->mid_hpf, low_cutoff, sample_rate);    // Mid band = Bandpass (HPF + LPF)
    lpf_init(&eq->mid_lpf, high_cutoff, sample_rate);   // Mid band = Bandpass (HPF + LPF)
}

void darthvader_init(DarthVader* dv, float low_gain, float mid_gain, float high_gain, 
                     float low_cutoff, float high_cutoff,
                     float pitch_factor, 
                     float reverb_delay_ms, float reverb_feedback,float reverb_mix, 
                     float distortion_threshold,
                     float volume_gain,
                     float sample_rate) {

    dv->distortion_threshold = distortion_threshold; // Set distortion threshold
    dv->volume_gain = volume_gain;                   // Set volume gain

    equalizer_init(&dv->eq, low_gain, mid_gain, high_gain, low_cutoff, high_cutoff, sample_rate);
    pitchshifter_init(&dv->ps, pitch_factor, sample_rate);
    reverb_init(&dv->reverb, reverb_delay_ms, reverb_feedback, reverb_mix, sample_rate);
}

// ---------------------------
// Custom Filter Apply Functions
// ---------------------------

float apply_equalizer(Equalizer* eq, float x) {
    // Low band: low-pass only
    float low = apply_lpf(&eq->lpf, x);

    // High band: high-pass only
    float high = apply_hpf(&eq->hpf, x);

    // Mid band: band-pass (HPF followed by LPF)
    float mid = apply_hpf(&eq->mid_hpf, x);
    mid = apply_lpf(&eq->mid_lpf, mid);

    // Apply gain
    return low * eq->low_gain + mid * eq->mid_gain + high * eq->high_gain;
}

// Function to increase volume of a single audio sample
float apply_volume_gain(float sample, float gain) {
    // Apply gain to the sample
    float amplified = sample * gain;

    // Clamp the result to the normalized range [-1.0f, 1.0f] to avoid clipping
    if (amplified > 1.0f) {
        amplified = 1.0f;
    } else if (amplified < -1.0f) {
        amplified = -1.0f;
    }

    return amplified;
}

float apply_darthvader(DarthVader* dv, float x) {

    x = apply_pitchshifter(&dv->ps, x);                 // Apply pitch shifter
    x = apply_distortion(x, dv->distortion_threshold);  // Apply distortion
	x = apply_equalizer(&dv->eq, x);                    // then equalizer
	x = apply_reverb(&dv->reverb, x);                   // reverb
    x = apply_volume_gain(x, dv->volume_gain);          // volume gain last

    return x;
}

