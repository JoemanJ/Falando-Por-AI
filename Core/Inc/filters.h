#ifndef FILTERS_H
#define FILTERS_H

#define MAX_DELAY_SAMPLES 960  // 1 sec max delay at 48kHz

// ======================================================================================================================
//                                                 BASE FILTERS SECTION
// ======================================================================================================================

// --------------------
// Base Filters Header
// --------------------
/**
 * Low-pass filter structure.
 */
typedef struct LPF {
    float alpha;
    float prev;
} LPF;

/**
 * High-pass filter structure.
 */
typedef struct HPF {
    float alpha;
    float prev_x;
    float prev_y;
} HPF;

/**
 * Echo effect structure.
 */
typedef struct Echo {
    float buffer[MAX_DELAY_SAMPLES];
    int size;
    int index;
    float decay;
    int delay_samples;
} Echo;

/**
 * Reverb effect structure.
 */
typedef struct Reverb {
    float buffer[MAX_DELAY_SAMPLES];
    int size;
    int index;
    float feedback;
    float mix;
} Reverb;

/**
 * Pitch shifter structure.
 */
typedef struct PitchShifter {
    float buffer[MAX_DELAY_SAMPLES];
    int write_index;
    float read_index;
    float pitch_factor; // <1.0 for pitch down, >1.0 for pitch up
    int size;
} PitchShifter;

// ---------------------------
// Filter initialization functions
// ---------------------------
/**
 * Initialize the low-pass filter.
 * @param f Pointer to LPF struct.
 * @param cutoff_freq Cutoff frequency in Hz.
 * @param sample_rate Sample rate of the audio.
 */
void lpf_init(LPF *f, float cutoff, float sample_rate);

/**
 * Initialize the high-pass filter.
 * @param f Pointer to HPF struct.
 * @param cutoff_freq Cutoff frequency in Hz.
 * @param sample_rate Sample rate of the audio.
 */
void hpf_init(HPF *f, float cutoff, float sample_rate);

/**
 * Initialize the echo effect.
 * @param e Pointer to Echo struct.
 * @param delay_ms Delay in milliseconds.
 * @param decay Decay factor for the echo.
 * @param sample_rate Sample rate of the audio.
 */
void echo_init(Echo* e, float delay_ms, float decay, float sample_rate);

/**
 * Initialize the reverb effect.
 * @param r Pointer to Reverb struct.
 * @param delay_ms Delay in milliseconds.
 * @param feedback Feedback amount for the reverb.
 * @param mix Mix amount for the reverb.
 * @param sample_rate Sample rate of the audio.
 */
void reverb_init(Reverb* r, float delay_ms, float feedback, float mix, float sample_rate);

/**
 * Initialize the pitch shifter.
 * @param ps Pointer to PitchShifter struct.
 * @param pitch_factor Factor for pitch shifting (e.g. 0.5 for down an octave).
 * @param sample_rate Sample rate of the audio.
 */
void pitchshifter_init(PitchShifter* ps, float pitch_factor, float sample_rate);

// ---------------------------
// Filter apply functions
// ---------------------------
float apply_lpf(LPF *f, float x);
float apply_hpf(HPF *f, float x);
float apply_distortion(float x, float threshold);
float apply_echo(Echo* e, float x);
float apply_reverb(Reverb* r, float x);
float apply_pitchshifter(PitchShifter* ps, float input);

// ======================================================================================================================
//                                                  CUSTOM FILTERS SECTION
// ======================================================================================================================

// --------------------
// Custom Filters Header
// --------------------

/**
 * Equalizer structure with low, mid, and high bands.
 */
typedef struct Equalizer {
    LPF lpf;   // Low-pass filter for low band
    HPF hpf;   // High-pass filter for high band
    HPF mid_hpf; // High-pass for mid band
    LPF mid_lpf; // Low-pass for mid band

    float low_gain;
    float mid_gain;
    float high_gain;
} Equalizer;

/**
 * Darth Vader filter structure combining multiple effects.
 */
typedef struct DarthVader {
    Equalizer eq;
    PitchShifter ps;
    Reverb reverb;
    float distortion_threshold;
    float volume_gain;
} DarthVader;

// ---------------------------
// Custom Filter initialization functions
// ---------------------------

/**
    * Initialize the equalizer with gains and cutoff frequencies. Equalizer has embedded filters low and high pass.
    * @param eq Pointer to Equalizer struct.
    * @param low_gain Gain for low band.
    * @param mid_gain Gain for mid band.
    * @param high_gain Gain for high band.
    * @param low_cutoff Cutoff frequency for low band.
    * @param high_cutoff Cutoff frequency for high band.
    * @param sample_rate Sample rate of the audio.
    */
void equalizer_init(Equalizer* eq,
                    float low_gain, float mid_gain, float high_gain,
                    float low_cutoff, float high_cutoff,
                    float sample_rate);

/**
 * Initialize the Darth Vader filter with various parameters.
 * @param dv Pointer to DarthVader struct.
 * @param low_gain Gain for low band.
 * @param mid_gain Gain for mid band.
 * @param high_gain Gain for high band.
 * @param low_cutoff Cutoff frequency for low band.
 * @param high_cutoff Cutoff frequency for high band.
 * @param pitch_factor Pitch shifting factor.
 * @param reverb_delay_ms Delay in milliseconds for reverb.
 * @param reverb_feedback Feedback amount for reverb.
 * @param reverb_mix Mix amount for reverb.
 * @param distortion_threshold Threshold for distortion.
 * @param volume_gain Gain for volume adjustment.
 * @param sample_rate Sample rate of the audio.
 * */
void darthvader_init(DarthVader* dv, float low_gain, float mid_gain, float high_gain,
                     float low_cutoff, float high_cutoff,
                     float pitch_factor, 
                     float reverb_delay_ms, float reverb_feedback, float reverb_mix, 
                     float distortion_threshold,
                     float volume_gain,
                     float sample_rate);

// ---------------------------
// Custom Filter apply functions
// ---------------------------

float apply_equalizer(Equalizer* eq, float x);
float apply_volume_gain(float sample, float gain);
float apply_darthvader(DarthVader* dv, float x);

#endif // FILTERS_H