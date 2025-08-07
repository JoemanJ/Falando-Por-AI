/*
 * audio_processing.c
 *
 *  Created on: Jul 17, 2025
 *      Author: diogo
 */

#include "audio_processing.h"
#include "filters.h"

uint16_t IN_BUFFER[BUFFER_SIZE] = {0};
int16_t OUT_BUFFER[BUFFER_SIZE*2] = {0};

uint16_t* process_in_buffer = &(IN_BUFFER[BUFFER_SIZE/2]);
int16_t* process_out_buffer = &(OUT_BUFFER[BUFFER_SIZE]);

float samplerate = 48000;
LPF lpf;
HPF hpf;
Reverb optimus_prime;
DarthVader dv;
PitchShifter hps, lps;
inline void filters_init()
{
	//DarthVader
	darthvader_init(&dv,
	                        1.5f,  // low_gain
	                        0.5f,  // mid_gain
	                        0.5f,  // high_gain
	                        200.0f, // low_cutoff
	                        5000.0f, // high_cutoff
	                        0.80f,   // pitch_factor
	                        20.0f,  // reverb_delay_ms
	                        0.15f,   // reverb_feedback
	                        0.35f,   // reverb_mix
	                        0.3f,   // distortion_threshold
	                        1.5f,   // volume_gain
							samplerate);

	// Filtro OPTIMUS PRIME (soh esse reverb msm)
	float r_delayms = 100;
	float r_feedback = 0.8;
	float r_mix = 0.5;
	reverb_init(&optimus_prime, r_delayms, r_feedback, r_mix, samplerate);

	//
	float lpf_cutoff = 2000;
	lpf_init(&lpf, lpf_cutoff, samplerate);

	float hpf_cutoff = 500;
	hpf_init(&hpf, hpf_cutoff, samplerate);

	float pitch_factor_high = 1.3;
	float pitch_factor_low = 0.7;
	pitchshifter_init(&hps, pitch_factor_high, samplerate);
	pitchshifter_init(&lps, pitch_factor_low, samplerate);
}

inline uint16_t clamp(uint16_t min, uint16_t x, uint16_t max)
{
	x = x<min? min: x;
	x = x>max? max: x;
	return x;
}

/*
	Receives a 12 bits unsigned audio input between 310 and 2792 and maps it
	into a float value between -1.0 and 1.0
*/
inline float normalizeAudio(uint16_t input)
{
//	clamp values to the working range
	input = clamp(310, input, 2792);
	return (input-1551)/(1241.0f);

//	return (input-2048)/4096.0;
}

/*
 * Receives a normalized audio input between -1.0 and 1.0 and converts it into
 * a signed 32bits uint32_t value, left-justified.
 */
inline int16_t deNormalizeAudio(float input)
{
	    input *= ((double) 32767.0f); //Maximum positive 16-bit value
	    return (int16_t) input;
}

/*
 * TODO
 */
void processHalfBuffer()
{
	uint16_t int_input = 0;
	float normalized_input = 0.0;
	float normalized_output = 0.0;
	int16_t int_output = 0;

	uint16_t i = 0;
	for (i=0; i<BUFFER_SIZE/2; i++)
	{
		int_input = process_in_buffer[i];
		normalized_input = normalizeAudio(int_input);
		normalized_output = processAudio(normalized_input) * OUTPUT_VOLUME; // COLOCAR EFEITO AQUI
		int_output = deNormalizeAudio(normalized_output) ;
		process_out_buffer[2*i] = int_output;
		process_out_buffer[2*i+1] = int_output;
	}

	return;
}

/*
 * Placeholder audio processing function that applies no effect
 */
float processAudio(float input)
{
	switch (CURRENT_FILTER)
	{
		case DARTH_VADER:
			return apply_darthvader(&dv, input);
			break;

		case OPTIMUS_PRIME:
			return apply_reverb(&optimus_prime, input);
			break;

		case LOW_PASS:
			return apply_lpf(&lpf, input);
			break;

		case HIGH_PASS:
			return apply_hpf(&hpf, input);
			break;

		case LOW_PITCH:
			return apply_pitchshifter(&lps, input);
			break;

		case HIGH_PITCH:
			return apply_pitchshifter(&hps, input);
			break;

		default:
			CURRENT_FILTER = 0;
			return processAudio(input);

//		case REVERB:
//			return apply_reverb(&optimus_prime, input);
//			break;
//
//		case DISTORTION:
//			return apply_distortion(&distortion, input);
//			break;
//
//		case ECHO:
//			return apply_echo(&echo, input);
//			break;
	}
}
