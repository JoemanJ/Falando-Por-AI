/*
 * audio_processing.c
 *
 *  Created on: Jul 17, 2025
 *      Author: diogo
 */

#include "audio_processing.h"

uint16_t IN_BUFFER[BUFFER_SIZE] = {0};
int16_t OUT_BUFFER[BUFFER_SIZE*2] = {0};

uint16_t* process_in_buffer = &(IN_BUFFER[BUFFER_SIZE/2]);
int16_t* process_out_buffer = &(OUT_BUFFER[BUFFER_SIZE]);

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
		int_output = deNormalizeAudio(normalized_output);
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
	return input;
}
