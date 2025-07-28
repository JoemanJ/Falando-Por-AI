/*
 * audio_processing.c
 *
 *  Created on: Jul 17, 2025
 *      Author: diogo
 */

#include "audio_processing.h"

uint16_t IN_BUFFER[BUFFER_SIZE] = {0};
int32_t OUT_BUFFER[BUFFER_SIZE*2] = {0};

uint16_t* process_in_buffer = &(IN_BUFFER[BUFFER_SIZE/2]);
int32_t* process_out_buffer = &(OUT_BUFFER[BUFFER_SIZE]);

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
inline int32_t deNormalizeAudio(float input)
{
//	    input *= ((double) 0x7FFFFFFF); //Maximum possible 32-bit value
//	    return (int32_t) input;

	// Clamp the input value to the valid range [-1.0, 1.0] to prevent overflow
	    if (input > 1.0f) {
	        input = 1.0f;
	    } else if (input < -1.0f) {
	        input = -1.0f;
	    }

	    // The maximum positive 24-bit signed value is 2^23 - 1 (8,388,607)
	    // The minimum negative 24-bit signed value is -2^23 (-8,388,608)
	    // We want to scale the float value [-1.0, 1.0] to this range.
	    // The full range of a 24-bit signed integer is 2^24.
	    // We can use 2^23 as a scaling factor for the positive range,
	    // and -2^23 for the negative range, or simply use 2^23 for the magnitude
	    // and preserve the sign.
	    // Multiplying by 2^23 (8388608.0f) will map -1.0 to -8388608 and 1.0 to 8388608.
	    // However, the maximum positive 24-bit signed value is 8388607.
	    // So, we should scale by (2^23 - 1) for the positive maximum.
	    // A simpler approach is to scale by 2^23 and then handle the maximum positive.

	    // Scaling factor for 23 bits of magnitude (excluding sign bit)
	    const float SCALE_FACTOR = (float)(1 << 23); // Equivalent to 8388608.0f

	    // Convert the float to a 24-bit signed integer value
	    // This will give a value in the range approx [-8388608, 8388608]
	    int32_t signed_24bit_val = (int32_t)(input * SCALE_FACTOR);

	    // Handle the positive maximum value case:
	    // If value was 1.0, signed_24bit_val might be 2^23 (8388608),
	    // but the max positive for 24-bit signed is 2^23 - 1 (8388607).
	    // So, if it's 2^23, cap it to 2^23 - 1.
	    // This effectively ensures that 1.0 maps to the largest positive 24-bit value.
	    if (signed_24bit_val == (1 << 23)) {
	        signed_24bit_val = (1 << 23) - 1;
	    }

	    // Left-justify the 24-bit value within the 32-bit frame.
	    // This means shifting it left by 8 bits (32 - 24 = 8).
	    // The 24-bit value will occupy bits 31 to 8.
	    return signed_24bit_val << 8;
}

/*
 * TODO
 */
void processHalfBuffer()
{
	uint16_t int_input = 0;
	float normalized_input = 0.0;
	float normalized_output = 0.0;
	int32_t int_output = 0;

	uint16_t i = 0;
	for (i=0; i<BUFFER_SIZE/2; i++)
	{
		int_input = process_in_buffer[i];
		normalized_input = normalizeAudio(int_input);
		normalized_output = processAudio(normalized_input) * AUDIO_VOLUME; // COLOCAR EFEITO AQUI
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
