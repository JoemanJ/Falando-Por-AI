/*
 * audio_processing.h
 *
 *  Created on: Jul 17, 2025
 *      Author: diogo
 */

#ifndef INC_AUDIO_PROCESSING_H_
#define INC_AUDIO_PROCESSING_H_

#include "inttypes.h"

#define BUFFER_SIZE 1024
#define OUTPUT_VOLUME 1

extern uint16_t IN_BUFFER[BUFFER_SIZE];
extern int16_t OUT_BUFFER[BUFFER_SIZE*2];

extern uint16_t* process_in_buffer;
extern int16_t* process_out_buffer;

void filters_init();

// Clamps an int value 'X" between min and max
uint16_t clamp(uint16_t min, uint16_t x, uint16_t max);
/*
	Receives a 12 bits unsigned audio input between 310 and 2792 and maps it
	into a float value between -1.0 and 1.0
*/
float normalizeAudio(uint16_t input);

/*
 * Receives a normalized audio input between -1.0 and 1.0 and converts it into
 * a signed 24bits uint32_t value, left-justified.
 */
int16_t deNormalizeAudio(float input);


/*
 * TODO
 */
void processHalfBuffer();

/*
 * Placeholder audio processing function that applies no effect
 */
float processAudio(float input);

#endif /* INC_AUDIO_PROCESSING_H_ */
