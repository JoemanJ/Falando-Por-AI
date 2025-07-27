/*
 * audio_processing.h
 *
 *  Created on: Jul 17, 2025
 *      Author: diogo
 */

#define BUFFER_SIZE 1024

extern uint16_t IN_BUFFER;
extern int32_t OUT_BUFFER;

#ifndef INC_AUDIO_PROCESSING_H_
#define INC_AUDIO_PROCESSING_H_

// Clamps an int value 'X" between min and max
inline uint16_t clamp(uint16_t min, uint16_t x, uint16_t max);

/*
	Receives a 12 bits unsigned audio input between 310 and 2792 and maps it
	into a float value between -1.0 and 1.0
*/
inline float normalizeAudio(uint16_t input);

/*
 * Receives a normalized audio input between -1.0 and 1.0 and converts it into
 * a signed 24bits uint32_t value, left-justified.
 */
inline uint32_t deNormalizeAudio(float input);

/*
 * TODO
 */
void processHalfBuffer();

/*
 * Placeholder audio processing function that applies no effect
 */
float processAudio(uint16_t input);

#endif /* INC_AUDIO_PROCESSING_H_ */
