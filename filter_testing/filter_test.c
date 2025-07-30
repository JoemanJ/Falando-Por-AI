#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"
#include "../Core/Inc/filters.h"

// ---------------------------
// Unified per-sample processor
// ---------------------------
float process_sample(float x) {
    static int initialized = 0;
    static DarthVader dv;

    if (!initialized) {
        float sample_rate = 44100.0f; // Or use the actual sampleRate variable
        darthvader_init(&dv, 
                        2.5f,  // low_gain
                        1.5f,  // mid_gain
                        1.5f,  // high_gain
                        200.0f, // low_cutoff
                        5000.0f, // high_cutoff
                        0.90f,   // pitch_factor
                        20.0f,  // reverb_delay_ms
                        0.15f,   // reverb_feedback
                        0.55f,   // reverb_mix
                        0.7f,   // distortion_threshold
                        2.5f,   // volume_gain
                        sample_rate);
        initialized = 1;
    }

    // Apply filters in order
    x = apply_darthvader(&dv, x);
    return x;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printf("Usage: %s input.wav output.wav\n", argv[0]);
        return 1;
    }

    // --- Load WAV (assumes 16-bit PCM) ---
    drwav_int16* pSampleData = NULL;
    drwav_uint64 totalSampleCount;
    unsigned int channels, sampleRate;

    drwav wav;
    if (!drwav_init_file(&wav, argv[1], NULL)) {
        printf("Failed to open WAV file.\n");
        return -1;
    }

    if (wav.bitsPerSample != 16) {
        printf("Only 16-bit PCM WAV files are supported.\n");
        drwav_uninit(&wav);
        return -1;
    }

    channels = wav.channels;
    sampleRate = wav.sampleRate;
    totalSampleCount = wav.totalPCMFrameCount * channels;

    pSampleData = malloc((size_t)totalSampleCount * sizeof(drwav_int16));
    if (!pSampleData) {
        drwav_uninit(&wav);
        printf("Memory allocation failed.\n");
        return -1;
    }

    drwav_read_pcm_frames_s16(&wav, wav.totalPCMFrameCount, pSampleData);
    drwav_uninit(&wav);

    // --- Process audio ---
    for (drwav_uint64 i = 0; i < totalSampleCount; ++i) {
        float x = pSampleData[i] / 32768.0f;  // Convert to float [-1.0, 1.0]
        
        x = process_sample(x);                // Apply filter (or none)
        
        x *= 32768.0f;                         // Convert back to int16 range

        // Clamp to avoid overflow
        if (x > 32767.0f) x = 32767.0f;
        if (x < -32768.0f) x = -32768.0f;
        pSampleData[i] = (drwav_int16)x;
    }

    // --- Save output WAV ---
    drwav_data_format format;
    format.container = drwav_container_riff;
    format.format = DR_WAVE_FORMAT_PCM;
    format.channels = channels;
    format.sampleRate = sampleRate;
    format.bitsPerSample = 16;

    drwav out_wav;
    if (!drwav_init_file_write(&out_wav, argv[2], &format, NULL)) {
        printf("Failed to open output file.\n");
        free(pSampleData);
        return -1;
    }

    drwav_write_pcm_frames(&out_wav, wav.totalPCMFrameCount, pSampleData);
    drwav_uninit(&out_wav);

    free(pSampleData);
    printf("WAV processed successfully.\n");
    return 0;
}
