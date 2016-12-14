#ifndef _DISCO_STICK_CORE_
#define _DISCO_STICK_CORE_

#include "Arduino.h"

class InputsTransform {
    private:
        int _adc_channel;
        int16_t _audio_capture[FFT_N];
        complex_t _butterfly_buf[FFT_N];
        uint16_t _spectrum[FFT_N];
        volatile byte _sample_index;
        int _column_levels[8][10];
        int _min_level_average[8];
        int _max_level_average[8];
        int _column_dividers[8];
        byte _spectral_energy[8];

        uint32_t _motion = 0;
        byte _led_count;
        byte _max_brightness;
        byte red = 0;
        byte blue = 0;
        byte green = 0;
        
        const byte ACCEL_I2C_ADDR = 0x1D;
        const byte ACCEL_INT1_PIN = 2;
        const byte ACCEL_INT2_PIN = 3;

        MMA8452Q _accel;
        LPD8806 _leds;

        void _init_audio();
        void _hsv_to_rgb(float h, float s, float v);
        void _process_audio();
        void _process_accel();
        unit32_t _color_sine(byte offset, float phase, byte weight);

    public:
        InputsTransform();
        void Step();

};

class DiscoCore {
    private:
        InputsTransform _inputs;
    public:

};

#endif //_DISCO_STICK_CORE_