#pragma once

#include <chrono>
#include <array>
#include <vector>
#include <frc/AddressableLED.h>
#include <frc/Timer.h>

class Lights
{
public:
    Lights();
    Lights(const Lights&) = delete;
    using Color = std::array<uint8_t, 3>;
    static constexpr size_t NUM_LIGHTS{19};
    static constexpr Color GREEN{15, 255, 0};
    static constexpr Color ORANGE{255, 100, 0};
    static constexpr Color BLACK{0, 0, 0};
    static constexpr Color WHITE{255, 255, 255};
    static constexpr Color BLUE{0, 0, 255};
    static constexpr Color RED{255, 0, 0};

    enum class Mode
    {
        SOLID,
        SWEEP,
        JITTER
    };

    void updateLights();
    void solid(uint8_t r, uint8_t g, uint8_t b);
    void solid(Color color);
    void sweep(frc::AddressableLED::LEDData fg, frc::AddressableLED::LEDData bg, uint8_t sz, double sd);
    void sweep(Color fg, Color bg, uint8_t sz, double sd);
    void jitter(frc::AddressableLED::LEDData fg, frc::AddressableLED::LEDData bg, uint8_t sz, double sd);
    void jitter(Color fg, Color bg, uint8_t sz, double sd);

private:
    void doSolid();
    void doSweep();
    void doJitter();
    frc::AddressableLED lights{0};
    Mode mode{Mode::SOLID};
    Color solid_color{0, 0, 255};
    frc::AddressableLED::LEDData foreground;
    frc::AddressableLED::LEDData background;
    double speed;
    double last_update_time;
    double sweep_position{0.0};
    double jitter_position{0.0};
    uint8_t size;
};

