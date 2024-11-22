#include "Lights.h"

Lights::Lights()
{
    lights.SetLength(NUM_LIGHTS);
    lights.Start();
    last_update_time = frc::Timer::GetFPGATimestamp().value();
}

void Lights::solid(uint8_t r, uint8_t g, uint8_t b)
{
    solid({r, g, b});
}

void Lights::solid(std::array<uint8_t, 3> color)
{
    mode = Mode::SOLID;
    solid_color = color;
}

void Lights::sweep(frc::AddressableLED::LEDData fg, frc::AddressableLED::LEDData bg, uint8_t sz, double sd)
{
    foreground = fg;
    background = bg;
    speed = sd;
    size = sz;
    mode = Mode::SWEEP;
}

void Lights::sweep(std::array<uint8_t, 3> fg, std::array<uint8_t, 3> bg, uint8_t sz, double sd)
{
    sweep(
        frc::AddressableLED::LEDData{fg[0], fg[1], fg[2]},
        frc::AddressableLED::LEDData{bg[0], bg[1], bg[2]},
        sz, 
        sd
    );
}

void Lights::jitter(frc::AddressableLED::LEDData fg, frc::AddressableLED::LEDData bg, uint8_t sz, double sd)
{
    foreground = fg;
    background = bg;
    speed = sd;
    size = sz;
    mode = Mode::JITTER;
}

void Lights::jitter(std::array<uint8_t, 3> fg, std::array<uint8_t, 3> bg, uint8_t sz, double sd)
{
    jitter(
        frc::AddressableLED::LEDData{fg[0], fg[1], fg[2]},
        frc::AddressableLED::LEDData{bg[0], bg[1], bg[2]},
        sz, 
        sd
    );
}

void Lights::updateLights()
{
    switch(mode)
    {
        case Mode::SOLID:
        {
            doSolid();
        }break;
        case Mode::SWEEP:
        {
            doSweep();
        }break;
        case Mode::JITTER:
        {
            doJitter();
        }break;
    }
}

void Lights::doSolid()
{
    std::vector<frc::AddressableLED::LEDData> data(NUM_LIGHTS);
    for(auto& led : data)
    {
        led.SetRGB(solid_color[0], 
                    solid_color[1], 
                    solid_color[2]);
    }

    lights.SetData(data);
}

void Lights::doSweep()
{
    double current_time = frc::Timer::GetFPGATimestamp().value();
    double delta_time = current_time - last_update_time;
    sweep_position += speed * delta_time;

    int whole_position = sweep_position;

    std::vector<frc::AddressableLED::LEDData> data(NUM_LIGHTS);
    for(auto& led : data)
    {
        led = background;
    }
    for(int i = 0; i < size; i++)
    {
        int index = (whole_position + i) % NUM_LIGHTS;
        data[index] = foreground;
    }
    last_update_time = current_time;

    lights.SetData(data);
}

void Lights::doJitter()
{
    double current_time = frc::Timer::GetFPGATimestamp().value();
    double delta_time = current_time - last_update_time;
    jitter_position += speed * delta_time;

    int whole_position = sweep_position;
    std::vector<frc::AddressableLED::LEDData> data(NUM_LIGHTS);

    for(int i = 0; i < NUM_LIGHTS; i++)
    {
        int offset = (i/size + whole_position) % 2;
        if(offset == 1)
        {
            data[i] = foreground;
        }
        else
        {
            data[i] = background;
        }
    }
    last_update_time = current_time;

    lights.SetData(data);
}
