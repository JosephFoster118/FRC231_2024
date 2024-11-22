#pragma once

#include <frc/GenericHID.h>

class PXN0082: public frc::GenericHID
{
public:
    PXN0082(int port);

    bool getLB();
    bool getLT();
    bool getX();
    bool getA();
    bool getY();
    bool getB();
    bool getRB();
    bool getRT();
    int getPOV();
    void setIsXbox(bool xbox);

private:
    bool xbox_mode = true;
};
