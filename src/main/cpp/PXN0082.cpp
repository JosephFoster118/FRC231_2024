#include "PXN0082.h"


PXN0082::PXN0082(int port):
    frc::GenericHID(port)
{

}

bool PXN0082::getLB()
{
    if(xbox_mode)
    {
        return GetRawButton(5);
    }
    else
    {
        return GetRawButton(5);
    }
    
}

bool PXN0082::getLT()
{
    if(xbox_mode)
    {
        return GetRawAxis(2) > 0.9;
    }
    else
    {
        return GetRawButton(7);
    }
}

bool PXN0082::getX()
{
    if(xbox_mode)
    {
        return GetRawButton(3);
    }
    else
    {
        return GetRawButton(1);
    }
}

bool PXN0082::getA()
{
    if(xbox_mode)
    {
        return GetRawButton(1);
    }
    else
    {
        return GetRawButton(2);
    }
}

bool PXN0082::getY()
{
    if(xbox_mode)
    {
        return GetRawButton(4);
    }
    else
    {
        return GetRawButton(4);
    }
}

bool PXN0082::getB()
{
    if(xbox_mode)
    {
        return GetRawButton(2);
    }
    else
    {
        return GetRawButton(3);
    }
}

bool PXN0082::getRB()
{
    if(xbox_mode)
    {
        return GetRawButton(6);
    }
    else
    {
        return GetRawButton(6);
    }
}

bool PXN0082::getRT()
{
    if(xbox_mode)
    {
        return GetRawAxis(3) > 0.9;
    }
    else
    {
        return GetRawButton(8);
    }
}

int PXN0082::getPOV()
{
    return GetPOV();
}

void PXN0082::setIsXbox(bool xbox)
{
    xbox_mode = xbox;
}
