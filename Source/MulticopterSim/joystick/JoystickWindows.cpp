
/*
 * Windows implementation of joystick/gamepad support for flight controllers
 *
 * Copyright (C) 2018 Simon D. Levy
 *
 * MIT License
 */

#ifdef _WIN32

#include "Joystick.h"

#define WIN32_LEAN_AND_MEAN

#undef TEXT
#include <shlwapi.h>
#include "joystickapi.h"

static void getAxes(float axes[6], DWORD axis0, DWORD axis1, DWORD axis2, DWORD axis3)
{
	axes[0] = (float)axis0;
	axes[1] = (float)axis1;
	axes[2] = (float)axis2;
	axes[3] = (float)axis3;
}

static void getAxes(float axes[6], DWORD axis0, DWORD axis1, DWORD axis2, DWORD axis3, DWORD axis4)
{
	getAxes(axes, axis0, axis1, axis2, axis3);

    axes[4] = (float)axis4;
}



Joystick::Joystick(const char * devname)
{
    JOYCAPS joycaps = {0};

    _productId = 0;

    _isRcTransmitter = false;

    // Grab the first available joystick
    for (_joystickId=0; _joystickId<16; _joystickId++)
        if (joyGetDevCaps(_joystickId, &joycaps, sizeof(joycaps)) == JOYERR_NOERROR)
            break;

    if (_joystickId < 16) {

        _productId = joycaps.wPid;

        _isRcTransmitter = (_productId == PRODUCT_TARANIS || _productId == PRODUCT_SPEKTRUM);
    }
}

// Convert InterLink aux switches to unique gamepad buttons
static void getAuxInterlink(float * axes, uint8_t buttons, uint8_t aux1, uint8_t aux2, float auxMid)
{
	axes[aux1] = -1;
	axes[aux2] = (buttons & 0x01) ? -1 : 1;

	switch (buttons) {

	case 3:
	case 2:
		axes[aux1] = auxMid;
		break;

	case 19:
	case 18:
		axes[aux1] = 1;
	}
}


Joystick::error_t Joystick::pollProduct(float axes[6], uint8_t & buttons)
{
    JOYINFOEX joyState;
    joyState.dwSize=sizeof(joyState);
    joyState.dwFlags=JOY_RETURNALL | JOY_RETURNPOVCTS | JOY_RETURNCENTERED | JOY_USEDEADZONE;
    joyGetPosEx(_joystickId, &joyState);

    // axes: 0=Thr 1=Ael 2=Ele 3=Rud 4=Aux

    switch (_productId) {

        case PRODUCT_SPEKTRUM:
            getAxes(axes, joyState.dwYpos, joyState.dwZpos, joyState.dwVpos, joyState.dwXpos, joyState.dwUpos);
            break;

        case PRODUCT_TARANIS:
            getAxes(axes, joyState.dwXpos, joyState.dwYpos, joyState.dwZpos, joyState.dwVpos, joyState.dwRpos);
            break;

        case PRODUCT_PS3_CLONE:      
        case PRODUCT_PS4:
            getAxes(axes, joyState.dwYpos, joyState.dwZpos, joyState.dwRpos, joyState.dwXpos);
            break;

        case PRODUCT_F310:
            getAxes(axes, joyState.dwYpos, joyState.dwZpos, joyState.dwRpos, joyState.dwXpos);
            break;

        case PRODUCT_XBOX360:  
        case PRODUCT_XBOX360_CLONE:
        case PRODUCT_XBOX360_CLONE2:
            getAxes(axes, joyState.dwYpos, joyState.dwUpos, joyState.dwRpos, joyState.dwXpos);
            break;

        case PRODUCT_EXTREMEPRO3D:  
            getAxes(axes, joyState.dwZpos, joyState.dwXpos, joyState.dwYpos, joyState.dwRpos);
            break;

        case PRODUCT_INTERLINK:
            getAxes(axes, joyState.dwZpos, joyState.dwXpos, joyState.dwYpos, joyState.dwRpos);
			getAuxInterlink(axes, joyState.dwButtons, AX_AU1, AX_AU2, AUX1_MID);
            break;

        default:

            return _productId ? ERROR_PRODUCT : ERROR_MISSING;
    }

    // Normalize the axes to demands to [-1,+1]
    for (uint8_t k=0; k<4; ++k) {
        axes[k] = axes[k] / 32767 - 1;
    }

    buttons = joyState.dwButtons;

    return Joystick::ERROR_NOERROR;
}


#endif
