#include "FlightManager.hpp"
#include "../MainModule/Utils.hpp"

#include <SDL.h>

static const char * LIBRARY_FILENAME = "hackflight_lib.dll";

FRustFlightManager::FRustFlightManager(APawn * pawn, Dynamics * dynamics)
    : FFlightManager(dynamics)
{
    _dynamics = dynamics;

    void * library_handle = SDL_LoadObject(LIBRARY_FILENAME);

    _run_hackflight =
        (hackflight_fun_t)SDL_LoadFunction(library_handle, "rust_run_hackflight");

    _joystick = new IJoystick();
}

FRustFlightManager::~FRustFlightManager()
{
}

float FRustFlightManager::scaleAxis(float value)
{
    return value * 670;
}

void FRustFlightManager::getMotors(double time, double* values)
{
    (void)time;

    double joyvals[10] = {};

    _joystick->poll(joyvals);

    demands_t demands = {
        (joyvals[0] + 1) / 2, // throttle [-1,+1] => [0,1]
        scaleAxis(joyvals[1]),
        scaleAxis(joyvals[2]),
        scaleAxis(joyvals[3]) 
    };

    static alt_hold_t _pid;

    hackflight_t hackflight = {demands, _dynamics->vstate, _pid};

    hackflight_t new_hackflight = _run_hackflight(&hackflight);

    motors_t motors = new_hackflight.motors;

    values[0] = motors.m1;
    values[1] = motors.m2;
    values[2] = motors.m3;
    values[3] = motors.m4;

    memcpy(&_pid, &new_hackflight.alt_hold_pid, sizeof(alt_hold_t));
}

void FRustFlightManager::tick(void)
{
}
