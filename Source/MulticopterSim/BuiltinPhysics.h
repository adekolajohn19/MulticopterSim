/*
 * BuiltinPhysics.h: Physics class using UE4 built-in physics 
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "Physics.h"
#include "VehiclePawn.h"
#include "Joystick.h"
#include "FlightManager.h"

// Math support
#define _USE_MATH_DEFINES
#include <math.h>


class BuiltinPhysics : public Physics {

    private:

        // XXX Can we replace these with an FVector ?
        float _eulerXPrev;
        float _eulerYPrev;
        float _eulerZPrev;

        FlightManager * _flightManager;

        FQuat getQuaternion(class AVehiclePawn * vehiclePawn);

        void computeForces(float deltaSeconds, TArray<float> motorValues, FVector & euler, FVector & rotationForce, FVector & translationForce);

        static constexpr float THRUST_FACTOR = 130.f;

        static float motorsToAngularForce(TArray<float> motorValues, uint8_t a, uint8_t b, uint8_t c, uint8_t d);

        static float sum(TArray<float> x);

    public:

        BuiltinPhysics(void);

        virtual void start(void) override;

        virtual TArray<float> update(float deltaSeconds, AVehiclePawn * vehiclePawn, class UStaticMeshComponent* vehicleMesh) override;

}; // BuiltinPhysics

