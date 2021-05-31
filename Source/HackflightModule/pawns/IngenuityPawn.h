/*
* Class declaration for Ingenuity pawn class using Hackflight flight manager
*
* Copyright (C) 2021 Simon D. Levy
*
* MIT License
*/

#pragma once

#include <mixers/coaxial.hpp>

#include <rft_motors/mockrotary.hpp>
#include <rft_motors/mockservo.hpp>

#include "../../MainModule/vehicles/multirotors/Ingenuity.hpp"

#include "../HackflightFlightManager.hpp"

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "IngenuityPawn.generated.h"

UCLASS(Config=Game)
class HACKFLIGHTMODULE_API AIngenuityPawn : public APawn {

    private:

        GENERATED_BODY()

        // Helper class
        Ingenuity _ingenuity;

        FHackflightFlightManager * _flightManager = NULL;

        rft::MockRotaryMotor _rotor1;
        rft::MockRotaryMotor _rotor2;
        rft::MockServoMotor _servo1;
        rft::MockServoMotor _servo2;

        hf::CoaxialMixer  _mixer = hf::CoaxialMixer(&_rotor1, &_rotor2, &_servo1, &_servo2);

    protected:

        // AActor overrides

        virtual void BeginPlay() override;

        virtual void Tick(float DeltaSeconds) override;

        virtual void PostInitializeComponents() override;

        virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

        // virtual void NotifyHit(...) override

    public:	

        AIngenuityPawn();

}; // IngenuityPawn