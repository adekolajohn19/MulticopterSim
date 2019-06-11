/*
* Class declaration for pawn class in MulticopterSim
*
* Copyright (C) 2019 Simon D. Levy, Daniel Katzav
*
* MIT License
*/

#pragma once

#include "vehicles/QuadXAP.hpp"
#include "GameFramework/Pawn.h"
#include "3DFly.generated.h"

UCLASS(Config=Game)
class MULTICOPTERSIM_API A3DFlyPawn : public APawn {

    private:

        GENERATED_BODY()

        static constexpr MultirotorDynamics::params_t _params = {

            // Estimated

            5.30E-07,               // b
            2.24E-06,               // d
            0.110,                  // m
            0.6,                    // l
            2,                      // Ix
            2,                      // Iy
            3,                      // Iz
            3.08E-04,               // Jr
            
            15000                  // maxrpm
        }; 

        static constexpr Vehicle::frame_t _frame = {

            0.0000,  // center X
            0.0000,  // center Y
           -0.0100,  // motor offset
            0.0375,  // width
            0.0375,  // length
            0.0050,  // motor Z
            0.0250   // propeller Z
        };

        QuadXAP * _vehicle;
        
    protected:

        // AActor overrides

        virtual void BeginPlay() override;

        virtual void Tick(float DeltaSeconds) override;

        virtual void PostInitializeComponents() override;

        virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

        // virtual void NotifyHit(...) override

    public:	

        A3DFlyPawn();

        ~A3DFlyPawn();

}; // A3DFlyPawn
