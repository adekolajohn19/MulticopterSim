{--
  PID control for roll/pitch angular velocity

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module RatePid(rateController)

where

import Language.Copilot

import State(dphi, dtheta)
import Demands
import PidController
import Utils

windupMax = 6.0 :: SFloat
rateMaxDps = 40 :: SFloat

rateController :: SFloat -> SFloat -> SFloat -> PidFun

rateController kp ki kd (state, rxdemands, piddemands) =
  (state, rxdemands, piddemands')

  where

    piddemands' = Demands (throttle piddemands) rollDemand pitchDemand (yaw piddemands)

    update op dmdfun stfun dmd' err' errI' = 

      let err = op (dmdfun piddemands) (stfun state)
          errI = constrain_abs (err + errI') windupMax
          errD = (err - err')
          dmd = kp * err + ki * errI + kd * errD

      in (dmd, err, errI, errD)

    (rollDemand, rollError, rollErrorI, rollErrorD)  =
      update (-) roll dphi rollDemand' rollError' rollErrorI'

    -- Pitch demand is nose-down positive, so we negate pitch-forward
    -- vehicle state (nose-down negative) to reconcile them
    (pitchDemand, pitchError, pitchErrorI, pitchErrorD)  =
      update (+) pitch dtheta pitchDemand' pitchError' pitchErrorI'

    -- Controller state
    rollError' = [0] ++ rollError
    rollErrorI' = [0] ++ rollErrorI
    rollDemand' = [0] ++ rollDemand
    pitchError' = [0] ++ pitchError
    pitchErrorI' = [0] ++ pitchErrorI
    pitchDemand' = [0] ++ pitchDemand