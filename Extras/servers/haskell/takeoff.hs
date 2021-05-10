{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Types
import Multicopter
import AltitudeController

main :: IO ()
main = runMulticopter (altitudeController 10.0 1.0 1.0 0.0 10.0) quadXAPMixer