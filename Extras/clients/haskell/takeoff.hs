{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Multicopter

controlFunc :: ControlFunc
controlFunc _ = [1,1,1,1]

main :: IO ()
main = runMulticopter controlFunc
