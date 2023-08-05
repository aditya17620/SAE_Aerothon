# SAE_Aerothon
SAE Aerothon Autonomous Mission Profile

This repository contains all the code developed for the autonomous control of the UAV developed for SAE Aerothon.
The code uses the camera feed taken from the PiCam 2 and runs Image detection (Pretrained on custom dataset) using ML and creos references its accuracy using OPenCV.

Once the target is identified with certain confidence, The cordinated of the centre are then relayed to the Pixhawk Flight controller which then guided the copter in that direction.

After the UAV has reached on top of the target, Servos are actuated to release the payload, and returns back to land.
