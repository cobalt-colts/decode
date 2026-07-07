# Cobalt Colts DECODE Game Code
This repository contains the code for FTC Team 6547's robot code for the 2025-2026 season.

## Our Stack
We use the following libraries:
 - PedroPathing for driving in TeleOp and path following in Autonomous
 - NextFTC for declarative actions management ("command based")
 - Sloth for fast pushing
 - SolversLib for motor control.

## Repository outline
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:
 - Contains all robot code
 - `auto/`
   - contains all autonomous routines and variants 
 - `pedroPathing/`
   - contains pedropathing tuning and configuration files
 - `teleop/`
   - contains teleop command declarations, teleop variants
   - `util/`
     - `paths/`
       - contains pathing files for autonomous
     - `subsystems.java`
       - contains all subsystems and commands for the entire robot
     - `ll.java`
       - contains limelight camera functions
     - rest of the files are various tuning and constant files