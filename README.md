[![CI](https://github.com/nerdherd/Crescendo2024/actions/workflows/main.yml/badge.svg)](https://github.com/nerdherd/Crescendo2024/actions/workflows/main.yml)

# Crescendo2024

FRC Team 687's code for its Crescendo 2024 competition robot, *Abruticus*.

## How to Use

This program requires WPILib VSCode 2024 and FRC Game Tools 2024 to be installed.
Clone this repository from <https://github.com/nerdherd/Crescendo2024>,
open it in VSCode, and press shift + F5 to deploy it to the robot. 
Then use Driver Station to enable the robot.

This program uses two PS4 Controllers for operator input. Due to hardware issues, a wrapper class `BadPS4` is used to translate button inputs to typical PS4 inputs.

## Contributing

Please branch off from the `main` branch for any changes, and then submit a PR when finished. 
Changes will be merged into the main branch and tagged as a release.

## 3rd Party Libraries

This repository uses:
- [NavX 2024.1.0](https://dev.studica.com/releases/2024/NavX.json)
- [Pathplanner 2024.2.8](https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json)
- [CTRE Phoenix v5](https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json)
- [CTRE Phoenix v6](https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-latest.json)
- [REVLib 2024.2.0](https://software-metadata.revrobotics.com/REVLib-2024.json)
- [WPIlib New Commands Library](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html)
