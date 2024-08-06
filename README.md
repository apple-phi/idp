# IDP 2024

In the IDP autonomous robotics competition, teams start with raw materials, an Arduino and very limited sensors
to design and build a delivery robot to navigate a maze and sort through parcels to delivery them to the right locations.

> [!NOTE]
> We were the *only* team to achieve a 100% score!

The Cambridge University Engineering Department published a [YouTube video](https://www.youtube.com/watch?v=8PjCkTg_oGc)
of us accomplishing this.

## Software theory

There are a few main theoretical components of the robot software:

- a PID control loop with first-order derivative filtering
- precomputed pathfinding using the A* algorithm
- an event loop which executes tasks based on the robot and game state singletons

All the C++ code was intentionally structured using modern design patterns to be composable, object-oriented, and easy to understand.

## Prerequisites

- The `ArxContainer` library, which can be installed using the Arduino library manager.

## Dev notes

- Since this is an Arduino project, the main file must be named `idp.ino`, the same as the git folder.
- Also, the C++ standard library is not available, so we have to use libraries specifically compatible with Arduino.
- The `.vscode/` folder has been included with some settings that may be useful for development under VSCode.
Any files that are machine-specific have been excluded from the repository using `.gitignore`.

Here's a video we took while prototyping the robot:

[![IDP 2024 Competition Robot](https://img.youtube.com/vi/qrxxvx1YnWg/0.jpg)](https://www.youtube.com/watch?v=qrxxvx1YnWg "IDP 2024 Competition Robot")
