# Code structure write-up

Full code available at https://github.com/apple-phi/idp. An explanation of the code structure, as required by the assignment. In general, the principles which guided the code structure were:

- **Encapsulation**: The project uses classes to encapsulate related data and functions. For example, the Robot class in `src/robot.h` encapsulates robot-related data and operations.
- **Modularity**: The code is divided into object-oriented classes and methods, each of which has a single responsibility.
- **Abstraction**: The code is written in such a way that the main loop is simple and easy to understand, with the details of the robot's behaviour abstracted away into methods.
- **Readability**: The code is written in a way that is easy to read and understand, with comments and descriptive variable names.
- **Use of Enums**: The project uses enums for better code readability and safety. For example, the Block_t enum in `src/robot.h` represents different types of blocks.
- **Namespace:** The project uses namespaces to avoid name collisions. For example, the Steering namespace in `src/steering.cpp`.
- **Use of Pointers:** The project uses pointers to objects to avoid copying large objects. For example, the robot object in `idp.ino` is a pointer to a Robot object.
- **Dependency Injection**: The project uses dependency injection to make the code more flexible and testable. For example, the Robot constructor in `src/robot.cpp` takes MotorPair and line_sensors as parameters.

## General strategy

The robot cycles through the following states, which are stored as an enum member of the `Robot` class:

```cpp
// robot.h
enum {
    NAVIGATE,
    ENTER_BLOCK_ZONE,
    GRAB,
    EXIT_BLOCK_ZONE,
    ENTER_DROP_ZONE,
    DROP_OFF,
    EXIT_DROP_ZONE
} deliveryTask = NAVIGATE;
```

The robot's behavior is determined by the current state, and the robot transitions between states based on the current state and the sensor readings. The robot's behavior is abstracted away into a series of methods, which are called in the main loop of the program.

## Entry-point: `idp.ino`

### Initialization

The `setup` function in `idp.ino` initializes the robot and its components, using a set of abstracted constructors and methods.

### Main loop

The main loop of the program is in `idp.ino`, the relevant excerpt of which is shown below:

```cpp
// idp.ino
void loop()
{
    if (startButton->pressed())
    {
        reset();
    }
    (*robot)
        .readSensors()
        .drive()
        .delayAndBlinkIfMoving(1000 * DT);
}
```

This simple loop abstracts out the robot's behavior into a series of chained method calls on the `robot` object. The `startButton` object is used to reset the robot's state when pressed. This design makes it easy to extend the robot's behavior by adding new methods to the `Robot` class.

## Navigation

The navigation system is pre-calculated using the Python `script/codegen.py` and is tested using Pytest with `test/test_codegen.py`. This script generates 2 different 2D arrays of integers, which iteratively tell the robot the next node and next direction it needs to reach to get to the destination. The robot uses this information to navigate to the destination.

## Robot components

The robot's components are encapsulated in classes, which are used to abstract away the details of the robot's behavior. The main components are:

- `Robot`: The main class that encapsulates the robot's behavior and state.
- `MotorPair`: A class that encapsulates the robot's wheel motors and provides an interface for controlling them.
- `LineSensors`: A class that encapsulates the robot's line sensors and provides an interface for reading their values. It encodes their values as a single binary number using bit-shift combination.
- `Steering`: A namespace that encapsulates the robot's steering behavior and provides methods for controlling the robot's steering.
- `Led`: A class that encapsulates the robot's LED and provides an interface for controlling it.
- `src/control.h`: the `PID` and `FirstOrderFilter` classes are used to control the robot's speed and steering using PID control and first-order filtering on the derivative term.
