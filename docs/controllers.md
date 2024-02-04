# Controllers
Under `frc.robot.subsystems.oi` is the pseudo-singleton class `ControlManager`, and the interface `Controller`.

This document explains the purpose and implementation of `ControlManager`, along with how to utilize the `Controller` interface to implement custom controllers, and also how to extend operator inputs.

## Primer
> If you're already familiar with the "Inverted Joystick Incident" of February 1st, feel free to skip most of this section.

`frc.robot.subsystems.oi` was introduced after the team started using multiple controllers, of different types (joystick vs. console), with the same robot and codebase, while still hard-coding constants for operator interfacing.

This resulted in:
* Broken controls due to differences in button mappings (i.e. button 1 on one controller would be different from button 1 on another).
* Uncomfortable controls due to differences in sensitivity.
* Unreliable controls due to differences in noise (and thus different requirements for deadband).

Because programmers (and robotics students in general) are better at technical engineering (i.e. creating an overcomplicated abstraction layer) rather than social mediation (i.e. agreeing to just use the same controllers), the `frc.robot.subsystems.oi` module was implemented.

Specifically, the module provides a framework for defining separate and distinct controllers, and switching between them as necessary, while still providing a singular output API, that is accessible to other code, such as  `SwerveJoystickDefaultCmd`.

## Introduction to HIDs
Hardware input devices (HIDs) are devices plugged (usually over USB) into the personal computer (that is running the driver station), which function as controllers.
They include joysticks and console controllers, along with other input devices, which may be used to control the robot.

HIDs can be uniquely identified by the driver station, based on low-level metadata information provided by the device firmware.
Once identified, the driver station "remembers" the HID, and assigns it a unique integer number.

> This assignment is visible and editable in the driver station application, under the tap with the USB icon on it.

The number assigned to the HID is unsigned (never negative), and the range of numbers available starts from 0.
It is also called the "port index," being the identifier for the virtual "port" the HID is connected to. 

HIDs are represented in code with the `GenericHID` class, provided by WPILib.
`Joystick` and `XboxController` are child classes of `GenericHID`.

## Operator vs. Driver vs. Gunner
For the sake of clarity for the remainder of this document:

* An operator is someone who is controlling the robot.
* A driver is an operator that specifically controls the movement of the robot.
* A gunner is an operator that specifically controls the other actions of the robot (i.e. arm, end-effector, intake, climb).

A drive team is a collection of:
* 1 driver.
* 1 gunner.
* 1 coach.
* 1 technician.

Coaches and technicians are not operators, nor are they drivers or gunners.
They handle coordination and technical support respectively, and are not in direct control of the robot.

## ControlManager
`ControlManager` is a class that contains a collection of static child classes, fields, and methods.

> It is never instantiated anywhere, and does not need to be instantiated anywhere.

As is obvious by its name, it manages the controllers available to the operators.

### Registry
The `ControlManager` has an internal `HashMap<Integer, Controller>`, which is a registry for all available `Controller`s, by their port index.

To tell the `ControlManager` about the existence of an available `Controller`, call the method `.registerController()` with the `Controller`.

If by accident, multiple `Controller`s are using the same port index, assuming the WPILib framework does not crash (it should and probably will), a warning will be printed to the console, and the previous `Controller` registered with the port index will be overwritten.

```text
Achtung! Handler for OI overwrote controller registered for port 0.
```

You should never deliberately assign multiple `Controller`s to one port.

### Active Controllers
The `ControlManager` tracks what port should be used to access the active `Controller`, for both the driver and gunner, by the port index.

The get-and-set data-flow pattern is used to manage what the active `Controller` should be.
Safety features are implemented to prevent the activation of `Controller`s that don't exist or are already being used by the other operator.

Only `Controller`s that are initialized and registered with the `ControlManager`'s registry can be activated.

### Outputs
The `ControlManager` has an exposed nested class, called `Outputs`, which should contain public variables that control the robot, based on the influence of operator input.

> It serves as the output API, that can be referenced by other parts of the codebase.

`Outputs` is updated by `ControlManager` when `.processDriver()` and `.processGunner()` are called.

### Processing Methods
`.processDriver()` and `.processGunner()` are methods which process inputs from the active `Controller`s for the driver and gunner respectively.
They should be called from a periodic function, preferrably `teleopPeriodic()` in `Robot`.

These methods each get their active `Controller` from the registry by port index, and then use methods defined by the `Controller` to access input values.
These values are then processed and finally used to update `Outputs`.

If a `Controller` is missing for either of the operators on the drive team, then the respective processing method will not update `Outputs`.
A warning message will be printed to the console.

```text
Achtung! Failed to get driver controller!
```

```text
Achtung! Failed to get gunner controller!
```

In the conditional block for handling missing `Controller`s, there should be additional code for safely stopping subsystems controlled by the operators, such as the drivetrain.

### Buttons
When a `Controller` is registered with the `ControlManager`, a series of `Trigger` objects are created.

> The `Trigger` class is provided by WPILib, and has methods such as `.onTrue()`, which allow API consumers to execute a `Command` upon a boolean condition changing.
>
> These `Command`s are executed by the `CommandScheduler`, inside one of its event loops specifically set aside for handling HIDs.
>
> `Trigger` also has child classes, including the perhaps more-familiar `JoystickButton` class.

These objects are used by `ControlManager` to implement button bindings.

```text
--------------    ------------------
|Binding     |    |                |
|- Button ID | => | ControlManager | ==\\ 
|- Command   |    |                |   ||
--------------    ------------------   \/

-----------------------------------------
Controllers

For each `Controller`:
1. Create `Trigger` object that activates,
   when the given button (by ID) on the `Controller` is pressed.
2. Bind a proxy `Command` to the button using the `Trigger`.
3. In the proxy `Command`...
   - If the `Controller` which provided the button to the `Trigger`,
     is active according to `ControlManager`,
     then schedule the original `Command`.
   - If the `Controller` which provided the button to the `Trigger`,
     is not active according to `ControlManager`,
     then do nothing.
   - `Command`s are separated into driver and gunner categories;
     the check for whether the parent `Controller` is active,
     will consider whether the `Command` matches the `Controller`.
------------------------------------------ 
```

In the `.registerController()` method, there are calls to the internal `.makeTriggerForButton()` method, which creates the `Trigger` objects and the proxy `Command`s.

The `Command`s behind the proxy `Command`s are defined in two exposed nested classes, called `DriverButtonCommands` and `GunnerButtonCommands`.
They are all set to be empty `InstantCommand`s by default; upon robot initialization, when all relevant subsystems have been initialized, the fields for `Command`s can be overwritten.

### Utility Methods
There are additional utility methods under `ControlManager`, which may be useful for implementing dashboard controls.

## Controllers
`Controller` is an interface.

It defines common methods which provide values, used by the processing methods under `ControlManager`, to update `Outputs`, and the `ControlManager` itself to perform its custodial work in managing the operator interface.

> To some extent, the `Controller` interface is a compatibility layer, for different implementations of operator controls, based off of the underlying `GenericHID` and the hardware it connects to. 

`Controller`s can be declared to be either configured for use by the driver, gunner, or both.
This is done so through overriding the method `.getIntendedUser()` in implementations of the `Controller` interface.

### Pre-Processing
`Controller`s should implement deadband, sensitivity scaling, and other adjustment code, on their own.
This is because these pre-processing features are dependent on the underlying HID.

The processing methods of `ControlManager` only handle the conversion of driver inputs to usable values (i.e. turning a decimal from -1 to 1, to velocity in meters per second).

### Creating Your Own Controllers
To create your own `Controller`:

1. Create a class.
2. State that the class `implements` the `Controller` interface.
3. Define a constant under the class, which states what (not already occupied) port the `Controller` uses.
    - This should correspond to the assignment of the HID; if the HID is assigned to port 0 by the driver station, then the constant should be 0.
4. Define the `GenericHID` or child of `GenericHID`, which represents the underlying HID.
5. Define a constructor where the `GenericHID` is initialized with the defined port index.
6. Define `.getPort()`, which should return the port index defined in step 3.
7. Define `.getGenericHID()`, which should return the `GenericHID` defined in step 4.
    - Children of `GenericHID` may be returned.
8. Define `.getIntendedUser()`.
    - If the `Controller` has method implementations and bindings for the driver, return an integer less than 0.
    - If the `Controller` has method implementations and bindings for the gunner, return an integer greater than 0.
    - If the `Controller` has method implementations and bindings for both the driver and gunner, return 0.
9. Respective driver-side and gunner-side method implementations and bindings should be defined, as documented in the `Controller` interface.
10. If a `Controller` only specializes in controls for the driver or gunner, method implementations and bindings for the other operator can be left as their default implementation.
11. For bindings, their implementations end in "button," and should return a integer button ID, for the button that the described function should be bound to.
12. Instantiate your `Controller` in `RobotContainer`, specifically after `DriverButtonCommands` and `GunnerButtonCommands` are overwritten.
13. Register your `Controller`.

## Operator Inputs
Operator inputs are values that the operators can influence, through use of HIDs, as defined by the `Controller` interface.

### Adding Numerical and Boolean Values
Numerical values include analog outputs, from joysticks, triggers, and other interfaces.

Boolean values include digital outputs, from buttons and other interfaces.

> Boolean values that trigger persistent effects (i.e. `Command`s) should generally be implemented using button bindings, instead of being harvested directly from the `GenericHID` (or `GenericHID` child) class.

To add a new numerical or boolean value as an operator input:

1. Add a field to `Outputs` under `ControlManager`.
2. Add a method under `Controller` with a default implementation.
    - The default implementation should return a safe value, that, should the `Controller` be used for the operator input, the robot will not behave in a way that jeopardizes safety, or performance.
    - Document whether the operator input is driver-side or gunner-side.
3. Add processing to `.processDriver()` or `.processGunner()`.
    - This should update `Outputs` based on the method in `Controller`.
    - If the operator input is driver-side, define the processing in `.processDriver()`, and vice versa if the operator input is gunner-side.
    - Include safeties if no `Controller` is available.
4. Update any existing `Controller`s as needed.
    - If a `Controller` is not used for the operator input (i.e. the operator input is driver-side, but the `Controller` is gunner-side), then no changes are needed.

### Adding Button Bindings
Button bindings are the linkage of `Command`s to the pressing of buttons on HIDs.

To add a new button binding as an operator input:

1. Add a field to `DriverButtonCommands` or `GunnerButtonCommands` for the `Command`.
    - If the operator input is driver-side, define the field in `DriverButtonCommands`, and vice versa if the operator input is gunner-side.
    - Set the field to `emptyButtonCommand` by default.
2. Add a method under `Controller` with a default implementation.
    - The default implementation should return -1, which disables the binding.
    - Document whether the operator input is driver-side or gunner-side.
3. Add a call to `.makeTriggerForButton()`.
    - Reference the method under `Controller` defined in step 2, for the method arguments.
    - Reference the field under `DriverButtonCommands` or `GunnerButtonCommands` defined in step 1, for the method arguments.
    - If the field defined in step 1 is under `DriverButtonCommands`, set the final argument to true, otherwise set it to false.
4. Overwrite the definition of the field defined in step 1 in `RobotContainer`.
    - It should come before any `Controller`s being registered.
5. Update any existing `Controller`s as needed.
    - If a `Controller` is not used for the operator input (i.e. the operator input is driver-side, but the `Controller` is gunner-side), then no changes are needed.