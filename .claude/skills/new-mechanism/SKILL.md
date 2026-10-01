---
name: new-mechanism
description: Create a new robot subsystem (arm, elevator, intake, shooter, climber...) on top of the Demacia StateBaseMechanism / BaseMechanism classes, with a state enum, constants, default command, optional calibration, and button bindings in RobotContainer. Use when the user asks to add or scaffold a mechanism or subsystem.
---

# New mechanism

Before starting, read `.claude/rules/mechanisms.md` and `.claude/rules/hardware.md` if they are not already loaded.

## 1. Ask or decide

- Name, motors (type, CAN ID, CAN bus, gear ratio, units), sensors (type, channel).
- Control mode per motor: `ANGLE` (rotating joint), `MOTION` (linear position with a profile), `VELOCITY` (rollers, shooters), `VOLTAGE` / `DUTYCYCLE` (simple open loop).
- The states and the target value of each motor in each state.
- Whether it needs calibration (zeroing against a limit switch).

Don't invent CAN IDs, ports or gear ratios. If they are unknown, use clearly named constants with a `// TODO: real value` comment and tell the user.

## 2. Create the files

Create them in `src/main/java/frc/robot/subsystems/<name>/`. This example (an arm with one TalonFX and a limit switch) compiles against the current library:

`ArmState.java`
```java
package frc.robot.subsystems.arm;

import frc.demacia.utils.mechanisms.StateBaseMechanism.MechanismState;

/** One target value per motor, in the same order as the motor array. Units: radians. */
public enum ArmState implements MechanismState {
    HOME(0.0),
    INTAKE(Math.toRadians(-10)),
    SCORE(Math.toRadians(80));

    private final double[] values;

    ArmState(double... values) {
        this.values = values;
    }

    @Override
    public double[] getValues() {
        return values;
    }
}
```

`ArmConstants.java`
```java
package frc.robot.subsystems.arm;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public final class ArmConstants {
    public static final String NAME = "Arm";
    public static final String MOTOR_NAME = "Arm Motor";

    public static final TalonFXConfig MOTOR_CONFIG = new TalonFXConfig(12, Canbus.Rio, MOTOR_NAME)
            .withRadiansMotor(50.0)                    // 50:1 gearbox → radians
            .withPID(8, 0, 0.1, 0.2, 0, 0, 0.35)       // kP kI kD kS kV kA kG
            .withMotionParam(4, 8, 40)                 // rad/s, rad/s², rad/s³ — required for ANGLE/MOTION
            .withCurrent(40)
            .withBrake(true);

    public static final LimitSwitchConfig LIMIT_CONFIG = new LimitSwitchConfig(0, "Arm Limit");

    public static final double CALIBRATION_POWER = -0.15;
    public static final double CALIBRATION_POSITION = Math.toRadians(-20); // angle at the limit switch

    private ArmConstants() {}
}
```

`Arm.java`
```java
package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.mechanisms.CalibratinCommand;
import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.SensorInterface;

public class Arm extends StateBaseMechanism {
    private final LimitSwitch limitSwitch;

    public Arm() {
        this(new TalonFXMotor(ArmConstants.MOTOR_CONFIG), new LimitSwitch(ArmConstants.LIMIT_CONFIG));
    }

    private Arm(TalonFXMotor motor, LimitSwitch limitSwitch) {
        super(ArmConstants.NAME, new MotorInterface[] { motor }, new SensorInterface[] { limitSwitch }, ArmState.class);
        this.limitSwitch = limitSwitch;
        setPositionMechanism();   // IDLE holds the current angle instead of going to 0
        withCalibration();        // motor setters do nothing until calibrated
        setDefaultCommand(new DefaultCommand(this, new ControlMode[] { ControlMode.ANGLE }));
    }

    public boolean isAtLimit() {
        return limitSwitch.get();
    }

    /** Drives down to the limit switch, then zeroes the encoder. */
    public Command calibrateCommand() {
        return new CalibratinCommand(this, ArmConstants.MOTOR_NAME, ArmConstants.CALIBRATION_POWER,
                this::isAtLimit, ArmConstants.CALIBRATION_POSITION);
    }
}
```

Variations:
- **No calibration:** remove `withCalibration()` and `calibrateCommand()`. Otherwise the mechanism will never move.
- **Several motors:** every state array needs one value per motor, and `ControlMode[]` needs one mode per motor, both in the same order as the motor array.
- **Rollers/shooter:** use `withMeterMotor` or no unit helper, and `ControlMode.VELOCITY`. Don't call `setPositionMechanism()`.
- **No state machine needed:** extend `BaseMechanism` instead, and add methods or commands that call `setPower` / `setVelocity` / `setMotion`.

## 3. Wire it in `RobotContainer`

```java
private final CommandController operator = new CommandController(1, ControllerType.kXbox);
private final Arm arm = new Arm();

// in configureBindings():
operator.upButton().onTrue(new InstantCommand(() -> arm.setState(ArmState.SCORE)));
operator.downButton().onTrue(new InstantCommand(() -> arm.setState(ArmState.INTAKE)));
operator.leftBumper().onTrue(arm.calibrateCommand());
```

The state-change `InstantCommand`s intentionally **don't** require `arm`. `setState` only changes a field, and requiring `arm` would interrupt a running calibration (and `CalibratinCommand.end()` zeroes the encoder even when interrupted).

## 4. Verify

1. Run `./gradlew compileJava` (or `bash ./gradlew compileJava` on Linux/macOS) and fix any errors.
2. Tell the user how to tune it on the real robot: pick `TESTING` in `"<Name>/State Chooser"` on the dashboard, edit `"<Name>/Test Values"`, and watch the logged target vs. position.
