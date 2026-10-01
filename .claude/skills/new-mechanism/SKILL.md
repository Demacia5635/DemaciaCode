---
name: new-mechanism
description: Create a new robot subsystem (arm, elevator, intake, shooter, climber...) on top of the Demacia StateBaseMechanism / BaseMechanism classes, with a state enum, constants, limits, calibration, default command and button bindings in RobotContainer. Use when the user asks to add or scaffold a mechanism or subsystem.
---

# New mechanism

Before starting, read `.claude/rules/mechanisms.md` and `.claude/rules/hardware.md` if they are not already loaded.

## 1. Ask or decide

- Name, motors (type, CAN ID, CAN bus, gear ratio, units), sensors (type, channel).
- Control mode per motor: `ANGLE` (rotating joint), `MAGIC_MOTION` (linear position with a profile), `VELOCITY` (rollers, shooters), `VOLTAGE` / `DUTYCYCLE` (simple open loop).
- The states and the target value of each motor in each state.
- Soft limits (min/max position), and how the encoder gets zeroed: a limit switch (`withAutoCalibration`), a manual button, or none.

Don't invent CAN IDs, ports, gear ratios or limits. If they are unknown, use clearly named constants with `// TODO: real value` and tell the user.

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

    public static final TalonFXConfig MOTOR_CONFIG = new TalonFXConfig(MOTOR_NAME, 12, Canbus.Rio)
            .withRadiansMotor(50.0)                              // 50:1 gearbox → radians
            .withPID(8, 0, 0.1, 0.2, 0, 0, 0, 0.35, 0)           // kP kI kD kS kV kA kG kCos kV2
            .withMotionParam(4, 8, 40)                           // rad/s, rad/s², rad/s³ — for MAGIC_MOTION/ANGLE
            .withCurrent(40)
            .withBrake(true);

    public static final LimitSwitchConfig LIMIT_CONFIG = new LimitSwitchConfig("Arm Limit", 0);

    public static final double MIN_ANGLE = Math.toRadians(-20);  // also the angle at the limit switch
    public static final double MAX_ANGLE = Math.toRadians(100);

    private ArmConstants() {}
}
```

`Arm.java`
```java
package frc.robot.subsystems.arm;

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
        addLimit(ArmConstants.MOTOR_NAME, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
        // Position setters are ignored until the limit switch is hit once; then the encoder is zeroed.
        withAutoCalibration(ArmConstants.MOTOR_NAME, this::isAtLimit, ArmConstants.MIN_ANGLE);
        setDefaultCommand(new DefaultCommand(this, new ControlMode[] { ControlMode.ANGLE }));
    }

    public boolean isAtLimit() {
        return limitSwitch.get();
    }

    @Override
    public void periodic() {
        super.periodic(); // required: runs auto-calibration
    }
}
```

Variations:
- **No zeroing needed** (absolute encoder, rollers): remove `withAutoCalibration`. Otherwise the position setters do nothing until the switch is hit.
- **Zero with a command instead:** `new CalibrationCommand(this, MOTOR_NAME, power, this::isAtLimit, resetPos)` together with `withCalibration(MOTOR_NAME)`. Remember that an interrupted `CalibrationCommand` still zeroes the encoder.
- **Several motors:** every state array needs one value per motor, and `ControlMode[]` needs one mode per motor, both in the same order as the motor array.
- **Rollers/shooter:** `ControlMode.VELOCITY`, no limits, no calibration.
- **No state machine:** extend `BaseMechanism`, and add methods or commands that call `setPower` / `setVelocity` / `setMotion`.
- If you override `periodic()`, always call `super.periodic()`.

## 3. Wire it in `RobotContainer`

```java
private final CommandController operator = new CommandController(1, ControllerType.kXbox);
private final Arm arm = new Arm();

// in configureBindings():
operator.upButton().onTrue(new InstantCommand(() -> arm.setState(ArmState.SCORE)));
operator.downButton().onTrue(new InstantCommand(() -> arm.setState(ArmState.INTAKE)));
operator.leftBumper().onTrue(new InstantCommand(arm::setStateIdle));   // IDLE stops the motors
```

The state-change commands intentionally **don't** require `arm`. `setState` only changes a field, and requiring `arm` would interrupt a running calibration command.

## 4. Verify

1. Run `./gradlew compileJava` (or `bash ./gradlew compileJava` on Linux/macOS) and fix any errors.
2. Tell the user how to tune it on the real robot: pick `TESTING` in `"Arm/Arm State Chooser"`, edit `"Arm Test Values"`, and tune PID live under `motors/Arm Motor`. Or run `sysID/sysidCommand` after a recorded run to fit the feed-forward automatically. The `elastic/Generate Layout` button builds a dashboard tab for the new mechanism.
