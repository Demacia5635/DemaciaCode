---
name: add-device
description: Add a motor or sensor to the robot using the Demacia config/wrapper classes, or write a new wrapper type (new sensor or motor controller) for the library. Use when the user asks to add, configure or support a motor, encoder, gyro, limit switch, beam break, or any other hardware device.
---

# Add a device

Before starting, read `.claude/rules/hardware.md` if it is not already loaded.

## A. Using an existing device type (most common)

1. Find the config constructor in the table in `hardware.md`. **The name always comes first:** `new TalonFXConfig("Arm Motor", 12, Canbus.Rio)`, `new SparkMaxConfig("Roller", 7)`, `new LimitSwitchConfig("Arm Limit", 0)`.
2. Put the config in the subsystem's `XxxConstants` class as a `public static final` field, with every number named. Don't invent CAN IDs or ports. If they are unknown, mark them `// TODO: real value` and tell the user.
3. Set units first: `withRadiansMotor(gearRatio)` for rotating joints, `withMeterMotor(gearRatio, diameterMeters)` for wheels and linear mechanisms. Then `withPID(kP, kI, kD, kS, kV, kA, kG, kCos, kV2)` (9 numbers), `withMotionParam`, `withCurrent`, `withBrake`.
4. Create the wrapper (`new TalonFXMotor(config)`, `new LimitSwitch(config)`, ...) inside the subsystem and pass it to `BaseMechanism` as a `MotorInterface` / `SensorInterface`.
5. Don't add logging, dashboard, Elastic or SysId registration yourself. The wrappers already do all of it.

## B. A new sensor type

Follow the existing pattern exactly. This beam-break example compiles against the current library:

`utils/sensors/BeamBreakConfig.java`
```java
package frc.demacia.utils.sensors;

public class BeamBreakConfig extends BaseSensorConfig<BeamBreakConfig> {
    public BeamBreakConfig(String name, int channel) {
        super(name, channel);
        sensorType = BeamBreak.class;
    }
}
```

`utils/sensors/BeamBreak.java`
```java
package frc.demacia.utils.sensors;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.elastic.ElasticGenerator;
import frc.demacia.utils.log.Log;
import frc.demacia.utils.log.Log.LogLevel;

/** Beam-break sensor: true when the beam is blocked. */
public class BeamBreak extends DigitalInput implements DigitalSensorInterface {
    private final BeamBreakConfig config;

    public BeamBreak(BeamBreakConfig config) {
        super(config.echoChannel);
        this.config = config;
        setName(config.name);
        addLog();
        SmartDashboard.putData("sensors/" + config.name, this);
        Log.log(config.name + " beam break initialized");
        ElasticGenerator.getInstance().registerSensor(this);
    }

    @SuppressWarnings("unchecked")
    private void addLog() {
        Log.putData(config.name + ": isBlocked", new Supplier[] { this::get }, LogLevel.LOG_ONLY, "sensors", false);
    }

    @Override
    public boolean get() {
        // beam-break receivers read false when blocked
        return config.isInverted == super.get();
    }

    @Override
    public void checkElectronics() {}

    @Override
    public String getName() {
        return config.name;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Digital Input");
        builder.addBooleanProperty("Value", this::get, null);
    }
}
```

Checklist for a sensor:
- The config extends `BaseSensorConfig<Self>`, takes the name first, and sets `sensorType`.
- The wrapper extends the WPILib/vendor class and implements `AnalogSensorInterface` (`double get()`), `DigitalSensorInterface` (`boolean get()`) or `ColorSensorInterface`.
- Constructor: `setName` → `addLog()` (grouped, `LOG_ONLY`, metadata `"sensors"`) → `SmartDashboard.putData("sensors/" + name, this)` → `Log.log("... initialized")` → `ElasticGenerator.getInstance().registerSensor(this)`.
- `checkElectronics()` reports faults with `Log.log(..., AlertType.kError)` (CTRE: fault signals; plain DIO: no-op).

## C. A new motor controller type

- Create `XxxConfig extends BaseMotorConfig<XxxConfig>` with constructors `(String name, int id)` and a copy constructor `(String name, int id, BaseMotorConfig<?> other)` that calls `copyBaseFields`. Set `motorClass`.
- Create `XxxMotor extends BaseMotor`. Hold the vendor object in a field and implement **only** the abstract hooks (`createMotor`, `configXxx`, `applyXxx`, `setSignals`, `changeMotorSlot`, `stopMotor`, `setMotorDuty/Voltage/Velocity/PositionVoltage/MotionMagic`) plus `checkElectronics`, `isConnected` and `setEncoderPosition`. Control modes, units, feed-forward, logging, the dashboard, Elastic and SysId all come from `BaseMotor`. Don't re-implement them.
- `setSignals()` must fill the `Data` fields (`positionSignal`, `velocitySignal`, ...). For Phoenix 6, wrap `StatusSignal`s in `new Data<>(signal, isRio())` so they are bulk-refreshed.
- Add an entry to `BaseMotorConfig.MotorControllerType`.
- Copy `TalonFXMotor` / `SparkMaxMotor` as the reference.

Remember the library sync: a new library class must also be added to the central repo `Demacia5635/2026UpdateRobotCodeEmpty`, or it will be lost when the sync runs. Tell the user.

## Verify

Run `./gradlew compileJava` (`bash ./gradlew compileJava` on Linux/macOS) and fix any errors.
