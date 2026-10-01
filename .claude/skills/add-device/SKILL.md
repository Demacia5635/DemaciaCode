---
name: add-device
description: Add a motor or sensor to the robot using the Demacia config/wrapper classes, or write a new wrapper type (new sensor or motor controller) for the library. Use when the user asks to add, configure or support a motor, encoder, gyro, limit switch, beam break, or any other hardware device.
---

# Add a device

Before starting, read `.claude/rules/hardware.md` if it is not already loaded.

## A. Using an existing device type (most common)

1. Find the config constructor in the table in `hardware.md`. Signatures differ, for example `TalonFXConfig(id, Canbus, name)` vs `SparkMaxConfig(id, name)`.
2. Put the config in the subsystem's `XxxConstants` class as a `public static final` field, with every number named. Don't invent CAN IDs or ports. If they are unknown, mark them `// TODO: real value` and tell the user.
3. Set units first: `withRadiansMotor(gearRatio)` for rotating joints, `withMeterMotor(gearRatio, diameterMeters)` for wheels and linear mechanisms. Then add PID, motion parameters, current limit and brake mode.
4. Create the wrapper (`new TalonFXMotor(config)`, `new LimitSwitch(config)`, ...) inside the subsystem and pass it to `BaseMechanism` as a `MotorInterface` / `SensorInterface`.
5. Don't add log entries for values the wrapper already logs (position, velocity, current, voltage, set-point).

## B. A new device type for the library

Follow the existing pattern exactly. This beam-break example compiles against the current library:

`utils/sensors/BeamBreakConfig.java`
```java
package frc.demacia.utils.sensors;

public class BeamBreakConfig extends BaseSensorConfig<BeamBreakConfig> {
    public BeamBreakConfig(int channel, String name) {
        super(channel, name);
        sensorType = BeamBreak.class;
    }
}
```

`utils/sensors/BeamBreak.java`
```java
package frc.demacia.utils.sensors;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;

/** Beam-break sensor: true when the beam is blocked. */
public class BeamBreak extends DigitalInput implements DigitalSensorInterface {
    private final BeamBreakConfig config;

    public BeamBreak(BeamBreakConfig config) {
        super(config.echoChannel);
        this.config = config;
        setName(config.name);
        addLog();
        LogManager.log(config.name + " beam break initialized");
    }

    @SuppressWarnings("unchecked")
    private void addLog() {
        LogManager.addEntry(config.name + ": isBlocked", this::get)
                .withLogLevel(LogLevel.LOG_ONLY_NOT_IN_COMP).build();
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

Checklist for a new wrapper:
- Config extends `BaseSensorConfig<Self>` / `BaseMotorConfig<Self>`, sets `sensorType` (sensors) or `motorClass` (motors), and every `withXxx()` returns `(T) this`.
- The wrapper extends the vendor/WPILib class and implements the right interface: `AnalogSensorInterface` (`double get()`), `DigitalSensorInterface` (`boolean get()`), `ColorSensorInterface`, or `MotorInterface`.
- Constructor order: apply config → `setName` → `addLog()` → `LogManager.log("... initialized")`.
- `checkElectronics()` reports hardware faults through `LogManager.log(..., AlertType.kError)` (CTRE devices: read the fault signals; simple DIO: no-op).
- **Motors only:** implement every `MotorInterface` method, even unsupported ones. Log a clear message the way `TalonSRXMotor.setMotion` does. Add an entry to `BaseMotorConfig.MotorControllerType` and a copy constructor `(int id, String name, BaseMotorConfig<?> other)` that calls `copyBaseFields`. For Phoenix 6 signals, use `Data` / `LogManager.addEntry(name, signals, isRio)` so they are bulk-refreshed. Don't call `refresh()` per signal.
- Remember the library sync. A new library class must also be added to the central repo `Demacia5635/2026UpdateRobotCodeEmpty`, or it will be lost when the sync runs. Tell the user.

## Verify

Run `./gradlew compileJava` (`bash ./gradlew compileJava` on Linux/macOS) and fix any errors.
