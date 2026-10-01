---
paths:
  - "src/main/java/frc/demacia/utils/log/**"
  - "src/main/java/frc/demacia/utils/Data.java"
  - "src/main/java/frc/demacia/sysID/**"
---

# Logging & telemetry (`frc/demacia/utils/log`, `utils/Data.java`, `sysID`)

`LogManager` is the backbone of debugging. `LogManager.log` is the most-called method in the library, and every motor and sensor registers itself here.

## API

```java
LogManager.log("Arm calibrated");                        // event → DataLog file + dashboard alert
LogManager.log("Bad config", AlertType.kError);          // with severity
LogManager.addEntry("Arm/angle", arm::getAngle)          // value logged every loop
        .withLogLevel(LogLevel.LOG_AND_NT)
        .build();
LogManager.addEntry("Arm/signals", statusSignalsArray, isRio);  // Phoenix 6 StatusSignals (bulk-refreshed)
```

Builder options: `withLogLevel`, `withMetaData(String)`, `withIsMotor()` (sets metadata `"motor"`, which SysId uses), `withIsSeparated(true)` (don't group). `build()` returns `null` and logs an error if the name is empty.

| `LogLevel` | File | NetworkTables | After `isComp = true` |
|-----------|:---:|:---:|---|
| `LOG_ONLY_NOT_IN_COMP` (**default**) | ✅ | ❌ | removed |
| `LOG_ONLY` | ✅ | ❌ | kept |
| `LOG_AND_NT_NOT_IN_COMP` | ✅ | ✅ | removed |
| `LOG_AND_NT` | ✅ | ✅ | kept |

Choose `LOG_AND_NT` only for values drivers need live in matches. Bandwidth and CPU matter on the roboRIO.

## How it works

- `LogManager` is a singleton `SubsystemBase` created by a **static initializer** the first time any class touches it. It starts `DataLogManager` and DriverStation logging.
- `periodic()` runs `Data.refreshAll()` first. This bulk-refreshes all Phoenix 6 signals (separately for the rio bus and the CANivore bus) and re-reads suppliers. It then expires console alerts and writes every entry. **All motor and sensor getters read values cached by this call.**
- To save work, entries are **grouped by data type + log level** into up to 24 combined array entries (`categoryLogEntries`) unless `withIsSeparated(true)` is used. A type mismatch falls back to a separate `LOG_ONLY` entry with a warning.
- `RobotContainer.setIsComp(true)` calls `LogManager.removeInComp()` once. It drops `*_NOT_IN_COMP` entries and can't be undone without a restart.
- `Data<T>` wraps either `StatusSignal`s or `Supplier`s, caches values and converts types (for example boolean → double).

## SysId

`sysID/SysidApp` is a **desktop** Swing app (`main` method). It does not run on the robot. It reads `.wpilog` files, finds entries with motor metadata, and fits kS/kV/kA/kG. It has its own `sysID/LogReader`, which is different from `utils/log/LogReader` (two classes with the same name, so check the package before editing).

## Rules

- New subsystems log through `addEntry`. Don't use `SmartDashboard.putNumber` for telemetry (dashboard buttons via `SmartDashboard.putData(Command)` are fine).
- Keep names hierarchical: `"<Subsystem>/<value>"`.
- Never log from a tight loop with `log(...)` (it creates an alert each time, capped by `ConsoleConstants.CONSOLE_LIMIT`). Use an entry instead.
