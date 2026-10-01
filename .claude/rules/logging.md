---
paths:
  - "src/main/java/frc/demacia/utils/log/**"
  - "src/main/java/frc/demacia/utils/Data.java"
  - "src/main/java/frc/demacia/utils/elastic/**"
  - "src/main/java/frc/demacia/utils/sysid/**"
  - "src/main/java/frc/demacia/sysID/**"
---

# Logging, dashboard, Elastic, SysId, replay

`Log` (`frc.demacia.utils.log.Log`, formerly `LogManager`) is the backbone of debugging. Every motor, sensor, mechanism and vision source goes through it.

## API

```java
Log.log("Arm calibrated");                           // event → DataLog file + dashboard alert
Log.log("Bad config", AlertType.kError);             // with severity
Log.putData("Arm/current", () -> motor.getCurrentCurrent());   // LOG_AND_NT, own entry: file + live NT
Log.putData("Arm/angle", statusSignal, isRio);                 // one Phoenix 6 signal, LOG_AND_NT
Log.putData("Arm/debug", new Supplier[] { () -> x },           // full control:
        LogLevel.LOG_ONLY, "myMeta", /* isSeparated */ false); //   level, metadata, grouping
Log.putData("Arm/tuning", someSendable);             // any Sendable → logged + on NT via DashboardBuilder
```

There is no builder anymore: `addEntry(...).withLogLevel(...).build()` is gone. `LogLevel` is `Log.LogLevel`.

| `LogLevel` | File | NetworkTables | Meant for |
|-----------|:---:|:---:|---|
| `LOG_ONLY_NOT_IN_COMP` | ✅ | ❌ | debug, dropped in competition |
| `LOG_ONLY` | ✅ | ❌ | always in the file |
| `LOG_AND_NT_NOT_IN_COMP` | ✅ | ✅ | live while testing |
| `LOG_AND_NT` | ✅ | ✅ | live in matches (this is the default of the short `putData`) |

## How it works

- `Log` is a singleton `SubsystemBase` created by a **static initializer** the first time any class touches it. On creation it starts `DataLogManager`, puts `sysID/sysidCommand` and `replay/LoadLatestLog` on the dashboard, and calls `RobotCommon.init()`.
- `periodic()`: `Data.refreshAll()` (bulk-refreshes all Phoenix 6 signals and suppliers; **all device getters read these cached values**), expires alerts, writes every entry, then updates all `DashboardBuilder`s.
- **Grouping:** `putData(..., isSeparated=false)` (or more than one `Data`) does not create its own entry. The values go into one of three shared arrays (`float[]`, `boolean[]`, `String[]`) whose name is all the member names joined with `" | "`. These groups are always **`LOG_ONLY`**, whatever level you pass. Motors and sensors log this way (metadata `"motors"` / `"sensors"`). Use `isSeparated=true` with one supplier to get a normal named entry.
- `DashboardBuilder` implements `NTSendableBuilder`. Each getter property is also logged (`Log.putData(table/key, getter)`), and setters are polled from NT.
- **Competition mode is broken:** `Log.removeInComp()` drops `*_NOT_IN_COMP` entries, but nothing calls it now (`RobotCommon.setIsComp` doesn't). Call it from the `isComp` setter if you need it.

## Tools built on the log

- **On-robot SysId:** `motors` metadata → `sysID/sysidCommand` (`utils/sysid/SysidCommand`) reads the **latest .wpilog** (`/home/lvuser/logs`, or `logs/` in sim), runs `sysID/Sysid` per motor registered with `Sysid.registerMotor` (done automatically by `BaseMotor`), and **applies** the fitted kS/kV/kA/kG(/kCos/kV2) and motion limits to slot 0 live. `motor.getSysidFlags()` chooses which terms to fit. `sysID/SysidApp` is the desktop Swing version. Note that the folder is `sysID` but the package is `frc.demacia.sysid`.
- **Replay:** `replay/LoadLatestLog` → `LogReplay` loads the latest log and republishes every entry under `replay/...`. Scrub with the `replay/time` slider. `LogFileChooser` (Windows PowerShell dialog, desktop only) picks a file manually.
- **Elastic:** `ElasticGenerator` collects registered motors, sensors, mechanisms, power commands, auto-calibrations, vision sources and the chassis. The `elastic/Generate Layout` button writes `Generated_Elastic_Layout.json` (to `/home/lvuser/elastic_layouts`, or the deploy dir in sim), served on port 5800. `ElasticNotification.sendNotification(...)` / `selectTab(...)` push pop-ups and switch tabs on the Elastic dashboard.

## Rules

- New telemetry → `Log.putData`. Use names in the form `"<Subsystem>/<value>"`.
- Choose the level on purpose. Use `LOG_AND_NT` only for values drivers need live in matches.
- Never call `Log.log(...)` every loop. It creates an alert each time (capped by `ConsoleConstants.CONSOLE_LIMIT`). For a condition you check in `periodic()` (overheating, disconnected), create a `ConsoleAlert` **field** once, for example `ConsoleAlert.error("Arm overheating")`, and call `alert.set(condition)` every loop. It logs and sends an Elastic pop-up only on the rising edge (see its Javadoc).
- `sendable`-style dashboard widgets: prefer `Log.putData(key, sendable)` over `SmartDashboard.putData` for anything you also want in the log file.
