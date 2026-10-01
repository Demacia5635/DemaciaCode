# DemaciaCode — FRC Team 5635 robot code

Java 17 · WPILib 2026 command-based · GradleRIO 2026.2.1 · team number 5635.

- `src/main/java/frc/robot/` — **season code**: `Robot`, `RobotContainer`, `Constants`, chassis constants for robots B and C (`robot/chassis`), camera configs (`robot/vision`). Game-specific logic goes here.
- `src/main/java/frc/demacia/` — **Demacia library**, reused every season: motors, sensors, mechanisms, swerve, pose estimation + vision, logging, Elastic dashboard, SysId, LEDs, controllers.
- `ARCHITECTURE.md` — long human-facing guide with diagrams. Open only the section you need; don't read it whole by default.

The team is rewriting this code for a new version. When writing new code, follow the rules below even where the old code doesn't.

## Module map — read the area's rule file before working there

Each area has a rule file. It loads automatically when you read a file in that area. When you are only planning (no code opened yet), read the matching rule file yourself first.

| Area | Code (under `src/main/java/frc/`) | Rule file |
|------|------|-----------|
| Motors & sensors (hardware wrappers) | `demacia/utils/motors`, `demacia/utils/sensors` | `.claude/rules/hardware.md` |
| Mechanisms (subsystem base classes, state machine) | `demacia/utils/mechanisms` | `.claude/rules/mechanisms.md` |
| Swerve, kinematics, pose estimation, path geometry | `demacia/utils/chassis`, `demacia/kinematics`, `demacia/RobotPose` (not `Vision/`), `demacia/path` | `.claude/rules/drive-and-pose.md` |
| Vision sources (Limelight 2D/3D, Quest) | `demacia/RobotPose/Vision` | `.claude/rules/vision.md` |
| Logging, dashboard, Elastic, SysId, replay | `demacia/utils/log`, `demacia/utils/Data.java`, `demacia/utils/elastic`, `demacia/utils/sysid`, `demacia/sysID` | `.claude/rules/logging.md` |
| Controllers, LEDs, math helpers, geometry, `RobotCommon` | `demacia/utils/controller`, `demacia/utils/leds`, `demacia/utils/*.java`, `demacia/utils/geometry` | `.claude/rules/utilities.md` |
| Season code | `robot` | `.claude/rules/season-code.md` |

Step-by-step recipes live in skills: `new-mechanism` (add a robot subsystem) and `add-device` (add a motor/sensor, or a new wrapper type).

## Commands

- Compile: `./gradlew compileJava`. Run this before saying a change is done.
- Full build: `./gradlew build` · Simulator: `./gradlew simulateJava` · Deploy: `./gradlew deploy` (needs the roboRIO connected)
- `gradlew` is not executable in git: on Linux/macOS use `bash ./gradlew ...`; on Windows use `gradlew.bat ...`.
- The first build downloads GradleRIO and vendor libraries (needs internet). A `429 Too Many Requests` from Maven is temporary; retry once.
- There are no unit tests yet (`src/test/java` does not exist). Pure-math classes are good candidates for JUnit tests.

## Golden rules

1. **Never block the 20 ms loop.** No `Thread.sleep`, busy-wait loops or blocking I/O in `periodic()`, `execute()`, or anything they call.
2. **Hardware only through the wrappers.** Create devices from a `XxxConfig` builder and use them as `MotorInterface` / `SensorInterface`. Never call vendor classes (`TalonFX`, `SparkMax`, ...) directly from robot code.
3. **Config constructors take the name first:** `new TalonFXConfig("Arm Motor", 12, Canbus.Rio)`, `new LimitSwitchConfig("Arm Limit", 0)`.
4. **Units.** Motor config `withMeterMotor` → meters and m/s. `withRadiansMotor` → radians and rad/s. Neither → mechanism rotations. Angles are radians unless the name says `Degrees`. Distances are meters.
5. **Commands own subsystems.** A command that moves hardware must `addRequirements(...)` every subsystem it controls.
6. **Log through `Log`** (`frc.demacia.utils.log.Log`). Use `Log.log("msg")` for events and `Log.putData(name, supplier)` for values (live on the dashboard and in the file). For file-only or competition-aware values, use the full `putData(name, Supplier[], LogLevel, metaData, isSeparated)`. Don't use `System.out.println` or `SmartDashboard.putNumber` for telemetry. `LogManager` and `LogEntryBuilder` no longer exist.
7. **Dependencies point down only:** `frc.robot` → mechanisms/chassis → hardware → log. `frc.demacia` must never import `frc.robot`. Pass values in (constructor parameters or `Supplier`s), the way `RobotPose.initialize` receives odometry. Don't add new `Chassis.getInstance()` calls inside the library.
8. **Game-specific names stay in `frc.robot`.** Hub, reef, turret, field positions, game-piece logic: none of it goes in `frc.demacia`.
9. **Alliance and competition flags:** `RobotCommon.getIsRed()` / `RobotCommon.getIsComp()` (`frc.demacia.utils`). They are toggled on the dashboard under `RC`. `isRed` defaults to **true**.
10. **Match the surrounding file.** Indentation mixes 2 and 4 spaces between files, so keep each file's style. Public library methods have Javadoc, and new ones should too. Code and comments are in English.

## Library sync — read before editing `frc/demacia`

`atouApdate.yaml` is meant to replace all of `src/main/java/frc/demacia/` with the copy from `Demacia5635/2026UpdateRobotCodeEmpty`. It currently never runs because it sits in the repo root instead of `.github/workflows/`. If it is activated, local edits under `frc/demacia` will be overwritten, so library fixes must also go to the central repo. Never put non-Java files (docs, CLAUDE.md, rules) inside `frc/demacia`.

## Known broken — check before relying on these

- `RobotContainer` uses `RobotCChassisConstants`, where the pigeon ID, all PID values and the steer offsets are `0 // TODO`. Robot B has real values. Details in `season-code.md`.
- `Chassis.setSpeedsFieldRel` does not do a field→robot conversion despite its name. Details in `drive-and-pose.md`.
- `RobotCommon.setIsComp(true)` no longer calls `Log.removeInComp()`, so `*_NOT_IN_COMP` entries are never removed. Details in `logging.md`.
- An interrupted `CalibrationCommand` still zeroes the encoder. Details in `mechanisms.md`.
- Stall detection: `withDetectStallInMotor` stores thresholds, but nothing reads them. Details in `hardware.md`.

## Tools

- If `codebase-memory-mcp` tools are available, use them for structural questions (callers, call chains, impact of a change) before grepping: `search_graph`, `trace_path`, `get_code_snippet`. Its call resolution is name-based. For example, `getInstance` matches several singletons, so confirm surprising edges in the source.
- Mermaid blocks in Markdown can be checked with `mmdc -i file.mmd -o out.svg` when Mermaid CLI is installed.
