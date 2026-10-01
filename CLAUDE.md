# DemaciaCode — FRC Team 5635 robot code

Java 17 · WPILib 2026 command-based · GradleRIO 2026.2.1 · team number 5635.

- `src/main/java/frc/robot/` — **season code**. Almost empty template (`Main`, `Robot`, `RobotContainer`, `Constants`). Game-specific logic goes here.
- `src/main/java/frc/demacia/` — **Demacia library**, reused every season: motors, sensors, mechanisms, swerve, odometry, vision, logging, LEDs, controllers.
- `ARCHITECTURE.md` — long human-facing guide with diagrams. Open only the section you need; don't read it whole by default.

The team is rewriting this code for a new version. When writing new code, follow the rules below even where the old code doesn't.

## Module map — read the area's rule file before working there

Each area has a rule file. It loads automatically when you read a file in that area. When you are only planning (no code opened yet), read the matching rule file yourself first.

| Area | Code (under `src/main/java/frc/`) | Rule file |
|------|------|-----------|
| Motors & sensors (hardware wrappers) | `demacia/utils/motors`, `demacia/utils/sensors` | `.claude/rules/hardware.md` |
| Mechanisms (subsystem base classes, state machine) | `demacia/utils/mechanisms` | `.claude/rules/mechanisms.md` |
| Swerve, kinematics, odometry, path geometry | `demacia/utils/chassis`, `demacia/kinematics`, `demacia/odometry`, `demacia/path` | `.claude/rules/drive-and-pose.md` |
| Vision (Limelight AprilTags, Quest, objects) | `demacia/vision` | `.claude/rules/vision.md` |
| Logging, telemetry, SysId | `demacia/utils/log`, `demacia/utils/Data.java`, `demacia/sysID` | `.claude/rules/logging.md` |
| Controllers, LEDs, math helpers, geometry | `demacia/utils/controller`, `demacia/utils/leds`, `demacia/utils/*.java`, `demacia/utils/geometry` | `.claude/rules/utilities.md` |
| Season code | `robot` | `.claude/rules/season-code.md` |

Step-by-step recipes live in skills: `new-mechanism` (add a robot subsystem) and `add-device` (add a motor/sensor, or a new wrapper type).

## Commands

- Compile: `./gradlew compileJava` — run this before saying a change is done.
- Full build: `./gradlew build` · Simulator: `./gradlew simulateJava` · Deploy: `./gradlew deploy` (needs the roboRIO connected)
- `gradlew` is not executable in git: on Linux/macOS use `bash ./gradlew ...`; on Windows use `gradlew.bat ...`.
- The first build downloads GradleRIO and vendor libraries (needs internet). A `429 Too Many Requests` from Maven is temporary; retry once.
- There are no unit tests yet (`src/test/java` does not exist). Pure-math classes are good candidates for JUnit tests.

## Golden rules

1. **Never block the 20 ms loop.** No `Thread.sleep`, busy-wait loops or blocking I/O in `periodic()`, `execute()`, or anything they call.
2. **Hardware only through the wrappers.** Create devices from a `XxxConfig` builder and use them as `MotorInterface` / `SensorInterface`. Never call vendor classes (`TalonFX`, `SparkMax`, ...) directly from robot code.
3. **Units.** Motor config `withMeterMotor` → meters and m/s. `withRadiansMotor` → radians and rad/s. Neither → mechanism rotations. Angles are radians unless the name says `Degrees`. Distances are meters.
4. **Commands own subsystems.** A command that moves hardware must `addRequirements(...)` every subsystem it controls.
5. **Log through `LogManager`.** Use `LogManager.log("msg")` for events and `LogManager.addEntry(name, supplier).withLogLevel(...).build()` for values. Don't use `System.out.println` or raw `SmartDashboard.put*` for telemetry.
6. **Dependencies point down only:** `frc.robot` → mechanisms/chassis → hardware → log. `frc.demacia` must never import `frc.robot`. Don't add new `Chassis.getInstance()` calls in vision, odometry or hardware code. Pass values in (constructor parameters or `Supplier`s) instead.
7. **Game-specific names stay in `frc.robot`.** Hub, reef, turret, field positions, game-piece logic: none of it goes in `frc.demacia`.
8. **Alliance and competition flags:** read them with `DemaciaUtils.getIsRed()` / `DemaciaUtils.getIsComp()`. `RobotContainer` sets them and the dashboard toggles them.
9. **Match the surrounding file.** Indentation mixes 2 and 4 spaces between files, so keep each file's style. Public library methods have Javadoc, and new ones should too. Code and comments are in English.

## Library sync — read before editing `frc/demacia`

`atouApdate.yaml` is meant to replace all of `src/main/java/frc/demacia/` with the copy from `Demacia5635/2026UpdateRobotCodeEmpty`. It currently never runs because it sits in the repo root instead of `.github/workflows/`. If it is activated, local edits under `frc/demacia` will be overwritten, so library fixes must also go to the central repo. Never put non-Java files (docs, CLAUDE.md, rules) inside `frc/demacia`.

## Known broken — check before relying on these

- Swerve kinematics receive an array of `null` module positions (`Chassis` constructor). Details in `drive-and-pose.md`.
- Field-relative vs robot-relative frames are inconsistent in the drive path. Details in `drive-and-pose.md`.
- Vision has no cameras configured (`VisionConstants.Tags.TAGS_ARRAY` is empty). Details in `vision.md`.
- Motor stall detection is configured but never triggered (nothing calls `updateStallDetection()`). Details in `hardware.md`.

## Tools

- If `codebase-memory-mcp` tools are available, use them for structural questions (callers, call chains, impact of a change) before grepping: `search_graph`, `trace_path`, `get_code_snippet`. Its call resolution is name-based. For example, `getInstance` matches both `Chassis` and `RobotPose`, so confirm surprising edges in the source.
- Mermaid blocks in Markdown can be checked with `mmdc -i file.mmd -o out.svg` when Mermaid CLI is installed.
