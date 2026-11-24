# Path code diagram

Overall flow of `frc.demacia.path`, from the button press to the swerve output.

Each class has one job, and data only moves forward:

1. **Build** (once): `TrajectoryBuilder` turns the waypoints into `Line, Arc, Line, ..., Line`.
2. **Follow** (every 20 ms): `DemaciaTrajectory` keeps the current segment and moves to the next one when it is done.
3. **Line / Arc logic**: `SegmentFollower` is the only class that knows the difference between a line and an arc.

```mermaid
flowchart TD
    subgraph RC[RobotContainer]
        BTN[Controller left button] --> PTS[addPoints: hard-coded Pose2d list]
        PTS --> TRJ[new DemaciaTrajectory]
        BTN --> CMD[new FollowTrajectoryCommand]
    end

    subgraph BUILD[Build path: TrajectoryBuilder.build]
        TRJ --> N{Number of waypoints}
        N -- "under 2" --> FIN0[Empty list: finished immediately]
        N -- "2 or more" --> LOOP[For each inner waypoint]
        LOOP --> CR[CornerRounder.round: center on bisector, entry tangent, mirrored exit]
        CR --> RCN[RoundedCorner: entry, exit, center, isLeftTurn]
        RCN --> ADD[Add Line to entry, then Arc to exit]
        ADD --> LOOP
        LOOP -- done --> LAST[Add last Line to final waypoint]
        LAST --> SEGS[pathSegments: Line, Arc, Line, Arc, ..., Line]
    end

    subgraph CMDLIFE[FollowTrajectoryCommand]
        CMD --> INIT[initialize: reset odometry to zero, trajectory.start]
        INIT --> EXEC[execute, every 20 ms]
        EXEC --> POSE[RobotPose.getEstimatedPose and chassis.getChassisSpeedsFieldRel]
        POSE --> CALC
        OUT[swerveCommand.setVelFieldRel] --> CH[Chassis drives]
        EXEC --> ISF{trajectory.isFinished?}
        ISF -- yes --> END[end: chassis.stop]
    end

    subgraph TRAJLOOP[DemaciaTrajectory.calculateChassisSpeeds]
        CALC[Get current segment] --> FOLLOW[SegmentFollower.calculateChassisSpeeds]
        FOLLOW --> DONE{SegmentFollower.isFinished?}
        DONE -- no --> RET[Return speeds]
        DONE -- "yes, not last" --> NEXT[currentSegmentIndex++] --> RET
        DONE -- "yes, last" --> ROT{Heading within FINAL_HEADING_TOLERANCE_RAD?}
        ROT -- yes --> FIN1[isTrajectoryFinished = true, return 0, 0, 0]
        ROT -- no --> SPIN[Return 0, 0, omega only]
    end

    subgraph SF[SegmentFollower]
        FOLLOW --> TYPE{Segment type}
        TYPE -- Line --> LINEV[Speed from DemaciaTrapezoid by distance to end. Heading mirrored across the line]
        TYPE -- Arc --> ARCV[Speed from DemaciaTrapezoid by remaining arc length. Heading tangent plus look-ahead]
        LINEV --> OMEGA[Omega: ProfiledPID on heading error to segment end pose]
        ARCV --> OMEGA
        OMEGA --> RETSF[ChassisSpeeds vx, vy, omega]
    end

    RETSF --> RET
    RET --> OUT
    FIN1 --> OUT
    SPIN --> OUT
```

## Segment-finished conditions (`SegmentFollower.isFinished`)

| Segment | Condition |
|---|---|
| Last line | Distance to end is below `FINAL_POINT_POSITION_THRESHOLD_METERS`. |
| Middle line | Distance to end is below `MID_PATH_POSITION_THRESHOLD_METERS`, or the end point has been passed (angle to end vs. line direction exceeds `MAX_ANGLE_TO_CONSIDER_POINT_PASSED_RAD`). |
| Arc | The velocity heading is within `MAX_HEADING_ERROR_TO_FINISH_ARC_RAD` of the exit tangent heading. |

## Folders

| Folder | Job |
|---|---|
| `DemaciaTrajectory` | Follows the list of segments, one after the other. |
| `builder/` | `TrajectoryBuilder`: waypoints to segments. |
| `geometry/` | `CornerRounder` + `RoundedCorner`: the math for one rounded corner. `TangentCalculator`: circle-to-circle helpers for later. |
| `segments/` | Data only: `PathSegment`, `LineSegment`, `ArcSegment`. |
| `following/` | `SegmentFollower` (line/arc speeds and finished check), `DemaciaTrapezoid` (velocity profile). |
| `commands/` | `FollowTrajectoryCommand`. |
| `constants/` | `PathConstants`. |

## Class structure

Arrows only go one way (top to bottom). `segments` uses no other path class.

```mermaid
classDiagram
    class FollowTrajectoryCommand
    class DemaciaTrajectory {
        pathSegments
        currentSegmentIndex
        calculateChassisSpeeds()
    }
    class TrajectoryBuilder {
        build(waypoints, radius)$
    }
    class CornerRounder {
        round(previous, corner, next, radius)$
    }
    class RoundedCorner {
        <<record>>
        entry
        exit
        center
        isLeftTurn
    }
    class SegmentFollower {
        headingController
        calculateChassisSpeeds()
        isFinished()
    }
    class DemaciaTrapezoid {
        nextVelocity()
    }
    class PathSegment {
        <<abstract>>
        startTranslation
        endPose
    }
    class LineSegment
    class ArcSegment {
        center
        isLeftTurn
        radius
    }

    FollowTrajectoryCommand --> DemaciaTrajectory
    DemaciaTrajectory ..> TrajectoryBuilder
    DemaciaTrajectory --> SegmentFollower
    DemaciaTrajectory o-- PathSegment
    TrajectoryBuilder ..> CornerRounder
    CornerRounder ..> RoundedCorner
    TrajectoryBuilder ..> PathSegment
    SegmentFollower --> DemaciaTrapezoid
    SegmentFollower ..> PathSegment
    PathSegment <|-- LineSegment
    PathSegment <|-- ArcSegment
```
