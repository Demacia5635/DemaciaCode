# Draft issues

Grouped by priority. Each item is meant to be one small issue. Items marked "verify" are suspected from reading the code and have not been reproduced.

## Path code: architecture (do in this order)

1. **Move the 5° heading tolerance into `PathConstants`**
   `DemaciaTrajectory:157` has `Math.toRadians(5)` with a TODO. Add a constant and use it.

2. **Extract `PathBuilder` from `DemaciaTrajectory`**
   The five `build...` methods only run at construction. Move them to a `PathBuilder` class that returns the segment list. `DemaciaTrajectory` keeps only the index, the finished flag and the loop.

3. **Move "segment finished" checks into the segments**
   `DemaciaTrajectory.isSegmentFinished` uses `instanceof LineSegment`. Add `isFinished(...)` to `PathSegment`, implemented in `LineSegment` and `ArcSegment`.

4. **Move line and arc velocity math into the segments**
   `SegmentFollower` uses `instanceof` to pick `calculateVelocityOnLine` or `calculateVelocityOnArc`. Put the direction math in each segment. The follower keeps the trapezoid and the heading PID.

## Path code: behavior

5. **Make omega proportional to line-segment length**
   Open item from the last commit. The heading target comes from a `ProfiledPIDController` with gain 1 in `SegmentFollower`. Decide whether to scale the target or the gain per segment.

6. **Reset the heading controller when a trajectory starts** (verify)
   `SegmentFollower` creates the `ProfiledPIDController` once and `start()` does not reset it, so state from a previous run may carry over.

7. **Arc finish check can trigger at low speed** (verify)
   `isSegmentFinished` uses heading `0` when the robot speed is zero. If an arc's exit heading is near 0, the arc may count as finished right away.

8. **`FollowTrajectoryCommand` resets odometry to zero on start** (verify intent)
   The path waypoints are used as-is. Check that this is wanted when the robot does not start at the origin.

## Path code: cleanup

9. **Remove unused and duplicate imports**
   `SegmentFollower` imports `DemaciaTrapezoid` twice and imports classes from its own package. `RobotContainer` has unused imports such as `java.security.PublicKey`.

10. **Fix `PathConstants`**
    It has "TODO: CHANGE THIS" and a note that one constant is unused. Set real values and delete what is unused.

11. **Remove hard-coded test waypoints from `RobotContainer`**
    `addPoints()` holds test points. Move them to a clearly named test path.

## Repo-wide (separate from the path work)

12. **Duplicate packages**
    `demacia/odometry` duplicates `demacia/RobotPose`. There are two `VisionConstants` files, two sysid and log-reader implementations, and three swerve stacks (custom `Chassis`, `wpilibSwerve`, `kinematics`). Pick one of each.

13. **Fix naming and typos**
    `WPILibKinmatics`, `WPILibOdemetry`, `MK5nChassisConstansRobotC`. Inconsistent packages: `RobotPose`, `sysID`.

14. **Check whether `bin/` and `build/` are tracked by git** (verify)
    Add them to `.gitignore` if they are.

15. **Finish the vision TODOs**
    Camera transforms in `robot/vision/VisionConstants`. 2026 Hub, Tower and Outpost offsets and Quest mount offsets in `demacia/vision`. Remove incorrectly located tags (`Vision.java:53`). Replace the Quest-disconnected log with an LED signal (`RobotPose:152`).

16. **Add tests**
    There are none. Start with `PathBuilder` and the segment `isFinished` logic, which need no robot hardware.
