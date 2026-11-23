package frc.demacia.RobotPose.Vision.OffSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.RobotPose.Vision.VisionTypes.Quest;
import frc.demacia.utils.chassis.Chassis;

public class OffSetsCheck extends SequentialCommandGroup {

  private Pose2d startPose;
  private Pose2d endPose;

  public OffSetsCheck(Quest quest, double omega) {
    Chassis chassis = Chassis.getInstance();

    addCommands(
        // 1. Wait to settle and record initial pose
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> {
          startPose = getQuestReportedPose(quest);
          System.out.println("Start Pose: " + startPose);
        }),

        // 2. Rotate in place (adjust timeout so the robot turns roughly 180 degrees)
        turnCommand(chassis, omega, 1.5),

        // 3. Wait to settle and record final pose
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> {
          endPose = getQuestReportedPose(quest);
          System.out.println("End Pose: " + endPose);
          calculateExactOffset();
        })
    );
  }

  private Command turnCommand(Chassis chassis, double omega, double seconds) {
    return Commands.runEnd(
        () -> chassis.setSpeedsRobotRel(new ChassisSpeeds(0, 0, omega)),
        () -> chassis.setSpeedsRobotRel(new ChassisSpeeds(0, 0, 0)),
        chassis
    ).withTimeout(seconds);
  }

  private Pose2d getQuestReportedPose(Quest quest) {
    var estimates = quest.getPoseEstimates();
    if (!estimates.isEmpty()) {
      return estimates.get(0).pose();
    }
    return RobotPose.getInstance().getEstimatedPose();
  }

  /**
   * Exact geometric calculation of the offset vector (robot center to Quest)
   * given two poses separated by a pure rotation.
   */
  private void calculateExactOffset() {
    if (startPose == null || endPose == null) {
      System.out.println("Missing poses for offset calculation!");
      return;
    }

    // Angle rotated between start and end
    Rotation2d deltaTheta = endPose.getRotation().minus(startPose.getRotation());
    double dThetaRad = deltaTheta.getRadians();

    if (Math.abs(dThetaRad) < Math.toRadians(30)) {
      System.out.println("Rotation angle too small (< 30 deg). Increase omega or time!");
      return;
    }

    // Field displacement of the Quest
    Translation2d deltaP = endPose.getTranslation().minus(startPose.getTranslation());

    // =========================================================================
    // HOW THE FORMULA WORKS:
    // When the robot rotates in place around its center (C), the Quest's field
    // position (P) at any angle (theta) is:
    //   P(theta) = C + Rotate(offset, theta)
    //
    // In 2D coordinates (X, Y), rotating the robot-relative offset (ox, oy) by theta is:
    //   Px = Cx + ox * cos(theta) - oy * sin(theta)
    //   Py = Cy + ox * sin(theta) + oy * cos(theta)
    //
    // Since the robot center (C) does NOT move during a pure tank turn,
    // subtracting P_start from P_end cancels out C completely:
    //   deltaX = Px_end - Px_start = ox * (cos_end - cos_start) - oy * (sin_end - sin_start)
    //   deltaY = Py_end - Py_start = ox * (sin_end - sin_start) + oy * (cos_end - cos_start)
    //
    // Let:
    //   a = cos_end - cos_start
    //   b = sin_end - sin_start
    //
    // Now we have a simple 2-equation linear system for (ox, oy):
    //   1) deltaX = a * ox - b * oy
    //   2) deltaY = b * ox + a * oy
    // =========================================================================

    double cosStart = startPose.getRotation().getCos();
    double sinStart = startPose.getRotation().getSin();
    double cosEnd = endPose.getRotation().getCos();
    double sinEnd = endPose.getRotation().getSin();

    // Change in cosine and sine between start and end headings
    double a = cosEnd - cosStart;
    double b = sinEnd - sinStart;

    // Determinant of the 2x2 matrix [[a, -b], [b, a]] -> (a*a) - (-b*b) = a^2 + b^2
    double denom = a * a + b * b;

    // Solving the 2x2 linear system using Cramer's rule / inverse matrix:
    // ox = ( a * deltaX + b * deltaY) / (a^2 + b^2)
    // oy = (-b * deltaX + a * deltaY) / (a^2 + b^2)
    double offsetX = (a * deltaP.getX() + b * deltaP.getY()) / denom;
    double offsetY = (-b * deltaP.getX() + a * deltaP.getY()) / denom;

    System.out.println("=== EXACT CALCULATED QUEST OFFSET ===");
    System.out.printf("Rotated Angle:      %.1f degrees%n", deltaTheta.getDegrees());
    System.out.printf("Offset X (Forward): %.4f meters%n", offsetX);
    System.out.printf("Offset Y (Left):    %.4f meters%n", offsetY);
  }
}
