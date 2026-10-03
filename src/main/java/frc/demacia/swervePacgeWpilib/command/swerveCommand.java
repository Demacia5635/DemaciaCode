// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.swervePacgeWpilib.command;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.swervePacgeWpilib.wpilibPoseEstimator.wpilibPoseEstimator;
import frc.demacia.swervePacgeWpilib.wpilib_kinmatics.wpilibKinmatics;
import frc.demacia.swervePacgeWpilib.wpilib_odmetry.wpilivOdmetry;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.vision.utils.Vision;
import frc.robot.RobotContainer;

public class SwerveCommand extends Command {

  private final Chassis chassis = Chassis.getInstance();
  private final wpilibKinmatics kinematics;
  private final wpilivOdmetry odometry;
  private final wpilibPoseEstimator poseEstimator;
  private final CommandController controller;
  private final Vision vision;

  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private SwerveModuleState[] moduleStates = new SwerveModuleState[4];
  private ChassisSpeeds speeds;
  private boolean isPrecisionMode = false;

  private Matrix<N3, N1> visionSTD;
  private Matrix<N3, N1> odmetryStd;

  public SwerveCommand(CommandController controller, Vision vision) {
    this.controller = controller;
    this.vision = vision;

    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = chassis.modules[i].getModulePosition();
    }

    kinematics = new wpilibKinmatics(chassis.modulePositions);
    odometry = new wpilivOdmetry(kinematics.kinematics(), chassis.getGyroAngle(), modulePositions);
    poseEstimator = new wpilibPoseEstimator(kinematics.kinematics(), odometry.getOdometry(), null, visionSTD);

    this.visionSTD = new Matrix<N3, N1>(new SimpleMatrix(new double[] { 0.3, 0.3, 0 }));
    this.odmetryStd = new Matrix<N3, N1>(new SimpleMatrix(new double[] { 0.3, 0.3, 0 }));

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double direction = RobotContainer.isRed ? 1.0 : -1.0;
    double joyX = controller.getLeftY() * direction;
    double joyY = controller.getLeftX() * direction;

    // Calculate rotation from trigger axes
    double rot = controller.getLeftTrigger() - controller.getRightTrigger();

    double velX = Math.pow(joyX, 2) * chassis.getConfig().maxDriveVelocity * Math.signum(joyX);
    double velY = Math.pow(joyY, 2) * chassis.getConfig().maxDriveVelocity * Math.signum(joyY);
    double velRot = Math.pow(rot, 2) * chassis.getConfig().maxRotationalVelocity * Math.signum(rot);

    if (isPrecisionMode) {
      velX /= 2.0;
      velY /= 2.0;
      velRot /= 2.0;
    }

    // במידה והנהיגה היא Robot-Oriented
    speeds = new ChassisSpeeds(velX, velY, -velRot);

    // במידה ותרצה Field-Oriented, השתמש בשורה הבאה במקום:
    // speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velX, velY, -velRot, chassis.getGyroAngle());

    moduleStates = kinematics.kinematics().toSwerveModuleStates(speeds);
    chassis.setModuleStates(moduleStates);

    if (vision != null && vision.getPoseEstimation() != null) {
      poseEstimator.updateVision(vision.getPoseEstimation(), 0.02);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // עצירת הרובוט בעת סיום/הפסקת הפקודה
    chassis.setModuleStates(
        kinematics.kinematics().toSwerveModuleStates(new ChassisSpeeds(0, 0, 0))
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setPrecisionMode(boolean precisionMode) {
    this.isPrecisionMode = precisionMode;
  }
}