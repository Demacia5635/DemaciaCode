// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.Pigeon;
import frc.robot.chassis.Mk4iChassisConstants;
import frc.robot.commands.ClimbCalibrationCommand;
import frc.robot.constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */

  private final TalonFXMotor leftMotor;
  private final TalonFXMotor rightMotor;
  private final LimitSwitch leftSwitch;
  private final LimitSwitch rightSwitch;
  private final Chassis chassis;
  private final Pigeon pigeon;
  private boolean leftCalibrated;
  private boolean rightCalibrated;

  public ClimbSubsystem() {
    leftMotor = new TalonFXMotor(ClimbConstants.LEFT_MOTOR_CONFIG);
    rightMotor = new TalonFXMotor(ClimbConstants.RIGHT_MOTOR_CONFIG);
    leftSwitch = new LimitSwitch(ClimbConstants.LEFT_SWITCH_CONFIG);
    rightSwitch = new LimitSwitch(ClimbConstants.RIGHT_SWITCH_CONFIG);
    Chassis.initialize(Mk4iChassisConstants.CHASSIS_CONFIG);
    chassis = Chassis.getInstance();
    pigeon = chassis.gyro;
    SmartDashboard.putData("Climb Subsystem", this);
    SmartDashboard.putData("Climb Calibration Command", new ClimbCalibrationCommand(this));

  }

  public void stop() {
    leftMotor.stop();
    rightMotor.stop();
  }

  public boolean isRightClosed() {
    return rightSwitch.get();
  }

  public boolean isLeftClosed() {
    return leftSwitch.get();
  }

  public double getRightPosition() {
    return rightMotor.getCurrentPosition();
  }

  public double getLeftPosition() {
    return leftMotor.getCurrentPosition();
  }

  public void setRightDuty(double duty) {
    rightMotor.setDuty(duty);
  }

  public void setLeftDuty(double duty) {
    leftMotor.setDuty(duty);
  }

  public double getRoll() {
    return pigeon.getRoll().getValueAsDouble();
  }

  public void startCalibration() {
    stop();
    leftCalibrated = false;
    rightCalibrated = false;
  }

  public void setLeftCalibrated() {
    leftMotor.stop();
    leftMotor.setEncoderPosition(0);
    leftCalibrated = true;
  }

  public void setRightCalibrated() {
    rightMotor.stop();
    rightMotor.setEncoderPosition(0.0);
    rightCalibrated = true;
  }

  public boolean isLeftCalibrated() {
    return leftCalibrated;
  }

  public boolean isRightCalibrated() {
    return rightCalibrated;
  }

  public boolean isCalibrated() {
    return leftCalibrated && rightCalibrated;
  }

  public void setPosition(double left, double right) {
    left = MathUtil.clamp(left, 0, ClimbConstants.OPEN_POSITION);
    right = MathUtil.clamp(right, 0, ClimbConstants.OPEN_POSITION);

    leftMotor.setPositionVoltage(left);
    rightMotor.setPositionVoltage(right);
  }

  public boolean isAtTarget(double target) {
    return Math.abs((getLeftPosition() + getRightPosition()) / 2 - target) <= ClimbConstants.POSITION_TOLERANCE;
  }

  public void balance(double target) {
    double roll = Math.toRadians(getRoll());
    if(Math.abs(roll) > ClimbConstants.BALANCE_TOLERANCE) {
      double correction = MathUtil.clamp(roll * ClimbConstants.BALANCE_FACTOR, -ClimbConstants.MAX_BALANCE_CORRECTION, ClimbConstants.MAX_BALANCE_CORRECTION);
      double leftTarget = target + correction;
      double rightTarget = target - correction;
      setPosition(leftTarget, rightTarget);
    }   
  }

  public boolean isBalanced() {
    double roll = Math.toRadians(getRoll());
    return Math.abs(roll) < ClimbConstants.BALANCE_TOLERANCE;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Left Position", this::getLeftPosition, null);
    builder.addDoubleProperty("Right Position", this::getRightPosition, null);
    builder.addDoubleProperty("Roll", this::getRoll, null);
    builder.addBooleanProperty("Left Limit Switch", this::isLeftClosed, null);
    builder.addBooleanProperty("Right Limit Switch", this::isRightClosed, null);
    builder.addBooleanProperty("Is Calibrated", this::isCalibrated, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
