// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.log.Log;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.Pigeon;
import frc.robot.commands.ClimbCalibrationCommand;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ClimbConstants.ClimbState;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */

  private final TalonFXMotor leftMotor;
  private final TalonFXMotor rightMotor;
  private final LimitSwitch leftSwitch;
  private final LimitSwitch rightSwitch;
  private final Pigeon gyro;
  private boolean leftCalibrated;
  private boolean rightCalibrated;
  private double balanceOffset = 0;
  private ClimbState state = ClimbState.IDLE;
  private boolean autoClimb;

  public ClimbSubsystem(Pigeon gyro) {
    leftMotor = new TalonFXMotor(ClimbConstants.LEFT_MOTOR_CONFIG);
    rightMotor = new TalonFXMotor(ClimbConstants.RIGHT_MOTOR_CONFIG);
    leftSwitch = new LimitSwitch(ClimbConstants.LEFT_SWITCH_CONFIG);
    rightSwitch = new LimitSwitch(ClimbConstants.RIGHT_SWITCH_CONFIG);
    this.gyro = gyro;
    SmartDashboard.putData("Climb Subsystem", this);
    SmartDashboard.putData("Climb Calibration Command", new ClimbCalibrationCommand(this));
    addSendableChooser();
    Log.putData("State", ()-> getState());
  }

  public void stop() {
    leftMotor.stop();
    rightMotor.stop();
  }

    public void startCalibration() {
    stop();
    leftCalibrated = false;
    rightCalibrated = false;
  }

  public boolean isRightClosed() {
    return rightSwitch.get();
  }

  public boolean isLeftClosed() {
    return leftSwitch.get();
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

  public void enableAutoClimb(){
    autoClimb = true;
  }

  public boolean isAutoClimb(){
    return autoClimb;
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
    return gyro.getRoll().getValueAsDouble();
  }

  public void setPosition(double left, double right) {
    left = MathUtil.clamp(left, 0, ClimbConstants.OPEN_POSITION);
    right = MathUtil.clamp(right, 0, ClimbConstants.OPEN_POSITION);

    leftMotor.setMotion(left);
    rightMotor.setMotion(right);
  }

  public boolean isAtTarget(double target) {
    return Math.abs((getLeftPosition() + getRightPosition()) / 2 - target) <= ClimbConstants.POSITION_TOLERANCE;
  }

  public void climbAndBalance(double target) {
    double roll = Math.toRadians(getRoll());
    double heightDiff = ClimbConstants.ARM_SEPERATION * Math.sin(roll);
    if(Math.abs(roll) > ClimbConstants.BALANCE_TOLERANCE) {
      double step = heightDiff / 2 * ClimbConstants.BALANCE_FACTOR;
      balanceOffset = MathUtil.clamp(balanceOffset + step, -ClimbConstants.MAX_BALANCE_CORRECTION, ClimbConstants.MAX_BALANCE_CORRECTION);
    }
      double leftTarget = target + balanceOffset;
      double rightTarget = target - balanceOffset;
      setPosition(leftTarget, rightTarget);   
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

  public void setState(ClimbState newState){
    state = newState;
  }

  public ClimbState getState(){
    return state;
  }

  public void addSendableChooser() {
    SendableChooser<ClimbState> stateChooser = new SendableChooser<>();

    for (ClimbState state : ClimbState.values()) {
      stateChooser.addOption(state.name(), state);
    }

    stateChooser.onChange(newState -> this.state = newState);
    SmartDashboard.putData("Climb State Chooser", stateChooser);
  }
}
