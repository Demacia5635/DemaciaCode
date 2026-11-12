// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Gripper.subsystems.subsystems;

import static frc.robot.Gripper.subsystems.constants.ConstantsGripper.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.OpticalSensor;
import frc.demacia.utils.sensors.UltraSonicSensor;

public class SubsystemGripper extends SubsystemBase {

  // Gripper intake members
  public TalonSRXMotor m_SRXSensorMotor;
  public OpticalSensor m_opticSensor;
  public UltraSonicSensor ultraSonicSensor;

  // Differential wrist (Kraken) members
  private final TalonFXMotor m_KRAKEN1Pitch;
  private final TalonFXMotor m_KRAKEN2Roll;
  private final LimitSwitch limitSwitchPitch;
  private final LimitSwitch limitSwitchRoll;

  public SubsystemGripper() {
    // Gripper intake initialization
    m_SRXSensorMotor = new TalonSRXMotor(kSnowBlowerConfig);
    // m_opticSensor = new OpticalSensor(kOpticSensorConfig); // Uncomment when sensor is connected
    ultraSonicSensor = new UltraSonicSensor(kUltraSonicSensorConfig);
    SmartDashboard.putData("Run Intake", new InstantCommand(() -> setPower(-0.6)));

    // Differential wrist initialization
    m_KRAKEN1Pitch = new TalonFXMotor(CONFIG);
    m_KRAKEN2Roll = new TalonFXMotor(CONFIG2);
    limitSwitchPitch = new LimitSwitch(LIMIT_SWITCH_CONFIG_PITCH);
    limitSwitchRoll = new LimitSwitch(LIMIT_SWITCH_CONFIG_ROLL);

    // Initialize dashboard target inputs
    SmartDashboard.putNumber("Differential Pitch (Degrees)", 0.0);
    SmartDashboard.putNumber("Differential Roll (Degrees)", 0.0);
    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // Automatically update telemetry values on the dashboard
    builder.addDoubleProperty("UltraSonicDistance", () -> ultraSonicSensor.get(), null);
    builder.addDoubleProperty("IntakeCurrent", () -> m_SRXSensorMotor.getCurrentCurrent(), null);
    builder.addDoubleProperty("Actual Pitch (Degrees)", this::getActualPitchDeg, null);
    builder.addDoubleProperty("Actual Roll (Degrees)", this::getActualRollDeg, null);
  }

  // --- Intake Methods ---

  public double getOpticSensor() {
    // Prevent NullPointerException if sensor is not initialized
    if (m_opticSensor == null) {
      return 0.0;
    }
    return m_opticSensor.get();
  }

  public void setPower(double power) {
    m_SRXSensorMotor.setDuty(power);
  }

  public double getUltraSonicSensor() {
    return ultraSonicSensor.get();
  }

  public double getCurrent() {
    return m_SRXSensorMotor.getCurrentCurrent();
  }

  // --- Differential Wrist Methods ---

  /**
   * Sets the target pitch and roll angles using differential kinematics.
   * Assumes motors are configured with radians as position units.
   */
  public void setPosition(double pitchRadians, double rollRadians) {
    m_KRAKEN1Pitch.setPositionVoltage(pitchRadians + rollRadians);
    m_KRAKEN2Roll.setPositionVoltage(pitchRadians - rollRadians);
  }

  public boolean isPitchLimitSwitchTriggered() {
    return limitSwitchPitch.get();
  }

  public boolean isRollLimitSwitchTriggered() {
    return limitSwitchRoll.get();
  }

  public void stop() {
    m_KRAKEN1Pitch.stop();
    m_KRAKEN2Roll.stop();
  }

  public final double getMotor1Position() {
    return m_KRAKEN1Pitch.getPosition().getValueAsDouble();
  }

  public final double getMotor2Position() {
    return m_KRAKEN2Roll.getPosition().getValueAsDouble();
  }

  /**
   * Calculates the actual pitch angle in degrees from both differential motors.
   */
  public double getActualPitchDeg() {
    return Math.toDegrees((getMotor1Position() + getMotor2Position()) / 2.0);
  }

  /**
   * Calculates the actual roll angle in degrees from both differential motors.
   */
  public double getActualRollDeg() {
    return Math.toDegrees((getMotor1Position() - getMotor2Position()) / 2.0);
  }

  /**
   * Safety check to stop motors if either axis exceeds +/- 90 degrees.
   */
  public final boolean isOutOfBounds() {
    return Math.abs(getActualPitchDeg()) > 90.0 || Math.abs(getActualRollDeg()) > 90.0;
  }

  public void resetEncoders() {
    m_KRAKEN1Pitch.setPosition(0);
    m_KRAKEN2Roll.setPosition(0);
  }

  public void pitchMove(double voltage) {
    m_KRAKEN1Pitch.setVoltage(voltage);
    m_KRAKEN2Roll.setVoltage(voltage);
  }

  public void rollMove(double voltage) {
    m_KRAKEN1Pitch.setVoltage(voltage);
    m_KRAKEN2Roll.setVoltage(-voltage);
  }

  @Override
  public void periodic() {
    // Zero encoders when both limit switches are pressed at the home position
    if (isPitchLimitSwitchTriggered() && isRollLimitSwitchTriggered()) {
      resetEncoders();
    }

    // Safety cutoff if the mechanism exceeds physical limits
    if (isOutOfBounds()) {
      stop();
      return;
    }

    // Read target angles from SmartDashboard (Note: remove this when using Commands)
    double targetPitchDeg = MathUtil.clamp(
        SmartDashboard.getNumber("Differential Pitch (Degrees)", 0.0), -90.0, 90.0);
    double targetRollDeg = MathUtil.clamp(
        SmartDashboard.getNumber("Differential Roll (Degrees)", 0.0), -90.0, 90.0);

    setPosition(Math.toRadians(targetPitchDeg), Math.toRadians(targetRollDeg));
  }
}
