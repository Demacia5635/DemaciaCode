// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Gripper.subsystems.subsystems;

import static frc.robot.Gripper.subsystems.constants.ConstantsGripper.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

  // GreeperPro members
  public TalonSRXMotor m_SRXSensorMotor;
  public OpticalSensor m_opticSensor;
  public UltraSonicSensor ultraSonicSensor;

  // SubsystemKraken members
  private final TalonFXMotor m_KRAKEN1Pitch;
  private final TalonFXMotor m_KRAKEN2Roll;
  private final LimitSwitch limitSwitchPitch;
  private final LimitSwitch limitSwitchRoll;

  public SubsystemGripper() {
    // GreeperPro init
    m_SRXSensorMotor = new TalonSRXMotor(frc.robot.Gripper.subsystems.constants.ConstantsGripper.kSnowBlowerConfig);
    // m_opticSensor = new OpticalSensor(ConstantsGripper.kOpticSensorConfig);
    ultraSonicSensor = new UltraSonicSensor(frc.robot.Gripper.subsystems.constants.ConstantsGripper.kUltraSonicSensorConfig);
    SmartDashboard.putData(new InstantCommand(() -> setPower(-0.6)));

    // SubsystemKraken init
    m_KRAKEN1Pitch = new TalonFXMotor(CONFIG);
    m_KRAKEN2Roll = new TalonFXMotor(CONFIG2);
    limitSwitchPitch = new LimitSwitch(LIMIT_SWITCH_CONFIG_PITCH);
    limitSwitchRoll = new LimitSwitch(LIMIT_SWITCH_CONFIG_ROLL);

    SmartDashboard.putNumber("Differential Pitch (Degrees)", 0.0);
    SmartDashboard.putNumber("Differential Roll (Degrees)", 0.0);
    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("OpticSensorState", () -> ultraSonicSensor.get(), null);
    builder.addDoubleProperty("Current", () -> m_SRXSensorMotor.getCurrentCurrent(), null);
  }

  // GreeperPro methods
  public double getOpticSensor() {
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

  // SubsystemKraken methods
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

  public final double getPitchPosition() {
    return m_KRAKEN1Pitch.getPosition().getValueAsDouble();
  }

  public final double getRollPosition() {
    return m_KRAKEN2Roll.getPosition().getValueAsDouble();
  }

  public final double getPitchPositionDegrees() {
    return Math.toDegrees(getPitchPosition());
  }

  public final double getRollPositionDegrees() {
    return Math.toDegrees(getRollPosition());
  }

  public final void Is_Out_Max() {
    if (getPitchPositionDegrees() > 90 || getRollPositionDegrees() > 90) {
      stop();
    }
  }

  public void resetPitchEncoder() {
    m_KRAKEN1Pitch.getCurrentPosition();
  }

  public void Pitchmove(double speed) {
    m_KRAKEN1Pitch.setVoltage(speed);
    m_KRAKEN2Roll.setVoltage(speed);
  }

  public void rollmove(double speed) {
    m_KRAKEN1Pitch.setVoltage(speed);
    m_KRAKEN2Roll.setVoltage(-speed);
  }

  @Override
  public void periodic() {
    // SubsystemKraken periodic logic
    if (isPitchLimitSwitchTriggered() && isRollLimitSwitchTriggered()) {
      m_KRAKEN1Pitch.setPosition(0);
      m_KRAKEN2Roll.setPosition(0);
    }

    double targetPitchDeg = MathUtil.clamp(
        SmartDashboard.getNumber("Differential Pitch (Degrees)", 0.0), -90.0, 90.0);
    double targetRollDeg = MathUtil.clamp(
        SmartDashboard.getNumber("Differential Roll (Degrees)", 0.0), -90.0, 90.0);

    setPosition(Math.toRadians(targetPitchDeg), Math.toRadians(targetRollDeg));

    double motor1Rad = getPitchPosition();
    double motor2Rad = getRollPosition();
    double actualPitchDeg = Math.toDegrees((motor1Rad + motor2Rad) / 2.0);
    double actualRollDeg = Math.toDegrees((motor1Rad - motor2Rad) / 2.0);

    SmartDashboard.putNumber("Actual Pitch (Degrees)", actualPitchDeg);
    SmartDashboard.putNumber("Actual Roll (Degrees)", actualRollDeg);
  }
}
