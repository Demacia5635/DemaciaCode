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
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.UltraSonicSensor;

public class SubsystemGripper extends SubsystemBase {

  // ------------------------------------------------- Snowblower Claw (TalonSRX) -----------------------------------------------------
  public TalonSRXMotor m_snowblowerClawMotor;
  private boolean isBottleSecured = false;
  private double ZeroPosition;



  // ------------------------------------------------------- Ultrasonic Sensor --------------------------------------------------------
  public UltraSonicSensor m_ultraSonicSensor;

  // -------------------------------------------------- Kraken Motors (TalonFX) -------------------------------------------------------
  private final TalonFXMotor m_krakenPitch;
  private final TalonFXMotor m_krakenRoll;

  // --------------------------------------------------------- Limit Switches ---------------------------------------------------------
  private final LimitSwitch m_limitSwitchPitch;
  private final LimitSwitch m_limitSwitchRoll;

  public SubsystemGripper() {
    // ----------------------------------------------- Snowblower Claw Init -----------------------------------------------------------
    m_snowblowerClawMotor = new TalonSRXMotor(SNOWBLOWER_CLAW_CONFIG);
    SmartDashboard.putData("Test Claw Reverse", new InstantCommand(() -> setClawPower(-0.6)));

    // ---------------------------------------------- Ultrasonic Sensor Init ----------------------------------------------------------
    m_ultraSonicSensor = new UltraSonicSensor(ULTRASONIC_SENSOR_CONFIG);

    // ----------------------------------------------- Kraken Motors Init -------------------------------------------------------------
    m_krakenPitch = new TalonFXMotor(KRAKEN_PITCH_CONFIG);
    m_krakenRoll = new TalonFXMotor(KRAKEN_ROLL_CONFIG);

    // ------------------------------------------------ Limit Switches Init -----------------------------------------------------------
    m_limitSwitchPitch = new LimitSwitch(KRAKEN_PITCH_LIMIT_SWITCH_CONFIG);
    m_limitSwitchRoll = new LimitSwitch(KRAKEN_ROLL_LIMIT_SWITCH_CONFIG);

    SmartDashboard.putNumber("Differential Pitch (Degrees)", 0.0);
    SmartDashboard.putNumber("Differential Roll (Degrees)", 0.0);
    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("UltraSonicDistance", this::getBottleDistanceMeters, null);
    builder.addDoubleProperty("Current", this::getClawMotorCurrent, null);
  }

  // ----------------------------------------------- Snowblower Claw Methods ----------------------------------------------------------
  public void setClawPower(double power) {
    m_snowblowerClawMotor.setDuty(power);
  }

  public void stopClaw() {
    m_snowblowerClawMotor.stop();
  }

  public double getClawMotorCurrent() {
    return m_snowblowerClawMotor.getCurrentCurrent();
  }

  public void runAutoBottleIntake() {
    double distance = getBottleDistanceMeters();
    if (DISTANCE_TO_BOTTLE < distance && DISTANCE_TO_BOTTLE_CLAMP > distance && !isBottleSecured) {
      setClawPower(0);
      LogManager.log("Bottle clamped - stopping claw");
    } else if (DISTANCE_TO_BOTTLE > distance && DISTANCE_TO_BOTTLE_CLAMP > distance && !isBottleSecured) {
      setClawPower(0.6);
      LogManager.log("Bottle detected - intaking");
    }
  }

  // ---------------------------------------------- Ultrasonic Sensor Methods ---------------------------------------------------------
  public double getBottleDistanceMeters() {
    return m_ultraSonicSensor.get();
  }

  // ------------------------------------------------ Limit Switch Methods ------------------------------------------------------------
  public boolean isPitchAtHomeLimit() {
    return m_limitSwitchPitch.get();
  }

  public boolean isRollAtHomeLimit() {
    return m_limitSwitchRoll.get();
  }

  public boolean isWristAtHomeLimit() {
    return isPitchAtHomeLimit() && isRollAtHomeLimit();
  }

  // ------------------------------------------------ Kraken Motor Methods ------------------------------------------------------------
  public void setWristPositionRadians(double targetPitchRadians, double targetRollRadians) {
    MathUtil.clamp(targetPitchRadians, KRAKEN_PITCH_MIN_RADIANS, KRAKEN_PITCH_MAX_RADIANS);
    m_krakenPitch.setPositionVoltage(targetPitchRadians + targetRollRadians);
    m_krakenRoll.setPositionVoltage(targetPitchRadians - targetRollRadians);
  }

  public void moveWristPitchVoltage(double volts) {
    m_krakenPitch.setVoltage(volts);
    m_krakenRoll.setVoltage(volts);
  }

  public void moveWristRollVoltage(double volts) {
    m_krakenPitch.setVoltage(volts);
    m_krakenRoll.setVoltage(-volts);
  }

  public void stopWristMotors() {
    m_krakenPitch.stop();
    m_krakenRoll.stop();
  }

  public void resetWristEncoders() {
    m_krakenPitch.setPosition(0);
    m_krakenRoll.setPosition(0);
  }

  public final double getPitchMotorPositionRadians() {
    return m_krakenPitch.getPosition().getValueAsDouble();
  }

  public final double getRollMotorPositionRadians() {
    return m_krakenRoll.getPosition().getValueAsDouble();
  }

  public final double getActualWristPitchRadians() {
    return (getPitchMotorPositionRadians() + getRollMotorPositionRadians()) / 2.0;
  }

  public final double getActualWristRollRadians() {
    return (getPitchMotorPositionRadians() - getRollMotorPositionRadians()) / 2.0;
  }
  //-------------------------------------------------------Calibrate-------------------------------------------
  public final void calibrate() {
    if (Is_Calibraete){
      
      setWristPositionRadians(1.57079633,1.57079633 );
      setWristPositionRadians(1.57079633,1.57079633 );
        if (isPitchAtHomeLimit()){


        }
      
      
    }

  }


  // ------------------------------------------------------- Periodic Loop ------------------------------------------------------------
  @Override
  public void periodic() {
  }
}
