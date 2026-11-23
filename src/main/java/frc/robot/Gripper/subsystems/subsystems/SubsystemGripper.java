package frc.robot.Gripper.subsystems.subsystems;

import static frc.robot.Gripper.subsystems.constants.ConstantsGripper.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.UltraSonicSensor;

public class SubsystemGripper extends SubsystemBase {

  // =========================================================================
  // Enums & State
  // =========================================================================

  public enum CalibrateSide {
    NONE,
    LEFT,
    RIGHT
  }

  private GripperState state = GripperState.IDLE;
  private CalibrateSide calibrateSide = CalibrateSide.NONE;

  // --- Snowblower Claw (TalonSRX) ---
  public TalonSRXMotor m_snowblowerClawMotor;
  private boolean isBottleSecured = false;

  // --- Ultrasonic Sensor ---
  public UltraSonicSensor m_ultraSonicSensor;

  // --- Kraken Motors (TalonFX) ---
  private final TalonFXMotor m_krakenPitch;
  private final TalonFXMotor m_krakenRoll;

  // --- Limit Switches ---
  private final LimitSwitch m_limitSwitchPitch;
  private final LimitSwitch m_limitSwitchRoll;

  // =========================================================================
  // Constructor & Telemetry
  // =========================================================================

  public SubsystemGripper() {
    // Snowblower Claw Init
    m_snowblowerClawMotor = new TalonSRXMotor(SNOWBLOWER_CLAW_CONFIG);
    SmartDashboard.putData("Test Claw Reverse", new InstantCommand(() -> setClawPower(-0.6)));

    // Ultrasonic Sensor Init
    m_ultraSonicSensor = new UltraSonicSensor(ULTRASONIC_SENSOR_CONFIG);

    // Kraken Motors Init
    m_krakenPitch = new TalonFXMotor(KRAKEN_PITCH_CONFIG);
    m_krakenRoll = new TalonFXMotor(KRAKEN_ROLL_CONFIG);

    // Limit Switches Init
    m_limitSwitchPitch = new LimitSwitch(KRAKEN_PITCH_LIMIT_SWITCH_CONFIG);
    m_limitSwitchRoll = new LimitSwitch(KRAKEN_ROLL_LIMIT_SWITCH_CONFIG);

    // Publish Subsystem & Elastic Choosers
    SmartDashboard.putData("Gripper Subsystem", this);
    addSendableChooser();
    addCalibrateChooser();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("UltraSonicDistance", this::getBottleDistanceMeters, null);
    builder.addDoubleProperty("Current", this::getClawMotorCurrent, null);
    builder.addStringProperty("Current State", () -> state.name(), null);
    builder.addStringProperty("Calibrate Side", () -> calibrateSide.name(), null);
    builder.addBooleanProperty("Pitch Limit Switch", this::isPitchAtHomeLimit, null);
    builder.addBooleanProperty("Roll Limit Switch", this::isRollAtHomeLimit, null);
    builder.addDoubleProperty("Pitch Position", m_krakenPitch::getCurrentPosition, null);
    builder.addDoubleProperty("Roll Position", m_krakenRoll::getCurrentPosition, null);
  }

  // =========================================================================
  // State & Calibration Choosers (Elastic)
  // =========================================================================

  public void setState(GripperState newState) {
    state = newState;
    LogManager.log("Gripper state changed to: " + state);
  }

  public GripperState getState() {
    return state;
  }

  public CalibrateSide getCalibrateSide() {
    return calibrateSide;
  }

  public void setCalibrateSide(CalibrateSide side) {
    calibrateSide = side;
    LogManager.log("Calibrate side changed to: " + calibrateSide);
  }

  public void addSendableChooser() {
    SendableChooser<GripperState> stateChooser = new SendableChooser<>();
    stateChooser.setDefaultOption(GripperState.IDLE.name(), GripperState.IDLE);
    for (GripperState s : GripperState.values()) {
      stateChooser.addOption(s.name(), s);
    }
    stateChooser.onChange(this::setState);
    SmartDashboard.putData("Gripper State Chooser", stateChooser);
  }

  public void addCalibrateChooser() {
    SendableChooser<CalibrateSide> calibrateChooser = new SendableChooser<>();
    calibrateChooser.setDefaultOption(CalibrateSide.NONE.name(), CalibrateSide.NONE);
    for (CalibrateSide side : CalibrateSide.values()) {
      calibrateChooser.addOption(side.name(), side);
    }
    calibrateChooser.onChange(this::setCalibrateSide);
    SmartDashboard.putData("Calibrate Side Chooser", calibrateChooser);
  }

  // =========================================================================
  // Snowblower Claw Control
  // =========================================================================

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

  // =========================================================================
  // Sensors & Limit Switches
  // =========================================================================

  public double getBottleDistanceMeters() {
    return m_ultraSonicSensor.get();
  }

  public boolean isPitchAtHomeLimit() {
    return m_limitSwitchPitch.get();
  }

  public boolean isRollAtHomeLimit() {
    return m_limitSwitchRoll.get();
  }

  public boolean isWristAtHomeLimit() {
    return isPitchAtHomeLimit() && isRollAtHomeLimit();
  }

  // =========================================================================
  // Kraken Wrist Control & Calibration
  // =========================================================================

  public void setWristPositionRadians(double targetPitchRadians, double targetRollRadians) {
    targetPitchRadians = MathUtil.clamp(targetPitchRadians, KRAKEN_PITCH_MIN_RADIANS, KRAKEN_PITCH_MAX_RADIANS);
    m_krakenPitch.setPositionVoltage(targetPitchRadians + targetRollRadians);
    m_krakenRoll.setPositionVoltage(targetPitchRadians - targetRollRadians);
  }

  public void setpowerwrist(double powerPitch, double powerRoll) {
    m_krakenPitch.setDuty(powerPitch + powerRoll);
    m_krakenRoll.setDuty(powerPitch - powerRoll);
  }

  public void moveWristVoltage(double voltsPitch, double voltsRoll) {
    m_krakenPitch.setVoltage(voltsPitch + voltsRoll);
    m_krakenRoll.setVoltage(voltsPitch - voltsRoll);
  }

  public void stopWristMotors() {
    m_krakenPitch.stop();
    m_krakenRoll.stop();
  }

  public void Rightcalibrate() {
    setpowerwrist(0.6, 0.6);
    if (isPitchAtHomeLimit()) {
      stopWristMotors();
      m_krakenPitch.setEncoderPosition(0);
      m_krakenRoll.setEncoderPosition(0);
      ZeroPositionCalibrte = 0;
    }
  }

  public void Leftcalibrate() {
    setpowerwrist(-0.6, -0.6);
    if (isPitchAtHomeLimit()) {
      stopWristMotors();
      m_krakenPitch.setEncoderPosition(0);
      m_krakenRoll.setEncoderPosition(0);
      ZeroPositionCalibrte = 0;
    }
  }

  // =========================================================================
  // Periodic Loop
  // =========================================================================

  @Override
  public void periodic() {}
}
