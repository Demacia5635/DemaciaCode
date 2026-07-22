package frc.demacia.utils.motors;

import java.util.function.Supplier;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.demacia.utils.dashboard.ElasticGenerator;
import frc.demacia.utils.log.Log;
import frc.demacia.utils.log.Log.LogLevel;
import frc.demacia.utils.sysid.Sysid;

/**
 * Wrapper class for the REV Spark Max motor controller.
 * <p>
 * Handles configuration, PID control, logging, and on-the-fly tuning via SmartDashboard.
 * Uses the REV Lib 2025 API.
 * </p>
 */
public class SparkMaxMotor extends SparkMax implements MotorInterface {

  SparkMaxConfig config;
  String name;
  com.revrobotics.spark.config.SparkMaxConfig cfg;

  ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;
  ControlType controlType = ControlType.kDutyCycle;

  double wantedValue;
  double testValue;

  ControlMode notDutyControlMode = ControlMode.DISABLE;
  ControlMode controlMode = ControlMode.DISABLE;
  ControlMode valueControlMode = ControlMode.DUTYCYCLE;
  SendableChooser<ControlMode> valueControlModeChooser = new SendableChooser<>();

  // Variables for manual velocity/acceleration calculation
  private double lastVelocity = 0;
  private double lastAcceleration = 0;
  private double lastTime = 0;

  // Motor Stalling
  private final Timer stallTimer = new Timer();
  private boolean conditionActive = false;
  private boolean isDone = false;
  private boolean isStalled = false;

  private boolean[] kFlags = {true, true, true, false, false, false};

  /**
   * Creates a new Spark Max motor wrapper.
   * @param config The configuration object
   */
  public SparkMaxMotor(SparkMaxConfig config) {
    super(config.id, SparkLowLevel.MotorType.kBrushless);
    this.config = config;
    name = config.name;
    configMotor();
    addLog();
    setName(name);
    SmartDashboard.putData("motors/" + name, this);
    Log.log(name + " motor initialized");
    ElasticGenerator.getInstance().registerMotor(this);
    Sysid.registerMotor(this);
  }

  public SparkMaxConfig getConfig() {
    return config;
  }

  /**
   * Applies the configuration to the motor.
   * Sets current limits, ramps, inversion, idle mode, and PID slots.
   */
  private void configMotor() {
    cfg = new com.revrobotics.spark.config.SparkMaxConfig();
    cfg.smartCurrentLimit((int) config.maxCurrent);
    cfg.openLoopRampRate(config.rampUpTime);
    cfg.closedLoopRampRate(config.rampUpTime);
    cfg.inverted(config.inverted);
    cfg.idleMode(config.brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    cfg.voltageCompensation(config.maxVolt);
    cfg.encoder.positionConversionFactor(1 / config.motorRatio);
    cfg.encoder.velocityConversionFactor(1 / config.motorRatio);
    updatePID(false);
    if (config.maxVelocity != 0) {
      cfg.closedLoop.maxMotion.cruiseVelocity(config.maxVelocity).maxAcceleration(config.maxAcceleration);
    }
    configure(cfg, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

  @SuppressWarnings({"unchecked"})
  private void addLog() {
    Log.putData(name + ": Position, Velocity, Acceleration, Voltage, Current, CloseLoopError, CloseLoopSP", 
        new Supplier[]{
          () -> getCurrentPosition(),
          () -> getCurrentVelocity(),
          () -> getCurrentAcceleration(),
          () -> getCurrentVoltage(),
          () -> getCurrentCurrent(),
          () -> getCurrentClosedLoopError(),
          () -> getCurrentClosedLoopSP(),
          () -> getCurrentControlModeInteger()
        }, LogLevel.LOG_ONLY, "motors", false);

    Log.putData("motors/" + name + "/wanted value", this::getWantedValue);
    Log.putData("motors/" + name + "/current value", this::getCurrentValue);
    Log.putData("motors/" + name + "/is Connected", this::isConnected);

    SmartDashboard.putData("motors/" + getName() + "/test value command", 
      new RunCommand(() -> applyControlModeValue(valueControlMode, testValue))
        .finallyDo(interrupted -> stop()));
    
    valueControlModeChooser.setDefaultOption(ControlMode.DUTYCYCLE.name(), ControlMode.DUTYCYCLE);
    for (ControlMode mode : ControlMode.class.getEnumConstants()) {
      if (mode == ControlMode.DISABLE) continue;
      valueControlModeChooser.addOption(mode.name(), mode);
    }
    valueControlModeChooser.onChange(mode -> this.valueControlMode = mode);
    SmartDashboard.putData("motors/" + getName() + "/Value Control Mode Chooser", valueControlModeChooser);

    configPidFf(0);
    configMotionMagic();
  }

  /**
   * Updates PID constants in the config object.
   * @param apply Whether to apply the config to the motor immediately
   */
  private void updatePID(boolean apply) {
    cfg.closedLoop.pid(config.pid[0].kP(), config.pid[0].kI(), config.pid[0].kD(), ClosedLoopSlot.kSlot0);
    cfg.closedLoop.feedForward.kV(config.pid[0].kV(), ClosedLoopSlot.kSlot0)
      .kA(config.pid[0].kA(), ClosedLoopSlot.kSlot0)
      .kS(config.pid[0].kS(), ClosedLoopSlot.kSlot0)
      .kG(config.pid[0].kG(), ClosedLoopSlot.kSlot0);
      
    cfg.closedLoop.pid(config.pid[1].kP(), config.pid[1].kI(), config.pid[1].kD(), ClosedLoopSlot.kSlot1);
    cfg.closedLoop.feedForward.kV(config.pid[1].kV(), ClosedLoopSlot.kSlot1)
      .kA(config.pid[1].kA(), ClosedLoopSlot.kSlot1)
      .kS(config.pid[1].kS(), ClosedLoopSlot.kSlot1)
      .kG(config.pid[1].kG(), ClosedLoopSlot.kSlot1);
      
    cfg.closedLoop.pid(config.pid[2].kP(), config.pid[2].kI(), config.pid[2].kD(), ClosedLoopSlot.kSlot2);
    cfg.closedLoop.feedForward.kV(config.pid[2].kV(), ClosedLoopSlot.kSlot2)
      .kA(config.pid[2].kA(), ClosedLoopSlot.kSlot2)
      .kS(config.pid[2].kS(), ClosedLoopSlot.kSlot2)
      .kG(config.pid[2].kG(), ClosedLoopSlot.kSlot2);
      
    cfg.closedLoop.pid(config.pid[3].kP(), config.pid[3].kI(), config.pid[3].kD(), ClosedLoopSlot.kSlot3);
    cfg.closedLoop.feedForward.kV(config.pid[3].kV(), ClosedLoopSlot.kSlot3)
      .kA(config.pid[3].kA(), ClosedLoopSlot.kSlot3)
      .kS(config.pid[3].kS(), ClosedLoopSlot.kSlot3)
      .kG(config.pid[3].kG(), ClosedLoopSlot.kSlot3);

    if (apply) {
      configure(cfg, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }
  }

  private double velocityFeedForward(double velocity) {
    return velocity * velocity * Math.signum(velocity) * config.kv2;
  }

  private double positionFeedForward(double position) {
    return Math.cos(position * config.posToRad) * config.kSin;
  }

  // --- MotorInterface Implementations ---

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void setName(String name) {
    MotorInterface.super.setName(name);
    this.name = name;
  }

  @Override
  public boolean isConnected() {
    return getFirmwareVersion() != 0;
  }

  @Override
  public void checkElectronics() {
    Faults faults = getFaults();
    boolean hasFault = faults.other || faults.motorType || faults.sensor || 
      faults.can || faults.temperature;

    if (hasFault) {
        Log.log(name + " Fault Detected: " + faults.toString(), AlertType.kError);
    }
  }

  @Override
  public void updateStallDetection() {
    if (config.conditionIsTrue == null || config.lowVelocityThreshold == 0)
      return;
    double currentVelocity = Math.abs(getCurrentVelocity());
    double currentCurrent = getCurrentCurrent();
    if (currentCurrent > config.highCurrentThreshold && currentVelocity < config.lowVelocityThreshold) {
      if (!conditionActive) {
        stallTimer.restart();
        conditionActive = true;
        isDone = false;
        isStalled = true;
      }
      if (stallTimer.hasElapsed(config.secondsThreshold) && !isDone) {
        config.conditionIsTrue.accept(config);
        isDone = true;
      }
    } else if (conditionActive) {
      stallTimer.stop();
      stallTimer.reset();
      conditionActive = false;
      isDone = false;
      isStalled = false;
    }
  }

  @Override
  public boolean getStallDetection() {
    return isStalled;
  }

  @Override
  public void changeSlot(int slot) {
    if (slot < 0 || slot > 3) {
      Log.log("Slot must be between 0 and 3", AlertType.kError);
      return;
    }
    this.closedLoopSlot = slot == 0 ? ClosedLoopSlot.kSlot0 : 
                          slot == 1 ? ClosedLoopSlot.kSlot1 : 
                          slot == 2 ? ClosedLoopSlot.kSlot2 : 
                          ClosedLoopSlot.kSlot3;
  }

  @Override
  public void setNeutralMode(boolean isBrake) {
    cfg.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    configure(cfg, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public double getWantedValue() {
    return wantedValue;
  }

  @Override
  public void stop() {
    stopMotor();
    wantedValue = 0;
    if (controlMode != ControlMode.DISABLE && controlMode != ControlMode.DUTYCYCLE){
      notDutyControlMode = controlMode;
    }
    controlMode = ControlMode.DISABLE;
  }

  @Override
  public void setDuty(double power) {
    super.set(power);
    wantedValue = power;
    controlType = ControlType.kDutyCycle;
    if (power == 0) {
      if (controlMode != ControlMode.DISABLE && controlMode != ControlMode.DUTYCYCLE){
        notDutyControlMode = controlMode;
      }
      controlMode = ControlMode.DISABLE;
    } else {
      if (controlMode != ControlMode.DISABLE && controlMode != ControlMode.DUTYCYCLE){
        notDutyControlMode = controlMode;
      }
      controlMode = ControlMode.DUTYCYCLE;
    }
  }

  @Override
  public void setVolt(double voltage) {
    super.setVoltage(voltage);
    wantedValue = voltage;
    controlType = ControlType.kVoltage;
    controlMode = ControlMode.VOLTAGE;
  }

  @Override
  public void setVelocity(double velocity, double feedForward) {
    getClosedLoopController().setSetpoint(velocity, ControlType.kMAXMotionVelocityControl, closedLoopSlot, feedForward + velocityFeedForward(velocity));
    wantedValue = velocity;
    controlType = ControlType.kMAXMotionVelocityControl;
    controlMode = ControlMode.VELOCITY;
  }

  @Override
  public void setVelocity(double velocity) {
    setVelocity(velocity, 0);
  }

  @Override
  public void setVelocityWithAcceleration(double velocity, Supplier<Double> wantedAccelerationSupplier) {
    setVelocity(velocity, wantedAccelerationSupplier.get() * config.pid[0].kA());
  }

  @Override
  public void setMotion(double position, double feedForward) {
    getClosedLoopController().setSetpoint(position, ControlType.kMAXMotionPositionControl, closedLoopSlot, feedForward + positionFeedForward(position));
    wantedValue = position;
    controlType = ControlType.kMAXMotionPositionControl;
    controlMode = ControlMode.MAGIC_MOTION;
  }

  @Override
  public void setMotion(double position) {
    setMotion(position, 0.0);
  }

  @Override
  public void setAngle(double angle, double feedForward) {
    setMotion(getCurrentPosition() + MathUtil.angleModulus(angle - getCurrentAngle()), feedForward);
    wantedValue = MathUtil.angleModulus(angle);
    controlMode = ControlMode.ANGLE;
  }

  @Override
  public void setAngle(double angle) {
    setAngle(angle, 0);
  }

  @Override
  public void setPositionVoltage(double position, double feedForward) {
    getClosedLoopController().setSetpoint(position, ControlType.kPosition, closedLoopSlot, feedForward + positionFeedForward(position));
    wantedValue = position;
    controlType = ControlType.kPosition;
    controlMode = ControlMode.POSITION_VOLTAGE;
  }

  @Override
  public void setPositionVoltage(double position) {
    setPositionVoltage(position, 0);
  }

  @Override
  public void setEncoderPosition(double position) {
    getEncoder().setPosition(position);
  }

  @Override
  public int getCurrentControlModeInteger() {
    return controlMode.ordinal();
  }

  @Override
  public ControlMode getCurrentControlMode() {
    return controlMode;
  }

  @Override
  public ControlMode getLastControlMode() {
    return notDutyControlMode;
  }

  @Override
  public double getCurrentClosedLoopSP() {
    return wantedValue;
  }

  @Override
  public double getCurrentClosedLoopError() {
    return getCalculatedError();
  }

  @Override
  public double getCurrentPosition() {
    return getEncoder().getPosition();
  }

  @Override
  public double getCurrentVelocity() {
    return getEncoder().getVelocity();
  }

  @Override
  public double getCurrentAcceleration() {
    double currentTimestamp = Timer.getFPGATimestamp();
    double dt = currentTimestamp - lastTime;

    if (dt < 0.001) { 
        return lastAcceleration;
    }

    double currentVelocity = getCurrentVelocity();
    
    lastAcceleration = (currentVelocity - lastVelocity) / dt;
    
    lastVelocity = currentVelocity;
    lastTime = currentTimestamp;

    return lastAcceleration;
  }

  @Override
  public double getCurrentAngle() {
    if (config.isRadiansMotor) {
      return MathUtil.angleModulus(getCurrentPosition());
    }
    return 0;
  }

  @Override
  public double getCurrentVoltage() {
    return getAppliedOutput() * 12.0; 
  }

  @Override
  public double getCurrentCurrent() {
    return getOutputCurrent();
  }

  // --- MotorInterface Abstract Hardware Hooks ---

  @Override
  public CloseLoopParam getPidParam(int slot) {
    return config.pid[slot];
  }

  @Override
  public void applyPidHardware(int slot) {
    ClosedLoopSlot revSlot = slot == 0 ? ClosedLoopSlot.kSlot0 : 
                             slot == 1 ? ClosedLoopSlot.kSlot1 : 
                             slot == 2 ? ClosedLoopSlot.kSlot2 : 
                             ClosedLoopSlot.kSlot3;

    cfg.closedLoop.pid(config.pid[slot].kP(), config.pid[slot].kI(), config.pid[slot].kD(), revSlot);
    cfg.closedLoop.feedForward.kV(config.pid[slot].kV(), revSlot)
      .kA(config.pid[slot].kA(), revSlot)
      .kS(config.pid[slot].kS(), revSlot)
      .kG(config.pid[slot].kG(), revSlot);

    configure(cfg, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void applyMotionMagicHardware() {
    cfg.closedLoop.maxMotion.cruiseVelocity(config.maxVelocity).maxAcceleration(config.maxAcceleration);
    configure(cfg, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public double getMaxVelocity() { return config.maxVelocity; }

  @Override
  public void setMaxVelocity(double velocity) { config.maxVelocity = velocity; }

  @Override
  public double getMaxAcceleration() { return config.maxAcceleration; }

  @Override
  public void setMaxAcceleration(double acceleration) { config.maxAcceleration = acceleration; }

  @Override
  public double getKSin() { return config.kSin; }

  @Override
  public void setKSin(double kSin) { config.kSin = kSin; }

  @Override
  public double getKV2() { return config.kv2; }

  @Override
  public void setKV2(double kV2) { config.kv2 = kV2; }

  @Override
  public void updatePid(CloseLoopParam newParams, int slot) {
    config.pid[slot].setKP(newParams.kP());
    config.pid[slot].setKI(newParams.kI());
    config.pid[slot].setKD(newParams.kD());
    config.pid[slot].setKS(newParams.kS());
    config.pid[slot].setKV(newParams.kV());
    config.pid[slot].setKA(newParams.kA());
    config.pid[slot].setKG(newParams.kG());
    applyPidHardware(slot);
  }

  @Override
  public boolean[] getSysidFlags() { return kFlags; }

  @Override
  public boolean isRadiansMotor() { return config.isRadiansMotor; }

  @Override
  public double getTestValue() { return testValue; }

  @Override
  public void setTestValue(double testValue) { this.testValue = testValue; }

  public double gearRatio() {
    return config.motorRatio;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    initCommonSendable(builder);
  }
}