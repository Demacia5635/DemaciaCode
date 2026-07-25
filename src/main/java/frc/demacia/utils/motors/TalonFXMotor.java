package frc.demacia.utils.motors;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.demacia.utils.Data;
import frc.demacia.utils.elastic.ElasticGenerator;
import frc.demacia.utils.log.Log;
import frc.demacia.utils.log.Log.LogLevel;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sysid.Sysid;

/**
 * Wrapper class for the TalonFX motor controller using Phoenix 6.
 * <p>
 * Handles configuration, control requests (Voltage, Velocity, MotionMagic),
 * and integrates with the logging system and SmartDashboard.
 * </p>
 */
public class TalonFXMotor extends TalonFX implements MotorInterface {

  TalonFXConfig config;
  String name;
  TalonFXConfiguration cfg;

  int slot = 0;

  // Phoenix 6 Control Requests
  DutyCycleOut dutyCycle = new DutyCycleOut(0);
  VoltageOut voltageOut = new VoltageOut(0);
  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(slot);
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(slot);
  PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(slot);
  MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0).withSlot(slot);

  // Data Signals for Logging
  Data<Double> closedLoopSPSignal;
  Data<Double> closedLoopErrorSignal;
  Data<Angle> positionSignal;
  Data<AngularVelocity> velocitySignal;
  Data<AngularAcceleration> accelerationSignal;
  Data<Voltage> voltageSignal;
  Data<Current> currentSignal;

  double wantedValue;
  double testValue;

  ControlMode notDutyControlMode = ControlMode.DISABLE;
  ControlMode controlMode = ControlMode.DISABLE;

  ControlMode valueControlMode = ControlMode.DUTYCYCLE;
  SendableChooser<ControlMode> valueControlModeChooser = new SendableChooser<>();

  private boolean[] kFlags = {true, true, true, false, false, false};

  // Motor Stalling
  private final Timer stallTimer = new Timer();
  private boolean conditionActive = false;
  private boolean isDone = false;
  private boolean isStalled = false;

  /**
   * Creates a new TalonFX motor wrapper.
   * @param config The configuration object for this motor
   */
  public TalonFXMotor(TalonFXConfig config) {
    super(config.id, config.canbus.canbus);
    this.config = config;
    name = config.name;
    configMotor();
    setSignals();
    addLog();
    setName(name);
    SmartDashboard.putData("motors/" + name, this);
    Log.log(name + " motor initialized");
    ElasticGenerator.getInstance().registerMotor(this);
    Sysid.registerMotor(this);
  }

  public TalonFXConfig getConfig() {
    return config;
  }

  /**
   * Applies the initial configuration to the motor.
   * Sets limits, ramps, PID, and Motion Magic parameters.
   */
  private void configMotor() {
    cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.SupplyCurrentLimit = config.maxCurrent;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = config.maxCurrent;
    cfg.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = config.rampUpTime;
    cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.rampUpTime;
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.rampUpTime;
    cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.rampUpTime;

    cfg.MotorOutput.Inverted = config.inverted ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    cfg.MotorOutput.PeakForwardDutyCycle = config.maxVolt / 12.0;
    cfg.MotorOutput.PeakReverseDutyCycle = config.minVolt / 12.0;
    cfg.Feedback.SensorToMechanismRatio = config.motorRatio;
    updatePID(false);
    cfg.Voltage.PeakForwardVoltage = config.maxVolt;
    cfg.Voltage.PeakReverseVoltage = config.minVolt;
    configureMotionMagic(false);

    getConfigurator().apply(cfg);
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

  public void configSoftwareLimit(double min, double max) {
    SoftwareLimitSwitchConfigs limitCfg = new SoftwareLimitSwitchConfigs();
    limitCfg.ForwardSoftLimitEnable = max != Double.MAX_VALUE;
    limitCfg.ReverseSoftLimitEnable = min != Double.MAX_VALUE;
    limitCfg.ForwardSoftLimitThreshold = max;
    limitCfg.ReverseSoftLimitThreshold = min;
    getConfigurator().apply(limitCfg);
  }

  private void configureMotionMagic(boolean apply) {
    cfg.MotionMagic.MotionMagicAcceleration = config.maxAcceleration;
    cfg.MotionMagic.MotionMagicCruiseVelocity = config.maxVelocity;
    cfg.MotionMagic.MotionMagicJerk = config.maxJerk;
    cfg.MotionMagic.MotionMagicExpo_kA = config.pid[0].kA();
    cfg.MotionMagic.MotionMagicExpo_kV = config.pid[0].kV();

    if (apply) {
      getConfigurator().apply(cfg.MotionMagic);
    }
  }

  private void updatePID(boolean apply) {
    cfg.Slot0.kP = config.pid[0].kP();
    cfg.Slot0.kI = config.pid[0].kI();
    cfg.Slot0.kD = config.pid[0].kD();
    cfg.Slot0.kS = config.pid[0].kS();
    cfg.Slot0.kV = config.pid[0].kV();
    cfg.Slot0.kA = config.pid[0].kA();
    cfg.Slot0.kG = config.pid[0].kG();
    cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    cfg.Slot1.kP = config.pid[1].kP();
    cfg.Slot1.kI = config.pid[1].kI();
    cfg.Slot1.kD = config.pid[1].kD();
    cfg.Slot1.kS = config.pid[1].kS();
    cfg.Slot1.kV = config.pid[1].kV();
    cfg.Slot1.kA = config.pid[1].kA();
    cfg.Slot1.kG = config.pid[1].kG();
    cfg.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    cfg.Slot2.kP = config.pid[2].kP();
    cfg.Slot2.kI = config.pid[2].kI();
    cfg.Slot2.kD = config.pid[2].kD();
    cfg.Slot2.kS = config.pid[2].kS();
    cfg.Slot2.kV = config.pid[2].kV();
    cfg.Slot2.kA = config.pid[2].kA();
    cfg.Slot2.kG = config.pid[2].kG();
    cfg.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    if (apply) {
      getConfigurator().apply(cfg.Slot0);
      getConfigurator().apply(cfg.Slot1);
      getConfigurator().apply(cfg.Slot2);
    }
  }

  public boolean isRio() {
    return config.canbus.equals(Canbus.Rio);
  }

  @Override
  public void setName(String name) {
    MotorInterface.super.setName(name);
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }

  private void setSignals() {
    closedLoopSPSignal = new Data<>(getClosedLoopReference(), isRio());
    closedLoopErrorSignal = new Data<>(getClosedLoopError(), isRio());
    positionSignal = new Data<>(getPosition(), isRio());
    velocitySignal = new Data<>(getVelocity(), isRio());
    accelerationSignal = new Data<>(getAcceleration(), isRio());
    voltageSignal = new Data<>(getMotorVoltage(), isRio());
    currentSignal = new Data<>(getStatorCurrent(), isRio());
  }

  @SuppressWarnings({ "unchecked" })
  private void addLog() {
    Log.putData(name + ": Position, Velocity, Acceleration, Voltage, Current, CloseLoopError, CloseLoopSP",
            new Data[] {
                positionSignal,
                velocitySignal,
                accelerationSignal,
                voltageSignal,
                currentSignal,
                closedLoopErrorSignal,
                closedLoopSPSignal,
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

  @Override
  public void checkElectronics() {
    int fault = getFaultField().getValue();
    if (fault != 0) {
      Log.log(name + " has fault num: " + fault, AlertType.kError);
    }
  }

  @Override
  public void changeSlot(int slot) {
    if (slot < 0 || slot > 2) {
      Log.log("Slot must be between 0 and 2", AlertType.kError);
      return;
    }
    this.slot = slot;
    velocityVoltage.withSlot(slot);
    motionMagicVoltage.withSlot(slot);
    positionVoltage.withSlot(slot);
  }

  @Override
  public void setNeutralMode(boolean isBrake) {
    cfg.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    getConfigurator().apply(cfg.MotorOutput);
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
    setControl(dutyCycle.withOutput(power));
    wantedValue = power;
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
    setControl(voltageOut.withOutput(voltage));
    wantedValue = voltage;
    controlMode = ControlMode.VOLTAGE;
  }

  @Override
  public void setVelocity(double velocity, double feedForward) {
    setControl(velocityVoltage.withVelocity(velocity).withFeedForward(feedForward + velocityFeedForward(velocity)));
    wantedValue = velocity;
    controlMode = ControlMode.VELOCITY;
  }

  @Override
  public void setVelocity(double velocity) {
    setVelocity(velocity, 0);
  }

  @Override
  public void setVelocityWithAcceleration(double velocity, Supplier<Double> wantedAccelerationSupplier) {
    setVelocity(velocity, wantedAccelerationSupplier.get() * config.pid[slot].kA());
  }

  @Override
  public void setMotion(double position, double feedForward) {
    setControl(motionMagicVoltage.withPosition(position).withFeedForward(feedForward));
    wantedValue = position;
    controlMode = ControlMode.MAGIC_MOTION;
  }

  @Override
  public void setMotion(double position) {
    setMotion(position, 0.0);
  }

  public void setMotionExpo(double position, double feedForward) {
    setControl(
        motionMagicExpoVoltage.withPosition(position).withFeedForward(feedForward + positionFeedForward(position)));
        wantedValue = position;
        controlMode = ControlMode.MAGIC_MOTION;
  }

  public void setMotionExpo(double position) {
    setMotionExpo(position, 0);
  }

  public void setMotion(double position, int slot) {
    setControl(motionMagicVoltage.withSlot(slot).withFeedForward(positionFeedForward(position)));
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
    setControl(positionVoltage.withPosition(position).withFeedForward(feedForward));
    wantedValue = position;
    controlMode = ControlMode.POSITION_VOLTAGE;
  }

  @Override
  public void setPositionVoltage(double position) {
    setPositionVoltage(position, 0);
  }

  private double velocityFeedForward(double velocity) {
    return velocity * velocity * Math.signum(velocity) * config.pid[0].kV2();
  }

  private double positionFeedForward(double position) {
    return Math.cos(position * config.posToRad) * config.pid[0].kCos();
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
    return closedLoopSPSignal.getDouble();
  }

  @Override
  public double getCurrentClosedLoopError() {
    double hardwareError = closedLoopErrorSignal.getDouble();
      if (hardwareError != 0) {
          return hardwareError;
      }
      return getCalculatedError();
  }

  @Override
  public double getCurrentPosition() {
    return positionSignal.getDouble();
  }

  @Override
  public double getCurrentVelocity() {
    return velocitySignal.getDouble();
  }

  @Override
  public double getCurrentAcceleration() {
    return accelerationSignal.getDouble();
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
    return voltageSignal.getDouble();
  }

  @Override
  public double getCurrentCurrent() {
    return currentSignal.getDouble();
  }

  @Override
  public void setEncoderPosition(double position) {
    setPosition(position);
  }

  // --- MotorInterface Abstract Hardware Hooks ---

  @Override
  public CloseLoopParam getPidParam(int slot) {
    return config.pid[slot];
  }

  @Override
  public void applyPidHardware(int slot) {
    int activeSlot = (slot >= 0 && slot <= 2) ? slot : 0;
    
    if (activeSlot == 0) {
        cfg.Slot0.kP = config.pid[0].kP();
        cfg.Slot0.kI = config.pid[0].kI();
        cfg.Slot0.kD = config.pid[0].kD();
        cfg.Slot0.kS = config.pid[0].kS();
        cfg.Slot0.kV = config.pid[0].kV();
        cfg.Slot0.kA = config.pid[0].kA();
        cfg.Slot0.kG = config.pid[0].kG();
        getConfigurator().apply(cfg.Slot0);
    } else if (activeSlot == 1) {
        cfg.Slot1.kP = config.pid[1].kP();
        cfg.Slot1.kI = config.pid[1].kI();
        cfg.Slot1.kD = config.pid[1].kD();
        cfg.Slot1.kS = config.pid[1].kS();
        cfg.Slot1.kV = config.pid[1].kV();
        cfg.Slot1.kA = config.pid[1].kA();
        cfg.Slot1.kG = config.pid[1].kG();
        getConfigurator().apply(cfg.Slot1);
    } else if (activeSlot == 2) {
        cfg.Slot2.kP = config.pid[2].kP();
        cfg.Slot2.kI = config.pid[2].kI();
        cfg.Slot2.kD = config.pid[2].kD();
        cfg.Slot2.kS = config.pid[2].kS();
        cfg.Slot2.kV = config.pid[2].kV();
        cfg.Slot2.kA = config.pid[2].kA();
        cfg.Slot2.kG = config.pid[2].kG();
        getConfigurator().apply(cfg.Slot2);
    }
  }

  @Override
  public void applyMotionMagicHardware() {
    cfg.MotionMagic.MotionMagicAcceleration = config.maxAcceleration;
    cfg.MotionMagic.MotionMagicCruiseVelocity = config.maxVelocity;
    cfg.MotionMagic.MotionMagicJerk = config.maxJerk;
    cfg.MotionMagic.MotionMagicExpo_kA = config.pid[0].kA();
    cfg.MotionMagic.MotionMagicExpo_kV = config.pid[0].kV();
    getConfigurator().apply(cfg.MotionMagic);
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
  public double getMaxJerk() { return config.maxJerk; }

  @Override
  public void setMaxJerk(double jerk) { config.maxJerk = jerk; }

  @Override
  public void updatePid(CloseLoopParam newParams, int slot) {
    config.pid[slot].setKP(newParams.kP());
    config.pid[slot].setKI(newParams.kI());
    config.pid[slot].setKD(newParams.kD());
    config.pid[slot].setKS(newParams.kS());
    config.pid[slot].setKV(newParams.kV());
    config.pid[slot].setKA(newParams.kA());
    config.pid[slot].setKG(newParams.kG());
    config.pid[slot].setKCos(newParams.kCos());
    config.pid[slot].setKV2(newParams.kV2());
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

  // Raw Signal Accessors
  public Data<Double> getClosedLoopErrorSignal() { return closedLoopErrorSignal; }
  public Data<Double> getClosedLoopSPSignal() { return closedLoopSPSignal; }
  public Data<Angle> getPositionSignal() { return positionSignal; }
  public Data<AngularVelocity> getVelocitySignal() { return velocitySignal; }
  public Data<AngularAcceleration> getAccelerationSignal() { return accelerationSignal; }
  public Data<Voltage> getVoltageSignal() { return voltageSignal; }
  public Data<Current> getCurrentSignal() { return currentSignal; }
}