package frc.demacia.utils.motors;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SlotConfigs;
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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.demacia.utils.Data;
import frc.demacia.utils.dashboard.ElasticGenerator;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;
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

  // Data Signals for Logging
  Data<Double> closedLoopSPSignal;
  Data<Double> closedLoopErrorSignal;
  Data<Angle> positionSignal;
  Data<AngularVelocity> velocitySignal;
  Data<AngularAcceleration> accelerationSignal;
  Data<Voltage> voltageSignal;
  Data<Current> currentSignal;

  double wantedValue;
  ControlMode controlMode = ControlMode.DISABLE;

  double testPower;

  private boolean[] kFlags = {true, true, true, false, false, false};

  // Motor Stalling
  private final Timer stallTimer = new Timer();
  private boolean conditionActive = false;
  private boolean IsDone = false;
  private boolean isStalled = false;


  /**
   * Creates a new TalonFX motor wrapper.
   * 
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
    SmartDashboard.putData("motors/" + name,this);
    LogManager.log(name + " motor initialized");
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

  public void updateStallDetection() {
    if (config.conditionIsTrue == null || config.lowVelocityThreshold == 0)
      return;
    double currentVelocity = Math.abs(getCurrentVelocity());
    double currentCurrent = getCurrentCurrent();
    if (currentCurrent > config.highCurrentThreshold && currentVelocity < config.lowVelocityThreshold) {
      if (!conditionActive) {
        stallTimer.restart();
        conditionActive = true;
        IsDone = false;
        isStalled = true;

      }
      if (stallTimer.hasElapsed(config.secondsThreshold) && !IsDone) {
        config.conditionIsTrue.accept(config);
        IsDone = true;
      }
    } else if (conditionActive) {
      stallTimer.stop();
      stallTimer.reset();
      conditionActive = false;
      IsDone = false;
      isStalled = false;
    }
  }
  public boolean getStallDetection() {
  return isStalled;
}

  public void configSoftwareLimit(double min, double max) {
    SoftwareLimitSwitchConfigs cfg = new SoftwareLimitSwitchConfigs();
    cfg.ForwardSoftLimitEnable = max != Double.MAX_VALUE;
    cfg.ReverseSoftLimitEnable = min != Double.MAX_VALUE;
    cfg.ForwardSoftLimitThreshold = max;
    cfg.ReverseSoftLimitThreshold = min;
    getConfigurator().apply(cfg);
  }

  /**
   * Configures Motion Magic parameters (Velocity, Acceleration, Jerk).
   * 
   * @param apply Whether to apply the config immediately to the hardware
   */
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

  /**
   * Updates PID constants from the config object to the Phoenix configuration.
   * 
   * @param apply Whether to apply the config immediately to the hardware
   */
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

  /** Initializes the data signals for telemetry */
  @SuppressWarnings("unchecked")
  private void setSignals() {
    closedLoopSPSignal = new Data<>(new StatusSignal[] { getClosedLoopReference() }, isRio());
    closedLoopErrorSignal = new Data<>(new StatusSignal[] { getClosedLoopError() }, isRio());
    positionSignal = new Data<>(new StatusSignal[] { getPosition() }, isRio());
    velocitySignal = new Data<>(new StatusSignal[] { getVelocity() }, isRio());
    accelerationSignal = new Data<>(new StatusSignal[] { getAcceleration() }, isRio());
    voltageSignal = new Data<>(new StatusSignal[] { getMotorVoltage() }, isRio());
    currentSignal = new Data<>(new StatusSignal[] { getStatorCurrent() }, isRio());
  }

  /** Registers the motor's signals with the LogManager */
  @SuppressWarnings("unchecked")
  private void addLog() {
    LogManager
        .addEntry(name + ": Position, Velocity, Acceleration, Voltage, Current, CloseLoopError, CloseLoopSP",
            new StatusSignal[] {
                positionSignal.getSignal(),
                velocitySignal.getSignal(),
                accelerationSignal.getSignal(),
                voltageSignal.getSignal(),
                currentSignal.getSignal(),
                closedLoopErrorSignal.getSignal(),
                closedLoopSPSignal.getSignal(),
            }, isRio())
        .withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP)
        .withIsMotor()
        .withIsSeparated(false).build();
    LogManager.addEntry(name + ": ControlMode",
        () -> getCurrentControlModeInteger())
        .withLogLevel(LogLevel.LOG_ONLY_NOT_IN_COMP)
        .withIsSeparated(false).build();
    LogManager.addEntry(name + ": wanted value", () -> getWantedValue(), 
      () -> getCurrentValue())
        .withIsSeparated(false).withLogLevel(LogLevel.LOG_AND_NT).build();
    LogManager.addEntry(name + ": is Connected", () -> isConnected())
        .withIsSeparated(false).withLogLevel(LogLevel.LOG_AND_NT).build();
    
    SmartDashboard.putData("motors/" + name + "/test power command", new StartEndCommand(
      () -> setDuty(testPower),
      () -> stop()));

      configPidFf(0);
      configMotionMagic();
  }

  @Override
  public void checkElectronics() {
    int fault = getFaultField().getValue();
    if (fault != 0) {
      LogManager.log(name + " have fault num: " + fault, AlertType.kError);
    }
  }

  @Override
  public void changeSlot(int slot) {
    if (slot < 0 || slot > 2) {
      LogManager.log("slot is not between 0 and 2", AlertType.kError);
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

  public double getWantedValue() {
    return wantedValue;
  }

  @Override
  public void stop() {
    stopMotor();
    wantedValue = 0;
    controlMode = ControlMode.DISABLE;
  }

  @Override
  public void setDuty(double power) {
    setControl(dutyCycle.withOutput(power));
    wantedValue = power;
    if (power == 0) {
      controlMode = ControlMode.DISABLE;
    } else {
      controlMode = ControlMode.DUTYCYCLE;
    }
  }

  public void setVolt(double voltage) {
    setVoltage(voltage);
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

  public void setMotionExpo(double position) {
    setMotionExpo(position, 0);
  }

  MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0).withSlot(slot);

  public void setMotionExpo(double position, double feedForward) {
    setControl(
        motionMagicExpoVoltage.withPosition(position).withFeedForward(feedForward + positionFeedForward(position)));
        wantedValue = position;
        controlMode = ControlMode.MAGIC_MOTION;
  }


  public void setMotion(double position, int slot){
    setControl(motionMagicVoltage.withSlot(slot).withFeedForward(positionFeedForward(position)));
  }
  @Override
  public void setMotion(double position) {
    setMotion(position, 0.0);
  }

  @Override
  public void setAngle(double angle, double feedForward) {
    setMotion(getCurrentPosition() + MathUtil.angleModulus(angle - getCurrentAngle()), feedForward);
    wantedValue = angle;
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
    return velocity * velocity * Math.signum(velocity) * config.kv2;
  }

  private double positionFeedForward(double position) {
    return Math.cos(position * config.posToRad) * config.kSin;
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
  public double getCurrentClosedLoopSP() {
    Double value = closedLoopSPSignal.getDouble();
    return value != null ? value : 0.0;
  }

  @Override
  public double getCurrentClosedLoopError() {
    Double value = closedLoopErrorSignal.getDouble();
    return value != null ? value : 0.0;
  }

  @Override
  public double getCurrentPosition() {
    Double value = positionSignal.getDouble();
    return value != null ? value : 0.0;
  }

  @Override
  public double getCurrentVelocity() {
    Double value = velocitySignal.getDouble();
    return value != null ? value : 0.0;
  }

  @Override
  public double getCurrentAcceleration() {
    Double value = accelerationSignal.getDouble();
    return value != null ? value : 0.0;
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
    Double value = voltageSignal.getDouble();
    return value != null ? value : 0.0;
  }

  @Override
  public double getCurrentCurrent() {
    Double value = currentSignal.getDouble();
    return value != null ? value : 0.0;
  }

  public double getCurrentValue() {
    switch (controlMode) {
      case DISABLE:
        return 0;
      case DUTYCYCLE:
        return getDutyCycle().getValueAsDouble();
      case VOLTAGE:
        return getCurrentVoltage();
      case VELOCITY:
        return getCurrentVelocity();
      case POSITION_VOLTAGE, MAGIC_MOTION:
        return getCurrentPosition();
      case ANGLE:
        return getCurrentAngle();
      default:
        return 0;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Talon Motor");
    builder.addBooleanProperty("Is" + name + "Connected", this::isConnected, null);
    builder.addDoubleProperty("CloseLoopError", this::getCurrentClosedLoopError, null);
    builder.addDoubleProperty("Position", this::getCurrentPosition, null);
    builder.addDoubleProperty("Velocity", this::getCurrentVelocity, null);
    builder.addDoubleProperty("Acceleration", this::getCurrentAcceleration, null);
    builder.addDoubleProperty("Voltage", this::getCurrentVoltage, null);
    builder.addDoubleProperty("Current", this::getCurrentCurrent, null);
    if (config.isRadiansMotor) {
      builder.addDoubleProperty("Angle", this::getCurrentAngle, null);
    }
    builder.addDoubleProperty("Value", this::getCurrentValue, null);
    builder.addDoubleProperty("ControlMode", this::getCurrentControlModeInteger, null);
    builder.addDoubleProperty(" Wanted Value", this::getWantedValue, null);
    builder.addDoubleProperty("test Power", () -> testPower, (testPower) -> this.testPower = testPower);
  }

  /**
   * Creates a command to configure PID and FeedForward parameters via the
   * Dashboard.
   * Useful for tuning without redeploying code.
   * 
   * @param slot The slot index to tune
   */
  public void configPidFf(int slot) {

    Command configPidFf = new InstantCommand(() -> {
      SlotConfigs cfg = new SlotConfigs();
      cfg.SlotNumber = slot;
      if (slot <= 2 && slot >= 0) {
        cfg.kP = config.pid[slot].kP();
        cfg.kI = config.pid[slot].kI();
        cfg.kD = config.pid[slot].kD();
        cfg.kS = config.pid[slot].kS();
        cfg.kV = config.pid[slot].kV();
        cfg.kA = config.pid[slot].kA();
        cfg.kG = config.pid[slot].kG();
      } else {
        cfg.kP = config.pid[0].kP();
        cfg.kI = config.pid[0].kI();
        cfg.kD = config.pid[0].kD();
        cfg.kS = config.pid[0].kS();
        cfg.kV = config.pid[0].kV();
        cfg.kA = config.pid[0].kA();
        cfg.kG = config.pid[0].kG();
      }
      getConfigurator().apply(cfg);
    }).ignoringDisable(true);

    SmartDashboard.putData("motors/" + name + "/PID+FF config", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PID+FF Config");
        builder.addDoubleProperty("KP", () -> config.pid[slot].kP(), (double newValue) -> config.pid[slot].setKP(newValue));
        builder.addDoubleProperty("KI", () -> config.pid[slot].kI(), (double newValue) -> config.pid[slot].setKI(newValue));
        builder.addDoubleProperty("KD", () -> config.pid[slot].kD(), (double newValue) -> config.pid[slot].setKD(newValue));
        builder.addBooleanProperty("USE_KS", () -> kFlags[0], (boolean newValue) -> kFlags[0] = newValue);
        builder.addDoubleProperty("KS", () -> config.pid[slot].kS(), (double newValue) -> config.pid[slot].setKS(newValue));
        builder.addBooleanProperty("USE_KV", () -> kFlags[1], (boolean newValue) -> kFlags[1] = newValue);
        builder.addDoubleProperty("KV", () -> config.pid[slot].kV(), (double newValue) -> config.pid[slot].setKV(newValue));
        builder.addBooleanProperty("USE_KA", () -> kFlags[2], (boolean newValue) -> kFlags[2] = newValue);
        builder.addDoubleProperty("KA", () -> config.pid[slot].kA(), (double newValue) -> config.pid[slot].setKA(newValue));
        builder.addBooleanProperty("USE_KG", () -> kFlags[3], (boolean newValue) -> kFlags[3] = newValue);
        builder.addDoubleProperty("KG", () -> config.pid[slot].kG(), (double newValue) -> config.pid[slot].setKG(newValue));
        builder.addBooleanProperty("USE_KSIN", () -> kFlags[4], (boolean newValue) -> kFlags[4] = newValue);
        builder.addDoubleProperty("KSIN", () -> config.kSin, (double newValue) -> config.kSin = newValue);
        builder.addBooleanProperty("USE_KV2", () -> kFlags[5], (boolean newValue) -> kFlags[5] = newValue);
        builder.addDoubleProperty("KV2", () -> config.kv2, (double newValue) -> config.kv2 = newValue);
        
        builder.addBooleanProperty("Update", () -> configPidFf.isScheduled(),
            value -> {
              if (value) {
                if (!configPidFf.isScheduled()) {
                  CommandScheduler.getInstance().schedule(configPidFf);
                }
              } else {
                if (configPidFf.isScheduled()) {
                  configPidFf.cancel();
                }
              }
            });
      }
    });
  }

  /**
   * Creates a command to configure Motion Magic parameters via the Dashboard.
   * Useful for tuning velocity/acceleration constraints live.
   */
  public void configMotionMagic() {
    Command configMotionMagic = new InstantCommand(() -> {
      cfg.MotionMagic.MotionMagicAcceleration = config.maxAcceleration;
      cfg.MotionMagic.MotionMagicCruiseVelocity = config.maxVelocity;
      cfg.MotionMagic.MotionMagicJerk = config.maxJerk;
      cfg.MotionMagic.MotionMagicExpo_kA = config.pid[0].kA();
      cfg.MotionMagic.MotionMagicExpo_kV = config.pid[0].kV();

      getConfigurator().apply(cfg);
    }).ignoringDisable(true);

    SmartDashboard.putData("motors/" + name + "/Motion Magic Config", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motion Magic Config");

        builder.addDoubleProperty("Vel", () -> config.maxVelocity, value -> config.maxVelocity = value);
        builder.addDoubleProperty("Acc", () -> config.maxAcceleration, value -> config.maxAcceleration = value);

        builder.addBooleanProperty("Update", () -> configMotionMagic.isScheduled(),
            value -> {
              if (value) {
                if (!configMotionMagic.isScheduled()) {
                  CommandScheduler.getInstance().schedule(configMotionMagic);
                }
              } else {
                if (configMotionMagic.isScheduled()) {
                  configMotionMagic.cancel();
                }
              }
            });
      }
    });
  }

  public void updatePid(CloseLoopParam newParams, int slot) {
    config.pid[slot].setKP(newParams.kP());
    config.pid[slot].setKI(newParams.kI());
    config.pid[slot].setKD(newParams.kD());
    config.pid[slot].setKS(newParams.kS());
    config.pid[slot].setKV(newParams.kV());
    config.pid[slot].setKA(newParams.kA());
    config.pid[slot].setKG(newParams.kG());
    
    SlotConfigs cfg = new SlotConfigs();

    cfg.SlotNumber = slot;
    if (slot <= 2 && slot >= 0) {
      cfg.kP = config.pid[slot].kP();
      cfg.kI = config.pid[slot].kI();
      cfg.kD = config.pid[slot].kD();
      cfg.kS = config.pid[slot].kS();
      cfg.kV = config.pid[slot].kV();
      cfg.kA = config.pid[slot].kA();
      cfg.kG = config.pid[slot].kG();
    } else {
      cfg.kP = config.pid[0].kP();
      cfg.kI = config.pid[0].kI();
      cfg.kD = config.pid[0].kD();
      cfg.kS = config.pid[0].kS();
      cfg.kV = config.pid[0].kV();
      cfg.kA = config.pid[0].kA();
      cfg.kG = config.pid[0].kG();
    }
    getConfigurator().apply(cfg);
  }

  public boolean[] getSysidFlags() {
    return kFlags;
  }

  public double gearRatio() {
    return config.motorRatio;
  }

  public String getName() {
    return name;
  }

  @Override
  public void setEncoderPosition(double position) {
    setPosition(position);
  }

  public Data<Double> getClosedLoopErrorSignal() {
    return closedLoopErrorSignal;
  }

  public Data<Double> getClosedLoopSPSignal() {
    return closedLoopSPSignal;
  }

  public Data<Angle> getPositionSignal() {
    return positionSignal;
  }

  public Data<AngularVelocity> getVelocitySignal() {
    return velocitySignal;
  }

  public Data<AngularAcceleration> getAccelerationSignal() {
    return accelerationSignal;
  }

  public Data<Voltage> getVoltageSignal() {
    return voltageSignal;
  }

  public Data<Current> getCurrentSignal() {
    return currentSignal;
  }
    
    /**
     * Checks if a specific motor has reached its target value within a specified tolerance.
     * * @param motorName The name of the motor
     * @param allowedError The allowable tolerance
     * @return true if the motor is within tolerance, false otherwise
     */
    public boolean isReady(double allowedError){
      switch (getCurrentControlMode()) {
        case DISABLE:
          break;
        case DUTYCYCLE:
          break;
        case VOLTAGE:
          if (Math.abs(getWantedValue() - getCurrentVoltage()) > allowedError){
            return false;
          }
            break;
        case VELOCITY:
          if (Math.abs(getWantedValue() - getCurrentVelocity()) > allowedError){
            return false;
          }
          break;
        case POSITION_VOLTAGE, MAGIC_MOTION, ANGLE:
          if (Math.abs(getWantedValue() - getCurrentPosition()) > allowedError){
            return false;
          }
          break;
        default:
          break;
      }
      return true;
    }
}