package frc.demacia.utils.motors;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.unmanaged.Unmanaged;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.demacia.utils.Data;
import frc.demacia.utils.RobotCommon;
import frc.demacia.utils.log.Log;

/**
 * Wrapper class for the CTRE Talon SRX motor controller using Phoenix 5.
 * Now extends BaseMotor to integrate with the uniform motor architecture.
 */
public class TalonSRXMotor extends BaseMotor {
  TalonSRX motor;

  private final double TICKS_PER_REV = 4096.0;

  private double lastVelocity = 0;
  private double lastAcceleration = 0;
  private double lastTime = 0;

  /**
   * Creates a new Talon SRX motor wrapper.
   * 
   * @param config The configuration object
   */
  @SuppressWarnings({ "rawtypes", "unchecked" })
  public TalonSRXMotor(TalonSRXConfig config) {
    super(config);

    if (RobotBase.isSimulation()) {
      new Data(() -> {
        Unmanaged.feedEnable(100);

        double simVoltage = motor.getSimCollection().getMotorOutputLeadVoltage();
        double velocity = calculateSimVelocity(simVoltage);

        motor.getSimCollection().setQuadratureVelocity((int) (velocity * getTicksPerUnit() / 10.0));
        motor.getSimCollection().addQuadraturePosition((int) (velocity * getTicksPerUnit() * RobotCommon.CYCLE_DT));
        return 0;
      });
    }
  }

  @Override
  protected void createMotor() {
    motor = new TalonSRX(config.id);
  }

  @Override
  protected void createMotorConfig() {
    motor.configFactoryDefault();
  }

  @Override
  protected void configMaxCurrent(double maxCurrent) {
    motor.configContinuousCurrentLimit((int) maxCurrent);
    motor.configPeakCurrentLimit((int) maxCurrent);
    motor.configPeakCurrentDuration(100);
    motor.enableCurrentLimit(true);
  }

  @Override
  protected void configRampUpTime(double rampUpTime) {
    motor.configClosedloopRamp(rampUpTime);
    motor.configOpenloopRamp(rampUpTime);
  }

  @Override
  protected void configIsInverted(boolean isInverted) {
    motor.setInverted(isInverted);
  }

  @Override
  protected void configNeutralMode(boolean isBrake) {
    motor.setNeutralMode(isBrake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  protected void configMaxVolt(double maxVolt) {
    motor.configPeakOutputForward(maxVolt / 12.0);
    motor.configVoltageCompSaturation(maxVolt);
    motor.enableVoltageCompensation(true);
  }

  @Override
  protected void configMinVolt(double minVolt) {
    motor.configPeakOutputReverse(minVolt / 12.0);
  }

  @Override
  protected void configMotorRatio(double motorRatio) {

  }

  @Override
  protected void configPidFf(CloseLoopParam[] pidFfParams) {
    for (int slot = 0; slot < 3; slot++) {
      if (pidFfParams.length > slot && pidFfParams[slot] != null) {
        double maxOutputNative = 1023.0;
        double voltageScale = config.maxVolt;
        double ticksPerUnit = getTicksPerUnit();

        double kP_Native = (pidFfParams[slot].kP() * maxOutputNative) / (voltageScale * ticksPerUnit);
        double kI_Native = (pidFfParams[slot].kI() * maxOutputNative) / (voltageScale * ticksPerUnit);
        double kD_Native = (pidFfParams[slot].kD() * 10.0 * maxOutputNative) / (voltageScale * ticksPerUnit);
        double kF_Native = (pidFfParams[slot].kV() * 10.0 * maxOutputNative) / (voltageScale * ticksPerUnit);

        motor.config_kP(slot, kP_Native);
        motor.config_kI(slot, kI_Native);
        motor.config_kD(slot, kD_Native);
        motor.config_kF(slot, kF_Native);
      }
    }
  }

  @Override
  protected void configMotionMagic(double maxVelocity, double maxAcceleration, double maxJerk) {
    double nativeCruiseVelocity = (maxVelocity * getTicksPerUnit()) / 10.0;
    double nativeAcceleration = (maxAcceleration * getTicksPerUnit()) / 10.0;

    motor.configMotionCruiseVelocity(nativeCruiseVelocity);
    motor.configMotionAcceleration(nativeAcceleration);
  }

  @Override
  protected void applyConfigs() {

  }

  @Override
  protected void applyPidFfConfigs(int slot) {
    configPidFf(config.pidFfParams);
  }

  @Override
  protected void applyMotionMagicConfigs() {
    configMotionMagic(config.maxVelocity, config.maxAcceleration, config.maxJerk);
  }

  @Override
  protected void applyNeutralModeConfigs() {
    motor.setNeutralMode(config.brake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  protected void setSignals() {
    positionSignal = new Data<>(() -> motor.getSelectedSensorPosition() / getTicksPerUnit());
    velocitySignal = new Data<>(() -> (motor.getSelectedSensorVelocity() * 10.0) / getTicksPerUnit());
    accelerationSignal = new Data<>(() -> {
      double currentTimestamp = Timer.getFPGATimestamp();
      double dt = currentTimestamp - lastTime;

      if (dt < 0.001) {
        return lastAcceleration;
      }

      double currentVelocity = (motor.getSelectedSensorVelocity() * 10.0) / getTicksPerUnit();

      lastAcceleration = (currentVelocity - lastVelocity) / dt;
      lastVelocity = currentVelocity;
      lastTime = currentTimestamp;

      return lastAcceleration;
    });
    voltageSignal = new Data<>(() -> motor.getMotorOutputVoltage());
    currentSignal = new Data<>(() -> motor.getStatorCurrent());
    closedLoopSPSignal = new Data<>(() -> isClosedLoop() ? motor.getClosedLoopTarget(0) / getTicksPerUnit() : 0);
    closedLoopErrorSignal = new Data<>(() -> isClosedLoop() ? motor.getClosedLoopError(0) / getTicksPerUnit() : 0);
  }

  private boolean isClosedLoop() {
    ControlMode mode = getCurrentControlMode();
    return mode == ControlMode.VELOCITY || 
        mode == ControlMode.POSITION_VOLTAGE || 
        mode == ControlMode.MAGIC_MOTION || 
        mode == ControlMode.ANGLE;
  }

  @Override
  protected void changeMotorSlot(int slot) {
    motor.selectProfileSlot(slot, 0);
  }

  @Override
  protected void stopMotor() {
    motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
  }

  @Override
  protected void setMotorDuty(double power) {
    motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, power);
  }

  @Override
  protected void setMotorVoltage(double voltage) {
    motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, voltage / config.maxVolt);
  }

  @Override
  protected void setMotorVelocity(double velocity, double feedForward) {
    double nativeVelocity = (velocity * getTicksPerUnit()) / 10.0;
    motor.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, nativeVelocity,
        DemandType.ArbitraryFeedForward, 
        (feedForward + staticFeedForward(velocity)) / config.maxVolt);
  }

  @Override
  protected void setMotorPositionVoltage(double position, double feedForward) {
    double nativePosition = position * getTicksPerUnit();
    motor.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, nativePosition,
        DemandType.ArbitraryFeedForward, 
        (feedForward + staticFeedForward(position - getCurrentPosition())) / config.maxVolt);
  }

  @Override
  protected void setMotorMotionMagic(double position, double feedForward) {
    double nativePosition = position * getTicksPerUnit();
    motor.set(com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic, nativePosition,
        DemandType.ArbitraryFeedForward, 
        (feedForward + staticFeedForward(getCurrentVelocity())) / config.maxVolt);
  }

  private double staticFeedForward(double direction) {
    return Math.signum(direction) * config.pidFfParams[slot].kS() + config.pidFfParams[slot].kG();
  }

  @Override
  public void checkElectronics() {
    Faults faults = new Faults();
    motor.getFaults(faults);
    if (faults.hasAnyFault()) {
      Log.log(getName() + " have fault num: " + faults.toString(), AlertType.kError);
    }
  }

  @Override
  public boolean isConnected() {
    return motor.getFirmwareVersion() >= 0;
  }

  @Override
  public void setEncoderPosition(double position) {
    motor.setSelectedSensorPosition(position * getTicksPerUnit());
  }

  private double getTicksPerUnit() {
    return config.motorRatio * TICKS_PER_REV;
  }
}