package frc.demacia.utils.motors;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.demacia.utils.elastic.ElasticGenerator;
import frc.demacia.utils.log.Log;
import frc.demacia.utils.log.Log.LogLevel;
import frc.demacia.utils.sysid.Sysid;

/**
 * Wrapper class for the CTRE Talon SRX motor controller using Phoenix 5.
 * <p>
 * Implements the MotorInterface for standard control.
 * <b>Note:</b> Many advanced control methods (Velocity, Motion Magic) are
 * currently
 * unimplemented in this wrapper and will log an error if called.
 * </p>
 */
public class TalonSRXMotor extends TalonSRX implements MotorInterface {
    TalonSRXConfig config;
    String name;

    int slot = 0;

    double wantedValue = 0.0;
    double testValue = 0.0;

    private double lastVelocity = 0;
    private double lastAcceleration = 0;
    private double lastTime = 0;

    ControlMode notDutyControlMode = ControlMode.DISABLE;
    ControlMode controlMode = ControlMode.DISABLE;
    ControlMode valueControlMode = ControlMode.DUTYCYCLE;
    SendableChooser<ControlMode> valueControlModeChooser = new SendableChooser<>();

    // Motor Stalling
    private final Timer stallTimer = new Timer();
    private boolean conditionActive = false;
    private boolean isDone = false;
    private boolean isStalled = false;

    private boolean[] kFlags = {true, true, true, false, false, false};

    private final double TICKS_PER_REV = 4096.0; 

    /**
     * Creates a new Talon SRX motor wrapper.
     * * @param config The configuration object
     */
    public TalonSRXMotor(TalonSRXConfig config) {
        super(config.id);
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

    /**
     * Applies the configuration to the motor using Phoenix 5 API.
     * Sets limits, ramps, inversion, and neutral mode.
     */
    private void configMotor() {
        configFactoryDefault();
        configContinuousCurrentLimit((int) config.maxCurrent);
        configPeakCurrentLimit((int) config.maxCurrent);
        configPeakCurrentDuration(100);
        enableCurrentLimit(true);
        configClosedloopRamp(config.rampUpTime);
        configOpenloopRamp(config.rampUpTime);
        setInverted(config.inverted);
        setNeutralMode(config.brake ? NeutralMode.Brake : NeutralMode.Coast);
        configPeakOutputForward(config.maxVolt / 12.0);
        configPeakOutputReverse(config.minVolt / 12.0);
        configVoltageCompSaturation(config.maxVolt);
        enableVoltageCompensation(true);
    }

    @Override
    public boolean isConnected() {
        return getFirmwareVersion() >= 0;
    }

    @Override
    public void setName(String name) {
        MotorInterface.super.setName(name);
        this.name = name;
    }

    /** Configures the logging entries for this motor */
    @SuppressWarnings("unchecked")
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

    @Override
    public void checkElectronics() {
        com.ctre.phoenix.motorcontrol.Faults faults = new com.ctre.phoenix.motorcontrol.Faults();
        getFaults(faults);
        if (faults.hasAnyFault()) {
            Log.log(name + " have fault num: " + faults.toString(), AlertType.kError);
        }
    }

    @Override
    public void changeSlot(int slot) {
        if (slot < 0 || slot > 2) {
            Log.log("slot is not between 0 and 2", AlertType.kError);
            return;
        }
        this.slot = slot;
        selectProfileSlot(slot, 0);
    }

    @Override
    public void setNeutralMode(boolean isBrake) {
        setNeutralMode(isBrake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public double getWantedValue() {
      return wantedValue;
    }

    @Override
    public void setDuty(double power) {
        set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, power);
        wantedValue = power;
        if (power == 0) {
            controlMode = ControlMode.DISABLE;
        } else {
            controlMode = ControlMode.DUTYCYCLE;
        }
    }

    @Override
    public void setVolt(double voltage) {
        set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, voltage / config.maxVolt);
        wantedValue = voltage;
        controlMode = ControlMode.VOLTAGE;
        notDutyControlMode = controlMode;
    }

    @Override
    public void setVelocity(double velocity, double feedForward) {
        double nativeVelocity = (velocity * getTicksPerUnit()) / 10.0; 
        set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, nativeVelocity, 
            DemandType.ArbitraryFeedForward, feedForward / config.maxVolt);
        wantedValue = velocity;
        controlMode = ControlMode.VELOCITY;
        notDutyControlMode = controlMode;
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
        double nativePosition = position * getTicksPerUnit();
        set(com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic, nativePosition, 
            DemandType.ArbitraryFeedForward, feedForward / config.maxVolt);
        wantedValue = position;
        controlMode = ControlMode.MAGIC_MOTION;
        notDutyControlMode = controlMode;
    }

    @Override
    public void setMotion(double position) {
        setMotion(position, 0);
    }

    @Override
    public void setAngle(double angle, double feedForward) {
        setMotion(getCurrentPosition() + MathUtil.angleModulus(angle - getCurrentAngle()), feedForward);
        wantedValue = angle;
        controlMode = ControlMode.ANGLE;
        notDutyControlMode = controlMode;
    }

    @Override
    public void setAngle(double angle) {
        setAngle(angle, 0);
    }

    @Override
    public void setPositionVoltage(double position, double feedForward) {
        Log.log("there is no PositionVoltage in SRX right now");
        wantedValue = position;
        controlMode = ControlMode.POSITION_VOLTAGE;
        notDutyControlMode = controlMode;
    }

    @Override
    public void setPositionVoltage(double position) {
        setPositionVoltage(position, 0);
    }

    @SuppressWarnings("unused")
    private double velocityFeedForward(double velocity) {
        return velocity * velocity * Math.signum(velocity) * config.pid[0].kV2();
    }

    @SuppressWarnings("unused")
    private double positionFeedForward(double position) {
        return Math.cos(position * config.posToRad) * config.pid[0].kSin();
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
        return getClosedLoopTarget(0) / getTicksPerUnit();
    }

    @Override
    public double getCurrentClosedLoopError() {
        return getClosedLoopError(0) / getTicksPerUnit();
    }

    @Override
    public double getCurrentPosition() {
        return getSelectedSensorPosition() / getTicksPerUnit();
    }

    @Override
    public double getCurrentAngle() {
        if (config.isRadiansMotor) {
            return MathUtil.angleModulus(getCurrentPosition());
        }
        return 0;
    }

    @Override
    public double getCurrentVelocity() {
        return (getSelectedSensorVelocity() * 10.0) / getTicksPerUnit();
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
    public double getCurrentVoltage() {
        return getMotorOutputVoltage();
    }

    @Override
    public double getCurrentCurrent() {
        return getStatorCurrent();
    }

    @Override
    public void updatePid(CloseLoopParam newParams, int slot) {
        config.pid[slot].setKP(newParams.kP());
        config.pid[slot].setKI(newParams.kI());
        config.pid[slot].setKD(newParams.kD());
        config.pid[slot].setKS(newParams.kS());
        config.pid[slot].setKV(newParams.kV());
        config.pid[slot].setKA(newParams.kA());
        config.pid[slot].setKG(newParams.kG());
        config.pid[slot].setKSin(newParams.kSin());
        config.pid[slot].setKV2(newParams.kV2());
        
        applyPidHardware(slot);
    }

    @Override
    public boolean[] getSysidFlags() {
        return kFlags;
    }

    @Override
    public void setEncoderPosition(double position) {
        setSelectedSensorPosition(position * getTicksPerUnit());
    }

    @Override
    public ControlMode getLastControlMode() {
        return notDutyControlMode;
    }

    @Override
    public CloseLoopParam getPidParam(int slot) {
        return config.pid[slot];
    }

    @Override
    public void applyPidHardware(int slot) {
        double maxOutputNative = 1023.0;
        double voltageScale = config.maxVolt;
        double ticksPerUnit = getTicksPerUnit();

        double kP_SI = config.pid[slot].kP();
        double kI_SI = config.pid[slot].kI();
        double kD_SI = config.pid[slot].kD();
        double kV_SI = config.pid[slot].kV(); 

        double kP_Native = (kP_SI * maxOutputNative) / (voltageScale * ticksPerUnit);
        double kI_Native = (kI_SI * maxOutputNative) / (voltageScale * ticksPerUnit);
        double kD_Native = (kD_SI * 10.0 * maxOutputNative) / (voltageScale * ticksPerUnit);
        double kF_Native = (kV_SI * 10.0 * maxOutputNative) / (voltageScale * ticksPerUnit);

        config_kP(slot, kP_Native);
        config_kI(slot, kI_Native);
        config_kD(slot, kD_Native);
        config_kF(slot, kF_Native); 
    }

    private double getTicksPerUnit() {
        return config.motorRatio * TICKS_PER_REV;
    }

    @Override
    public void applyMotionMagicHardware() {
        double nativeCruiseVelocity = (config.maxVelocity * getTicksPerUnit()) / 10.0;
        double nativeAcceleration = (config.maxAcceleration * getTicksPerUnit()) / 10.0;
        
        configMotionCruiseVelocity(nativeCruiseVelocity);
        configMotionAcceleration(nativeAcceleration);
    }

    @Override
    public String getName() {
        return name;
    }

    public double gearRatio() {
        return config.motorRatio;
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

    public boolean getStallDetection() {
        return isStalled;
    }

    @Override
    public void stop() {
        setDuty(0);
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
    public boolean isRadiansMotor() { return config.isRadiansMotor; }

    @Override
    public double getTestValue() { return testValue; }

    @Override
    public void setTestValue(double testValue) { this.testValue = testValue; }

    @Override
    public void initSendable(SendableBuilder builder) {
        initCommonSendable(builder);
    }
}