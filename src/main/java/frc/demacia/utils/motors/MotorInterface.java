package frc.demacia.utils.motors;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
/**
 * Common interface for all motor controllers in the robot.
 * <p>
 * Defines standard methods for controlling motors (Voltage, Velocity, Position, etc.)
 * and retrieving telemetry data, ensuring interchangeable use of different motor types.
 * </p>
 */
public interface MotorInterface extends Sendable {

    /** Enumeration of supported control modes */
    enum ControlMode {
        DISABLE, DUTYCYCLE, VOLTAGE, VELOCITY, POSITION_VOLTAGE, MAGIC_MOTION, ANGLE
    }

    /** @return The name of the motor */
    String getName();

    /**
     * Sets the name of the motor in the SendableRegistry.
     * @param name The new name
     */
    default void setName(String name) {
        SendableRegistry.setName(this, name);
    }

    boolean isConnected();

    /**
     * Checks for hardware faults and logs them.
     */
    void checkElectronics();

    /**
     * Sets the neutral mode of the motor.
     * @param isBrake true for Brake mode, false for Coast mode
     */
    void setNeutralMode(boolean isBrake);

    /**
     * Changes the active control slot (PID profile).
     * @param slot The slot index (typically 0, 1, or 2)
     */
    void changeSlot(int slot);

    /**
     * Stops the motor immediately and disables control.
     */
    void stop();

    /**
     * Sets the motor output as a duty cycle (percent output).
     * @param power The power output [-1.0, 1.0]
     */
    void setDuty(double power);

    /**
     * Sets the motor output voltage.
     * @param voltage The voltage to apply
     */
    void setVolt(double voltage);

    /**
     * Sets the target velocity with an optional feedforward.
     * @param velocity The target velocity
     * @param feedForward Arbitrary feedforward value
     */
    void setVelocity(double velocity, double feedForward);

    /**
     * Sets the target velocity.
     * @param velocity The target velocity
     */
    void setVelocity(double velocity);

    /**
     * Sets the target velocity using Ka.
     * @param velocity The target velocity
     * @param wantedAccelerationSupplier The supplier for the acceleration
     */
    void setVelocityWithAcceleration(double velocity, Supplier<Double> wantedAccelerationSupplier);

    /**
     * Sets the target position using Motion Magic.
     * @param position The target position
     * @param feedForward Arbitrary feedforward value
     */
    void setMotion(double position, double feedForward);

    /**
     * Sets the target position using Motion Magic.
     * @param position The target position
     */
    void setMotion(double position);

    /**
     * Sets the target angle using Motion Magic.
     * @param angle The target angle (in radians)
     * @param feedForward Arbitrary feedforward value
     */
    void setAngle(double angle, double feedForward);

    /**
     * Sets the target angle using Motion Magic.
     * @param angle The target angle
     */
    void setAngle(double angle);

    /**
     * Sets the target position using Position Voltage.
     * @param position The target position
     * @param feedForward Arbitrary feedforward value
     */
    void setPositionVoltage(double position, double feedForward);

    /**
     * Sets the target position using Position Voltage.
     * @param position The target position
     */
    void setPositionVoltage(double position);

    /** @return The integer representation of the current control mode */
    int getCurrentControlModeInteger();

    /** @return The current control mode */
    ControlMode getCurrentControlMode();

    /** @return The current closed-loop setpoint */
    double getCurrentClosedLoopSP();

    /**
     * Calculates the software-based closed-loop error.
     * Automatically handles the shortest-path modulus for ANGLE mode.
     */
    default double getCalculatedError() {
        ControlMode mode = getCurrentControlMode();
        
        if (mode == ControlMode.VOLTAGE || mode == ControlMode.DUTYCYCLE || mode == ControlMode.DISABLE) {
            return 0.0;
        }
        
        double error = getWantedValue() - getCurrentValue();
        
        if (mode == ControlMode.ANGLE) {
            return MathUtil.angleModulus(error);
        }
        
        return error;
    }

    /** @return The current closed-loop error */
    double getCurrentClosedLoopError();

    /** @return The current position (in mechanism units) */
    double getCurrentPosition();

    /** @return The current angle (in mechanism units) */
    double getCurrentAngle();

    /** @return The current velocity (in mechanism units/sec) */
    double getCurrentVelocity();

    /** @return The current acceleration (in mechanism units/sec^2) */
    double getCurrentAcceleration();

    /** @return The current motor voltage */
    double getCurrentVoltage();

    /** @return The current stator current (Amps) */
    double getCurrentCurrent();
    
    /** @return The current calculated value based on control modes */
    default double getCurrentValue() {
        ControlMode mode = getCurrentControlMode();
        
        if (mode == ControlMode.DISABLE || mode == ControlMode.DUTYCYCLE) {
            mode = getLastControlMode();
        }
        
        switch (mode) {
            case VOLTAGE:
                return getCurrentVoltage();
            case VELOCITY:
                return getCurrentVelocity();
            case MAGIC_MOTION, POSITION_VOLTAGE:
                return getCurrentPosition();
            case ANGLE:
                return getCurrentAngle();
            default:
                return 0.0;
        }
    }

    /** @return The last active control mode (used for fallback when disabled/dutycycle) */
    ControlMode getLastControlMode();

    /**
     * Overrides the internal encoder position.
     * @param position The new position to set
     */
    void setEncoderPosition(double position);
    /**
     * Checks if the motor is stalled based on current, velocity, and time thresholds.
     * @return true if the motor is stalled, false otherwise
     */
    void updateStallDetection();
    /** 
     * Checks if stall detection is enabled.
     * @return true if stall detection is enabled, false otherwise
     */
    boolean getStallDetection();

    double getWantedValue();

    /**
     * @return The flags used for Sysid OLS calculation [kS, kV, kA, kG, kCos, kV2]
     */
    boolean[] getSysidFlags();

    // --- Abstract Hooks for Configuration (Template Method Pattern) ---
    CloseLoopParam getPidParam(int slot);
    
    // Abstract hardware updaters
    void applyPidHardware(int slot);

    void applyMotionMagicHardware();

    // Motion Magic Getters/Setters
    double getMaxVelocity();

    void setMaxVelocity(double velocity);

    double getMaxAcceleration();

    void setMaxAcceleration(double acceleration);

    void updatePid(CloseLoopParam newParams, int slot);

    /**
     * Creates a command to configure PID and FeedForward parameters via the Dashboard.
     * @param slot The slot index to tune
     */
    default void configPidFf(int slot) {
        Command configPidCmd = new InstantCommand(() -> {
            applyPidHardware(slot);
        }).ignoringDisable(true);

        SmartDashboard.putData("motors/" + getName() + "/PID+FF config", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("PID+FF Config");
                CloseLoopParam pid = getPidParam(slot);
                boolean[] flags = getSysidFlags();
                
                builder.addDoubleProperty("KP", pid::kP, pid::setKP);
                builder.addDoubleProperty("KI", pid::kI, pid::setKI);
                builder.addDoubleProperty("KD", pid::kD, pid::setKD);
                builder.addBooleanProperty("USE_KS", () -> flags[0], (v) -> flags[0] = v);
                builder.addDoubleProperty("KS", pid::kS, pid::setKS);
                builder.addBooleanProperty("USE_KV", () -> flags[1], (v) -> flags[1] = v);
                builder.addDoubleProperty("KV", pid::kV, pid::setKV);
                builder.addBooleanProperty("USE_KA", () -> flags[2], (v) -> flags[2] = v);
                builder.addDoubleProperty("KA", pid::kA, pid::setKA);
                builder.addBooleanProperty("USE_KG", () -> flags[3], (v) -> flags[3] = v);
                builder.addDoubleProperty("KG", pid::kG, pid::setKG);
                builder.addBooleanProperty("USE_KSIN", () -> flags[4], (v) -> flags[4] = v);
                builder.addDoubleProperty("KSIN", pid::kSin, pid::setKSin);
                builder.addBooleanProperty("USE_KV2", () -> flags[5], (v) -> flags[5] = v);
                builder.addDoubleProperty("KV2", pid::kV2, pid::setKV2);
                
                builder.addBooleanProperty("Update", () -> configPidCmd.isScheduled(),
                    value -> {
                        if (value && !configPidCmd.isScheduled()) {
                            CommandScheduler.getInstance().schedule(configPidCmd);
                        } else if (!value && configPidCmd.isScheduled()) {
                            configPidCmd.cancel();
                        }
                    });
            }
        });
    }

    /**
     * Creates a command to configure Motion Magic parameters via the Dashboard.
     */
    default void configMotionMagic() {
        Command configMotionMagicCmd = new InstantCommand(this::applyMotionMagicHardware).ignoringDisable(true);

        SmartDashboard.putData("motors/" + getName() + "/Motion Magic Config", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Motion Magic Config");
                builder.addDoubleProperty("Vel", MotorInterface.this::getMaxVelocity, MotorInterface.this::setMaxVelocity);
                builder.addDoubleProperty("Acc", MotorInterface.this::getMaxAcceleration, MotorInterface.this::setMaxAcceleration);
                builder.addBooleanProperty("Update", () -> configMotionMagicCmd.isScheduled(),
                    value -> {
                        if (value && !configMotionMagicCmd.isScheduled()) {
                            CommandScheduler.getInstance().schedule(configMotionMagicCmd);
                        } else if (!value && configMotionMagicCmd.isScheduled()) {
                            configMotionMagicCmd.cancel();
                        }
                    });
            }
        });
    }

    /**
     * Applies a target value to the motor based on the specified control mode.
     */
    default void applyControlModeValue(ControlMode mode, double value) {
        switch (mode) {
            case VOLTAGE:
                setVolt(value);
                break;
            case VELOCITY:
                setVelocity(value);
                break;
            case POSITION_VOLTAGE:
                setPositionVoltage(value);
                break;
            case MAGIC_MOTION:
                setMotion(value);
                break;
            case ANGLE:
                setAngle(value);
                break;
            case DUTYCYCLE:
            case DISABLE:
            default:
                setDuty(value);
                break;
        }
    }

    /**
   * Checks if a specific motor has reached its target value within a specified tolerance.
   * * @param motorName The name of the motor
   * @param allowedError The allowable tolerance
   * @return true if the motor is within tolerance, false otherwise
   */
    default boolean isReady(double allowedError){
        return Math.abs(getCurrentClosedLoopError()) < allowedError;
    }

    boolean isRadiansMotor();

    double getTestValue();

    void setTestValue(double testValue);

    /**
     * Initializes the common Sendable properties for all motors.
     */
    default void initCommonSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor");
        builder.addBooleanProperty("Is Connected", this::isConnected, null);
        builder.addDoubleProperty("CloseLoopError", this::getCurrentClosedLoopError, null);
        builder.addDoubleProperty("Position", this::getCurrentPosition, null);
        builder.addDoubleProperty("Velocity", this::getCurrentVelocity, null);
        builder.addDoubleProperty("Acceleration", this::getCurrentAcceleration, null);
        builder.addDoubleProperty("Voltage", this::getCurrentVoltage, null);
        builder.addDoubleProperty("Current", this::getCurrentCurrent, null);
        if (isRadiansMotor()) {
            builder.addDoubleProperty("Angle", this::getCurrentAngle, null);
        }
        builder.addDoubleProperty("Value", this::getCurrentValue, null);
        builder.addDoubleProperty("ControlMode", this::getCurrentControlModeInteger, null);
        builder.addDoubleProperty("Wanted Value", this::getWantedValue, null);

        builder.addDoubleProperty("test Value", this::getTestValue, (value) -> setTestValue(value));
    }
}