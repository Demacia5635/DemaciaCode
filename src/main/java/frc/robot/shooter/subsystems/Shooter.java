package frc.robot.shooter.subsystems;

import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.RobotContainer;
import static frc.robot.shooter.ShooterConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shooter.commands.HoodCalibrationCommand;
import static frc.robot.shooter.ShooterConstants.FlywheelConstants.*;
import static frc.robot.shooter.ShooterConstants.HoodConstants.*;
import static frc.robot.shooter.ShooterConstants.FeederConstants.*;
import static frc.robot.shooter.ShooterConstants.HoodMinLimitSwitchConstants.*;

public class Shooter extends StateBaseMechanism {
    private static Shooter instance;

    private Shooter() {
        super(SHOOTER_NAME, 
        new MotorInterface[] {
            new TalonFXMotor(FLYWHEEL_CONFIG),
            new TalonFXMotor(HOOD_CONFIG),
            new TalonFXMotor(FEEDER_CONFIG),
        }, 
        new SensorInterface[] {
            new LimitSwitch(HOOD_MIN_LIMIT_SWITCH_CONFIG),
        }, 
        ShooterStates.class);

        addLimit(HOOD_NAME, HOOD_MIN_LIMIT, HOOD_MAX_LIMIT);
        withPowerCommand(HOOD_NAME, () -> RobotContainer.controller.getRightX());
        withPowerCommand(FLYWHEEL_NAME, () -> RobotContainer.controller.getRightX());
        withAutoCalibration(HOOD_NAME, this::atHoodResetPos, HOOD_AUTO_CALIBRATION_RESET_POS);
        SmartDashboard.putData(SHOOTER_NAME + "/" + HOOD_NAME + " Calibration Command", new HoodCalibrationCommand(this));
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public void setFlywheelPower(double power) {
        setPower(FLYWHEEL_NAME, power);
    }

    public void setFlywheelVelocity(double velocity) {
        setVelocity(FLYWHEEL_NAME, velocity);
    }

    public double getFlywheelVelocity() {
        return getMotor(FLYWHEEL_NAME).getCurrentVelocity();
    }

    public void setHoodPower(double power) {
        setPower(HOOD_NAME, power);
    }

    public void setHoodMotion(double motion) {
        setMotion(HOOD_NAME, motion);
    }

    public double getHoodPosition() {
        return getMotor(HOOD_NAME).getCurrentPosition();
    }

    public void setFeederPower(double power) {
        setPower(FEEDER_NAME, power);
    }

    public boolean getHoodMin() {
        return ((LimitSwitch) getSensor(HOOD_MIN_LIMIT_SWITCH_NAME)).get();
    }

    public double[] getShooterValues() {
        switch ((ShooterStates) state) {
            case SHOOTING:
                break;
            case DELIVERY:
                break;
            case TRANCH:
                break;
            default:
                break;
        }
        
        return new double[] {}; // TODO: Unimplemented method 'getShooterValues'
    }

    public boolean atHoodResetPos() {
        return ((LimitSwitch) getSensor(HOOD_MIN_LIMIT_SWITCH_NAME)).get();
    }

}
