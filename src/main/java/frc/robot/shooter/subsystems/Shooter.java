package frc.robot.shooter.subsystems;

import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.RobotContainer;
import static frc.robot.shooter.ShooterConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shooter.ShooterConstants.ShooterStates;
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
        withPowerCommand(FLYWHEEL_NAME, () -> RobotContainer.controller.getRightX());
        withPowerCommand(HOOD_NAME, () -> RobotContainer.controller.getRightY());
        withAutoCalibration(HOOD_NAME, this::atHoodAutoResetPos, HOOD_AUTO_CALIBRATION_RESET_POS);
        SmartDashboard.putData(SHOOTER_NAME + "/" + HOOD_NAME + " Calibration Command", new HoodCalibrationCommand(this));
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public double[] getShooterValues() {
        switch ((ShooterStates) state) {
            case SHOOTING:
                break;
            case DELIVERY:
                break;
            case GETTING_READY:
                break;
            default:
                break;
        }
        
        return new double[3];
    }

    public boolean atHoodResetPos() {
        // TODO Auto-generated method stub
        return false;
    }

    public boolean atHoodAutoResetPos() {
        return ((LimitSwitch) getSensor(HOOD_MIN_LIMIT_SWITCH_NAME)).get();
    }

}
