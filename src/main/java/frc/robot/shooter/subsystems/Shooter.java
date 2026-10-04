package frc.robot.shooter.subsystems;

import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shooter.ShooterConstants.ShooterStates;
import frc.robot.shooter.commands.HoodCalibrationCommand;
import frc.robot.shootingValues.ShootingValues;
import frc.robot.shootingValues.ShootingValuesConstants;
import frc.robot.shootingValues.ShootingValuesRecord;
import static frc.robot.shooter.ShooterConstants.*;
import static frc.robot.shooter.ShooterConstants.FlywheelConstants.*;
import static frc.robot.shooter.ShooterConstants.HoodConstants.*;
import static frc.robot.shooter.ShooterConstants.FeederConstants.*;
import static frc.robot.shooter.ShooterConstants.HoodMinLimitSwitchConstants.*;

public class Shooter extends StateBaseMechanism<ShooterStates> {
    private static Shooter instance;

    private double[] shooterValues;

    private Shooter() {
        super(SHOOTER_NAME, 
        new MotorInterface[] {
            new TalonFXMotor(FLYWHEEL_CONFIG),
            new TalonFXMotor(HOOD_CONFIG),
            new TalonFXMotor(FEEDER_CONFIG),
        }, 
        new SensorInterface[] {
            new LimitSwitch(HOOD_MIN_LIMIT_SWITCH_CONFIG),
        });

        shooterValues = new double[3];

        addLimit(HOOD_NAME, HOOD_MIN_LIMIT, HOOD_MAX_LIMIT);
        withPowerCommand(FLYWHEEL_NAME, () -> RobotContainer.controller.getRightX());
        withPowerCommand(HOOD_NAME, () -> RobotContainer.controller.getRightX());
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

    public boolean isFlywheelReady() {
        return isReady(FLYWHEEL_NAME, FLYWHEEL_ALLOWED_ERROR);
    }
    public boolean isHoodReady() {
        return isReady(HOOD_NAME, HOOD_ALLOWED_ERROR);
    }
    public boolean isShooterReady() {
        return isFlywheelReady() && isHoodReady();
    }

    public boolean isStuckHood() {
        return getMotor(HOOD_NAME).isStuck();
    }
    public boolean isStuckFeeder() {
        return getMotor(FEEDER_NAME).isStuck();
    }
    public double[] getShooterValues() {
        switch ((ShooterStates) state) {
            case SHOOTING:
                ShootingValuesRecord shootingValues = ShootingValues.getInstance().getShootingValues();

                shooterValues[0] = shootingValues.velocity();
                shooterValues[1] = shootingValues.hoodAngle();
                shooterValues[2] = FEEDER_POWER;
                break;
            case DELIVERY:
                shooterValues[0] = ShootingValuesConstants.LOOK_UP_TABLE.get(ShootingValues.getInstance().distanceFromHubAfterTime(ShootingValuesConstants.PREDICTING_TIME))[0];
                shooterValues[1] = ShootingValuesConstants.LOOK_UP_TABLE.get(ShootingValues.getInstance().distanceFromHubAfterTime(ShootingValuesConstants.PREDICTING_TIME))[1];
                shooterValues[2] = FEEDER_POWER;
                break;
            case TRANCH:
                shooterValues[1] = 0;
                break;
            default:
                break;
        }
        
        return shooterValues;
    }

    public boolean getHoodMin() {
        return ((LimitSwitch) getSensor(HOOD_MIN_LIMIT_SWITCH_NAME)).get();
    }

    public boolean atHoodAutoResetPos() {
        return getHoodMin();
    }

    public boolean atHoodResetPos() {
        return getHoodMin() || isStuckHood();
    }

    @Override
    public void periodic() {
        super.periodic();

        ShootingValues.getInstance().updateShootingValues();
    }
}
