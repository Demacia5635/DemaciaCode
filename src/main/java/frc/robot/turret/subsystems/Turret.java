package frc.robot.turret.subsystems;

import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.RobotContainer;
import frc.robot.shootingValues.ShootingValues;
import frc.robot.shootingValues.ShootingValuesConstants;

import static frc.robot.turret.TurretConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.turret.TurretConstants.TurretStates;
import frc.robot.turret.commands.TurretMotorCalibrationCommand;
import static frc.robot.turret.TurretConstants.TurretMotorConstants.*;
import static frc.robot.turret.TurretConstants.MinLimitSwitchConstants.*;

public class Turret extends StateBaseMechanism<TurretStates> {
    private static Turret instance;

    private double[] turretVal;

    private Turret() {
        super(TURRET_NAME, 
        new MotorInterface[] {
            new TalonFXMotor(TURRET_MOTOR_CONFIG),
        }, 
        new SensorInterface[] {
            new LimitSwitch(MIN_LIMIT_SWITCH_CONFIG),
        });

        turretVal = new double[1];

        addLimit(TURRET_MOTOR_NAME, TURRET_MOTOR_MIN_LIMIT, TURRET_MOTOR_MAX_LIMIT);
        withPowerCommand(TURRET_MOTOR_NAME, () -> RobotContainer.controller.getRightX());
        withAutoCalibration(TURRET_MOTOR_NAME, this::atTurretMotorAutoResetPos, TURRET_MOTOR_AUTO_CALIBRATION_RESET_POS);
        SmartDashboard.putData(TURRET_NAME + "/" + TURRET_MOTOR_NAME + " Calibration Command", new TurretMotorCalibrationCommand(this));
    }

    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    public void setTurretMotorPower(double power) {
        setPower(TURRET_MOTOR_NAME, power);
    }

    public void setTurretMotorMotion(double motion) {
        setMotion(TURRET_MOTOR_NAME, motion);
    }

    public double getTurretMotorPosition() {
        return getMotor(TURRET_MOTOR_NAME).getCurrentPosition();
    }

    public boolean isTurretMotorReady() {
        return isReady(TURRET_MOTOR_NAME, TURRET_MOTOR_ALLOWED_ERROR);
    }
    public boolean isTurretReady() {
        return isTurretMotorReady();
    }

    public double[] getTurretValues() {
        switch ((TurretStates) state) {
            case SHOOTING:
                turretVal[0] = ShootingValues.getInstance().getShootingValues().turretAngle();
                break;
            case DELIVERY:
                turretVal[0] = ShootingValues.getInstance().angleFromHubAfterTime(ShootingValuesConstants.PREDICTING_TIME);
                break;
            default:
                break;
        }
        
        return turretVal;
    }

    public boolean getMin() {
        return ((LimitSwitch) getSensor(MIN_LIMIT_SWITCH_NAME)).get();
    }

    public boolean atTurretMotorResetPos() {
        return getMin();
    }

    public boolean atTurretMotorAutoResetPos() {
        return getMin();
    }

}
