package frc.robot.turret.subsystems;

import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.RobotContainer;
import static frc.robot.turret.TurretConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.turret.commands.TurretMotorCalibrationCommand;
import static frc.robot.turret.TurretConstants.TurretMotorConstants.*;
import static frc.robot.turret.TurretConstants.TurretMinLimitSwitchConstants.*;

public class Turret extends StateBaseMechanism {
    private static Turret instance;

    private Turret() {
        super(TURRET_NAME, 
        new MotorInterface[] {
            new TalonFXMotor(TURRET_MOTOR_CONFIG),
        }, 
        new SensorInterface[] {
            new LimitSwitch(TURRET_MIN_LIMIT_SWITCH_CONFIG),
        }, 
        TurretStates.class);

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

    public double[] getTurretValues() {
        switch ((TurretStates) state) {
            case SHOOTING:
                break;
            case DELIVERY:
                break;
            default:
                break;
        }
        
        return new double[1];
    }

    public boolean atTurretMotorResetPos() {
        return ((LimitSwitch) getSensor(TURRET_MIN_LIMIT_SWITCH_NAME)).get();
    }

    public boolean atTurretMotorAutoResetPos() {
        return false;
    }

}
