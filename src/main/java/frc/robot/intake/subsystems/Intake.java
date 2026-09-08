package frc.robot.intake.subsystems;

import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.RobotContainer;
import static frc.robot.intake.IntakeConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.intake.commands.IntakeDeployCalibrationCommand;
import static frc.robot.intake.IntakeConstants.IntakeRollerConstants.*;
import static frc.robot.intake.IntakeConstants.IntakeDeployConstants.*;
import static frc.robot.intake.IntakeConstants.IntakeDeployMaxLimitSwitchConstants.*;

public class Intake extends StateBaseMechanism {
    private static Intake instance;

    private Intake() {
        super(INTAKE_NAME, 
        new MotorInterface[] {
            new TalonFXMotor(INTAKE_ROLLER_CONFIG),
            new TalonFXMotor(INTAKE_DEPLOY_CONFIG),
        }, 
        new SensorInterface[] {
            new LimitSwitch(INTAKE_DEPLOY_MAX_LIMIT_SWITCH_CONFIG),
        }, 
        IntakeStates.class);

        addLimit(INTAKE_DEPLOY_NAME, INTAKE_DEPLOY_MIN_LIMIT, INTAKE_DEPLOY_MAX_LIMIT);
        withPowerCommand(INTAKE_DEPLOY_NAME, () -> RobotContainer.controller.getRightX());
        withAutoCalibration(INTAKE_DEPLOY_NAME, this::atIntakeDeployAutoResetPos, INTAKE_DEPLOY_AUTO_CALIBRATION_RESET_POS);
        SmartDashboard.putData(INTAKE_NAME + "/" + INTAKE_DEPLOY_NAME + " Calibration Command", new IntakeDeployCalibrationCommand(this));
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public boolean atIntakeDeployResetPos() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'atIntakeDeployResetPos()'");
    }

    public boolean atIntakeDeployAutoResetPos() {
        return ((LimitSwitch) getSensor(INTAKE_DEPLOY_MAX_LIMIT_SWITCH_NAME)).get();
    }

}
