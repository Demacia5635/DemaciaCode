package frc.robot.intake.subsystems;

import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.RobotContainer;
import static frc.robot.intake.IntakeConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.intake.IntakeConstants.IntakeStates;
import frc.robot.intake.commands.IntakeDeployCalibrationCommand;
import static frc.robot.intake.IntakeConstants.IntakeRollersConstants.*;
import static frc.robot.intake.IntakeConstants.IntakeDeployConstants.*;
import static frc.robot.intake.IntakeConstants.IntakeDeployMaxLimitSwitchConstants.*;

public class Intake extends StateBaseMechanism<IntakeStates> {
    private static Intake instance;

    private Intake() {
        super(INTAKE_NAME, 
        new MotorInterface[] {
            new TalonFXMotor(INTAKE_ROLLERS_CONFIG),
            new TalonFXMotor(INTAKE_DEPLOY_CONFIG),
        }, 
        new SensorInterface[] {
            new LimitSwitch(INTAKE_DEPLOY_MAX_LIMIT_SWITCH_CONFIG),
        });

        addLimit(INTAKE_DEPLOY_NAME, INTAKE_DEPLOY_MIN_LIMIT, INTAKE_DEPLOY_MAX_LIMIT);
        withPowerCommand(INTAKE_DEPLOY_NAME, () -> RobotContainer.controller.getRightX());
        withAutoCalibration(INTAKE_DEPLOY_NAME, this::atIntakeDeployResetPos, INTAKE_DEPLOY_AUTO_CALIBRATION_RESET_POS);
        SmartDashboard.putData(INTAKE_NAME + "/" + INTAKE_DEPLOY_NAME + " Calibration Command", new IntakeDeployCalibrationCommand(this));
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void setIntakeRollersPower(double power) {
        setPower(INTAKE_ROLLERS_NAME, power);
    }

    public void setIntakeDeployPower(double power) {
        setPower(INTAKE_DEPLOY_NAME, power);
    }

    public void setIntakeDeployMotion(double motion) {
        setMotion(INTAKE_DEPLOY_NAME, motion);
    }

    public double getIntakeDeployPosition() {
        return getMotor(INTAKE_DEPLOY_NAME).getCurrentPosition();
    }

    public boolean isIntakeDeployReady() {
        return getMotor(INTAKE_DEPLOY_NAME).getCurrentAngle() < INTAKE_DEPLOY_ALLOWED_POS;
    }
    public boolean isIntakeReady() {
        return isIntakeDeployReady();
    }

    public boolean isStuckIntakeRollers() {
        return getMotor(INTAKE_ROLLERS_NAME).isStuck();
    }
    public boolean getIntakeDeployMax() {
        return ((LimitSwitch) getSensor(INTAKE_DEPLOY_MAX_LIMIT_SWITCH_NAME)).get();
    }

    public boolean atIntakeDeployResetPos() {
        return getIntakeDeployMax();
    }

}
