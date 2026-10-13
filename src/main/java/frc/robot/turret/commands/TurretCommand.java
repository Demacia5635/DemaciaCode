package frc.robot.turret.commands;

import static frc.robot.turret.TurretConstants.TurretMotorConstants.*;

import frc.demacia.utils.log.Log;
import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.turret.TurretConstants.TurretStates;
import frc.robot.turret.subsystems.Turret;

public class TurretCommand extends DefaultCommand<TurretStates> {
    private final Turret turret = Turret.getInstance();
    protected Runnable control;
    
    public TurretCommand() {
        super(Turret.getInstance(), new ControlMode[] {
            ControlMode.ANGLE
        });

        control = () -> turret.setTurretAngle(turret.getValue(TURRET_MOTOR_NAME));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (mechanism.getMechanismState().equals(mechanism.TESTING_STATE) && Math.abs(turret.getMotor(TURRET_MOTOR_NAME).getCurrentVelocity()) < 0.01){
            Log.alert(Math.abs(turret.getTurretMotorAngle() - turret.getValue(TURRET_MOTOR_NAME)) + "");
            if (Math.abs(turret.getTurretMotorAngle() - turret.getValue(TURRET_MOTOR_NAME)) > Math.toRadians(40)) {
                turret.getMotor(TURRET_MOTOR_NAME).setSlot(0);
                control = () -> turret.setTurretAngle(turret.getValue(TURRET_MOTOR_NAME));
            } else {
                turret.getMotor(TURRET_MOTOR_NAME).setSlot(1);
                control = () -> turret.setTurretMotorPositionVoltage(turret.getValue(TURRET_MOTOR_NAME));
            }

            control.run();
            return;
        }

        //if state is testing or idle getState() is null and DefaultCommand will handle it
        if (turret.getState() == null) {
            super.execute();
            return;
        }

        switch (turret.getState()) {
            case SHOOTING, DELIVERY:
                turret.setTurretAngle(turret.getValue(TURRET_MOTOR_NAME));
                break;
            default:
                break;
        }
    }
}
