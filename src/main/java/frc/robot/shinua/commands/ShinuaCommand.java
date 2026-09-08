package frc.robot.shinua.commands;

import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.shinua.subsystems.Shinua;

public class ShinuaCommand extends DefaultCommand {
    
    public ShinuaCommand() {
        super(Shinua.getInstance(), new ControlMode[] {
            ControlMode.DUTYCYCLE,
            ControlMode.DUTYCYCLE
        });
    }
}
