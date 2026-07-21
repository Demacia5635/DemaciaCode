package frc.demacia.utils.sysid;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.demacia.utils.motors.CloseLoopParam;
import frc.demacia.utils.motors.MotorInterface;

public class SysidCommand extends InstantCommand {
    public SysidCommand() {
        super(() -> {
            Map<String, CloseLoopParam> sysidResults = Sysid.getPidParams();
            if (sysidResults == null || sysidResults.isEmpty()) {
                return;
            }
            for (MotorInterface motor : Sysid.getMotors()) {
                String rawName = motor.getName();
    
                if (sysidResults.containsKey(rawName)) {
                    CloseLoopParam params = sysidResults.get(rawName);
                    motor.updatePid(params, 0);
                }
            }
        });
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}