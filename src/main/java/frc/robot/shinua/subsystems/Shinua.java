package frc.robot.shinua.subsystems;

import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import static frc.robot.shinua.ShinuaConstants.*;
import static frc.robot.shinua.ShinuaConstants.MechanomConstants.*;
import static frc.robot.shinua.ShinuaConstants.ShinuaRollersConstants.*;

public class Shinua extends StateBaseMechanism {
    private static Shinua instance;

    private Shinua() {
        super(SHINUA_NAME, 
        new MotorInterface[] {
            new TalonFXMotor(MECHANOM_CONFIG),
            new TalonFXMotor(SHINUA_ROLLERS_CONFIG),
        }, 
        new SensorInterface[] {
        }, 
        ShinuaStates.class);

    }

    public static Shinua getInstance() {
        if (instance == null) {
            instance = new Shinua();
        }
        return instance;
    }

}
