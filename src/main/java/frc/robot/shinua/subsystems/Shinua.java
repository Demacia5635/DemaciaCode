package frc.robot.shinua.subsystems;

import frc.demacia.utils.mechanisms.StateBaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import static frc.robot.shinua.ShinuaConstants.*;
import static frc.robot.shinua.ShinuaConstants.ShinuaRollersConstants.*;
import static frc.robot.shinua.ShinuaConstants.MecanumConstants.*;

public class Shinua extends StateBaseMechanism<ShinuaStates> {
    private static Shinua instance;

    private Shinua() {
        super(SHINUA_NAME, 
        new MotorInterface[] {
            new TalonFXMotor(SHINUA_ROLLERS_CONFIG),
            new TalonFXMotor(MECANUM_CONFIG),
        }, 
        new SensorInterface[] {
        });

    }

    public static Shinua getInstance() {
        if (instance == null) {
            instance = new Shinua();
        }
        return instance;
    }

    public void setShinuaRollersPower(double power) {
        setPower(SHINUA_ROLLERS_NAME, power);
    }

    public void setMecanumPower(double power) {
        setPower(MECANUM_NAME, power);
    }

    public boolean isStuckShinuaRollers() {
        return getMotor(SHINUA_ROLLERS_NAME).isStuck();
    }
    public boolean isStuckMecanum() {
        return getMotor(MECANUM_NAME).isStuck();
    }
}
