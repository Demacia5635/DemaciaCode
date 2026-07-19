package frc.robot.shootet.subsystems;

import frc.demacia.utils.mechanisms.BaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.Cancoder;
import frc.robot.RobotContainer;
import static frc.robot.shootet.ShootetConstants.*;
import static frc.robot.shootet.ShootetConstants.UnnamedMotor1Constants.*;
import static frc.robot.shootet.ShootetConstants.HoodConstants.*;
import static frc.robot.shootet.ShootetConstants.UnnamedMotor3Constants.*;
import static frc.robot.shootet.ShootetConstants.UnnamedSensor1CancoderConstants.*;

public class Shootet extends BaseMechanism {
    private static Shootet instance;

    private Shootet() {
        super(SHOOTET_NAME, 
        new MotorInterface[] {
            new TalonFXMotor(UNNAMED_MOTOR_1_CONFIG),
            new TalonFXMotor(HOOD_CONFIG),
            new TalonFXMotor(UNNAMED_MOTOR_3_CONFIG),
        }, 
        new SensorInterface[] {
            new Cancoder(UNNAMED_SENSOR_1_CANCODER_CONFIG),
        });

        withPowerCommand(UNNAMED_MOTOR_1_NAME, () -> RobotContainer.controller.getRightX());
        withPowerCommand(HOOD_NAME, () -> RobotContainer.controller.getRightY());
    }

    public static Shootet getInstance() {
        if (instance == null) {
            instance = new Shootet();
        }
        return instance;
    }

}
