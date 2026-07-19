// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  double millis;
  long count;
  private int warmupCount;
  double random;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    long start = System.nanoTime();

    CommandScheduler.getInstance().run();

    long end = System.nanoTime();

    random=Math.random();
    m_robotContainer.sim.setRawRotorPosition(random);
    m_robotContainer.sim.setRotorVelocity(random*2);
    m_robotContainer.sim.setRotorAcceleration(random*3);

    m_robotContainer.sim2.setRawRotorPosition(random);
    m_robotContainer.sim2.setRotorVelocity(random*2);
    m_robotContainer.sim2.setRotorAcceleration(random*3);

    m_robotContainer.sim3.setRawRotorPosition(random);
    m_robotContainer.sim3.setRotorVelocity(random*2);
    m_robotContainer.sim3.setRotorAcceleration(random*3);

    m_robotContainer.sim4.setRawRotorPosition(random);
    m_robotContainer.sim4.setRotorVelocity(random*2);
    m_robotContainer.sim4.setRotorAcceleration(random*3);

    m_robotContainer.sim5.setRawRotorPosition(random);
    m_robotContainer.sim5.setRotorVelocity(random*2);
    m_robotContainer.sim5.setRotorAcceleration(random*3);

    m_robotContainer.sim6.setRawRotorPosition(random);
    m_robotContainer.sim6.setRotorVelocity(random*2);
    m_robotContainer.sim6.setRotorAcceleration(random*3);

    m_robotContainer.sim7.setRawRotorPosition(random);
    m_robotContainer.sim7.setRotorVelocity(random*2);
    m_robotContainer.sim7.setRotorAcceleration(random*3);

    m_robotContainer.sim8.setRawRotorPosition(random);
    m_robotContainer.sim8.setRotorVelocity(random*2);
    m_robotContainer.sim8.setRotorAcceleration(random*3);

    m_robotContainer.sim9.setRawRotorPosition(random);
    m_robotContainer.sim9.setRotorVelocity(random*2);
    m_robotContainer.sim9.setRotorAcceleration(random*3);

    m_robotContainer.sim10.setRawRotorPosition(random);
    m_robotContainer.sim10.setRotorVelocity(random*2);
    m_robotContainer.sim10.setRotorAcceleration(random*3);

    m_robotContainer.sim11.setRawRotorPosition(random);
    m_robotContainer.sim11.setRotorVelocity(random*2);
    m_robotContainer.sim11.setRotorAcceleration(random*3);

    m_robotContainer.sim12.setRawRotorPosition(random);
    m_robotContainer.sim12.setRotorVelocity(random*2);
    m_robotContainer.sim12.setRotorAcceleration(random*3);

    m_robotContainer.sim13.setRawRotorPosition(random);
    m_robotContainer.sim13.setRotorVelocity(random*2);
    m_robotContainer.sim13.setRotorAcceleration(random*3);

    m_robotContainer.sim14.setRawRotorPosition(random);
    m_robotContainer.sim14.setRotorVelocity(random*2);
    m_robotContainer.sim14.setRotorAcceleration(random*3);

    m_robotContainer.sim15.setRawRotorPosition(random);
    m_robotContainer.sim15.setRotorVelocity(random*2);
    m_robotContainer.sim15.setRotorAcceleration(random*3);

    m_robotContainer.sim16.setRawRotorPosition(random);
    m_robotContainer.sim16.setRotorVelocity(random*2);
    m_robotContainer.sim16.setRotorAcceleration(random*3);

    m_robotContainer.sim17.setRawRotorPosition(random);
    m_robotContainer.sim17.setRotorVelocity(random*2);
    m_robotContainer.sim17.setRotorAcceleration(random*3);

    m_robotContainer.sim18.setRawRotorPosition(random);
    m_robotContainer.sim18.setRotorVelocity(random*2);
    m_robotContainer.sim18.setRotorAcceleration(random*3);

    m_robotContainer.sim19.setRawRotorPosition(random);
    m_robotContainer.sim19.setRotorVelocity(random*2);
    m_robotContainer.sim19.setRotorAcceleration(random*3);

    m_robotContainer.sim20.setRawRotorPosition(random);
    m_robotContainer.sim20.setRotorVelocity(random*2);
    m_robotContainer.sim20.setRotorAcceleration(random*3);

    m_robotContainer.sim21.setRawRotorPosition(random);
    m_robotContainer.sim21.setRotorVelocity(random*2);
    m_robotContainer.sim21.setRotorAcceleration(random*3);

    m_robotContainer.sim22.setRawRotorPosition(random);
    m_robotContainer.sim22.setRotorVelocity(random*2);
    m_robotContainer.sim22.setRotorAcceleration(random*3);

    m_robotContainer.sim23.setRawRotorPosition(random);
    m_robotContainer.sim23.setRotorVelocity(random*2);
    m_robotContainer.sim23.setRotorAcceleration(random*3);

    m_robotContainer.sim24.setRawRotorPosition(random);
    m_robotContainer.sim24.setRotorVelocity(random*2);
    m_robotContainer.sim24.setRotorAcceleration(random*3);

    m_robotContainer.sim25.setRawRotorPosition(random);
    m_robotContainer.sim25.setRotorVelocity(random*2);
    m_robotContainer.sim25.setRotorAcceleration(random*3);

    m_robotContainer.sim26.setRawRotorPosition(random);
    m_robotContainer.sim26.setRotorVelocity(random*2);
    m_robotContainer.sim26.setRotorAcceleration(random*3);

    m_robotContainer.sim27.setRawRotorPosition(random);
    m_robotContainer.sim27.setRotorVelocity(random*2);
    m_robotContainer.sim27.setRotorAcceleration(random*3);

    m_robotContainer.sim28.setRawRotorPosition(random);
    m_robotContainer.sim28.setRotorVelocity(random*2);
    m_robotContainer.sim28.setRotorAcceleration(random*3);

    m_robotContainer.sim29.setRawRotorPosition(random);
    m_robotContainer.sim29.setRotorVelocity(random*2);
    m_robotContainer.sim29.setRotorAcceleration(random*3);

    m_robotContainer.sim30.setRawRotorPosition(random);
    m_robotContainer.sim30.setRotorVelocity(random*2);
    m_robotContainer.sim30.setRotorAcceleration(random*3);

    if (warmupCount < 50*120) {
      warmupCount++;
      return;
    }

    millis += (end - start) / 1e6;
    count++;
    SmartDashboard.putNumber("Periodic Time ms", millis / count);

    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
