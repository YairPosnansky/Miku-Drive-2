// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Swerve.PID;
import frc.robot.subsystems.Drive.DriveSubsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private ShuffleboardTab SwerveDataTab;
  private ShuffleboardTab drivePIDTab;
  private ShuffleboardTab steerPIDTab;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Shuffleboard setup
    SwerveDataTab = Shuffleboard.getTab("Swerve Data");
    drivePIDTab = Shuffleboard.getTab("Drive PID");
    steerPIDTab = Shuffleboard.getTab("Steer PID");

    // Add PID constants to Shuffleboard
    addPIDConstantsToShuffleboardDrive(drivePIDTab, "Drive");
    addPIDConstantsToShuffleboardSteer(steerPIDTab, "Steer");
  }

  private void updateCurrentGyroAngle() {
    SmartDashboard.putNumber("Swerve c Gyro Angle", m_robotContainer.getDriveSubsystem().getGyroAngleInRotation2d().getDegrees());
  }


  private void updateCurrentAngle() {
    SmartDashboard.putNumber("Swerve c bl", m_robotContainer.getDriveSubsystem().get_bl().getModuleState().angle.getDegrees());
    SmartDashboard.putNumber("Swerve c br", m_robotContainer.getDriveSubsystem().get_br().getModuleState().angle.getDegrees());
    SmartDashboard.putNumber("Swerve c fl", m_robotContainer.getDriveSubsystem().get_fl().getModuleState().angle.getDegrees());
    SmartDashboard.putNumber("Swerve c fr", m_robotContainer.getDriveSubsystem().get_fr().getModuleState().angle.getDegrees());
  }

  private void updateTargetVelocity() {
    SmartDashboard.putNumber("Swerve target velocity bl", m_robotContainer.getDriveSubsystem().get_bl().getTargetRotorVelocityRPM());
    SmartDashboard.putNumber("Swerve target velocity br", m_robotContainer.getDriveSubsystem().get_br().getTargetRotorVelocityRPM());
    SmartDashboard.putNumber("Swerve target velocity fl", m_robotContainer.getDriveSubsystem().get_fl().getTargetRotorVelocityRPM());
    SmartDashboard.putNumber("Swerve target velocity fr", m_robotContainer.getDriveSubsystem().get_fr().getTargetRotorVelocityRPM());
  }

  private void updateCurrentVelocity() {
    SmartDashboard.putNumber("Swerve current velocity bl", m_robotContainer.getDriveSubsystem().get_bl().getDriveMotor().getRotorVelocity().getValueAsDouble() *60);
    SmartDashboard.putNumber("Swerve current velocity br", m_robotContainer.getDriveSubsystem().get_br().getDriveMotor().getRotorVelocity().getValueAsDouble() *60);
    SmartDashboard.putNumber("Swerve current velocity fl", m_robotContainer.getDriveSubsystem().get_fl().getDriveMotor().getRotorVelocity().getValueAsDouble() *60);
    SmartDashboard.putNumber("Swerve current velocity fr", m_robotContainer.getDriveSubsystem().get_fr().getDriveMotor().getRotorVelocity().getValueAsDouble() *60);
  }

  private void updateTargetAngle() {
    SmartDashboard.putNumber("Swerve t bl", m_robotContainer.getDriveSubsystem().get_bl().getTargetState().angle.getDegrees());
    SmartDashboard.putNumber("Swerve t br", m_robotContainer.getDriveSubsystem().get_br().getTargetState().angle.getDegrees());
    SmartDashboard.putNumber("Swerve t fl", m_robotContainer.getDriveSubsystem().get_fl().getTargetState().angle.getDegrees());
    SmartDashboard.putNumber("Swerve t fr", m_robotContainer.getDriveSubsystem().get_fr().getTargetState().angle.getDegrees());
  }

  private void addPIDConstantsToShuffleboardDrive(ShuffleboardTab tab, String name) {
    tab.add(name + " kS", PID.Drive.kS);
    tab.add(name + " kV", PID.Drive.kV);
    tab.add(name + " kA", PID.Drive.kA);
    tab.add(name + " kP", PID.Drive.kP);
    tab.add(name + " kI", PID.Drive.kI);
    tab.add(name + " kD", PID.Drive.kD);
  }

  private void addPIDConstantsToShuffleboardSteer(ShuffleboardTab tab, String name) {
    tab.add(name + " kS", PID.Steer.kS);
    tab.add(name + " kV", PID.Steer.kV);
    tab.add(name + " kA", PID.Steer.kA);
    tab.add(name + " kP", PID.Steer.kP);
    tab.add(name + " kI", PID.Steer.kI);
    tab.add(name + " kD", PID.Steer.kD);
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode-specific periodic functions but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    updateCurrentAngle();
    updateTargetAngle();
    updateCurrentGyroAngle();
    updateCurrentVelocity();
    updateTargetVelocity();

    // Update Shuffleboard data here if needed
  }


  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
      m_robotContainer.getClass();
    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
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
