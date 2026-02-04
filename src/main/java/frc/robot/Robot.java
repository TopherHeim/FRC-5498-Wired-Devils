// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
//import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.loggingUtil.LogManager;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.ArmStuff.ShooterTestManager;
import frc.robot.subsystems.swerve.Swerve;


public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
    private  ShooterTestManager shooterTestManager;
  
  
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    FollowPathCommand.warmupCommand().schedule();
  }

  
  @Override
  public void robotPeriodic() {
    
    SmartDashboard.putNumber("Match Timer", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    LogManager.log();
  }

  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  
  @Override
  public void autonomousPeriodic() {
    
  }

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

  
  @Override
  public void teleopPeriodic() {}
//You Are A F***in Wizard ^u^
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    double cameraHeightMeters = 0.6;
    double targetHeightMeters = 2.64;  
    double cameraAngleDegrees = 20.0;  

    shooterTestManager = new ShooterTestManager(m_robotContainer.getShooter(), cameraHeightMeters, targetHeightMeters, cameraAngleDegrees);
}

  
  @Override 
  
  public void testPeriodic() {
    if (shooterTestManager != null) {
        shooterTestManager.periodic();
    }
  }
}
