// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autons.AutonomousSelector.modes;
import frc.robot.autons.modes.FOUR_HALF_PIECE;
import frc.robot.autons.modes.FOUR_PIECE;
import frc.robot.autons.modes.FOUR_PIECE_THREE_B_A;
import frc.robot.autons.modes.PRELOAD_AMP;
import frc.robot.autons.modes.PRELOAD_LEAVE_AMP;
import frc.robot.autons.modes.PRELOAD_LEAVE_MID;
import frc.robot.autons.modes.PRELOAD_LEAVE_SOURCE;
import frc.robot.autons.modes.PRELOAD_MID;
import frc.robot.autons.modes.PRELOAD_SOURCE;
import frc.robot.autons.modes.TWO_PIECE_AMP_ONE;
import frc.robot.autons.modes.TWO_PIECE_MID;
import frc.robot.autons.modes.TWO_PIECE_SOURCE;
import frc.robot.autons.modes.TWO_PIECE_SOURCE_THREE;
import frc.robot.autons.modes.choreotest;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //public static PathPlannerPath TWOPieceMid = PathPlannerPath.fromChoreoTrajectory("choreoPaths");
  SequentialCommandGroup mid_2p_b;
  SequentialCommandGroup mid_4p_b_c_a;
  SequentialCommandGroup mid_preload;
  SequentialCommandGroup mid_four_race_three;
  SequentialCommandGroup mid_preload_leave;
  SequentialCommandGroup mid_four_half_piece;
  SequentialCommandGroup amp_preload;
  SequentialCommandGroup amp_preload_leave;
  SequentialCommandGroup amp_two_piece_race_one;
  SequentialCommandGroup source_two_piece;
  SequentialCommandGroup source_two_piece_three;
  SequentialCommandGroup source_preload;
  SequentialCommandGroup source_preload_leave;
  
  private boolean built = false;

  @Override
  public void robotInit() {
    SignalLogger.setPath("/media/sda1/");
    
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.*/
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (DriverStation.getAlliance().isPresent() && !built){

    mid_preload = new PRELOAD_MID(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    mid_four_half_piece = new FOUR_HALF_PIECE(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    mid_2p_b = new TWO_PIECE_MID(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    mid_4p_b_c_a = new FOUR_PIECE(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    mid_four_race_three = new FOUR_PIECE_THREE_B_A(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    mid_preload_leave = new PRELOAD_LEAVE_MID(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    amp_preload = new PRELOAD_AMP(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    amp_preload_leave = new PRELOAD_LEAVE_AMP(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    amp_two_piece_race_one = new TWO_PIECE_AMP_ONE(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    source_two_piece = new TWO_PIECE_SOURCE(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    source_two_piece_three = new TWO_PIECE_SOURCE_THREE(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    source_preload_leave = new PRELOAD_LEAVE_SOURCE(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
    source_preload = new PRELOAD_SOURCE(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());

    built = true;
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
     if(m_robotContainer.getAutonomousCommand() == modes.MID_PRELOAD){
      m_autonomousCommand = mid_preload;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.MID_2p_B){
      m_autonomousCommand = mid_2p_b;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.MID_4p_B_C_A){
      m_autonomousCommand = mid_4p_b_c_a;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.TESTABLE_MID_FOUR_HALF_PIECE){
      m_autonomousCommand = mid_four_half_piece;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.TESTABLE_MID_FOUR_RACE_THREE){
      m_autonomousCommand = mid_four_race_three;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.MID_PRELOAD_LEAVE){
      m_autonomousCommand = mid_preload_leave;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.AMP_PRELOAD){
      m_autonomousCommand = amp_preload;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.AMP_PRELOAD_LEAVE){
      m_autonomousCommand = amp_preload_leave;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.TESTABLE_AMP_TWO_PIECE_ONE){
      m_autonomousCommand = amp_two_piece_race_one;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.TESTABLE_SOURCE_TWO_PIECE){
      m_autonomousCommand = source_two_piece;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.TESTABLE_SOURCE_TWO_PIECE_THREE){
      m_autonomousCommand = source_two_piece_three;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.SOURCE_PRELOAD){
      m_autonomousCommand = source_preload;
    }
    else if(m_robotContainer.getAutonomousCommand() == modes.SOURCE_PRELOAD_LEAVE){
      m_autonomousCommand = source_preload_leave;
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
