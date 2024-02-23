// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;

public class shootVelocity extends Command {
  private final Shooter Shooter;
  private final Intake handoff;
  boolean zero;

  /** Creates a new runIntake. */
  public shootVelocity(Shooter Shooter, Intake handoff, boolean zero) {
    this.Shooter = Shooter;
    this.handoff = handoff;
    this.zero = zero;


    addRequirements(Shooter, handoff);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(zero){
      Shooter.zeroVelocity();
      handoff.spinIntake(0);
    }
    else{
    Shooter.shootVelocity();
    handoff.spinIntake(2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}


