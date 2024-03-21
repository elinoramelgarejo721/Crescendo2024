// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class DefaultClimbers extends Command {

  Climber leftClimber;
  Climber rightClimber;

  /** Creates a new DefaultSlides. */
  public DefaultClimbers(Climber leftClimber, Climber rightClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leftClimber = leftClimber;
    this.rightClimber = rightClimber;

    addRequirements(leftClimber, rightClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(this.leftClimber.lgetState()) {

      case 0:
        this.leftClimber.lrunToState(ClimberConstants.lposition1);
        break;

      case 1:
        this.leftClimber.lrunToState(ClimberConstants.lposition2);
        break;

      default:
        break;

    }

    switch(this.rightClimber.rgetState()) {

      case 0:
        this.rightClimber.rrunToState(ClimberConstants.rposition1);
        break;

      case 1:
        this.rightClimber.rrunToState(ClimberConstants.rposition2);
        break;

      default:
        break;

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
