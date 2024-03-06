// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SlidesConstants;
import frc.robot.subsystems.Slides;

public class DefaultSlides extends Command {

  Slides slides;

  /** Creates a new DefaultSlides. */
  public DefaultSlides(Slides slides) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.slides = slides;
    addRequirements(slides);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(this.slides.getState()) {

      case 0:
        this.slides.runToState(SlidesConstants.position1);
        break;

      case 1:
        this.slides.runToState(SlidesConstants.position2);
        break;

      case 2:
        this.slides.runToState(SlidesConstants.position3);
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
