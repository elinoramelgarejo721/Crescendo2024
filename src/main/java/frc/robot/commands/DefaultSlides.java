// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SlidesConstants;
import frc.robot.subsystems.Slides;
public class DefaultSlides extends Command {

  Slides slides;
  // private DigitalInput toplimitSwitch;
  // private DigitalInput bottomlimitSwitch;
  private double position1 = SlidesConstants.position1;
  private double position2 = SlidesConstants.position2;
  private double position3 = SlidesConstants.position3;

  /** Creates a new DefaultSlides. */
  public DefaultSlides(Slides slides) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.slides = slides;
    addRequirements(slides);

    // this.bottomlimitSwitch = new DigitalInput(0);
    // this.toplimitSwitch    = new DigitalInput(1);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(this.slides.getState()) {

      case 0:
        this.slides.runToState(position1);
        break;

      case 1:
        if (this.slides.bottomSwitchNotHit()){
          this.slides.runToState(position2);
        }  
        else if (this.slides.bottomSwitchHit()) {
          position2 += 1;
        } 
        break;

      case 2:
        if (this.slides.topSwitchNotHit()) {
          this.slides.runToState(position3);
        } 
        else if (this.slides.topSwitchHit()) {
          position3 -= 1;
        } 
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
