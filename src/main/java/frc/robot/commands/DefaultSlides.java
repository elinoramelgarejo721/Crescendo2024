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

      /* SLIDES STATES */

      // if the bottom limit switch is hit, it tells the encoder that it's at position 1 (Makes sure it doesn't drive over the limit)
      case 0:
        if (this.slides.bottomSwitchNotHit()){
          this.slides.runToState(SlidesConstants.position1);
        }  
        else if (this.slides.bottomSwitchHit()) {
          this.slides.SetEncoderPosition(SlidesConstants.position1);
        } 
        break;

      // Runs to the starting position (Middle of the Elevator)
      case 1:
        this.slides.runToState(SlidesConstants.position2);
        // if (this.slides.bottomSwitchHit()) {
        //   this.slides.SetEncoderPosition(SlidesConstants.position1);
        // }
        break;

      // if the top limit switch is hit, it tells the encoder that it's at position 2 (Makes sure it doesn't drive over the limit)
      case 2:
        if (this.slides.topSwitchNotHit()) {
          this.slides.runToState(SlidesConstants.position3);
        } 
        else if (this.slides.topSwitchHit()) {
          this.slides.SetEncoderPosition(SlidesConstants.position3);
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
