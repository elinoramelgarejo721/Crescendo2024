// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SlidesConstants;
import frc.robot.subsystems.Launcher;
public class DefaultLauncher extends Command {

  Launcher launcher;

  /** Creates a new DefaultSlides. */
  public DefaultLauncher(Launcher launcher) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcher = launcher;
    addRequirements(launcher);

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
