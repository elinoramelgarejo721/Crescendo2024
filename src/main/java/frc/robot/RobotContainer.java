// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Java imports
import java.util.*;

// FIRST imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

// Pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.CommandUtil;

// Subsystems
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
// import frc.robot.subsystems.SwerveAutoBuild;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Slides;
import frc.robot.Constants.DriveConstants.MotorPosition;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DefaultClimbers;
// import frc.robot.commands.ExampleAuto;
import frc.robot.commands.DefaultSlides;
// Commands
import frc.robot.commands.TeleOpSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final SwerveDrive s_SwerveDrive   = new SwerveDrive();
  private final Climber         s_Climber   = new Climber();
  private final Intake          s_Intake    = new Intake();
  private final Launcher       s_Launcher   = new Launcher();
  private final Slides           s_Slides   = new Slides();

  private final SendableChooser<Command> autoChooser;

  /* Commands */

  // Reset to Absolute
  InstantCommand reset_to_abs = new InstantCommand(() -> {this.s_SwerveDrive.resetToAbsolute();}, this.s_SwerveDrive);

  // Left Climber
  InstantCommand run_left_climber_up   = new InstantCommand(() -> {this.s_Climber.LeftRun(0.75); }, this.s_Climber);
  InstantCommand run_left_climber_down = new InstantCommand(() -> {this.s_Climber.LeftRun(-0.75); }, this.s_Climber);

  InstantCommand rClimber_increment_state = new InstantCommand( () -> {this.s_Climber.rcounterUp();} );
  InstantCommand rClimber_decrement_state = new InstantCommand( () -> {this.s_Climber.rcounterDown();} );

  // Right Climber
  InstantCommand run_right_climber_up   = new InstantCommand( () -> {this.s_Climber.RightRun(0.75); }, this.s_Climber );
  InstantCommand run_right_climber_down = new InstantCommand( () -> {this.s_Climber.RightRun(-0.75); }, this.s_Climber );

  // Left Climber
  InstantCommand lClimber_increment_state = new InstantCommand( () -> {this.s_Climber.lcounterUp();} );
  InstantCommand lClimber_decrement_state = new InstantCommand( () -> {this.s_Climber.lcounterDown();} );

  // Slides
  InstantCommand slides_increment_state = new InstantCommand( () -> {this.s_Slides.counterUp();} );
  InstantCommand slides_decrement_state = new InstantCommand( () -> {this.s_Slides.counterDown();} );

  // Xbox Controller
  private final CommandXboxController driverController =
      new CommandXboxController(ControllerConstants.driver_controller_id);

  private final CommandXboxController driverController2 =
      new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    s_SwerveDrive.setDefaultCommand(
      new TeleOpSwerve(
      s_SwerveDrive, 
      () -> -driverController.getLeftY(), 
      () -> driverController.getLeftX(), 
      () -> -driverController.getRightX(),
      () -> driverController.back().getAsBoolean()
      )
    );

    // s_Slides.setDefaultCommand(
    //   new DefaultSlides(s_Slides)
    // );

    // s_Climber.setDefaultCommand(
    //   new DefaultClimbers(s_Climber, s_Climber)
    // );

    // Register Named Commands
    NamedCommands.registerCommand("RunSpeaker", new SequentialCommandGroup(new InstantCommand(() -> s_Launcher.RunSpeaker()), new InstantCommand(() -> s_Intake.feedToLauncher())));
    NamedCommands.registerCommand("RunAmp", new SequentialCommandGroup(new InstantCommand(() -> s_Launcher.RunAmp()), new InstantCommand(() -> s_Intake.feedToLauncher())));
    NamedCommands.registerCommand("SpeakerDone", new SequentialCommandGroup(new InstantCommand(() -> s_Launcher.Off()), new InstantCommand(() -> s_Intake.Off())));
    NamedCommands.registerCommand("IntakeOff", new InstantCommand(() -> s_Intake.Off()));
    NamedCommands.registerCommand("SlidesUp", slides_increment_state);
    NamedCommands.registerCommand("SlidesDown", slides_decrement_state);
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> s_Intake.intake()));
    NamedCommands.registerCommand("LauncherOff", new InstantCommand(() -> s_Launcher.Off()));
    NamedCommands.registerCommand("ResetModules", new InstantCommand(() -> s_SwerveDrive.resetToAbsolute()));
    NamedCommands.registerCommand("ResetPigeon", new InstantCommand(() -> s_SwerveDrive.zero_imu()));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Aligning the Modules
    driverController.start().whileTrue(reset_to_abs);
    driverController.povUp().whileTrue(reset_to_abs);

    // Zeroing the IMU
    // driverController.start().whileTrue(new InstantCommand(()-> s_SwerveDrive.zero_imu()));
    driverController.povDown().whileTrue(new InstantCommand(()-> s_SwerveDrive.zero_imu()));

    // Launcher
    driverController.a().onTrue(
        new SequentialCommandGroup(
          new InstantCommand(() -> s_Launcher.RunAmp()), 
          new InstantCommand(() -> s_Intake.feedToLauncher())
        )
      ).onFalse(
        new SequentialCommandGroup(
          new InstantCommand(() -> {s_Launcher.Off(); }, s_Launcher), 
          new InstantCommand(() -> {s_Intake.Off(); }, s_Intake)
        )
      );
    driverController.b().onTrue(
        new SequentialCommandGroup(
          new InstantCommand(() -> s_Launcher.RunSpeaker()), 
          new InstantCommand(() -> s_Intake.feedToLauncher())
        )
      ).onFalse(
        new SequentialCommandGroup(
          new InstantCommand(() -> {s_Launcher.Off(); }, s_Launcher), 
          new InstantCommand(() -> {s_Intake.Off(); }, s_Intake)
        )
      );

    driverController2.a().onTrue(new InstantCommand(() -> s_Launcher.setSetpoint(0)));
    driverController2.b().onTrue(new InstantCommand(() -> s_Launcher.setSetpoint(2000)));
    driverController2.y().onTrue(new InstantCommand(() -> s_Launcher.setSetpoint(4000)));
    driverController2.x().onTrue(new InstantCommand(() -> s_Launcher.setSetpoint(6000)));

    // Intake 
    driverController.rightTrigger().onTrue(new InstantCommand(() -> s_Intake.intake())).onFalse(new InstantCommand(() -> {s_Intake.Off(); }, s_Intake));
    driverController.leftTrigger().onTrue(new InstantCommand(() -> s_Intake.Outtake())).onFalse(new InstantCommand(() -> {s_Intake.Off(); }, s_Intake));

    // Slides 
    driverController.y().onTrue(new RunCommand(() -> s_Slides.SlidesUp(), s_Slides)).onFalse(new InstantCommand(() -> {s_Slides.SlidesOff(); }, s_Slides));;
    driverController.x().onTrue(new RunCommand(() -> s_Slides.SlidesDown(), s_Slides)).onFalse(new InstantCommand(() -> {s_Slides.SlidesOff(); }, s_Slides));
    // driverController.y().onTrue(slides_increment_state).onFalse(new InstantCommand(() -> {s_Slides.SlidesOff(); }, s_Slides));
    // driverController.x().onTrue(slides_decrement_state).onFalse(new InstantCommand(() -> {s_Slides.SlidesOff(); }, s_Slides));

    // Left Climber
    driverController.leftBumper().onTrue(run_left_climber_up).onFalse(new InstantCommand(() -> {s_Climber.LeftRun(0); }, s_Climber));
    driverController.leftStick().onTrue(run_left_climber_down).onFalse(new InstantCommand(() -> {s_Climber.LeftRun(0); }, s_Climber));
    // driverController.leftBumper().onTrue(lClimber_increment_state).onFalse(new InstantCommand(() -> {s_Climber.LeftRun(0); }, s_Climber));
    // driverController.leftStick().onTrue(lClimber_decrement_state).onFalse(new InstantCommand(() -> {s_Climber.LeftRun(0); }, s_Climber));

    // Right Climber
    driverController.rightBumper().onTrue(run_right_climber_up).onFalse(new InstantCommand(() -> {s_Climber.RightRun(0); }, s_Climber));
    driverController.rightStick().onTrue(run_right_climber_down).onFalse(new InstantCommand(() -> {s_Climber.RightRun(0); }, s_Climber));
    // driverController.rightBumper().onTrue(rClimber_increment_state).onFalse(new InstantCommand(() -> {s_Climber.RightRun(0); }, s_Climber));
    // driverController.rightStick().onTrue(rClimber_decrement_state).onFalse(new InstantCommand(() -> {s_Climber.RightRun(0); }, s_Climber));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }
}
