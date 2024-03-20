
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {

  // Left Climber
  private CANSparkMax launcher;
  private RelativeEncoder launcherEncoder;
  private PIDController launcherPID;

  /** Creates a new ExampleSubsystem. */
  public Launcher() {

    // Left Climber
    this.launcher = new CANSparkMax(LauncherConstants.launcher_id, MotorType.kBrushless);
    launcher.setInverted(false);
    this.launcherPID = new PIDController(
        LauncherConstants.launcher_kp,
        LauncherConstants.launcher_ki,
        LauncherConstants.launcher_kd);
    this.launcherPID.setTolerance(200);
    this.launcherPID.setSetpoint(0);
    this.launcherEncoder = launcher.getEncoder();

    Shuffleboard.getTab("Game").addDouble(
        "Launcher" + "Pos", () -> launcherEncoder.getPosition());

  }

  // Launcher
  public void Run(double speed) {
    launcher.set(speed);
  }

  public void RunAmp() {
    launcher.set(0.4);
  }

  public void RunAmp_SlidesDown() {
    launcher.set(0.9);
  }

  public void RunSpeaker() {
    launcher.set(1);
  }

  public void Off() {
    launcher.set(0);
  }

  public void setSetpoint(double setpoint) {
    launcherPID.setSetpoint(setpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    launcher.set(launcherPID.calculate(launcherEncoder.getVelocity()/3));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}