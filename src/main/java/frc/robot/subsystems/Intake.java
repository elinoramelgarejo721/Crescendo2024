
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.util.PIDController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;

public class Intake extends SubsystemBase {

  // Left Climber
  private CANSparkMax intake;
  private RelativeEncoder intakeEncoder;

  private PIDController intakePID;

  /** Creates a new ExampleSubsystem. */
  public Intake() {

    // Left Climber
    this.intake = new CANSparkMax(IntakeConstants.intake_id, MotorType.kBrushless);
    this.intake.setInverted(true);
    this.intake.setIdleMode(IdleMode.kBrake);
    this.intakeEncoder = intake.getEncoder();
    this.intakePID = new PIDController(
      IntakeConstants.intake_kp, 
      IntakeConstants.intake_ki, 
      IntakeConstants.intake_kd
    );
    this.intakePID.setTolerance(0);
    this.intakePID.setSetpoint(0);

    Shuffleboard.getTab("Game").addDouble(
        "Intake" + "Pos", () -> intakeEncoder.getPosition()
    );

  }

  // Intake
  public void intake() {
    intakePID.setSetpoint(IntakeConstants.intakeSetpoint);
  }

  public void Outtake() {
    intakePID.setSetpoint(IntakeConstants.outtakeSetpoint);   
  }

  public void feedToLauncher() {
    intakePID.setSetpoint(IntakeConstants.feedToLauncher);
  }

  public void Off() {
    intakePID.setSetpoint(IntakeConstants.Off);
  }

  public void setSetpoint(double setpoint) {
    intakePID.setSetpoint(setpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intake.set(intakePID.calculateForPercent(intakeEncoder.getVelocity(), IntakeConstants.max_RPM));
    SmartDashboard.putNumber("IntakeRPM", intakeEncoder.getVelocity());
  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}