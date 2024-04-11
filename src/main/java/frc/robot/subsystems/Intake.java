
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.util.PIDController;
import static frc.robot.Constants.IntakeConstants.*;
import frc.robot.Constants.LauncherConstants;

public class Intake extends SubsystemBase {

  // Left Climber
  private CANSparkMax intake;
  private RelativeEncoder intakeEncoder;

  private PIDController intakePID;

  private LinearFilter currentFilter = LinearFilter.movingAverage(2);
  private double filteredCurrent;

  /** Creates a new ExampleSubsystem. */
  public Intake() {

    // Left Climber
    this.intake = new CANSparkMax(intake_id, MotorType.kBrushless);
    this.intake.setInverted(true);
    this.intake.setIdleMode(IdleMode.kCoast);
    this.intakeEncoder = intake.getEncoder();
    this.intakePID = new PIDController(
      intake_kp, 
      intake_ki, 
      intake_kd
    );
    this.intakePID.setTolerance(0);
    this.intakePID.setSetpoint(0);

    Shuffleboard.getTab("Game").addDouble(
        "Intake" + "Pos", () -> intakeEncoder.getPosition()
    );

  }

  // Intake
  public void intake() {
    intakePID.setSetpoint(intakeSetpoint);
    // intake.set(1.0);
  }

  public void Outtake() {
    intakePID.setSetpoint(outtakeSetpoint);   
    // intake.set(-1.0);
  }

  public void feedToLauncher() {
    intakePID.setSetpoint(feedToLauncher);
    // intake.set(1.0);
  }

  public void Off() {
    intakePID.setSetpoint(Off);
    // intake.set(0);
  }

  public void setSetpoint(double setpoint) {
    intakePID.setSetpoint(setpoint);
  }

  public double getSetpoint() {
    return intakeEncoder.getVelocity();
  }

  // Note Detection
  public Command intakeNoteCommand() {
    
    Debouncer debounce = new Debouncer(1, Debouncer.DebounceType.kRising);
    
    return

    // set the intake to note intaking speed
    run( () -> { this.intake(); } )

    // Wait until current spike is detected for more than 1s
    .until(() -> debounce.calculate(getFilteredCurrent() > INTAKE_STALL_DETECTION))

    // Stop the motor
    .finallyDo( (interrupted) -> 

      {
      System.out.println("ended");
      this.Off();
      }

    );

  }

  public double getCurrent() {
    return intake.getOutputCurrent();
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public double getOutput() {
    return intake.getAppliedOutput();
  }


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    intake.set(intakePID.calculateForPercent(intakeEncoder.getVelocity(), max_RPM));
    SmartDashboard.putNumber("IntakeRPM", intakeEncoder.getVelocity());
    filteredCurrent = currentFilter.calculate(getCurrent());

    SmartDashboard.putNumber("outputCurrent", getCurrent());
    SmartDashboard.putNumber("filteredCurrent", getFilteredCurrent());
    SmartDashboard.putNumber("appliedOutput", getOutput());
    
  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}