
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SlidesConstants;

public class Climber extends SubsystemBase {

  // Left Climber
  private CANSparkMax leftClimber;
  private RelativeEncoder leftClimbEncoder;
  private SparkPIDController left_PID;

  private int lclimber_state;

  // Right Climber
  private CANSparkMax rightClimber;
  private RelativeEncoder rightClimbEncoder;
  private SparkPIDController right_PID;

  private int rclimber_state;

  /** Creates a new ExampleSubsystem. */
  public Climber() {

    // Left Climber
    this.leftClimber = new CANSparkMax(ClimberConstants.left_climber_id, MotorType.kBrushless);
    this.leftClimber.setInverted(true);
    this.leftClimbEncoder = leftClimber.getEncoder();
    this.leftClimber.setSmartCurrentLimit(ClimberConstants.left_climber_current_limit);
    this.leftClimber.setSecondaryCurrentLimit(ClimberConstants.left_climber_current_limit);
    this.leftClimbEncoder.setPosition(0);
    this.left_PID = this.leftClimber.getPIDController();
    this.left_PID.setP(ClimberConstants.lclimber_kp);
    this.left_PID.setI(ClimberConstants.lclimber_ki);
    this.left_PID.setD(ClimberConstants.lclimber_kd);
    this.left_PID.setFF(ClimberConstants.lclimber_kFF);

    this.lclimber_state = 0;

    Shuffleboard.getTab("Game").addDouble(
        "Climber" + " LeftPos", () -> leftClimbEncoder.getPosition()
    );

    // Right Climber
    this.rightClimber = new CANSparkMax(ClimberConstants.right_climber_id, MotorType.kBrushless);
    this.rightClimber.setInverted(false);
    this.rightClimbEncoder = rightClimber.getEncoder();
    this.rightClimber.setSmartCurrentLimit(ClimberConstants.right_climber_current_limit);
    this.rightClimber.setSecondaryCurrentLimit(ClimberConstants.right_climber_current_limit);
    this.rightClimbEncoder.setPosition(0);
    this.right_PID = this.rightClimber.getPIDController();
    this.right_PID.setP(ClimberConstants.rclimber_kp);
    this.right_PID.setI(ClimberConstants.rclimber_ki);
    this.right_PID.setD(ClimberConstants.rclimber_kd);
    this.right_PID.setFF(ClimberConstants.rclimber_kFF);

    this.rclimber_state = 0;

    Shuffleboard.getTab("Game").addDouble(
        "Climber" + " RightPos", () -> rightClimbEncoder.getPosition()
    );
    
  }

  // Left Climber
  public void LeftRun(double speed) {
    leftClimber.set(speed);
  }

  // Right Climber
  public void RightRun(double speed) {
    rightClimber.set(speed);
  }

  // PID Climber
  
  /* Left Climber */
  public void lrunToState(double target) {
    this.left_PID.setReference(target, ControlType.kPosition);
  }

  public int lgetState() {
    return this.lclimber_state;
  }

  public void lcounterUp() {
    if (this.lclimber_state == 1)
      return;
    this.lclimber_state++;
  }

  public void lcounterDown() {
    if (this.lclimber_state == 0)
      return;
    this.lclimber_state--;
  }

  /* Right Climber */
  public void rrunToState(double target) {
    this.right_PID.setReference(target, ControlType.kPosition);
  }

  public int rgetState() {
    return this.rclimber_state;
  }

  public void rcounterUp() {
    if (this.rclimber_state == 1)
      return;
    this.rclimber_state++;
  }

  public void rcounterDown() {
    if (this.rclimber_state == 0)
      return;
    this.rclimber_state--;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}