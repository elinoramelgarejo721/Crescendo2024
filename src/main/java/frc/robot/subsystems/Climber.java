package frc.robot.subsystems;

// REV IMPORTS
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

// WPI IMPORTS
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// ROBOT IMPORTS
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SlidesConstants;

public class Climber extends SubsystemBase {

 /* ----- Declares Motors & PID Controllers ----- */

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

  /* Creates a new ExampleSubsystem. */
  public Climber() {

    // Left Climber

    // ID
    this.leftClimber = new CANSparkMax(ClimberConstants.left_climber_id, MotorType.kBrushless);

    // Inverts the Motor
    this.leftClimber.setInverted(true);

    // Encoder
    this.leftClimbEncoder = leftClimber.getEncoder();

    // Sets the Current Limit
    this.leftClimber.setSmartCurrentLimit(ClimberConstants.left_climber_current_limit);
    this.leftClimber.setSecondaryCurrentLimit(ClimberConstants.left_climber_current_limit);

    // Zeroes the Encoder Position when the robot turns on
    this.leftClimbEncoder.setPosition(0);

    // Sets the values for the PID Controller
    this.left_PID = this.leftClimber.getPIDController();
    this.left_PID.setP(ClimberConstants.climbers_kp);
    this.left_PID.setI(ClimberConstants.climbers_ki);
    this.left_PID.setD(ClimberConstants.climbers_kd);
    this.left_PID.setFF(ClimberConstants.climbers_kFF);

    // Sets the Default Climber State
    this.lclimber_state = 1;

    // Adds the Data to Shuffleboard
    Shuffleboard.getTab("Game").addDouble(
        "Climber" + " LeftPos", () -> leftClimbEncoder.getPosition()
    );

    // Right Climber

    // ID
    this.rightClimber = new CANSparkMax(ClimberConstants.right_climber_id, MotorType.kBrushless);

    // Inverts the Motor
    this.rightClimber.setInverted(false);

    // Encoder
    this.rightClimbEncoder = rightClimber.getEncoder();

    // Sets the Current Limit
    this.rightClimber.setSmartCurrentLimit(ClimberConstants.right_climber_current_limit);
    this.rightClimber.setSecondaryCurrentLimit(ClimberConstants.right_climber_current_limit);

    // Zeroes the Encoder Position when the robot turns on
    this.rightClimbEncoder.setPosition(0);

    // Sets the values for the PID Controller
    this.right_PID = this.rightClimber.getPIDController();
    this.right_PID.setP(ClimberConstants.climbers_kp);
    this.right_PID.setI(ClimberConstants.climbers_ki);
    this.right_PID.setD(ClimberConstants.climbers_kd);
    this.right_PID.setFF(ClimberConstants.climbers_kFF);

    // Sets the Default Climber State
    this.rclimber_state = 1;

    // Adds the Data to Shuffleboard
    Shuffleboard.getTab("Game").addDouble(
        "Climber" + " RightPos", () -> rightClimbEncoder.getPosition()
    );
    
  }

  // Left Climber Run Function
  public void LeftRun(double speed) {
    leftClimber.set(speed);
  }

  // Right Climber Run Function
  public void RightRun(double speed) {
    rightClimber.set(speed);
  }

  // PID Commands for the Climbers
  
  /* --------- Left Climber --------- */

  // run to desired state
  public void lrunToState(double target) {
    this.left_PID.setReference(target, ControlType.kPosition);
  }

  // get the state
  public int lgetState() {
    return this.lclimber_state;
  }

  // incremental counter for the states
  public void lcounterUp() {
    if (this.lclimber_state == 3)
      return;
    this.lclimber_state++;
  }

  // decremental counter for the states
  public void lcounterDown() {
    if (this.lclimber_state == 0)
      return;
    this.lclimber_state--;
  }

  /* Right Climber */

  // run to desired state
  public void rrunToState(double target) {
    this.right_PID.setReference(target, ControlType.kPosition);
  }

  // get the state
  public int rgetState() {
    return this.rclimber_state;
  }

  // incremental counter for the states
  public void rcounterUp() {
    if (this.rclimber_state == 3)
      return;
    this.rclimber_state++;
  }

  // decremental counter for the states
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