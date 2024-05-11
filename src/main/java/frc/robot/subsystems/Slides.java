
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SlidesConstants;

public class Slides extends SubsystemBase {

  // Left Climber
  private CANSparkMax slides;
  private RelativeEncoder slidesEncoder;
  private SparkPIDController slidesPID;

  private DigitalInput toplimitSwitch;
  private DigitalInput bottomlimitSwitch;

  private int slides_state;

  /** Creates a new ExampleSubsystem. */
  public Slides() {

    // Slides Config
    this.slides = new CANSparkMax(SlidesConstants.slides_id, MotorType.kBrushless);
    this.slidesEncoder = this.slides.getEncoder();
    this.slidesEncoder.setPosition(0);
    this.slidesPID = this.slides.getPIDController();
    slidesPID.setFeedbackDevice(slidesEncoder);
    this.slidesPID.setFF(0);
    this.slidesPID.setP(SlidesConstants.slides_kp);
    this.slidesPID.setI(SlidesConstants.slides_ki);
    this.slidesPID.setD(SlidesConstants.slides_kd);

    this.bottomlimitSwitch = new DigitalInput(0);
    this.toplimitSwitch    = new DigitalInput(1);
    
    this.slides_state = 1;

    Shuffleboard.getTab("Game").addDouble(
        "Slides" + "Pos", () -> getPosition()
    );

    Shuffleboard.getTab("Game").addBoolean(
        "BottomLimit", () -> bottomlimitSwitch.get()
    );

    Shuffleboard.getTab("Game").addBoolean(
        "TopLimit" , () -> toplimitSwitch.get()
    );


  }

  // Slides
  public void SlidesUp() {
      if (toplimitSwitch.get() == true) {
          // Limit switch not tripped
          slides.set(0.4);
      } 
        else if(toplimitSwitch.get() == false){
          // Limit Switch Tripped
          slides.set(0);
    }
  }

  public boolean topSwitchHit() {
    return toplimitSwitch.get() == false;
  }

  public boolean topSwitchNotHit() {
    return toplimitSwitch.get() == true;
  }

  public void SlidesDown() {
    if (bottomlimitSwitch.get() == true) {
          // Limit switch not tripped
          slides.set(-0.4);
      } 
        else if(bottomlimitSwitch.get() == false){
          // Limit Switch Tripped
          slides.set(0);
    }
  }

  public boolean bottomSwitchHit() {
    return bottomlimitSwitch.get() == false;
  }

  public boolean bottomSwitchNotHit() {
    return bottomlimitSwitch.get() == true;
  }

  public void SlidesOff() {
        System.out.println("baddd: "+this.slides_state);

    slides.set(0);
  }

  public void SetEncoderPosition(double setpoint) {
    this.slidesEncoder.setPosition(setpoint);
  }

  public double getPosition() {
    return this.slides.getEncoder().getPosition();
  }

  public void runToState(double target) {
    this.slidesPID.setReference(target, ControlType.kPosition);
  }

  public int getState() {
    return this.slides_state;
  }

  public void counterUp() {
    this.slides_state=MathUtil.clamp(++this.slides_state,0,2);
    System.out.println("Swichhhh: "+this.slides_state);
  }

  public void counterDown() {
      this.slides_state=MathUtil.clamp(--this.slides_state,0,2);

  }

  public boolean getBottomLimitSwitch() {
    return this.bottomlimitSwitch.get();
  }

  public boolean getTopLimitSwitch() {
    return this.toplimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // runToState(target);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}