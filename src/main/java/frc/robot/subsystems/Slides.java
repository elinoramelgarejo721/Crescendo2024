
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

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
    this.slidesEncoder = this.slides.getAlternateEncoder(8192);
    this.slidesEncoder.setPosition(0);
    this.slidesPID = this.slides.getPIDController();
    this.slidesPID.setFF(0);
    this.slidesPID.setP(SlidesConstants.slides_kp);
    this.slidesPID.setI(SlidesConstants.slides_ki);
    this.slidesPID.setD(SlidesConstants.slides_kd);

    this.bottomlimitSwitch = new DigitalInput(0);
    this.toplimitSwitch    = new DigitalInput(1);
    
    this.slides_state = 0;

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
      if (toplimitSwitch.get() == false) {
          // Limit switch not tripped
          slides.set(0.4);
      } 
        else if(bottomlimitSwitch.get() == true){
          // Limit Switch Tripped
          slides.set(0);
    }
  }

  public boolean topSwitchHit() {
    return toplimitSwitch.get() == true;
  }

  public boolean topSwitchNotHit() {
    return toplimitSwitch.get() == false;
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
    slides.set(0);
  }

  public void SetEncoderPosition(double setpoint) {
    this.slidesEncoder.setPosition(setpoint);
  }
  
  // PID Slides DON'T UTILIZE YET - NOT TUNED, MOST LIKELY NEED ABSOLUTE ENCODER
  // public void setSetpoint(double setpoint) {
  //   slidesPID.setSetpoint(setpoint);
  // }

  // public void runPID() {
  //   slides.set(slidesPID.calculate(getPositionInches()));
  // }

  public double getPosition() {
    return this.slides.getEncoder().getPosition();
  }

  // DON'T USE YET!!! MIGHT BREAK THINGS- PEOPLE WILL CRY!!! 
  // public void incrementUp() {
  //   if(Math.abs(SlidesConstants.position1 - getPositionInches()) <= SlidesConstants.tolerance) {
  //     slidesPID.setSetpoint(SlidesConstants.position2);
  //   }
  //   else if(Math.abs(SlidesConstants.position2 - getPositionInches()) <= SlidesConstants.tolerance) {
  //     slidesPID.setSetpoint(SlidesConstants.position3);
  //   }
  //   else if(Math.abs(SlidesConstants.position3 - getPositionInches()) <= SlidesConstants.tolerance) {
  //     slidesPID.setSetpoint(SlidesConstants.position1);
  //   }
  // }

  // public void incrementDown() {
  //   if(Math.abs(SlidesConstants.position1 - getPositionInches()) <= SlidesConstants.tolerance) {
  //     slidesPID.setSetpoint(SlidesConstants.position3);
  //   }
  //   else if(Math.abs(SlidesConstants.position2 - getPositionInches()) <= SlidesConstants.tolerance) {
  //     slidesPID.setSetpoint(SlidesConstants.position1);
  //   }
  //   else if(Math.abs(SlidesConstants.position3 - getPositionInches()) <= SlidesConstants.tolerance) {
  //     slidesPID.setSetpoint(SlidesConstants.position2);
  //   }
  // }

  public void runToState(double target) {
    this.slidesPID.setReference(target, ControlType.kPosition);
  }

  public int getState() {
    return this.slides_state;
  }

  public void counterUp() {
    if (this.slides_state == 2)
      return;
    this.slides_state++;
  }

  public void counterDown() {
    if (this.slides_state == 0)
      return;
    this.slides_state--;
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