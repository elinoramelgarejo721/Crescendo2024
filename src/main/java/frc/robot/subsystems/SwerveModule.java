package frc.robot.subsystems;

// FIRST imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
// import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;

// Pheonix imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

// Custom imports
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.lib.math.OnboardModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DriveConstants;
// import frc.robot.lib.util.CANSparkMaxUtil.Usage;
// import frc.robot.lib.util.CANSparkMaxUtil;

// Java imports
import java.util.*;

public class SwerveModule {
    
    public int module_number;

    private Rotation2d prev_angle;
    private Rotation2d angle_offset;

    private CANSparkMax angle_motor;
    private CANSparkMax drive_motor;

    private CANcoder can_coder;
    private StatusSignal pos;

    private RelativeEncoder angle_encoder;
    private RelativeEncoder drive_encoder;

    private SparkPIDController angle_controller;
    private PIDController drive_controller;
    
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModule(int module_number, SwerveModuleConstants module_constants) {

        this.module_number = module_number;

        this.can_coder = new CANcoder(module_constants.CANCoder_id);
        this.pos = this.can_coder.getAbsolutePosition();

        this.angle_motor = new CANSparkMax(module_constants.angle_motor_id, MotorType.kBrushless);
        this.angle_encoder = this.angle_motor.getEncoder(); 
        this.angle_controller = this.angle_motor.getPIDController();
        this.configAngleMotor();

        this.drive_motor = new CANSparkMax(module_constants.drive_motor_id, MotorType.kBrushless);
        this.drive_encoder = this.drive_motor.getEncoder();
        this.drive_controller = new PIDController(
            SwerveConstants.drive_kP, 
            SwerveConstants.drive_kI, 
            SwerveConstants.drive_kD
            );
        this.configDriveMotor();

        this.prev_angle = this.getState().angle;

    }

    public void resetToAbsolute() {
        Double absolutePosition = getCANDouble();
        angle_encoder.setPosition(absolutePosition);
    }

    private void configAngleMotor() {

        this.angle_motor.restoreFactoryDefaults();
        this.angle_motor.setInverted(SwerveConstants.angle_invert);
        this.angle_motor.setIdleMode(SwerveConstants.angle_idle_mode);
        this.angle_motor.setSmartCurrentLimit(SwerveConstants.angle_smart_current_limit);

        this.angle_encoder.setPositionConversionFactor(SwerveConstants.angleConversionFactor);
        this.angle_encoder.setPosition(0.0);

        this.angle_controller.setP(SwerveConstants.angle_kP);
        this.angle_controller.setI(SwerveConstants.angle_kI);
        this.angle_controller.setD(SwerveConstants.angle_kD);
        this.angle_controller.setFF(SwerveConstants.angle_kFF);

        this.angle_motor.enableVoltageCompensation(SwerveConstants.voltage_comp);
        this.angle_motor.burnFlash();

        resetToAbsolute();
    }

    private void configDriveMotor() {

        this.drive_motor.restoreFactoryDefaults();
        this.drive_motor.setInverted(SwerveConstants.drive_invert);
        this.drive_motor.setIdleMode(SwerveConstants.drive_idle_mode);
        this.drive_motor.setSmartCurrentLimit(SwerveConstants.drive_smart_current_limit);

        this.drive_encoder.setPositionConversionFactor(SwerveConstants.driveConversionPositionFactor);
        this.drive_encoder.setVelocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);

        this.drive_controller.setP(SwerveConstants.drive_kP);
        this.drive_controller.setI(SwerveConstants.drive_kI);
        this.drive_controller.setD(SwerveConstants.drive_kD);

        this.angle_motor.enableVoltageCompensation(SwerveConstants.voltage_comp);
        this.angle_motor.burnFlash();
        this.drive_encoder.setPosition(0.0);

    }

    public void setDesiredState(SwerveModuleState desired_state, boolean is_open_loop) {

        desired_state = OnboardModuleState.optimize(desired_state, getState().angle);

        this.setAngle(desired_state);
        this.setSpeed(desired_state, is_open_loop);

    }

    public void setTargetState(SwerveModuleState targetState) {
        // Optimize the state
        currentState = SwerveModuleState.optimize(targetState, currentState.angle);
  
        currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }

    public void setSpeed(SwerveModuleState desired_state, boolean is_open_loop) {

        if (is_open_loop) {

            double percent_output = desired_state.speedMetersPerSecond / SwerveConstants.maxSpeed;
            this.drive_motor.set(percent_output);

        } else {

            this.drive_motor.setVoltage(
                drive_controller.calculate(drive_encoder.getVelocity(), desired_state.speedMetersPerSecond)
            );

        }

    }

    public SwerveModulePosition getPosition() {
        return currentPosition;
    }

    public void setAngle(SwerveModuleState desired_state) {
   
        Rotation2d angle = 
            (Math.abs(desired_state.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
                ? prev_angle
                : desired_state.angle;
            
        // this.angle_motor.setVoltage(angle_controller.calculate(angle_encoder.getPosition(), angle.getDegrees()));
        angle_controller.setReference(angle.getDegrees(), ControlType.kPosition);
        this.prev_angle = angle;

    }

    public Rotation2d getAngle() {

        return Rotation2d.fromDegrees(this.angle_encoder.getPosition());

    }

    public SwerveModuleState getState() {

        return new SwerveModuleState(this.drive_encoder.getVelocity(), this.getAngle());

    }

    public Rotation2d getCANCoder() {
        return Rotation2d.fromDegrees(this.can_coder.getAbsolutePosition().getValue() * 360);
    }

    public Double getCANDouble() {
        return (this.can_coder.getAbsolutePosition().getValue() * 360);
    }

}
