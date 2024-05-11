// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// FIRST imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Translation2d;

// REV imports
import com.revrobotics.CANSparkBase.IdleMode;

// Java imports
import java.util.*;

// Pheonix imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants {

    public static enum MotorPosition {
      C_FRONT_LEFT_DRIVE, C_FRONT_RIGHT_DRIVE, C_REAR_LEFT_DRIVE, C_REAR_RIGHT_DRIVE,
      C_FRONT_LEFT_STEER, C_FRONT_RIGHT_STEER, C_REAR_LEFT_STEER, C_REAR_RIGHT_STEER

    };

    // Motors
    public static final int front_left_drive_id     = 1;
    public static final int front_right_drive_id    = 2;
    public static final int rear_left_drive_id      = 3;
    public static final int rear_right_drive_id     = 4;

    public static final int front_left_steer_id     = 5;
    public static final int front_right_steer_id    = 6;
    public static final int rear_left_steer_id      = 7;
    public static final int rear_right_steer_id     = 8;

    // CANCoders
    public static final int front_left_CANcoder_id  = 15;
    public static final int front_right_CANcoder_id = 16;
    public static final int rear_left_CANcoder_id   = 17;
    public static final int rear_right_CANcoder_id  = 18;

    // IMU
    public static final boolean invert_imu = false;

    // Pigeon
    public static final int pigeon_id = 20;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (6.75*3 / 1.0); // 12.8:1

    // Math
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    
  }

  // Swerve Module Constants
  public static class SwerveModuleConstants {

    public int angle_motor_id;
    public int drive_motor_id;
    public int CANCoder_id;
    public Rotation2d angle_offset;

    public SwerveModuleConstants(int angle_motor_id, int drive_motor_id, int CANCoder_id) {

      this.angle_motor_id = angle_motor_id;
      this.drive_motor_id = drive_motor_id;
      this.CANCoder_id = CANCoder_id;
    }

  }

  // Other Swerve Constants
  public static class SwerveConstants {
    
    public static final boolean angle_invert = false;
    public static final boolean drive_invert = true;

    public static final IdleMode angle_idle_mode = IdleMode.kBrake;
    public static final IdleMode drive_idle_mode = IdleMode.kBrake;

    public static final int angle_smart_current_limit = 20;
    public static final int drive_smart_current_limit = 110;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
      (DriveConstants.wheelDiameter * Math.PI) / DriveConstants.driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360 / DriveConstants.angleGearRatio;


    // Angle PID Values
    public static final double angle_kP  = 0.01;
    public static final double angle_kI  = 0.000015;
    public static final double angle_kD  = 0.0;
    public static final double angle_kFF = 0.0;

    // Drive PID Values
    public static final double drive_kP  = 1.0;
    public static final double drive_kI  = 0.0;
    public static final double drive_kD  = 0.0;
    public static final double drive_kFF = 0.0;

    /* Drive Motor Characterization Values */
    // public static final double driveKS = 0.667;
    // public static final double driveKV = 2.44;
    // public static final double driveKA = 0.27;

    // Voltage Compensation
    public static final double voltage_comp = 12.0;

    // Math Utilized for Swerve Kinematics
    public static final double wheel_base = Units.inchesToMeters(24.75);
    public static final double track_width = Units.inchesToMeters(24.75);

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5; // 11.5

    // Set the kinematics for the wheels -/+ value show (x, y) for wheel positions
    public static final SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(
      new Translation2d(-wheel_base / 2.0, track_width / 2.0),    // -+
      new Translation2d(-wheel_base / 2.0, -track_width / 2.0),   // --
      new Translation2d(wheel_base / 2.0, track_width / 2.0),     // ++ 
      new Translation2d(wheel_base / 2.0, -track_width / 2.0)     // +-
    );

    // Took out angle offsets because we zeroed the modules from the CANCoders directly
    public static final SwerveModuleConstants[] module_constants = new SwerveModuleConstants[]{
      new SwerveModuleConstants(DriveConstants.front_left_steer_id, DriveConstants.front_left_drive_id, DriveConstants.front_left_CANcoder_id),
      new SwerveModuleConstants(DriveConstants.front_right_steer_id, DriveConstants.front_right_drive_id, DriveConstants.front_right_CANcoder_id),
      new SwerveModuleConstants(DriveConstants.rear_left_steer_id, DriveConstants.rear_left_drive_id, DriveConstants.rear_left_CANcoder_id),
      new SwerveModuleConstants(DriveConstants.rear_right_steer_id, DriveConstants.rear_right_drive_id, DriveConstants.rear_right_CANcoder_id)
    };

  }

  // Controller Constants
  public static class ControllerConstants {

    // Driver ID
    public static final int driver_controller_id = 0;

    // Deadband
    public static final double stickDeadband = 0.1;

    // Controller joystick 
    public static final Joystick driver1 = new Joystick(0);

  }

  // Intake Constants
  public static class IntakeConstants {

    // Intake Motor Controller ID
    public static final int intake_id = 10;

    // Intake PID Values
    public static final double intake_kp = 0.1;
    public static final double intake_ki = 0.0;
    public static final double intake_kd = 0.0;

    // Max RPM
    public static final double max_RPM      = 5676; //5676

    // Setpoint Constants
    public static final double intakeSetpoint   = 6000;
    public static final double outtakeSetpoint  = -6000;
    public static final double feedToLauncher   = 6000;
    public static final double Off              = 0;

    public static final double INTAKE_STALL_DETECTION = 9.1; // Amps
  }

  public static class ClimberConstants {

    // Climber Motor Controller IDs
    public static final int left_climber_id = 9;
    public static final int right_climber_id = 14;

    // Current Limits to preserve voltage
    public static final int left_climber_current_limit  = 40;
    public static final int right_climber_current_limit = 40;

    // Climber PID Constants
    public static final double climbers_kp  = 0.15;
    public static final double climbers_ki  = 0.0;
    public static final double climbers_kd  = 0.0;
    public static final double climbers_kFF = 0.0;

    // Left Climber Positions

    public static final double lposition0 = -3;
    public static final double lposition1 = 0;
    public static final double lposition2 = 150;
    public static final double lposition3 = 360;

    // Right Climber Positions

    public static final double rposition0 = -3;
    public static final double rposition1 = 0;
    public static final double rposition2 = 235;
    public static final double rposition3 = 395;

  }

  public static class LauncherConstants {

    // Launcher Motor Controller IDs
    public static final int launcher_id = 12;

    // Launcher PID Values
    public static final double launcher_kp  = 0.5;
    public static final double launcher_ki  = 0.0;
    public static final double launcher_kd  = 0.0;
    public static final double launcher_kFF = 0.0;

    // Setpoint Constants
    public static final double Amp      = 1800;
    public static final double Speaker  = 6000;
    public static final double Off      = 0;

    // Max RPM for the Motors
    public static final double max_RPM      = 5676; //5676

    // Set the Gear Ratio
    public static final double gear_ratio   = 3;
  }

  public static class SlidesConstants {

    // Slides Constants
    public static final int slides_id = 13;

    // Slides PID Values
    public static final double slides_kp = 1.0; // 0.15
    public static final double slides_ki = 0.0;
    public static final double slides_kd = 0.0;

    // Slide Positions
    public static final double position1 = -159;
    public static final double position2 = 0; // -140
    public static final double position3 = 74;
    public static final double tolerance = 0.5;

  }

  public static class AutoConstants {

    // Not being used currently, this is alternative code

    // Auto
    public static final Translation2d flModuleOffset = new Translation2d(0.7366, 0.7366);
    public static final Translation2d frModuleOffset = new Translation2d(0.7366, -0.7366);
    public static final Translation2d blModuleOffset = new Translation2d(-0.7366, 0.7366);
    public static final Translation2d brModuleOffset = new Translation2d(-0.7366, -0.7366);

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = DriveConstants.front_left_drive_id;
      public static final int angleMotorID = DriveConstants.front_left_steer_id;
      public static final int canCoderID = DriveConstants.front_left_CANcoder_id;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(angleMotorID, driveMotorID, canCoderID);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = DriveConstants.front_right_drive_id;
      public static final int angleMotorID = DriveConstants.front_right_steer_id;
      public static final int canCoderID = DriveConstants.front_right_CANcoder_id;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(angleMotorID, driveMotorID, canCoderID);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = DriveConstants.rear_left_drive_id;
      public static final int angleMotorID = DriveConstants.rear_left_steer_id;
      public static final int canCoderID = DriveConstants.rear_left_CANcoder_id;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(angleMotorID, driveMotorID, canCoderID);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = DriveConstants.rear_right_drive_id;
      public static final int angleMotorID = DriveConstants.rear_right_steer_id;
      public static final int canCoderID = DriveConstants.rear_right_CANcoder_id;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(angleMotorID, driveMotorID, canCoderID);
    }

    // Max Module Speed
    public static final double maxModuleSpeed = 4.5;

    // Pathplanner Config
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(10.0, 15.0, 20.0), // Translation constants 
      new PIDConstants(10.0, 20.0, 25.0), // Rotation constants 
      maxModuleSpeed, 
      1.04171, // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );

  }

}
