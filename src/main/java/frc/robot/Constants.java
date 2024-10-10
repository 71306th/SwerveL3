// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.FSLib.swerve.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class RobotConstants {
    public static final String canbusName = "GTX7130";
    public static final int DriverControllerID = 0;
  }

  public static final class FieldConstants {
    public static final Pose2d BLUE_LB = 
      new Pose2d(new Translation2d(0.7, 6.70), Rotation2d.fromDegrees(60));
    public static final Pose2d BLUE_MB =
      new Pose2d(new Translation2d(1.37, 5.50), Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_RB =
      new Pose2d(new Translation2d(0.7, 4.40), Rotation2d.fromDegrees(-60));
    public static final Pose2d RED_LB =
      new Pose2d(new Translation2d(15.85, 4.38), Rotation2d.fromDegrees(-120));
    public static final Pose2d RED_MB =
      new Pose2d(new Translation2d(15.17, 5.50), Rotation2d.fromDegrees(-180));
    public static final Pose2d RED_RB =
      new Pose2d(new Translation2d(15.85, 6.75), Rotation2d.fromDegrees(-120));

    public static final Pose2d X1 =
      new Pose2d(new Translation2d(2.56, 6.99), Rotation2d.fromDegrees(0));
    public static final Pose2d X2 =
      new Pose2d(new Translation2d(2.56, 5.55), Rotation2d.fromDegrees(0));
    public static final Pose2d X3 =
      new Pose2d(new Translation2d(2.56, 4.10), Rotation2d.fromDegrees(0));
  }

  // public static enum UpperState {
  //   DEFAULT,
  //   GROUND,
  //   AMP,
  //   BASE,
  //   FAR,
  //   LFIGHT,
  //   MGROUND,
  //   SHOOT,
  //   TRAP,
  //   NULL,
  //   PREENDGAME,
  //   ENDGAME
  // }

  // public static UpperState state;

  public static final class SwerveConstants {
    public static final double axisDeadBand = 0.05;
    public static final int pigeon1 = 1;
    public static final boolean invertGyro = false;

    /* Drivetrain Constants */
    public static final double trackWidth = 0.583; // meters, length between two side's wheels, need to adjust
    public static final double wheelBase = 0.583; // meters, length between same side's wheels, need to adjust
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    // public static final double openLoopRamp = 0.25;
    // public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12244897959 / 1.0); // 6.12:1 (6.12244897959), for MK4i(L3)
    public static final double angleGearRatio = (150.0 / 7.0 / 1.0); // 150/7 : 1, for MK4i(all)
    public static final double steerReduction = (14.0 / 50.0) * (10.0 / 60.0);

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 10;
    public static final int driveContinuousCurrentLimit = 40;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.008;
    public static final double angleKI = 0.05;
    public static final double angleKD = 0.005;

    /* Angle Motor Auto-Facing PID Values */
    public static final double faceKP = 0.8;
    public static final double faceKI = 0.0;
    public static final double faceKD = 0.1;
    public static final double faceiWindup = 0.0;
    public static final double faceiLimit = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.08; // 0.12
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0025; 
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxModuleSpeed = 4; // m/s
    public static final double maxModuleAccleration = 3; // m/s square
    public static final double maxAngularVelocity = 13.5; // rad/s
    public static final double maxAngularAccleration = 12; // rad/s square

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

    /* Field Oriented */ // nigaa
    public static boolean fieldOriented = true;

    /* Slow Mode */
    public static boolean slow = false;

    public static final Translation2d LFModuleOffset = new Translation2d(0.3, 0.3);
    public static final Translation2d RFModuleOffset = new Translation2d(0.3, -0.3);
    public static final Translation2d LRModuleOffset = new Translation2d(-0.3, 0.3);
    public static final Translation2d RRModuleOffset = new Translation2d(-0.3, -0.3);

    public static final edu.wpi.first.math.geometry.Translation2d[] swerveModuleLocations = {
            new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };
    
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
    public static final int driveMotorID = 1;
    public static final int angleMotorID = 2;
    public static final int canCoderID = 0;
    public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.590682);
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
}

/* Front Right Module - Module 1 */
public static final class Mod1 {
    public static final int driveMotorID = 11;
    public static final int angleMotorID = 12;
    public static final int canCoderID = 1;
    public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.37416);
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
}


/* Rear Left Module - Module 2 */
public static final class Mod2 {
    public static final int driveMotorID = 21;
    public static final int angleMotorID = 22;
    public static final int canCoderID = 2;
    public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.457124);
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
}

/* Rear Right Module - Module 3 */
public static final class Mod3 {
    public static final int driveMotorID = 31;
    public static final int angleMotorID = 32;
    public static final int canCoderID = 3;
    public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.059325);
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
}

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      SwerveConstants.LFModuleOffset, 
      SwerveConstants.RFModuleOffset, 
      SwerveConstants.LRModuleOffset, 
      SwerveConstants.RRModuleOffset
  );

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        steerReduction *
        wheelDiameter * Math.PI; // =5.47147871

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 
        (MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(trackWidth / 2.0, wheelBase / 2.0));

  public static final PIDConstants translationConstants = new PIDConstants(3, 0.0, 0.0); // need adjust
  public static final PIDConstants rotationConstants = new PIDConstants(0.5, 0.0, 0.0);
}
public static class VisionConstants {

    /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(//37" 35" 18.3"
        new Translation3d(inchesToMeters(0), inchesToMeters(18.3), inchesToMeters(7.4)),
        new Rotation3d(0.0, degreesToRadians(15.0), degreesToRadians(180)));

    public static final String LIMELIGHT_NAME = "limelight";
    
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
    public static final Pose2d FLIPPING_POSE = new Pose2d(
        new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
        new Rotation2d(Math.PI));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }

  public static class AutoConstants {
    public static final double PATH_THETA_kP = 2.6;
    public static final double PATH_THETA_kI = 0.001;
    public static final double PATH_THETA_kD = 0.0;

    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI, 2 / Math.PI);
    public static final double THETA_kP = 6.0;
    public static final double THETA_kI = 0.02;
    public static final double THETA_kD = 0.0;

    public static final double X_kP = 5.0;
    public static final double X_kI = 0.0;
    public static final double X_kD = 0.0;

    public static final double Y_kP = 5.0;
    public static final double Y_kI = 0.0;
    public static final double Y_kD = 0.0;
  }
}