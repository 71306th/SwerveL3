// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class Swerve extends SubsystemBase {
      
  RobotConfig config;

  private static Swerve mInstance;

  private final Pigeon2 gyro = new Pigeon2(SwerveConstants.pigeon1, RobotConstants.canbusName);

  public SwerveModule[] mSwerveMods = new SwerveModule[] {
    new SwerveModule(0, SwerveConstants.Mod0.constants),
    new SwerveModule(1, SwerveConstants.Mod1.constants),
    new SwerveModule(2, SwerveConstants.Mod2.constants),
    new SwerveModule(3, SwerveConstants.Mod3.constants)
  };

  private SwerveDriveKinematics kinematics = SwerveConstants.kinematics;

  private SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(kinematics, getYaw(), getPositions());

  private PoseEstimatorSubsystem poseEstimater;

  private Field2d m_field = new Field2d();

  private double AngularVel;

  private Pose2d[] mModulePoses = new Pose2d[4];

  private Pose2d mRobotPose = new Pose2d();

  ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

  private final SwerveSetpointGenerator setpointGenerator;

  private SwerveSetpoint previousSetpoint;

  public Swerve() {

    gyro.setYaw(0);

        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }

      AutoBuilder.configure(
        this::getPoseEstimated, // Robot pose supplier (getOdometryPose or getPoseEstimated)
        this::resetOdometryPose,  // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController(
            Constants.SwerveConstants.translationConstants,
            Constants.SwerveConstants.rotationConstants
          ), 
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
      );

    setpointGenerator = new SwerveSetpointGenerator(
                config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
                Units.rotationsToRadians(10.0) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
            );

        // Initialize the previous setpoint to the robot's current speeds & module states
        ChassisSpeeds currentSpeeds = getRobotRelativeSpeeds(); // Method to get current robot-relative chassis speeds
        SwerveModuleState[] currentStates = getStates(); // Method to get the current swerve module states
        previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, null);

  SmartDashboard.putData("Field_Swerve", m_field);
}

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxModuleSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxModuleSpeed);
    for (SwerveModule mod : mSwerveMods) { 
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getOdometryPose() {
    return swerveOdometry.getPoseMeters();
  }

  public Pose2d getPoseEstimated() {
    return poseEstimater.getCurrentPose();
  }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public void resetOdometryPose(Pose2d pose) {
    poseEstimater.resetPosition(getGyroscopeRotation(), getPositions(), pose);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getOdometryPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    // SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    // setModuleStates(targetStates);

    /* Swerve Setpoint Grnerator */
    // Note: it is important to not discretize speeds before or after
    // using the setpoint generator, as it will discretize them for you
      previousSetpoint = setpointGenerator.generateSetpoint(
      previousSetpoint, // The previous setpoint
      robotRelativeSpeeds, // The desired target speeds
      0.02 // The loop time of the robot code, in seconds
      );
      setModuleStates(previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  // return Robot Relative Speeds
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        kinematics.toChassisSpeeds(getStates()),
        getYaw());

    return chassisSpeeds;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  // for pose estimater
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(mSwerveMods).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
  }

  // for pose estimater
  public Rotation2d getGyroscopeRotation() {
    return gyro.getRotation2d();
  }

  public Rotation2d getYaw(){
    return Rotation2d.fromDegrees(gyro.getYaw().getValue());
  }

  public void setYaw(Rotation2d pos) {
    gyro.setYaw(pos.getDegrees());
  }

  public static Swerve getInstance() {
    if (mInstance == null) {
        mInstance = new Swerve();
    }
    return mInstance;
  }

  public void stop() {
    driveFieldRelative(new ChassisSpeeds());
  }

  private void updateSwervePoses() {
    if(getOdometryPose() != null) mRobotPose = getOdometryPose();
    else mRobotPose = new Pose2d();
    for (int i = 0; i < mModulePoses.length; i++) {
        Translation2d updatedPosition = Constants.SwerveConstants.swerveModuleLocations[i]
                .rotateBy(mRobotPose.getRotation()).plus(mRobotPose.getTranslation());
        Rotation2d updatedRotation = getStates()[i].angle.plus(mRobotPose.getRotation());
        if(getStates()[i].speedMetersPerSecond < 0.0) {
            updatedRotation = updatedRotation.plus(Rotation2d.fromDegrees(180));;
        }
        mModulePoses[i] = new Pose2d(updatedPosition, updatedRotation);
    }
  }

  @Override
  public void periodic() {
    updateSwervePoses();
    m_field.setRobotPose(mRobotPose);
    // m_field.getObject("Swerve Modules Pose").setPoses(mModulePoses);
    
    swerveOdometry.update(getYaw(), getPositions());
    AngularVel = Math.max(gyro.getAngularVelocityZWorld().getValueAsDouble(), AngularVel);
    tab.add("Yaw", getGyroscopeRotation())
      .withSize(2, 4)
      .withPosition(0, 0);
    tab.add("AngularVel", AngularVel)
      .withSize(2, 4)
      .withPosition(2, 0);
    tab.add("OdometryX", getOdometryPose().getX())
      .withSize(2, 4)
      .withPosition(4, 0);
    tab.add("OdometryY", getOdometryPose().getY())
      .withSize(2, 4)
      .withPosition(6, 0);
    // tab.add("OdometryRotation", getOdometryPose().getRotation().getRotations());

    // publisher.set(new SwerveModuleState[] {
    //   LFState,
    //   RFState,
    //   LRState,
    //   RRState
    // });
    // SmartDashboard.putDate("States", getStates());
  }
}
