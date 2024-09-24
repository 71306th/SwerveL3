// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.SwerveAim;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  private final XboxController controller = new XboxController(RobotConstants.DriverControllerID);

  private static Swerve s_Swerve = new Swerve();

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  private final PoseEstimatorSubsystem poseEstimator =
      new PoseEstimatorSubsystem(s_Swerve::getYaw, s_Swerve::getModulePositions);

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(s_Swerve, controller);

  private final SwerveAim aimSwerve = new SwerveAim(s_Swerve, poseEstimator, controller);

  private ShuffleboardTab SwerveTab = Shuffleboard.getTab("SwerveTab");

  public RobotContainer() {
    // s_Swerve.setDefaultCommand(teleopSwerve);
    s_Swerve.setDefaultCommand(aimSwerve);
    s_Swerve.setPose(FieldConstants.RED_MB);
    // fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
    //     drivetrainSubsystem,
    //     () -> poseEstimator.getCurrentPose().getRotation(),
    //     controlBindings.translationX(),
    //     controlBindings.translationY(),
    //     controlBindings.omega());

    // fieldHeadingDriveCommand = new FieldHeadingDriveCommand(
    //     drivetrainSubsystem,
    //     () -> poseEstimator.getCurrentPose().getRotation(),
    //     controlBindings.translationX(),
    //     controlBindings.translationY(),
    //     controlBindings.heading());
    
    // s_Upper.setDefaultCommand(new RunCommand(()->{
      
    //   if (controller.getYButton()) s_Upper.setElbow(0.2);
    //   else if (controller.getAButton()) s_Upper.setElbow(-0.2);
    //   else s_Upper.setElbow(0);

    //   if (controller.getRightBumper()) s_Upper.setIntake(-0.5);
    //   else s_Upper.setIntake(0);

    //   if (controller.getRightTriggerAxis() > 0.05) {
    //     s_Upper.setRightShooter(-0.8);
    //     s_Upper.setIntake(-1);
    //   }
    // }, s_Upper));
    autoChooser.setDefaultOption("u need to choose", null);
    autoChooser.addOption("X1", new PathPlannerAuto("X1"));
    SmartDashboard.putData("autoChooser", autoChooser);
    /**** Vision tab ****/
    final var visionTab = Shuffleboard.getTab("Vision");

    // Pose estimation
    poseEstimator.addDashboardWidgets(visionTab);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // delete the photonvision images
  public void deleteImages() {
    Path imagesPath = Paths.get(
      " /opt/photonvision/photonvision_config/imgSaves" +
      "");

    try {
      Files.delete(imagesPath);
      System.out.println("File "
              + imagesPath.toAbsolutePath().toString()
              + " successfully removed");
    } catch (IOException e) {
      System.err.println("Unable to delete "
              + imagesPath.toAbsolutePath().toString()
              + " due to...");
      e.printStackTrace();
    }
  }
}
