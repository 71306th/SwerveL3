// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.OneSec;
import frc.robot.commands.SwerveAim;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  private final XboxController controller = new XboxController(RobotConstants.DriverControllerID);

  private static Swerve s_Swerve = new Swerve();

  private final SendableChooser<Command> autoChooser;

  private final PoseEstimatorSubsystem poseEstimator =
      new PoseEstimatorSubsystem(s_Swerve::getYaw, s_Swerve::getModulePositions);

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(s_Swerve, controller);

  private final SwerveAim aimSwerve = new SwerveAim(s_Swerve, poseEstimator, controller);

  // private ShuffleboardTab SwerveTab = Shuffleboard.getTab("SwerveTab");

  private Field2d field_PP = new Field2d();

  public RobotContainer() {
    s_Swerve.setDefaultCommand(teleopSwerve);
    // s_Swerve.setDefaultCommand(aimSwerve);


    //////////////////////////////////* REMEMBER TO CHANGE EVERY TIME *//////////////////////////////////
    s_Swerve.setPose(FieldConstants.BLUE_MB);
    //////////////////////////////////* REMEMBER TO CHANGE EVERY TIME *//////////////////////////////////

    
    // Register named commands
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));

    // Use event markers as triggers
    new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("u need to choose", null);
    autoChooser.addOption("X1", new PathPlannerAuto("X1"));
    autoChooser.addOption("X2", new PathPlannerAuto("X2"));
    SmartDashboard.putData("autoChooser", autoChooser);

    /**** Vision tab ****/
    final var visionTab = Shuffleboard.getTab("Vision");

    // Pose estimation
    poseEstimator.addDashboardWidgets(visionTab);

    // Field base on PathPlanner
    SmartDashboard.putData("Field_PP", field_PP);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        field_PP.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        field_PP.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        // Do whatever you want with the poses here
        field_PP.getObject("path").setPoses(poses);
    });
  }

  private void configureBindings() {
    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    SmartDashboard.putData("X1", new PathPlannerAuto("X1"));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Go To X1", AutoBuilder.pathfindToPose(
      FieldConstants.X1, 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ).alongWith(new OneSec()));
    SmartDashboard.putData("Go To X2", AutoBuilder.pathfindToPose(
      FieldConstants.X2, 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ).alongWith(new OneSec()));
    SmartDashboard.putData("Go To X3", AutoBuilder.pathfindToPose(
      FieldConstants.X3, 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ).andThen(new OneSec()));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 1m in the +X Field_PP direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = s_Swerve.getOdometryPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(1.0, 0.0)), new Rotation2d());

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        waypoints, 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }).alongWith(new OneSec()));
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
