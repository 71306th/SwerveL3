package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.math.simplePID;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private XboxController controller;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private double translationVal;
  private double strafeVal;
  private double rotationVal;

  private simplePID zPID = new simplePID(2, 0);

  public TeleopSwerve(Swerve s_Swerve, XboxController controller) {
    this.s_Swerve = s_Swerve;
    this.controller = controller;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {

    if (controller.getBackButton()) {
      s_Swerve.setYaw(Rotation2d.fromDegrees(0));
      s_Swerve.setPose(new Pose2d());
    }

    /* Get Values, Deadband */
    translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(-controller.getLeftY(), Constants.SwerveConstants.axisDeadBand));
    strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(-controller.getLeftX(), Constants.SwerveConstants.axisDeadBand));
    rotationVal = 
      rotationLimiter
        .calculate(MathUtil.applyDeadband(-controller.getRightX() * 0.3, Constants.SwerveConstants.axisDeadBand));

    // if (controller.getLeftBumper()) {

    //   double zTarget;
    //   if (controllerStation.getAlliance().isPresent() ) {
    //     if (controllerStation.getAlliance().get() == Alliance.Blue) {
    //       zTarget = FieldConstants.blueSpeakerCoord.minus(s_Swerve.getOdometryPose().getTranslation()).getAngle().getRotations();
    //     } else {
    //       zTarget = FieldConstants.redSpeakerCoord.minus(s_Swerve.getOdometryPose().getTranslation()).getAngle().getRotations();
    //     }
    //   } else {
    //     zTarget = s_Swerve.getYaw().getRotations()-4p;
    //   }

      // double z = s_Swerve.getYaw().getRotations();
      // z -= Math.floor(z);
      // if (z > 0.5) z -= 1;
      // s_Swerve.drive(
      //   new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxModuleSpeed),
      //   zPID.calculate(z, zTarget) * SwerveConstants.maxAngularVelocity,
      //   true,
      //   false
      // );

    // } else {
      s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxModuleSpeed),
        rotationVal * Constants.SwerveConstants.maxAngularVelocity,
         true,
        false
      );
    // }
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}