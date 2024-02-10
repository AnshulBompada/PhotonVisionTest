// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private Pose2d prevPose = new Pose2d();
  private Field2d field = new Field2d();

  public RobotContainer() {
    camera = new PhotonCamera("LLLeft");
    poseEstimator = new PhotonPoseEstimator(
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      camera, 
      new Transform3d());

    configureBindings();
  }

  public void update() {
    // poseEstimator.setReferencePose(prevPose);
    PhotonPipelineResult result = camera.getLatestResult();

    SmartDashboard.putNumber("Result Time", result.getTimestampSeconds() );
    Pose2d resultPose = new Pose2d();
    SmartDashboard.putBoolean("Is detected", result.getMultiTagResult().estimatedPose.isPresent);
    if(result.getMultiTagResult().estimatedPose.isPresent) {
      resultPose = toPose2d( result.getMultiTagResult().estimatedPose.best );
    }

    SmartDashboard.putBoolean("Has Target", result.hasTargets());
    // if(result.hasTargets()) {
    //   EstimatedRobotPose pose = poseEstimator.update().get();
    //   prevPose = pose.estimatedPose.toPose2d();
    // }

    if(result.hasTargets()) {
      field.getObject("AYYYY").setPose( toPose2d( result.getBestTarget().getBestCameraToTarget() ) );
      field.getObject("PhotonPoseEstimatorAnshul").setPose(prevPose);
    }
    // field.getObject("PhotonVision").setPose(resultPose);
    SmartDashboard.putData("FIELDS!", field);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private Pose2d toPose2d(Transform3d transform3d) {
    return new Pose2d(transform3d.getTranslation().toTranslation2d(), transform3d.getRotation().toRotation2d());
  }
}
