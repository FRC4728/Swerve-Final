// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  // private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // public static RobotPoseEstimator robotPoseEstimator;

  // Location of all april tags on field, check if id is correct

  final AprilTag tag01 = new AprilTag(
      1,
      new Pose3d(
          Units.inchesToMeters(610.77),
          Units.inchesToMeters(42.19),
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI)));

  final AprilTag tag02 = new AprilTag(
      2,
      new Pose3d(
          Units.inchesToMeters(610.77),
          Units.inchesToMeters(108.19),
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI)));
  final AprilTag tag03 = new AprilTag(
      3,
      new Pose3d(
          Units.inchesToMeters(610.77),
          Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI)));
  final AprilTag tag04 = new AprilTag(
      4,
      new Pose3d(
          Units.inchesToMeters(636.96),
          Units.inchesToMeters(265.74),
          Units.inchesToMeters(27.38),
          new Rotation3d(0.0, 0.0, Math.PI)));

  final AprilTag tag05 = new AprilTag(
      5,
      new Pose3d(
          Units.inchesToMeters(14.25),
          Units.inchesToMeters(265.74),
          Units.inchesToMeters(27.38),
          new Rotation3d(0.0, 0.0, Math.PI)));
  final AprilTag tag06 = new AprilTag(
      6,
      new Pose3d(
          Units.inchesToMeters(40.45),
          Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI)));
  final AprilTag tag07 = new AprilTag(
      7,
      new Pose3d(
          Units.inchesToMeters(40.45),
          Units.inchesToMeters(108.19),
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI)));
  final AprilTag tag08 = new AprilTag(
      8,
      new Pose3d(
          Units.inchesToMeters(40.45),
          Units.inchesToMeters(42.19),
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI)));

  Pose3d[] apriltaglocations = new Pose3d[] {
      new Pose3d(tag01.pose.getX(), tag01.pose.getY(), tag01.pose.getZ(), tag01.pose.getRotation()),
      new Pose3d(tag02.pose.getX(), tag02.pose.getY(), tag02.pose.getZ(), tag02.pose.getRotation()),
      new Pose3d(tag03.pose.getX(), tag03.pose.getY(), tag03.pose.getZ(), tag03.pose.getRotation()),
      new Pose3d(tag04.pose.getX(), tag04.pose.getY(), tag04.pose.getZ(), tag04.pose.getRotation()),
      new Pose3d(tag05.pose.getX(), tag05.pose.getY(), tag05.pose.getZ(), tag05.pose.getRotation()),
      new Pose3d(tag06.pose.getX(), tag06.pose.getY(), tag06.pose.getZ(), tag06.pose.getRotation()),
      new Pose3d(tag07.pose.getX(), tag07.pose.getY(), tag07.pose.getZ(), tag07.pose.getRotation()),
      new Pose3d(tag08.pose.getX(), tag08.pose.getY(), tag08.pose.getZ(), tag08.pose.getRotation()) };

      AprilTagFieldLayout fieldLayout;

  // cameraName is actual name of camera, not nickname
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  // where cam is on robot fix
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, Units.inchesToMeters(3)),
      new Rotation3d(0, 0, 0));

  PhotonPoseEstimator poseEstimator;

  public PhotonVisionSubsystem() {

    // worthless for now, but sets up an entire field for pose estimation
    ArrayList<AprilTag> aprilTags = new ArrayList<AprilTag>();
    aprilTags.add(tag01);
    aprilTags.add(tag02);
    aprilTags.add(tag03);
    aprilTags.add(tag04);
    aprilTags.add(tag05);
    aprilTags.add(tag06);
    aprilTags.add(tag07);
    aprilTags.add(tag08);

     fieldLayout = new AprilTagFieldLayout(aprilTags, Units.inchesToMeters(651.25),
        Units.inchesToMeters(315.5));

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(camera, robotToCam));

    poseEstimator = new PhotonPoseEstimator(fieldLayout,
        PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);

  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> VisionPoseEstimator = poseEstimator.update();
    SmartDashboard.putString("PoseEsimator location on field", VisionPoseEstimator.toString());
    // This method will be called once per scheduler run
  }
/* 
  // does everything right now, returns nothing useful
  public Translation2d CameraGet() {
    // take picture
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // get best target may not be optimal, check for better options
      PhotonTrackedTarget target = result.getBestTarget();
      int targetID = target.getFiducialId();
      int i = targetID;
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      double skew = target.getSkew();

      SmartDashboard.putNumber("Photon Yaw", yaw);
      SmartDashboard.putNumber("Photon Pitch", pitch);
      SmartDashboard.putNumber("Photon Area", area);
      SmartDashboard.putNumber("Photon Skew", skew);
      SmartDashboard.putNumber("Photon ID", targetID);

      // double apriltagX;
      // double apriltagY;

      var currenTag = new AprilTag[targetID];
      for (int index = 0; index < targetID; index++) {
        var apriltagX = apriltaglocations[index].getX();
        var apriltagY = apriltaglocations[index].getY();
        var apriltagZ = apriltaglocations[index].getZ();
        var apriltagRotate = apriltaglocations[index].getRotation();

        // currenTag[targetID] = new AprilTag(targetID, new Pose3d(apriltagX, apriltagY,
        // apriltagZ, apriltagRotate));

        double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
            Units.inchesToMeters(42.5),
            Units.inchesToMeters(18.22),
            Units.degreesToRadians(-33),
            Units.degreesToRadians(target.getPitch()));

        // PhotonUtils.estimateFieldToRobot(null, null, null);
        // finds translation to target for trajectory generation
        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
            distanceMeters, Rotation2d.fromDegrees(-target.getYaw()));

        SmartDashboard.putString("Translation to target",
            new Translation2d(Units.metersToInches(translation.getX()), Units.metersToInches(translation.getY()))
                .toString());
        SmartDashboard.putNumber("Distance To Target", distanceMeters);

        // Transform2d cameratotarget = PhotonUtils.estimateCameraToTarget(translation,
        // new Pose2d(apriltagX, apriltagY, new Rotation2d(0))
        // , new Rotation2d(drive1.getTheta()));

        // SmartDashboard.putString("cameratotarget", cameratotarget.toString());
        if (apriltagX < 7.62) {
          SmartDashboard.putString("New RobotPose",
              new Translation2d(Units.metersToInches(apriltagX + translation.getX()),
                  Units.metersToInches(apriltagY + translation.getY())).toString());
          return new Translation2d(apriltagX + translation.getX(), apriltagY + translation.getY());
        } else if (apriltagX > 7.62) {
          SmartDashboard.putString("New RobotPose",
              (new Translation2d(Units.metersToInches(apriltagX - translation.getX()),
                  Units.metersToInches(apriltagY - translation.getY())).toString()));
          return new Translation2d(apriltagX - translation.getX(), apriltagY - translation.getY());
        }
      }
    }
    return new Translation2d(0, 0);
  }

*/

  // does everything right now, returns nothing useful
  public EstimatedRobotPose CameraGet() {
    // take picture
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // get best target may not be optimal, check for better options
      PhotonTrackedTarget target = result.getBestTarget();
      int targetID = target.getFiducialId();
      int i = targetID;
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      double skew = target.getSkew();

      SmartDashboard.putNumber("Photon Yaw", yaw);
      SmartDashboard.putNumber("Photon Pitch", pitch);
      SmartDashboard.putNumber("Photon Area", area);
      SmartDashboard.putNumber("Photon Skew", skew);
      SmartDashboard.putNumber("Photon ID", targetID);

      // double apriltagX;
      // double apriltagY;

        // currenTag[targetID] = new AprilTag(targetID, new Pose3d(apriltagX, apriltagY,
        // apriltagZ, apriltagRotate));
      
      EstimatedRobotPose VisionRobotPose = lowestAmbiguityStrategy(result);
      SmartDashboard.putString("Robot Vision on button", VisionRobotPose.toString());
      }
    
    return new EstimatedRobotPose(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), 0);
  }

  private EstimatedRobotPose lowestAmbiguityStrategy(PhotonPipelineResult result) {
    PhotonTrackedTarget lowestAmbiguityTarget = null;

    double lowestAmbiguityScore = 10;

    for (PhotonTrackedTarget target : result.targets) {
        double targetPoseAmbiguity = target.getPoseAmbiguity();
        // Make sure the target is a Fiducial target.
        if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
            lowestAmbiguityScore = targetPoseAmbiguity;
            lowestAmbiguityTarget = target;
        }
    }

    // Although there are confirmed to be targets, none of them may be fiducial
    // targets.

    int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

    Optional<Pose3d> targetPosition = fieldLayout.getTagPose(targetFiducialId);

   

    return 
            new EstimatedRobotPose(
                    targetPosition
                            .get()
                            .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                            .transformBy(robotToCam.inverse()),
                    result.getTimestampSeconds());
}



  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    // SmartDashboard.putString("vision pose",
    // poseEstimator.update().get().estimatedPose.getTranslation().toString());
    return poseEstimator.update();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
