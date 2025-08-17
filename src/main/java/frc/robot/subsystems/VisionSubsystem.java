package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {

  PhotonCamera cam = new PhotonCamera("Good Camera");
  SwerveSubsystem swerve;
  PhotonPoseEstimator photonPoseEstimator;

  public VisionSubsystem(PhotonCamera camera, SwerveSubsystem swerve) {

    //Forward Camera
    this.cam = camera;

    this.swerve = swerve;



    // Construct PhotonPoseEstimator
    this.photonPoseEstimator = new PhotonPoseEstimator(Constants.AprilTags.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.AprilTags.CameraConstants.kRobotToCam);
    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = Constants.AprilTags.kSingleTagStdDevs;
    var targets = this.cam.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
        var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
                tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = Constants.AprilTags.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    PhotonPipelineResult pipelineResult = this.cam.getLatestResult();
    boolean hasTargets = pipelineResult.hasTargets();

    return photonPoseEstimator.update(pipelineResult);
  }

  public void updateGlobalPosition() {
    Pose2d previousPose2d = swerve.getPose();

    //System.out.println("Old Pose: " + previousPose2d.toString());

    Optional<EstimatedRobotPose> estimatedPosition = getEstimatedGlobalPose(previousPose2d);
    //System.out.println(estimatedPosition);
    //System.out.println("Got optional variable for position");
    estimatedPosition.ifPresent(est -> {
      //System.out.println("Position Estimated!");
      final Pose2d estPose = est.estimatedPose.toPose2d();
      final double estTime = est.timestampSeconds;

      //System.out.println("Estimated Pose: " + estPose.toString());

      // Change our trust in the measurement based on the tags we can see
      Matrix<N3, N1> estStdDevs = getEstimationStdDevs(estPose);

      final double distance = 2;  //get this from the apriltag somehow
      estStdDevs = VecBuilder.fill(distance / 2, distance / 2, 100);

      swerve.addVisionReading(estPose, estTime, estStdDevs);
      
    });
  }
}