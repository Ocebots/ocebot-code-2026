package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionConfig {
  public static PhotonPipelineResult result = new PhotonPipelineResult();
  public static final String CAMERA_NAME = "forwardAprilTag"; // left
  public static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  public static final PhotonPoseEstimator.PoseStrategy STRATEGY =
      PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
  public static final Transform3d FORWARD_CAMERA_POSITION =
      new Transform3d(
          Units.inchesToMeters(-0.79),
          Units.inchesToMeters(-11.55),
          Units.inchesToMeters(14.88),
          new Rotation3d(0.0, Units.degreesToRadians(16), 0.0));
  Optional<EstimatedRobotPose> visionEst = Optional.empty();
  public static PhotonPoseEstimator photonPoseEstimatorForward =
      new PhotonPoseEstimator(LAYOUT, FORWARD_CAMERA_POSITION);
}
