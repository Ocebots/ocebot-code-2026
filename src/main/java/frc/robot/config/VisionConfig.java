package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionConfig {
  public static PhotonPipelineResult result = new PhotonPipelineResult();
  public static final String FRONT_CAMERA_NAME = "forwardAprilTag"; 
  // Per-camera measurement standard deviations: [x (m), y (m), theta (rad)]
  public static final Matrix<N3, N1> FRONT_VISION_STDDEVS =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(3.0));
  public static final Matrix<N3, N1> REAR_VISION_STDDEVS =
      VecBuilder.fill(0.10, 0.10, Units.degreesToRadians(5.0));
  public static final String REAR_CAMERA_NAME = "rearAprilTag";
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
public static final Transform3d REAR_CAMERA_POSITION =
      new Transform3d(
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(0.0),
          new Rotation3d(0.0, Units.degreesToRadians(0.0), 0.0));
}
