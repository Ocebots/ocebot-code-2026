package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.VisionConfig;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {
  public static final PhotonCamera FrontCameraApril =
      new PhotonCamera(VisionConfig.FRONT_CAMERA_NAME);
}
