package frc.robot.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.controller.ProfiledPIDController;

@CustomLoggerFor(ProfiledPIDController.class)
public class ProfiledPIDControllerLogger extends ClassSpecificLogger<ProfiledPIDController> {
  public ProfiledPIDControllerLogger() {
    super(ProfiledPIDController.class);
  }

  @Override
  protected void update(EpilogueBackend backend, ProfiledPIDController controller) {
    backend.log("P", controller.getP());
    backend.log("I", controller.getI());
    backend.log("D", controller.getD());
    backend.log("At Goal Position", controller.atSetpoint());
    backend.log("Positional Error", controller.getPositionError());
    backend.log("Velocity Error", controller.getVelocityError());
    backend.log("Velocity Tolerance", controller.getVelocityTolerance());
    backend.log("Positional Tolerance", controller.getVelocityError());
  }
}
