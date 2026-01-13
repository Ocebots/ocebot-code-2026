package frc.robot.loggers;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@CustomLoggerFor(SwerveDriveState.class)
public class SwerveDriveStateLogger extends ClassSpecificLogger<SwerveDriveState> {
  public SwerveDriveStateLogger() {
    super(SwerveDriveState.class);
  }

  @Override
  protected void update(EpilogueBackend backend, SwerveDriveState state) {
    backend.log("Pose", state.Pose, Pose2d.struct);
    backend.log("Speeds", state.Speeds, ChassisSpeeds.struct);
    backend.log("ModuleStates", state.ModuleStates, SwerveModuleState.struct);
    backend.log("ModuleTargets", state.ModuleTargets, SwerveModuleState.struct);
    backend.log("ModulePositions", state.ModulePositions, SwerveModulePosition.struct);
    backend.log("RawHeading", state.RawHeading, Rotation2d.struct);
    backend.log("Timestamp", state.Timestamp);
    backend.log("OdometryPeriod", state.OdometryPeriod);
    backend.log("SuccessfulDaqs", state.SuccessfulDaqs);
    backend.log("FailedDaqs", state.FailedDaqs);
  }
}
