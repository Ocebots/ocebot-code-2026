package frc.robot.loggers;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;

@CustomLoggerFor(SwerveDrivetrain.class)
public class SwerveDrivetrainLogger extends ClassSpecificLogger<SwerveDrivetrain> {
  public SwerveDrivetrainLogger() {
    super(SwerveDrivetrain.class);
  }

  String[] moduleNames = {"FL", "FR", "BL", "BR"};

  @Override
  protected void update(EpilogueBackend backend, SwerveDrivetrain drivetrain) {
    backend.log("Pose", drivetrain.getState().Pose, Pose2d.struct);
    backend.log("Speeds", drivetrain.getState().Speeds, ChassisSpeeds.struct);
    backend.log("ModuleStates", drivetrain.getState().ModuleStates, SwerveModuleState.struct);
    backend.log("ModuleTargets", drivetrain.getState().ModuleTargets, SwerveModuleState.struct);
    backend.log(
        "ModulePositions", drivetrain.getState().ModulePositions, SwerveModulePosition.struct);
    backend.log("RawHeading", drivetrain.getState().RawHeading, Rotation2d.struct);
    backend.log("Timestamp", drivetrain.getState().Timestamp);
    backend.log("OdometryPeriod", drivetrain.getState().OdometryPeriod);
    backend.log("SuccessfulDaqs", drivetrain.getState().SuccessfulDaqs);
    backend.log("FailedDaqs", drivetrain.getState().FailedDaqs);
    backend.log("Pigeon Yaw", drivetrain.getPigeon2().getYaw().getValue());
    backend.log(
        "Pigeon Angular Velocity Z World",
        drivetrain.getPigeon2().getAngularVelocityZWorld().getValue());
    backend.log("Pigeon Supply Voltage", drivetrain.getPigeon2().getSupplyVoltage().getValue());
    backend.log(
        "Operator Forward Direction", drivetrain.getOperatorForwardDirection(), Rotation2d.struct);
    backend.log("Battery Voltage", RobotController.getBatteryVoltage());
    var modules = drivetrain.getModules();

    for (int i = 0; i < modules.length; i++) {
      var module = modules[i];
      backend.log(
          moduleNames[i] + " Drive Supply Current",
          module.getDriveMotor().getSupplyCurrent().getValue());
      backend.log(
          moduleNames[i] + " Steer Supply Current",
          module.getSteerMotor().getSupplyCurrent().getValue());
      backend.log(
          moduleNames[i] + " Drive Stator Current",
          module.getDriveMotor().getStatorCurrent().getValue());
      backend.log(
          moduleNames[i] + " Steer Stator Current",
          module.getSteerMotor().getStatorCurrent().getValue());
      backend.log(
          moduleNames[i] + " Drive Supply Voltage",
          module.getDriveMotor().getSupplyVoltage().getValue());
      backend.log(
          moduleNames[i] + " Steer Supply Voltage",
          module.getSteerMotor().getSupplyVoltage().getValue());
      backend.log(
          moduleNames[i] + " Drive Closed Loop Error",
          module.getDriveMotor().getClosedLoopError().getValue());
      backend.log(
          moduleNames[i] + " Steer Closed Loop Error",
          module.getSteerMotor().getClosedLoopError().getValue());
    }

    double totalDriveSupply = 0;
    double totalSteerSupply = 0;

    for (var module : modules) {
      totalDriveSupply += module.getDriveMotor().getSupplyCurrent().getValueAsDouble();
      totalSteerSupply += module.getSteerMotor().getSupplyCurrent().getValueAsDouble();
    }
    backend.log("Total Supply Current", totalDriveSupply + totalSteerSupply);
  }
}
