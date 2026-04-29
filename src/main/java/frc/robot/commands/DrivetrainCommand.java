package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.TunerConstants;
import frc.robot.helpers.ApplyModuleStates;
import frc.robot.helpers.ShotCalculator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

@Logged
public class DrivetrainCommand extends Command {
  public static enum Position {
    TELEOP,
    STILL_SHOT,
    SOTM,
    AUTO_ALIGN_HUB
  }

  private CommandSwerveDrivetrain subsystem;
  private DrivetrainCommand.Position pose;
  private DoubleSupplier leftY;
  private DoubleSupplier leftX;
  private DoubleSupplier rightX;
  public static String drivetrainState = "Stopped";

  private final double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.25)
          .withRotationalDeadband(MaxAngularRate * 0.25) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType
                  .OpenLoopVoltage); // Use open-loop control for drive motors

  SwerveModuleState[] states = {
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(225))
  };
  private final ApplyModuleStates applyRequest = new ApplyModuleStates();
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private final ProfiledPIDController autoAlignPidController;
  private Rotation2d m_targetAngle = Rotation2d.kZero;
  private static final double FUEL_EXIT_VELOCITY = 10.0; // This needs to be tuned

  public DrivetrainCommand(
      CommandSwerveDrivetrain subsystem,
      DrivetrainCommand.Position pose,
      DoubleSupplier leftX,
      DoubleSupplier leftY,
      DoubleSupplier rightX) {
    this.pose = pose;
    this.subsystem = subsystem;
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
    double alignP = 60;
    double alignI = 0;
    double alignD = 10;
    this.autoAlignPidController =
        new ProfiledPIDController(
            alignP,
            alignI,
            alignD,
            new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularRate / 0.2));
    this.autoAlignPidController.enableContinuousInput(
        -Math.PI, Math.PI); // Swerve angles are continuous (-180 to 180 deg)
    // Don't compute hub-facing target in the constructor (DriverStation alliance may be
    // unavailable during initialization). Initialize to current heading; execute() will
    // recompute the actual hub-facing target each loop.
    this.m_targetAngle = subsystem.getState().Pose.getRotation();

    addRequirements(subsystem);
  }

  public double getAutoAlignRotationalOutput() {
    Rotation2d currentAngle = subsystem.getState().Pose.getRotation();
    double current = currentAngle.getRadians();
    double target = m_targetAngle.getRadians();
    double output = autoAlignPidController.calculate(current, target);
    if (output > MaxAngularRate) {
      output = MaxAngularRate;
    } else if (output < -MaxAngularRate) {
      output = -MaxAngularRate;
    }
    return output;
  }

  @Override
  public void execute() {
    switch (pose) {

      // Default driving mode
      case TELEOP:
        subsystem.setControl(
            drive
                .withVelocityX(
                    -leftY.getAsDouble() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-leftX.getAsDouble() * MaxSpeed) // Drive left with negative X
                .withRotationalRate(
                    -rightX.getAsDouble()
                        * MaxAngularRate)); // Drive counterclockwise with negative X
        drivetrainState = "Teleop Drive";
        break;

      // (Needs check) Mode for when shots are from a still position
      case STILL_SHOT:
        // make X with wheels, set wheels to brake mode
        applyRequest.ModuleStates = states;
        subsystem.setControl(applyRequest);
        drivetrainState = "Still Shot";
        break;

      // (Incomplete) Mode for moving shots
      case SOTM:
        drivetrainState = "Shoot on the Move Drive";
        double vx = -leftY.getAsDouble() * MaxSpeed;
        double vy = -leftX.getAsDouble() * MaxSpeed;
        Translation2d robotPosition = subsystem.getState().Pose.getTranslation();
        ChassisSpeeds speeds = subsystem.getState().Speeds;
        Translation2d robotVelocity =
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Translation2d goal = ShotCalculator.calculateHubPosition();
        double distance = robotPosition.getDistance(goal);
        double flightTime = distance / FUEL_EXIT_VELOCITY;
        Translation2d toGoal = goal.minus(robotPosition);
        Translation2d toGoalDir =
            toGoal.getNorm() > 1e-6 ? toGoal.div(toGoal.getNorm()) : new Translation2d();
        Translation2d lateralVelocity =
            robotVelocity.minus(toGoalDir.times(robotVelocity.dot(toGoalDir)));
        Translation2d virtualGoal = goal.minus(lateralVelocity.times(flightTime));
        Rotation2d targetAngle = virtualGoal.minus(robotPosition).getAngle();
        double currentHeading = subsystem.getState().Pose.getRotation().getRadians();
        double targetHeading = targetAngle.getRadians();
        double error = targetHeading - currentHeading;
        error = Math.atan2(Math.sin(error), Math.cos(error));
        double kP = 50; // tune
        double omega = error * kP;
        omega = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, omega));
        subsystem.setControl(drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));

        break;

      case AUTO_ALIGN_HUB:
        // Recompute the desired heading toward the hub each loop
        m_targetAngle =
            ShotCalculator.getRotationTowardsHub(
                ShotCalculator.calculateHubPosition(), subsystem.getState().Pose.getTranslation());
        double currentRad = subsystem.getState().Pose.getRotation().getRadians();
        double targetRad = m_targetAngle.getRadians();
        // Normalize error1 to [-pi, pi]
        double error1 =
            Math.atan2(Math.sin(targetRad - currentRad), Math.cos(targetRad - currentRad));
        double absError = Math.abs(error1);
        double angleTolerance = Math.toRadians(0.5); // stop within 1 degree

        double rotOutput = getAutoAlignRotationalOutput();

        if (absError < angleTolerance) {
          // Aligned: stop rotating (and hold position)
          subsystem.setControl(drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
        } else {
          // Not aligned: apply rotational output
          subsystem.setControl(
              drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(rotOutput));
        }
        drivetrainState = "Auto Align";
        break;

      default:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (pose != Position.AUTO_ALIGN_HUB) {
      return false;
    }

    Translation2d hubPos = ShotCalculator.calculateHubPosition();

    Translation2d currentTrans = subsystem.getState().Pose.getTranslation();
    if (currentTrans == null) {
      return false;
    }

    Rotation2d target = ShotCalculator.getRotationTowardsHub(hubPos, currentTrans);
    double current = subsystem.getState().Pose.getRotation().getRadians();
    double targetRad = target.getRadians();
    double err = Math.atan2(Math.sin(targetRad - current), Math.cos(targetRad - current));
    double angleTolerance = Math.toRadians(1.0); // 1 degree
    return Math.abs(err) < angleTolerance;
  }
}
