package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
    double alignP = 1;
    double alignI = 1;
    double alignD = 1;
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
        break;

      // (Needs check) Mode for when shots are from a still position
      case STILL_SHOT:
        // make X with wheels, set wheels to brake mode
        applyRequest.ModuleStates = states;
        subsystem.setControl(applyRequest);
        System.out.println("Drivetrain: Still Shot Configuration");
        break;

      // (Incomplete) Mode for moving shots
      case SOTM:
        // shoot on the move, reference Mechanical Advantage build log
        System.out.println("Drivetrain: SOTM Drive");
        break;

      case AUTO_ALIGN_HUB:
        // Recompute the desired heading toward the hub each loop
        m_targetAngle =
            ShotCalculator.getRotationTowardsHub(
                ShotCalculator.calculateHubPosition(), subsystem.getState().Pose.getTranslation());
        System.out.println(
            "Auto Align Current Pose: "
                + subsystem.getState().Pose.getX()
                + " "
                + subsystem.getState().Pose.getY());

        double currentRad = subsystem.getState().Pose.getRotation().getRadians();
        double targetRad = m_targetAngle.getRadians();
        // Normalize error to [-pi, pi]
        double error =
            Math.atan2(Math.sin(targetRad - currentRad), Math.cos(targetRad - currentRad));
        double absError = Math.abs(error);
        double angleTolerance = Math.toRadians(1.0); // stop within 1 degree

        double rotOutput = getAutoAlignRotationalOutput();

        if (absError < angleTolerance) {
          // Aligned: stop rotating (and hold position)
          subsystem.setControl(drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
        } else {
          // Not aligned: apply rotational output
          System.out.println("new rotational output: " + rotOutput);
          subsystem.setControl(
              drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(rotOutput));
        }

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

    edu.wpi.first.math.geometry.Translation2d hubPos = ShotCalculator.calculateHubPosition();
    if (hubPos == null) {
      return false;
    }

    edu.wpi.first.math.geometry.Translation2d currentTrans =
        subsystem.getState().Pose.getTranslation();
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
