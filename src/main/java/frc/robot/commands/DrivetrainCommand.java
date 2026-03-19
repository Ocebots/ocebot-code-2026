package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    SOTM
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
          .withDeadband(MaxSpeed * 0.2)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType
                  .OpenLoopVoltage); // Use open-loop control for drive motors

  private static final double NOTE_EXIT_VELOCITY = 10.0; // This needs to be tuned

  SwerveModuleState[] states = {
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(225))
  };
  private final ApplyModuleStates applyRequest = new ApplyModuleStates();
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

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

    addRequirements(subsystem);
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

      // (Incomplete) Mode for moving shots; pretty much finished but needs tuning and testing, and
      // may need to be changed to use a different method of calculating the shot
      case SOTM:
        double vx = -leftY.getAsDouble() * MaxSpeed;
        double vy = -leftX.getAsDouble() * MaxSpeed;
        Translation2d robotPosition = subsystem.getState().Pose.getTranslation();
        ChassisSpeeds speeds = subsystem.getState().Speeds;
        Translation2d robotVelocity =
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Translation2d goal = ShotCalculator.calculateGoalPosition();
        double distance = robotPosition.getDistance(goal);
        double flightTime = distance / NOTE_EXIT_VELOCITY;
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
        double kP = 4.0; // tune
        double omega = error * kP;
        omega = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, omega));
        subsystem.setControl(drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));

        break;
      default:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
