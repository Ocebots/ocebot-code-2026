package frc.robot.Commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class DrivetrainCommand extends Command {
  public static enum Position {
    TELEOP,
    STILL_SHOT,
    SOTM
  }

  private CommandSwerveDrivetrain subsystem;
  private DrivetrainCommand.Position pose;
  private double leftY;
  private double leftX;
  private double rightX;

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
  public static final double DEADBAND = 0.05;

  public DrivetrainCommand(
      CommandSwerveDrivetrain subsystem,
      DrivetrainCommand.Position pose,
      double leftX,
      double leftY,
      double rightX) {
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
        subsystem.applyRequest(
            () ->
                drive
                    .withVelocityX(-leftY * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-leftX * MaxSpeed) // Drive left with negative X
                    .withRotationalRate(
                        -rightX * MaxAngularRate)); // Drive counterclockwise with negative X
        System.out.println("Drivetrain: Teleop Drive");
        break;

      // (Incomplete) Mode for when shots are from a still position
      case STILL_SHOT:
        // make X with wheels
        System.out.println("Drivetrain: Still Shot Configuration");
        break;

      // (Incomplete) Mode for moving shots
      case SOTM:
        // shoot on the move, reference Mechanical Advantage build log
        System.out.println("Drivetrain: SOTM Drive");
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
