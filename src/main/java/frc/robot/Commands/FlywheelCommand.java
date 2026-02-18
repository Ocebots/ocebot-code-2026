package frc.robot.Commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.FlywheelConfig;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.ShotCalculator;

@Logged
public class FlywheelCommand extends Command {
  public static enum Position {
    SHOOT_SIMPLE,
    SHOOT_CALCULATED,
    PASS,
    OUTTAKE
  }

  private Flywheel subsystem;
  private FlywheelCommand.Position pose;
  private CommandSwerveDrivetrain drivetrain;

  public FlywheelCommand(Flywheel subsystem, Position pose) {
    this.pose = pose;
    this.subsystem = subsystem;
    this.drivetrain = drivetrain;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {
      case SHOOT_SIMPLE:
        subsystem.shoot(FlywheelConfig.FLYWHEEL_SHOOT_SPEED);
        System.out.println("Flywheel: Simple Shot");
        break;

      case SHOOT_CALCULATED:
        subsystem.shoot(
            ShotCalculator.calculateFlywheelShot(
                drivetrain.getState().Pose.getTranslation(),
                new Translation2d(
                    drivetrain.getState().Speeds.vxMetersPerSecond,
                    drivetrain.getState().Speeds.vyMetersPerSecond)));
        System.out.println("Flywheel: Calculated Shot");
        break;

      case PASS:
        subsystem.shoot(
            ShotCalculator.calculateFlywheelPass(
                drivetrain.getState().Pose.getTranslation(),
                new Translation2d(
                    drivetrain.getState().Speeds.vxMetersPerSecond,
                    drivetrain.getState().Speeds.vyMetersPerSecond)));
        System.out.println("Flywheel: Pass");

        break;

      case OUTTAKE:
        subsystem.outtake(FlywheelConfig.FLYWHEEL_OUTTAKE_SPEED);
        System.out.println("Flywheel: Outtakef");
        break;

      default:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return subsystem.atVelocity();
  }
}
