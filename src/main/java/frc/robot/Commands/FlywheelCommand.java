package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.FlywheelConfig;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.ShotCalculator;

public class FlywheelCommand extends Command {
  public static enum Position {
    SHOOT,
    PASS,
    OUTTAKE
  }

  private Flywheel subsystem;
  private FlywheelCommand.Position pose;
  private CommandSwerveDrivetrain drivetrain;

  public FlywheelCommand(
      Flywheel subsystem, FlywheelCommand.Position pose, CommandSwerveDrivetrain drivetrain) {
    this.pose = pose;
    this.subsystem = subsystem;
    this.drivetrain = drivetrain;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {
      case SHOOT:
        subsystem.shoot(
            ShotCalculator.calculateFlywheelShot(
                drivetrain.getState().Pose.getTranslation(),
                new Translation2d(
                    drivetrain.getState().Speeds.vxMetersPerSecond,
                    drivetrain.getState().Speeds.vyMetersPerSecond)));
        break;

      case PASS:
        subsystem.shoot(
            ShotCalculator.calculateFlywheelPass(
                drivetrain.getState().Pose.getTranslation(),
                new Translation2d(
                    drivetrain.getState().Speeds.vxMetersPerSecond,
                    drivetrain.getState().Speeds.vyMetersPerSecond)));
        break;

      case OUTTAKE:
        subsystem.outtake(FlywheelConfig.FLYWHEEL_OUTTAKE_SPEED);
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
