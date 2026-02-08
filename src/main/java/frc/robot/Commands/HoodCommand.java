package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShotCalculator;

public class HoodCommand extends Command {
  public static enum Position {
    SHOOT,
    PASS
  }

  private Hood subsystem;
  private HoodCommand.Position pose;
  private CommandSwerveDrivetrain drivetrain;

  public HoodCommand(
      Hood subsystem, HoodCommand.Position pose, CommandSwerveDrivetrain drivetrain) {
    this.pose = pose;
    this.subsystem = subsystem;
    this.drivetrain = drivetrain;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {
      case SHOOT:
        subsystem.rotate(
            ShotCalculator.calculateHoodShot(
                drivetrain.getState().Pose.getTranslation(),
                new Translation2d(
                    drivetrain.getState().Speeds.vxMetersPerSecond,
                    drivetrain.getState().Speeds.vyMetersPerSecond)));
        break;
      case PASS:
        subsystem.rotate(
            ShotCalculator.calculateHoodPass(
                drivetrain.getState().Pose.getTranslation(),
                new Translation2d(
                    drivetrain.getState().Speeds.vxMetersPerSecond,
                    drivetrain.getState().Speeds.vyMetersPerSecond)));
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
    return subsystem.atPosition();
  }
}
