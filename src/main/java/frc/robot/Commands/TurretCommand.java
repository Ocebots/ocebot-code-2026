package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.subsystems.Turret;

public class TurretCommand extends Command {
  public static enum Position {
    STILL_SHOT,
    PASS,
    MOVING_SHOT
  }

  private Turret subsystem;
  private TurretCommand.Position pose;
  private CommandSwerveDrivetrain drivetrain;

  public TurretCommand(
      Turret subsystem, TurretCommand.Position pose, CommandSwerveDrivetrain drivetrain) {
    this.pose = pose;
    this.subsystem = subsystem;
    this.drivetrain = drivetrain;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {
      case STILL_SHOT:
        subsystem.rotate(
            ShotCalculator.calculateTurretStillShot(
                drivetrain.getState().Pose.getTranslation(),
                new Translation2d(
                    drivetrain.getState().Speeds.vxMetersPerSecond,
                    drivetrain.getState().Speeds.vyMetersPerSecond)));
        break;

      case PASS:
        subsystem.rotate(
            ShotCalculator.calculateTurretPass(
                drivetrain.getState().Pose.getTranslation(),
                new Translation2d(
                    drivetrain.getState().Speeds.vxMetersPerSecond,
                    drivetrain.getState().Speeds.vyMetersPerSecond)));
        break;

      case MOVING_SHOT:
        subsystem.rotate(
            ShotCalculator.calculateTurretSOTM(
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
