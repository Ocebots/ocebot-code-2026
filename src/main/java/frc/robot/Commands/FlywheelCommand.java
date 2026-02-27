package frc.robot.Commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.FlywheelConfig;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.ShotCalculator;

@Logged
public class FlywheelCommand extends Command {
  public static enum Position {
    HUB_SHOT,
    TOWER_SHOT,
    CALCULATED_SHOT,
    PASS,
    DEFAULT_SHOT,
    OUTTAKE
  }

  private Flywheel subsystem;
  private FlywheelCommand.Position pose;
  private CommandSwerveDrivetrain drivetrain;

  public FlywheelCommand(Flywheel subsystem, Position pose, CommandSwerveDrivetrain drivetrain) {
    this.pose = pose;
    this.subsystem = subsystem;
    this.drivetrain = drivetrain;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    switch (pose) {
      // Spins flywheels at proper speed for shooting from hub
      case HUB_SHOT:
        subsystem.shoot(FlywheelConfig.FLYWHEEL_HUB_SHOT_SPEED);
        System.out.println("Flywheel: Hub Shot");
        break;

      // Spins flywheels at proper speed for shooting from tower
      case TOWER_SHOT:
        subsystem.shoot(FlywheelConfig.FLYWHEEL_TOWER_SHOT_SPEED);
        System.out.println("Flywheel: Tower Shot");
        break;

      // Spins flywheels at estimated speed given distance from hub
      case CALCULATED_SHOT:
        subsystem.shoot(
            ShotCalculator.calculateFlywheelShot(drivetrain.getState().Pose.getTranslation()));
        System.out.println("Flywheel: Calculated Shot");
        break;

      // Determines if robot should be prepared to shoot (if during active period or 5 seconds
      // before one), and runs at middle speed close to actual speeds as to not overheat motors but
      // to minimize spin-up time
      case DEFAULT_SHOT:
        subsystem.shoot(
            ShotCalculator.calculateFlywheelDefaultShot(
                drivetrain.getState().Pose.getTranslation()));
        System.out.println("Flywheel: Default Shot");
        break;

      // Runs flywheels in outtake direction in case of jam
      case OUTTAKE:
        subsystem.outtake(FlywheelConfig.FLYWHEEL_OUTTAKE_SPEED);
        System.out.println("Flywheel: Outtake");
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
    return false;
  }
}
