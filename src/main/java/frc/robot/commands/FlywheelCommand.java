package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.FlywheelConfig;
import frc.robot.helpers.ShotCalculator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;

@Logged
public class FlywheelCommand extends Command {
  public static enum Position {
    HUB_SHOT,
    TOWER_SHOT,
    TRENCH_SHOT,
    CALCULATED_SHOT,
    DEFAULT_SHOT,
    OUTTAKE
  }

  private Flywheel subsystem;
  private Position pose;
  private CommandSwerveDrivetrain drivetrain;
  public static String isOn = "";

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
        isOn = "true";
        //                 System.out.println("Flywheel: Hub Shot");
        break;

      // Spins flywheels at proper speed for shooting from tower
      case TOWER_SHOT:
        subsystem.shoot(FlywheelConfig.FLYWHEEL_TOWER_SHOT_SPEED);
        isOn = "true";

        //                 System.out.println("Flywheel: Tower Shot");
        break;

      case TRENCH_SHOT:
        subsystem.shoot(FlywheelConfig.FLYWHEEL_TRENCH_SHOT_SPEED);
        isOn = "true";

        //        System.out.println("Flywheel: Trench Shot");
        break;

      // Spins flywheels at estimated speed given distance from hub
      case CALCULATED_SHOT:
        isOn = "true";

        subsystem.shoot(
            ShotCalculator.calculateFlywheelShot(drivetrain.getState().Pose.getTranslation()));
        //         System.out.println("Flywheel: Calculated Shot");
        break;

      // Determines if robot should be prepared to shoot (if during active period or 5 seconds
      // before one), and runs at middle speed close to actual speeds as to not overheat motors but
      // to minimize spin-up time
      case DEFAULT_SHOT:
        subsystem.shoot(
            ShotCalculator.calculateFlywheelDefaultShot(
                drivetrain.getState().Pose.getTranslation()));
        //         System.out.println("Flywheel: Default Shot");
        break;

      // Runs flywheels in outtake direction in case of jam
      case OUTTAKE:
        subsystem.outtake(FlywheelConfig.FLYWHEEL_OUTTAKE_SPEED);
        // System.out.println("Flywheel: Outtake");
        break;

      default:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
    isOn = "false";

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
