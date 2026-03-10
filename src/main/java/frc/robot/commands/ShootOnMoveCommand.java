package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.FlywheelConfig;
import frc.robot.subsystems.*;

// I (stole) Thought of a new design that might work better, but I don't have the time to fully
// implement it right now. The general idea is we just adapt this for swerve
// https://www.youtube.com/watch?v=dATlviyccY0
// Then again we can't do much until vision in configured

@Logged
public class ShootOnMoveCommand extends Command {
  public static enum Position {
    SHOOTING,
    OFF
  }

  private CommandSwerveDrivetrain drivetrain;
  private Flywheel flywheel;
  private Position pose;

  public ShootOnMoveCommand(CommandSwerveDrivetrain drivetrain, Flywheel flywheel, Position pose) {
    this.drivetrain = drivetrain;
    this.flywheel = flywheel;
    this.pose = pose;

    addRequirements(drivetrain, flywheel);
  }

  // Temp
  @Override
  public void initialize() {
    switch (pose) {
      case SHOOTING:
        flywheel.shoot(FlywheelConfig.FLYWHEEL_HUB_SHOT_SPEED);
        break;
      case OFF:
        flywheel.shoot(0);
        break;
      default:
        break;
    }
  }
}
