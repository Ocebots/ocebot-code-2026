package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.ClimbConfig;
import frc.robot.subsystems.Climb;

@Logged
public class ClimbCommand extends Command {
  public static enum Position {
    DIRECTIONAL_CLIMB,
    DIRECTIONAL_UNCLIMB,
    POSITIONAL_CLIMB,
    POSITIONAL_UNCLIMB
  }

  private Climb subsystem;
  private Position pose;

  public ClimbCommand(Climb subsystem, Position pose) {
    this.pose = pose;
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {

      // Pulls down on climber (moves climber in down direction)
      case DIRECTIONAL_CLIMB:
        subsystem.retractDirectional(ClimbConfig.CLIMB_CLIMB_SPEED);
        System.out.println("Climb: Directional Climb");
        break;

      // Pushes up on climber (moves climber in up direction)
      case DIRECTIONAL_UNCLIMB:
        subsystem.extendDirectional(ClimbConfig.CLIMB_UNCLIMB_SPEED);
        System.out.println("Climb: Directional Unclimb");
        break;

      // Moves climber to climb position (pulls robot up to climb position)
      case POSITIONAL_CLIMB:
        subsystem.move(ClimbConfig.CLIMB_CLIMB_ROTATION);
        System.out.println("Climb: Positional Climb");
        break;

      // Moves climber to unclimb position (lets robot back down to climb position)
      case POSITIONAL_UNCLIMB:
        subsystem.move(ClimbConfig.CLIMB_UNCLIMB_ROTATION);
        System.out.println("Climb: Positional Unclimb");
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
