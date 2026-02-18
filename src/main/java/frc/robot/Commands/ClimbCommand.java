package frc.robot.Commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.ClimbConfig;
import frc.robot.subsystems.Climb;

@Logged
public class ClimbCommand extends Command {
  public static enum Position {
    CLIMB,
    UNCLIMB
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
      case CLIMB:
        subsystem.move(ClimbConfig.CLIMB_CLIMB_ROTATION);
        System.out.println("Climb: Climbing");
        break;

      case UNCLIMB:
        subsystem.move(ClimbConfig.CLIMB_UNCLIMB_ROTATION);
        System.out.println("Climb: Unclimbing");
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
