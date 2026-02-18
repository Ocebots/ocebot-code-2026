package frc.robot.Commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.HopperConfig;
import frc.robot.subsystems.Hopper;

@Logged
public class HopperCommand extends Command {
  public static enum Position {
    EXTEND,
    RETRACT
  }

  private Hopper subsystem;
  private HopperCommand.Position pose;

  public HopperCommand(Hopper subsystem, HopperCommand.Position pose) {
    this.pose = pose;
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {
      case EXTEND:
        subsystem.move(HopperConfig.HOPPER_EXTEND_ROTATION);
        System.out.println("Hopper: Extending");
        break;

      case RETRACT:
        subsystem.move(HopperConfig.HOPPER_RETRACT_ROTATION);
        System.out.println("Hopper: Retracting");
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
