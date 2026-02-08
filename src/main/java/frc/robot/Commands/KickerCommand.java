package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.KickerConfig;
import frc.robot.subsystems.Kicker;

public class KickerCommand extends Command {
  public static enum Position {
    INTAKE,
    OUTTAKE
  }

  private Kicker subsystem;
  private KickerCommand.Position pose;

  public KickerCommand(Kicker subsystem, KickerCommand.Position pose) {
    this.pose = pose;
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {
      case INTAKE:
        subsystem.intake(KickerConfig.KICKER_INTAKE_SPEED);
        break;

      case OUTTAKE:
        subsystem.outtake(KickerConfig.KICKER_INTAKE_SPEED);
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
