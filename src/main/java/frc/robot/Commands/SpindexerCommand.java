package frc.robot.Commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

@Logged
public class SpindexerCommand extends Command {
  public static enum Position {
    INDEX
  }

  private Spindexer subsystem;
  private SpindexerCommand.Position pose;

  public SpindexerCommand(Spindexer subsystem, SpindexerCommand.Position pose) {
    this.pose = pose;
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (pose) {
      case INDEX:
        subsystem.index();
        System.out.println("Spindexer: Indexing");
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
