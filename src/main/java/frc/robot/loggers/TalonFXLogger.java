package frc.robot.loggers;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  protected void update(EpilogueBackend backend, TalonFX motor) {
    backend.log("Motor Current (A)", motor.getStatorCurrent().getValueAsDouble());
    backend.log("Motor Applied Output", motor.getDutyCycle().getValueAsDouble());
    backend.log("Motor Requested Output", motor.getClosedLoopReference().getValueAsDouble());
    backend.log("Error", motor.getClosedLoopError().getValueAsDouble());
    backend.log("Bus Voltage", motor.getSupplyVoltage().getValueAsDouble());
    backend.log("Relative Position", motor.getPosition().getValueAsDouble());
    backend.log("Velocity", motor.getVelocity().getValueAsDouble());
  }
}
