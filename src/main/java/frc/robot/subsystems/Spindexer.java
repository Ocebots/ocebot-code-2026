package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.SpindexerConfig;

public class Spindexer extends SubsystemBase {
  protected TalonFX spindexer;

  public Spindexer() {
    spindexer = new TalonFX(CANMappings.SPINDEXER_MOTOR_ID);
    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();

    spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    spindexerConfig.CurrentLimits.SupplyCurrentLimit =
        SpindexerConfig.SPINDEXER_SUPPLY_CURRENT_LIMIT;
    spindexerConfig.CurrentLimits.StatorCurrentLimit =
        SpindexerConfig.SPINDEXER_STATOR_CURRENT_LIMIT;

    spindexerConfig.Feedback.SensorToMechanismRatio = SpindexerConfig.SPINDEXER_GEAR_RATIO;
    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spindexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    spindexer.getConfigurator().apply(spindexerConfig);
  }

  public void rotate(double speed) {
    spindexer.setControl(new DutyCycleOut(speed));
  }

  public void index() {
    this.rotate(SpindexerConfig.SPINDEXER_INDEXING_SPEED);
  }

  public void stop() {
    spindexer.stopMotor();
  }
}
