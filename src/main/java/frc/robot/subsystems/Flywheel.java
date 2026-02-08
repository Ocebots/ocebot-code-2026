package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CANMappings;
import frc.robot.config.FlywheelConfig;

@Logged
public class Flywheel extends SubsystemBase {
  protected TalonFX flywheel;
  private double flywheelScoreSpeed;

  public Flywheel() {
    flywheel = new TalonFX(CANMappings.FLYWHEEL_MOTOR_ID);
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit = FlywheelConfig.FLYWHEEL_STATOR_CURRENT_LIMIT;

    flywheelConfig.CurrentLimits.SupplyCurrentLimit = FlywheelConfig.FLYWHEEL_SUPPLY_CURRENT_LIMIT;

    flywheelConfig.MotionMagic.MotionMagicCruiseVelocity =
        FlywheelConfig.FLYWHEEL_MAX_CRUISE_VELOCITY;
    flywheelConfig.MotionMagic.MotionMagicAcceleration =
        FlywheelConfig.FLYWHEEL_TARGET_ACCELERATION;

    flywheelConfig.Slot0.kP = FlywheelConfig.FLYWHEEL_P;
    flywheelConfig.Slot0.kI = FlywheelConfig.FLYWHEEL_I;
    flywheelConfig.Slot0.kD = FlywheelConfig.FLYWHEEL_D;
    flywheelConfig.Slot0.kS = FlywheelConfig.FLYWHEEL_S;
    flywheelConfig.Slot0.kV = FlywheelConfig.FLYWHEEL_V;
    flywheelConfig.Slot0.kA = FlywheelConfig.FLYWHEEL_A;

    flywheelConfig.Feedback.SensorToMechanismRatio = FlywheelConfig.FLYWHEEL_GEAR_RATIO;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheel.getConfigurator().apply(flywheelConfig);
  }

  public void shoot(double speed) {
    speed = Math.abs(speed);
    flywheel.setControl(new MotionMagicVoltage(speed));
  }

  public void outtake(double speed) {
    speed = Math.abs(speed);
    flywheel.setControl(new DutyCycleOut(speed));
  }

  public void stop() {
    flywheel.stopMotor();
  }

  public boolean atVelocity() {
    return (Math.abs(flywheel.getClosedLoopError().getValueAsDouble())
        <= FlywheelConfig.FLYWHEEL_TOLERANCE);
  }
}
