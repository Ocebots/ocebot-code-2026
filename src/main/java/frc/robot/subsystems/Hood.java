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
import frc.robot.config.HoodConfig;
@Logged
public class Hood extends SubsystemBase {
  protected TalonFX hood;
  private double hoodScoreRotation;

  public Hood() {
    hood = new TalonFX(CANMappings.HOOD_MOTOR_ID);
    hood = new TalonFX(CANMappings.HOOD_MOTOR_ID);
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.StatorCurrentLimit = HoodConfig.HOOD_STATOR_CURRENT_LIMIT;

    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConfig.HOOD_SUPPLY_CURRENT_LIMIT;

    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = HoodConfig.HOOD_MAX_CRUISE_VELOCITY;
    hoodConfig.MotionMagic.MotionMagicAcceleration = HoodConfig.HOOD_TARGET_ACCELERATION;

    hoodConfig.Slot0.kP = HoodConfig.HOOD_P;
    hoodConfig.Slot0.kI = HoodConfig.HOOD_I;
    hoodConfig.Slot0.kD = HoodConfig.HOOD_D;

    hoodConfig.Feedback.SensorToMechanismRatio = HoodConfig.HOOD_GEAR_RATIO;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hood.getConfigurator().apply(hoodConfig);
  }

  public void rotate(double rotation) {
    hood.setControl(new MotionMagicVoltage(rotation));
  }

  public void rotateDirectional(double speed) {
    hood.setControl(new DutyCycleOut(speed));
  }

  public void stop() {
    hood.stopMotor();
  }

  public void zero() {
    hood.setPosition(0.0);
  }

  public boolean atPosition() {
    return (Math.abs(hood.getClosedLoopError().getValueAsDouble()) <= HoodConfig.HOOD_TOLERANCE);
  }
}
