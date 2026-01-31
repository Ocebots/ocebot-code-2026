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
import frc.robot.config.HopperConfig;

@Logged
public class Hopper extends SubsystemBase {
  protected TalonFX hopper;

  public Hopper() {

    hopper = new TalonFX(CANMappings.HOPPER_MOTOR_ID);

    TalonFXConfiguration hopperConfig = new TalonFXConfiguration();

    hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hopperConfig.CurrentLimits.StatorCurrentLimit = HopperConfig.HOPPER_STATOR_CURRENT_LIMIT;

    hopperConfig.CurrentLimits.SupplyCurrentLimit = HopperConfig.HOPPER_SUPPLY_CURRENT_LIMIT;

    hopperConfig.MotionMagic.MotionMagicCruiseVelocity = HopperConfig.HOPPER_MAX_CRUISE_VELOCITY;
    hopperConfig.MotionMagic.MotionMagicAcceleration = HopperConfig.HOPPER_TARGET_ACCELERATION;

    hopperConfig.Slot0.kP = HopperConfig.HOPPER_P;
    hopperConfig.Slot0.kI = HopperConfig.HOPPER_I;
    hopperConfig.Slot0.kD = HopperConfig.HOPPER_D;

    hopperConfig.Feedback.SensorToMechanismRatio = HopperConfig.HOPPER_GEAR_RATIO;

    hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hopper.getConfigurator().apply(hopperConfig);
  }

  public void move(double rotation) {
    hopper.setControl(new MotionMagicVoltage(rotation));
  }

  public void directionalMove(double speed) {
    hopper.setControl(new DutyCycleOut(speed));
  }

  public void stop() {
    hopper.stopMotor();
  }

  public void zero() {
    hopper.setPosition(0.0);
  }

  public boolean atPosition() {
    return (Math.abs(hopper.getClosedLoopError().getValueAsDouble())
        <= HopperConfig.HOPPER_TOLERANCE);
  }

  public void extend() {
    this.move(HopperConfig.HOPPER_EXTEND_ROTATION);
  }

  public void retract() {
    this.move(HopperConfig.HOPPER_RETRACT_ROTATION);
  }
}
