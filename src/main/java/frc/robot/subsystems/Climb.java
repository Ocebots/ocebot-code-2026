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
import frc.robot.config.ClimbConfig;

@Logged
public class Climb extends SubsystemBase {
  protected TalonFX climb;

  public Climb() {
    climb = new TalonFX(CANMappings.CLIMB_MOTOR_ID);

    TalonFXConfiguration climbConfig = new TalonFXConfiguration();

    climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbConfig.CurrentLimits.StatorCurrentLimit = ClimbConfig.CLIMB_STATOR_CURRENT_LIMIT;
    ;
    climbConfig.CurrentLimits.SupplyCurrentLimit = ClimbConfig.CLIMB_SUPPLY_CURRENT_LIMIT;
    ;

    climbConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConfig.CLIMB_MAX_CRUISE_VELOCITY;
    climbConfig.MotionMagic.MotionMagicAcceleration = ClimbConfig.CLIMB_TARGET_ACCELERATION;

    climbConfig.Slot0.kP = ClimbConfig.CLIMB_P;
    climbConfig.Slot0.kI = ClimbConfig.CLIMB_I;
    climbConfig.Slot0.kD = ClimbConfig.CLIMB_D;

    climbConfig.Feedback.SensorToMechanismRatio = ClimbConfig.CLIMB_GEAR_RATIO;
    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    climb.getConfigurator().apply(climbConfig);
  }

  public void move(double rotation) {
    climb.setControl(new MotionMagicVoltage(rotation));
  }

  public void directionalMove(double speed) {
    climb.setControl(new DutyCycleOut(speed));
  }

  public void stop() {
    climb.stopMotor();
  }

  public void zero() {
    climb.setPosition(0.0);
  }

  public boolean atPosition() {
    return (Math.abs(climb.getClosedLoopError().getValueAsDouble()) <= ClimbConfig.CLIMB_TOLERANCE);
  }

  public void climb() {
    this.move(ClimbConfig.CLIMB_CLIMB_ROTATION);
  }

  public void unclimb() {
    this.move(ClimbConfig.CLIMB_UNCLIMB_ROTATION);
  }
}
