package frc.robot.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.FlywheelConfig;
import java.util.Optional;

public class ShotCalculator {
  /* Calculation Methods */
  public static double calculateFlywheelShot(Translation2d robotPosition) {
    // 1. Calculate distance between goal and robot position
    double distanceToGoal = calculateGoalPosition().getDistance(robotPosition);
    // 2. Retrieve correlating RPS from map
    return DISTANCE_TO_FLYWHEEL.get(distanceToGoal);
  }

  public static double calculateFlywheelDefaultShot(Translation2d robotPosition) {
    // 1. Determine if within shooter run period (five seconds before or during active period)
    if (shouldRunShooter(5)) {
      // 2. If within proper shooting period, run at default speed
      return FlywheelConfig.FLYWHEEL_DEFAULT_SHOT_SPEED;
    }
    // 3. Otherwise, do not run shooter
    return 0;
  }

  /* Helper Methods */
  public static Translation2d calculateGoalPosition() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        return new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
      } else if (alliance.get() == DriverStation.Alliance.Red) {
        return new Translation2d(
            Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84));
      }
    }
    return null;
  }

  // Determines active based on current match time
  // (Needs check)
  private static boolean computeActive(double timeLeft, boolean shift1ActiveForMe) {
    if (timeLeft > 130) { // transition into teleop
      return true;
    } else if (timeLeft > 105) { // Shift 1
      return shift1ActiveForMe;
    } else if (timeLeft > 80) { // Shift 2
      return !shift1ActiveForMe;
    } else if (timeLeft > 55) { // Shift 3
      return shift1ActiveForMe;
    } else if (timeLeft > 30) { // Shift 4
      return !shift1ActiveForMe;
    } else { // End Game
      return true;
    }
  }

  // Determine if should run shooter (if hub is active or 5 seconds from being active)
  // (Needs check)
  public static boolean shouldRunShooter(double graceSeconds) {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    // If no alliance, return false
    if (alliance.isEmpty()) {
      return false;
    }

    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    // If not teleop, return false
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    double matchTime = DriverStation.getMatchTime(); // time left
    String gameData = DriverStation.getGameSpecificMessage();

    // If no data yet, assume active
    if (gameData.isEmpty()) {
      return true;
    }

    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    boolean iAmRed = alliance.get() == DriverStation.Alliance.Red;
    boolean shift1ActiveForMe = iAmRed ? !redInactiveFirst : redInactiveFirst;

    // Get active state for current match time
    boolean currentActive = computeActive(matchTime, shift1ActiveForMe);

    if (currentActive) {
      return true;
    }

    // If not active now, check if within graceSec of next active period
    // Next active transitions (end of shift boundary) times:
    double[] boundaries1 = {130, 105, 80, 55, 30};

    for (double boundary1 : boundaries1) {
      if (matchTime <= boundary1 + graceSeconds && matchTime >= boundary1 - graceSeconds) {

        // Check active right after boundary1:
        boolean activeAfter = computeActive(boundary1 - 0.01, shift1ActiveForMe);

        if (activeAfter) {
          return true;
        }
      }
    }
    return false;
  }

  /* Maps */
  // Distance (m) -> Speed (RPS)
  private static final InterpolatingDoubleTreeMap DISTANCE_TO_FLYWHEEL =
      new InterpolatingDoubleTreeMap();

  // Set map inputs
  static {
    DISTANCE_TO_FLYWHEEL.put(Units.inchesToMeters(171), 87.0);
    DISTANCE_TO_FLYWHEEL.put(Units.inchesToMeters(141), 70.0);
    DISTANCE_TO_FLYWHEEL.put(Units.inchesToMeters(90), 60.0);
    DISTANCE_TO_FLYWHEEL.put(Units.inchesToMeters(61), 50.0);
  }
}
