package BobcatLib.Hardware.Motors.SensorHelpers;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * A wrapper class for managing the neutral or disabled state of a motor controller bridge. This
 * class provides a way to represent and convert motor controller states between different enums,
 * such as `NeutralModeValue` and `IdleMode`.
 */
public class NeutralModeWrapper {
  /** The current mode of the motor controller bridge. */
  disabledMode mode = disabledMode.None;

  /** Enum representing the possible disabled or neutral states of the motor controller. */
  public enum disabledMode {
    Coast, // Motor is in coast mode (freewheeling)
    Brake, // Motor is in brake mode (braking when idle)
    None // No neutral or disabled state applied
  }

  /**
   * Constructor to initialize the neutral mode using a `NeutralModeValue`.
   *
   * @param mode2 The neutral mode value to set the motor controller to. It can be either
   *     `NeutralModeValue.Brake` or `NeutralModeValue.Coast`.
   */
  public NeutralModeWrapper(NeutralModeValue mode2) {
    mode = disabledMode.Coast;
    if (mode2 == NeutralModeValue.Brake) {
      mode = disabledMode.Brake;
    }
  }

  /**
   * Constructor to initialize the neutral mode using an `IdleMode` value from the Spark motor
   * controller.
   *
   * @param value The idle mode value to set the motor controller to. It can be either
   *     `IdleMode.kBrake` or `IdleMode.kCoast`.
   */
  public NeutralModeWrapper(IdleMode value) {
    mode = disabledMode.Coast;
    if (value == IdleMode.kBrake) {
      mode = disabledMode.Brake;
    }
  }

  /**
   * Constructor to initialize the neutral mode directly using a `disabledMode` enum value.
   *
   * @param value The `disabledMode` value to set the motor controller to. It can be `Coast`,
   *     `Brake`, or `None`.
   */
  public NeutralModeWrapper(disabledMode value) {
    mode = value;
  }

  /**
   * Converts the current state of the motor controller to the corresponding `IdleMode` for Spark
   * motor controllers.
   *
   * @return The `IdleMode` representing the current neutral or disabled state of the motor
   *     controller. Returns `IdleMode.kBrake` if the state is `Brake`, otherwise returns
   *     `IdleMode.kCoast`.
   */
  public IdleMode asIdleMode() {
    if (mode == disabledMode.Brake) {
      return IdleMode.kBrake;
    }
    return IdleMode.kCoast;
  }

  /**
   * Converts the current state of the motor controller to the corresponding `NeutralModeValue`.
   *
   * @return The `NeutralModeValue` representing the current neutral or disabled state of the motor
   *     controller. Returns `NeutralModeValue.Brake` if the state is `Brake`, otherwise returns
   *     `NeutralModeValue.Coast`.
   */
  public NeutralModeValue asNeutralModeValue() {
    if (mode == disabledMode.Brake) {
      return NeutralModeValue.Brake;
    }
    return NeutralModeValue.Coast;
  }
}
