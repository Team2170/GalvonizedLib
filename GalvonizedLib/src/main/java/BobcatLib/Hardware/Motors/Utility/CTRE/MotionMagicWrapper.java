package BobcatLib.Hardware.Motors.Utility.CTRE;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

/** Immutable wrapper for configuring Motion Magic parameters on a TalonFX motor controller. */
public final class MotionMagicWrapper {
  private final TalonFXConfiguration config;

  /**
   * Constructs a new MotionMagicWrapper with the given motor configuration.
   *
   * @param motorConfiguration the TalonFXConfiguration to be used
   */
  public MotionMagicWrapper(TalonFXConfiguration motorConfiguration) {
    this.config = motorConfiguration;
    this.config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
  }

  /**
   * Returns a new MotionMagicWrapper with the specified cruise velocity.
   *
   * @param mmcv the Motion Magic cruise velocity
   * @return a new MotionMagicWrapper instance with the updated configuration
   */
  public MotionMagicWrapper withCruiseVelocity(double mmcv) {
    config.MotionMagic.MotionMagicCruiseVelocity = mmcv;
    return new MotionMagicWrapper(config);
  }

  /**
   * Returns a new MotionMagicWrapper with the specified kA (acceleration gain).
   *
   * @param kA the Motion Magic exponential kA value
   * @return a new MotionMagicWrapper instance with the updated configuration
   */
  public MotionMagicWrapper with_kA(double kA) {
    config.MotionMagic.MotionMagicExpo_kA = kA;
    return new MotionMagicWrapper(config);
  }

  /**
   * Returns a new MotionMagicWrapper with the specified kV (velocity gain).
   *
   * @param kV the Motion Magic exponential kV value
   * @return a new MotionMagicWrapper instance with the updated configuration
   */
  public MotionMagicWrapper with_kV(double kV) {
    config.MotionMagic.MotionMagicExpo_kV = kV;
    return new MotionMagicWrapper(config);
  }

  /**
   * Returns a new MotionMagicWrapper with the specified kG (gravity gain).
   *
   * @param kG the gravity feedforward gain
   * @return a new MotionMagicWrapper instance with the updated configuration
   */
  public MotionMagicWrapper with_kG(double kG) {
    config.Slot0.kG = kG;
    return new MotionMagicWrapper(config);
  }

  /**
   * Returns a new MotionMagicWrapper with the specified kS (static friction gain).
   *
   * @param kS the static friction feedforward gain
   * @return a new MotionMagicWrapper instance with the updated configuration
   */
  public MotionMagicWrapper with_kS(double kS) {
    config.Slot0.kS = kS;
    return new MotionMagicWrapper(config);
  }

  /**
   * Gets the current TalonFX configuration.
   *
   * @return the TalonFXConfiguration instance
   */
  public TalonFXConfiguration getConfig() {
    return config;
  }

  /**
   * Gets the current Motion Magic configuration.
   *
   * @return the MotionMagicConfigs instance
   */
  public MotionMagicConfigs getMMConfig() {
    return config.MotionMagic;
  }

  /**
   * Gets the current kG (gravity gain) value from the PID controller.
   *
   * @return the kG value
   */
  public double getPID_kG() {
    return config.Slot0.kG;
  }

  /**
   * Gets the current kS (static friction gain) value from the PID controller.
   *
   * @return the kS value
   */
  public double getPID_kS() {
    return config.Slot0.kS;
  }
}
