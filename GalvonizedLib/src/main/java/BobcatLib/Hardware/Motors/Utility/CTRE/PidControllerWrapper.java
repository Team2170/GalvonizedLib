package BobcatLib.Hardware.Motors.Utility.CTRE;

import com.ctre.phoenix6.configs.Slot0Configs;

/**
 * A wrapper class around Slot0Configs that provides immutable updates for PID constants. Each
 * method in this class returns a new instance with the updated configuration.
 */
public class PidControllerWrapper {

  private final Slot0Configs pidSlot0Configs;

  /**
   * Constructs a PidControllerWrapper with the given Slot0Configs.
   *
   * @param pidSlot0Configs The initial PID configuration.
   */
  public PidControllerWrapper(Slot0Configs pidSlot0Configs) {
    this.pidSlot0Configs = pidSlot0Configs;
  }

  /**
   * Returns the Slot0Configs associated with this PidControllerWrapper.
   *
   * @return The current Slot0Configs.
   */
  public Slot0Configs getSlot0Config() {
    return pidSlot0Configs;
  }

  /**
   * Returns a new PidControllerWrapper with the updated kP value.
   *
   * @param kP The new kP value.
   * @return A new PidControllerWrapper with the updated kP value.
   */
  public PidControllerWrapper with_kP(double kP) {
    pidSlot0Configs.kP = kP;
    return new PidControllerWrapper(pidSlot0Configs);
  }

  /**
   * Returns a new PidControllerWrapper with the updated kI value.
   *
   * @param kI The new kI value.
   * @return A new PidControllerWrapper with the updated kI value.
   */
  public PidControllerWrapper with_kI(double kI) {
    pidSlot0Configs.kI = kI;
    return new PidControllerWrapper(pidSlot0Configs);
  }

  /**
   * Returns a new PidControllerWrapper with the updated kD value.
   *
   * @param kD The new kD value.
   * @return A new PidControllerWrapper with the updated kD value.
   */
  public PidControllerWrapper with_kD(double kD) {
    pidSlot0Configs.kD = kD;
    return new PidControllerWrapper(pidSlot0Configs);
  }

  /**
   * Returns a new PidControllerWrapper with the updated kA value.
   *
   * @param kA The new kA value.
   * @return A new PidControllerWrapper with the updated kA value.
   */
  public PidControllerWrapper with_kA(double kA) {
    pidSlot0Configs.kA = kA;
    return new PidControllerWrapper(pidSlot0Configs);
  }

  /**
   * Returns a new PidControllerWrapper with the updated kG value.
   *
   * @param kG The new kG value.
   * @return A new PidControllerWrapper with the updated kG value.
   */
  public PidControllerWrapper with_kG(double kG) {
    pidSlot0Configs.kG = kG;
    return new PidControllerWrapper(pidSlot0Configs);
  }

  /**
   * Returns a new PidControllerWrapper with the updated kS value.
   *
   * @param kS The new kS value.
   * @return A new PidControllerWrapper with the updated kS value.
   */
  public PidControllerWrapper with_kS(double kS) {
    pidSlot0Configs.kS = kS;
    return new PidControllerWrapper(pidSlot0Configs);
  }

  /**
   * Returns a new PidControllerWrapper with the updated kV value.
   *
   * @param kV The new kV value.
   * @return A new PidControllerWrapper with the updated kV value.
   */
  public PidControllerWrapper with_kV(double kV) {
    pidSlot0Configs.kV = kV;
    return new PidControllerWrapper(pidSlot0Configs);
  }
}
