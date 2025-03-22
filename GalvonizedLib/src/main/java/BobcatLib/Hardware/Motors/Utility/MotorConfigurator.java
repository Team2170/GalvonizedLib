package BobcatLib.Hardware.Motors.Utility;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;

/** A utility class for configuring motor controllers. */
public class MotorConfigurator {
  private TalonFXConfiguration ctreConfig;
  private SparkMaxConfig revConfig;

  /**
   * Constructs a new MotorConfigurator with default configurations for both CTRE and REV motors.
   */
  public MotorConfigurator() {
    this.ctreConfig = new TalonFXConfiguration();
    this.revConfig = new SparkMaxConfig();
  }

  /**
   * Constructs a new MotorConfigurator with a specified CTRE configuration.
   *
   * @param ctreConfig the TalonFXConfiguration to be used
   */
  public MotorConfigurator(TalonFXConfiguration ctreConfig) {
    this.ctreConfig = ctreConfig;
  }

  /**
   * Constructs a new MotorConfigurator with a specified REV configuration.
   *
   * @param revConfig the SparkMaxConfig to be used
   */
  public MotorConfigurator(SparkMaxConfig revConfig) {
    this.revConfig = revConfig;
  }

  /**
   * Gets the CTRE motor configuration.
   *
   * @return the current TalonFXConfiguration
   */
  public TalonFXConfiguration getCtreConfig() {
    return ctreConfig;
  }

  /**
   * Gets the REV motor configuration.
   *
   * @return the current SparkMaxConfig
   */
  public SparkMaxConfig getRevConfig() {
    return revConfig;
  }

  /**
   * Updates the CTRE motor configuration.
   *
   * @param ctreConfig the new TalonFXConfiguration to be applied
   */
  public void update(TalonFXConfiguration ctreConfig) {
    this.ctreConfig = ctreConfig;
  }

  /**
   * Updates the REV motor configuration.
   *
   * @param revConfig the new SparkMaxConfig to be applied
   */
  public void update(SparkMaxConfig revConfig) {
    this.revConfig = revConfig;
  }
}
