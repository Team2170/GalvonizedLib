package BobcatLib.Hardware.Motors;

public class OptionalConfigCtre {
  public class RampAndLoopConfiguration {
    private final double DutyCycleRampPeriod;
    private final double VoltageRampPeriod;
    private final double TorqueRampPeriod;

    /**
     * Constructs an immutable RampAndLoopConfiguration with the specified ramp periods.
     *
     * @param dutycycle The duty cycle ramp period.
     * @param torque The torque cycle ramp period.
     * @param voltage The voltage ramp period.
     */
    public RampAndLoopConfiguration(double dutycycle, double voltage, double torque) {
      this.DutyCycleRampPeriod = dutycycle;
      this.TorqueRampPeriod = torque;
      this.VoltageRampPeriod = voltage;
    }

    /**
     * Constructs an immutable RampAndLoopConfiguration with the specified ramp periods.
     *
     * @param dutycycle The duty cycle ramp period.
     * @param voltage The voltage ramp period.
     */
    public RampAndLoopConfiguration(double dutycycle, double voltage) {
      this.DutyCycleRampPeriod = dutycycle;
      this.TorqueRampPeriod = 0.00;
      this.VoltageRampPeriod = voltage;
    }

    /**
     * Retrieves the duty cycle ramp period.
     *
     * @return The duty cycle ramp period.
     */
    public double getDutyCycleRampPeriod() {
      return DutyCycleRampPeriod;
    }

    /**
     * Retrieves the voltage ramp period.
     *
     * @return The voltage ramp period.
     */
    public double getVoltageRampPeriod() {
      return VoltageRampPeriod;
    }

    /**
     * Retrieves the torque ramp period.
     *
     * @return The torque ramp period.
     */
    public double getTorqueRampPeriod() {
      return TorqueRampPeriod;
    }
  }

  /**
   * Flag to indicate whether Field-Oriented Control (FOC) is enabled for the motor. FOC allows more
   * efficient control of the motor, particularly in situations requiring precision or dynamic
   * performance.
   */
  public boolean useFOC;

  /**
   * Flag to enable or disable stator supply current monitoring for the motor. When enabled, it
   * allows monitoring of the stator current, which can be useful for diagnostics and safety
   * purposes.
   */
  public boolean StatorSupplyCurrentEnable;

  /**
   * The limit on the stator supply current (in Amps) to protect the motor from overcurrent
   * situations. This limit is applied when stator current monitoring is enabled.
   */
  public double StatorSupplyCurrentLimit;

  /**
   * Flag to enable or disable the supply current limit for the motor. When enabled, it imposes a
   * limit on the amount of current supplied to the motor to prevent damage or overuse.
   */
  public boolean SupplyCurrentLimitEnable;

  /**
   * The maximum supply current limit (in Amps) for the motor. This is the threshold beyond which
   * current is limited to protect the motor from excessive current draw.
   */
  public double SupplyCurrentLimit;

  /**
   * The threshold (in Amps) for detecting supply current spikes. This is used in combination with
   * the supply current limit to trigger actions when current exceeds the threshold.
   */
  public double SupplyCurrentThreshold;

  /**
   * The time threshold (in seconds) for supply current spikes. If the supply current exceeds the
   * threshold for a period longer than this value, a protective action might be taken.
   */
  public double SupplyTimeThreshold;

  /**
   * Configuration for open-loop motor control, which refers to motor control without feedback,
   * typically used for speed control or simpler applications. This configuration includes ramping
   * and looping parameters specific to open-loop control.
   */
  public RampAndLoopConfiguration openLoop;

  /**
   * Configuration for closed-loop motor control, which uses feedback (e.g., from a sensor) to
   * maintain a set speed or position. This configuration includes ramping and looping parameters
   * specific to closed-loop control.
   */
  public RampAndLoopConfiguration closedLoop;
}
