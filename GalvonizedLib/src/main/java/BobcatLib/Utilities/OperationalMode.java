package BobcatLib.Utilities;
/**
 * Represents the operational mode of a robot. This class is immutable and provides factory methods
 * to create instances with specific modes.
 */
public final class OperationalMode {

  /** The current mode of operation. */
  private final Mode currentMode;

  /** Enumeration of possible operational modes. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY,
    /** No default? */
    NONE
  }

  /**
   * Private constructor to enforce immutability.
   *
   * @param mode the operational mode
   */
  private OperationalMode(Mode mode) {
    this.currentMode = mode;
  }

  /**
   * Private constructor to enforce immutability.
   *
   * @param isReal is real
   */
  private OperationalMode(boolean isReal) {
    if (isReal) {
      this.currentMode = Mode.REAL;
    } else {
      this.currentMode = Mode.SIM;
    }
  }
  /**
   * Private constructor to enforce immutability.
   *
   * <p>use factory classes!
   */
  private OperationalMode() {
    this.currentMode = Mode.NONE;
  }

  /**
   * Creates an instance based on whether the robot is real or simulated.
   *
   * @param isReal if the robot is real use "RobotBase.isReal()" as input here.
   * @return a new OperationalMode instance
   */
  public static OperationalMode withMode(boolean isReal) {
    return new OperationalMode(isReal ? Mode.REAL : Mode.SIM);
  }

  /**
   * Creates an instance representing the simulation mode.
   *
   * @return a new OperationalMode instance in SIM mode
   */
  public static OperationalMode asSim() {
    return new OperationalMode(Mode.SIM);
  }

  /**
   * Creates an instance representing the real mode.
   *
   * @return a new OperationalMode instance in REAL mode
   */
  public static OperationalMode asReal() {
    return new OperationalMode(Mode.REAL);
  }

  /**
   * Creates an instance representing the replay mode.
   *
   * @return a new OperationalMode instance in REPLAY mode
   */
  public static OperationalMode asReplay() {
    return new OperationalMode(Mode.REPLAY);
  }

  /**
   * Gets the current mode of operation.
   *
   * @return the current mode
   */
  public Mode getCurrentMode() {
    return currentMode;
  }
}
