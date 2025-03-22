package GalvonizedLib.Utilities;

/**
 * The {@code GarbageHandler} class is responsible for periodically triggering garbage collection to
 * help manage memory usage in the application.
 */
public class GarbageHandler {
  private final GalvonizedRunner executeCleanup;

  /**
   * Constructs a {@code GarbageHandler} that schedules periodic garbage collection execution.
   *
   * @param seconds The interval in seconds at which garbage collection is triggered.
   */
  public GarbageHandler(int seconds) {
    executeCleanup = new GalvonizedRunner(this::executeCleanup, seconds, true);
  }

  /** Executes garbage collection by invoking {@code System.gc()}. */
  public void executeCleanup() {
    System.gc();
  }

  /** Stops the scheduled garbage collection execution. */
  public void stop() {
    executeCleanup.endExecution();
  }
}
