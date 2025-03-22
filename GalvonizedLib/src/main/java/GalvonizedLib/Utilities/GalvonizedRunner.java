package GalvonizedLib.Utilities;

import edu.wpi.first.wpilibj.Notifier;

/**
 * The {@code BobcatRunner} class provides a mechanism to execute a given method periodically or as
 * a one-time execution after a specified delay.
 */
public class GalvonizedRunner {
  private Notifier executer = new Notifier(() -> {});

  /**
   * Constructs an instance of the {@code BobcatRunner}, allowing a method to be executed
   * periodically or once after a given delay.
   *
   * @param callback The method to be executed.
   * @param delayInSeconds The delay in seconds before the execution starts.
   * @param isContinuous If {@code true}, the method runs periodically with the given delay;
   *     otherwise, it runs only once after the delay.
   */
  public GalvonizedRunner(Runnable callback, int delayInSeconds, boolean isContinuous) {
    executer.setCallback(callback);
    if (isContinuous) {
      executer.startPeriodic(delayInSeconds);
    } else {
      executer.startSingle(delayInSeconds);
    }
  }

  /** Stops the execution of the scheduled task. */
  public void endExecution() {
    executer.stop();
  }
}
