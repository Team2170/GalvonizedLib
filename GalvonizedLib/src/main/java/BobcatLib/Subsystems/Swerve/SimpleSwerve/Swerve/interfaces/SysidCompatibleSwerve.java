package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.interfaces;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;

public interface SysidCompatibleSwerve {
  // TODO: better documentation
  /** set all modules to supplied voltage */
  public default void sysidVoltage(Measure<VoltageUnit> volts) {}

  /** volts */
  public default double getModuleVoltage(int moduleNumber) {
    return 0;
  }
  /** meters */
  public default double getModuleDistance(int moduleNumber) {
    return 0;
  }
  /** meters/sec */
  public default double getModuleSpeed(int moduleNumber) {
    return 0;
  }
}
