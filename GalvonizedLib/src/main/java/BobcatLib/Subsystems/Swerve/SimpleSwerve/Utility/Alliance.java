package BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility;

import edu.wpi.first.wpilibj.DriverStation;

public class Alliance {
  private DriverStation.Alliance alliance;

  public Alliance() {
    this.alliance = getNullSafe();
  }

  public Alliance(DriverStation.Alliance ally) {
    alliance = ally;
  }

  public boolean isBlueAlliance() {
    return alliance == DriverStation.Alliance.Blue;
  }

  public boolean isRedAlliance() {
    return alliance == DriverStation.Alliance.Red;
  }

  public DriverStation.Alliance get() {
    return alliance;
  }

  /**
   * If FMS isnt attached, Driverstation.getAlliance() returns null, this defaults to blue instead
   *
   * @return blue if FMS isnt attached, actual alliance otherwise
   */
  public DriverStation.Alliance getNullSafe() {
    return DriverStation.getAlliance().isEmpty()
        ? DriverStation.Alliance.Blue
        : DriverStation.getAlliance().get();
  }
}
