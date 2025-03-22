package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser;

public class ModuleEncoderJson {
  public ModuleEncoderJson() {}

  public ModuleEncoderJson(int id, String canbus) {
    this.id = id;
    this.canbus = canbus;
  }

  /** The device type, e.g. CANCoder , CanAndCoder */
  public String type = "";
  /** The CAN ID or pin ID of the device. */
  public int id = 0;
  /** The CAN bus name which the device resides on if using CAN. */
  public String canbus = "";
  /** The offset of the absolute encoder. */
  public double offset = 0.00;
  /** sets the inversion state of the gyro. */
  public boolean inverted = false;
}
