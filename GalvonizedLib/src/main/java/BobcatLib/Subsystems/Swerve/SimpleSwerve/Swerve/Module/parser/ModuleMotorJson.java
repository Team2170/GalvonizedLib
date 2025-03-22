package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser;

public class ModuleMotorJson {
  public ModuleMotorJson() {}

  public ModuleMotorJson(int id, String canbus) {
    this.id = id;
    this.canbus = canbus;
  }

  /** The device type, e.g. Kraken,Falcon,Vortex,NEO */
  public String motor_type = "";
  /** The CAN ID or pin ID of the device. */
  public int id = 0;
  /** The CAN bus name which the device resides on if using CAN. */
  public String canbus = "";
  /** sets the inversion state of the motor. */
  public boolean inverted = false;
}
