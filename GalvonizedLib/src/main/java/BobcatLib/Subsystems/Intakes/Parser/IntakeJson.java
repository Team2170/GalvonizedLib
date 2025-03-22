package BobcatLib.Subsystems.Intakes.Parser;

public class IntakeJson {
  public String name = "";
  public boolean usePivotMotor = false;
  // Section Handles Pivot Motor Constants
  public double upperPivotRange = 0.00;
  public double lowerPivotRange = 0.00;
  public boolean pivotMotorInverted = false;
  public boolean pivotBrakeMode = true;
  public double angleOffset = 0.00;
  public double pivotPid_kP = 0.00;
  public double pivotPid_kI = 0.00;
  public double pivotPid_kD = 0.0;
  public double mechanismCircumference = 0.00;
  public int pivotMotorId = 0;
  public boolean useAbsEncoder = false;
  public int absPivotEncoderId = 0;
  // Section Handles Roller Motor Constants
  public boolean rollerMotorInverted = false;
  public boolean rollerBrakeMode = false;
  public double rollerPid_kP = 0.00;
  public double rollerPid_kI = 0.00;
  public double rollerPid_kD = 0.0;
  public int rollerMotorId = 0;
}
