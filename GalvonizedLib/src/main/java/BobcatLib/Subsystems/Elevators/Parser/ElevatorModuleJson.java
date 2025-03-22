package BobcatLib.Subsystems.Elevators.Parser;

public class ElevatorModuleJson {
  public class ElevatorJson {
    public String name;
  }

  public class ElevatorLimitsJson {
    public double upperLimit;
    public double lowerLimit;
  }

  public class ElevatorMotorJson {
    public String type;
    public String idle_type;
    public int id;
    public String canbus;
    public boolean inverted;
  }

  public class ElevatorPID {
    public double kP;
    public double kI;
    public double kD;
    public double kF;
  }

  public ElevatorJson elevator;
  public ElevatorLimitsJson limits;
  public ElevatorMotorJson motor;
  public ElevatorPID elevatorPid;
}
