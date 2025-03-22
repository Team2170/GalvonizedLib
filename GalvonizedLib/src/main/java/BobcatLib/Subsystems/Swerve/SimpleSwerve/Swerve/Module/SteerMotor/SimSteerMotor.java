package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.ModuleConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimSteerMotor implements SteerWrapper {
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;

  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private final DCMotorSim turnSim;

  private double turnAppliedVolts = 0.0;
  public ModuleConstants chosenModule;
  public ModuleLimitsJson limits;

  public SimSteerMotor(ModuleConstants chosenModule, ModuleLimitsJson limits) {
    this.chosenModule = chosenModule;
    this.limits = limits;
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TURN_GEARBOX, 0.01, chosenModule.angleGearRatio),
            TURN_GEARBOX);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void withSupplyCurrent() {}

  public void withStatorCurrent() {}

  public void updateOutputs() {
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    turnSim.update(0.02);
  }

  /*
   *
   * sets the angle of the motor by setting the internal pid.
   *
   * @param rotations
   */
  public void setAngle(double rotations) {
    turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    turnController.setSetpoint(rotations);
  }

  /**
   * sets the position of the steer motor given the desired rotations.
   *
   * @param absolutePosition
   */
  public void setPosition(double absolutePosition) {

    turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    turnController.setSetpoint(absolutePosition);
  }

  /**
   * gets the position of the steer motor.
   *
   * @return position of the steer motor in rotations.
   */
  public double getPosition() {
    return turnSim.getAngularPositionRotations();
  }

  /** Stops the motors properly. */
  public void stopMotor() {}
  ;
}
