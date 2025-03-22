package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.ModuleConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.math.Conversions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimDriveMotor implements DriveWrapper {
  // TunerConstants doesn't support separate sim constants, so they are declared
  // locally
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT =
      0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private final DCMotorSim driveSim;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  public ModuleConstants chosenModule;
  public ModuleLimitsJson limits;

  public SimDriveMotor(ModuleConstants chosenModule, ModuleLimitsJson limits) {
    this.chosenModule = chosenModule;
    this.limits = limits;
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, 0.01, chosenModule.driveGearRatio),
            DRIVE_GEARBOX);
  }

  public void withSupplyCurrent() {}

  public void withStatorCurrent() {}

  public void updateOutputs() {}

  /**
   * gets the velocity of the drive motor in rotations per second either using percentage output (
   * dutycycleout ) or velocity control triggered by the isOpenLoop parameter
   *
   * @param desiredState desired State
   * @param isOpenLoop open Loop
   */
  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double angularVelocityRadPerSec;
    if (!isOpenLoop) {
      double rotationsPerSeconds =
          Conversions.MPSToRPS(desiredState.speedMetersPerSecond, chosenModule.wheelCircumference);
      angularVelocityRadPerSec = rotationsPerSeconds / (Math.PI * 2);
      driveAppliedVolts = driveFFVolts + driveController.calculate(angularVelocityRadPerSec);
      driveFFVolts =
          DRIVE_KS * Math.signum(angularVelocityRadPerSec) + DRIVE_KV * angularVelocityRadPerSec;
      driveController.setSetpoint(angularVelocityRadPerSec);
    } else {
      driveController.reset();
    }
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
  }

  /**
   * I need to fix this for real , I hate / dislike the new measurement implementation.
   *
   * @return position
   */
  public double getPosition() {
    double pos = driveSim.getAngularPositionRotations();
    return pos;
  }

  /**
   * gets the velocity of the drive motor in rotations per second.
   *
   * @return velocity
   */
  public double getVelocity() {
    double rotPerSec = driveSim.getAngularVelocity().magnitude() / 60;
    return rotPerSec;
  }

  /**
   * Sets up the control mode for SYSID ONLY!
   *
   * @param volts
   */
  public void setControl(double volts) {}

  /**
   * Gets motor Voltage of the given motor
   *
   * @return motorVoltage
   */
  public double getMotorVoltage() {
    return 0;
  }

  /** Stops the motors properly. */
  public void stopMotor() {}
}
