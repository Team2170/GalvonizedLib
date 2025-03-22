package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor;

import BobcatLib.Logging.Alert;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.ModuleConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.math.Conversions;
import BobcatLib.Utilities.CANDeviceDetails;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class KrakenDriveMotor implements DriveWrapper {
  private TalonFX mDriveMotor;
  public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
  /* Feed Forward Setup */
  private final SimpleMotorFeedforward driveFeedForward;
  /** An {@link Alert} for if the CAN ID is greater than 40. */
  public static final Alert canIdWarning =
      new Alert(
          "JSON",
          "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
          Alert.AlertType.WARNING);

  public ModuleConstants chosenModule;
  public ModuleLimitsJson limits;
  public CANDeviceDetails details;

  public KrakenDriveMotor(
      CANDeviceDetails details,
      int id,
      ModuleConstants chosenModule,
      String canivorename,
      ModuleLimitsJson limits) {
    this.details = details;
    if (id >= 40) {
      canIdWarning.set(true);
    }
    this.chosenModule = chosenModule;
    this.limits = limits;
    double driveKS = 0.00;
    double driveKV = 0.00;
    double driveKA = 0.0;
    driveFeedForward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);
    /* Drive Motor Config */
    if (canivorename == "" || canivorename == "rio") {
      mDriveMotor = new TalonFX(id);
    } else {
      mDriveMotor = new TalonFX(id, canivorename);
    }
    configDriveMotor();
    mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
    mDriveMotor.getConfigurator().setPosition(0.0);
  }

  /** Updates the Motor Outputs */
  public void updateOutputs() {
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/Drive/MotorVoltage",
        getMotorVoltage());
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/Drive/RelativePosition",
        Rotation2d.fromRotations(getPosition()));
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/Drive/Velocity",
        getVelocity());
  }

  public void configDriveMotor() {
    /** Swerve Drive Motor Configuration */
    /* Motor Inverts and Neutral Mode */
    swerveDriveFXConfig.MotorOutput.Inverted = chosenModule.driveMotorInvert.asCTRE();
    swerveDriveFXConfig.MotorOutput.NeutralMode =
        chosenModule.driveNeutralMode.asNeutralModeValue();

    /* Gear Ratio Config */
    swerveDriveFXConfig.Feedback.SensorToMechanismRatio = chosenModule.driveGearRatio;

    /* PID Config */
    swerveDriveFXConfig.Slot0.kP = chosenModule.drivePID.kP;
    swerveDriveFXConfig.Slot0.kI = chosenModule.drivePID.kI;
    swerveDriveFXConfig.Slot0.kD = chosenModule.drivePID.kD;

    /* Open and Closed Loop Ramping */
    swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = chosenModule.json.openLoopRamp;
    swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = chosenModule.json.openLoopRamp;

    swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        chosenModule.json.closedLoopRamp;
    swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        chosenModule.json.closedLoopRamp;
  }

  public void withSupplyCurrent() {
    /* Current Limiting */
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        chosenModule.json.driveEnableSupplyCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit =
        chosenModule.json.driveSupplyCurrentLimit;
  }

  public void withStatorCurrent() {
    /* Current Limiting */
    swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable =
        chosenModule.json.driveEnableStatorCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit =
        chosenModule.json.driveStatorCurrentLimit;
  }

  /**
   * gets the velocity of the drive motor in rotations per second either using percentage output (
   * dutycycleout ) or velocity control triggered by the isOpenLoop parameter
   *
   * @param desiredState
   * @param isOpenLoop
   */
  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double maxSpeed = limits.maxSpeed;
    if (isOpenLoop) {
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / maxSpeed;
      mDriveMotor.setControl(driveDutyCycle);
    } else {
      driveVelocity.Velocity =
          Conversions.MPSToRPS(desiredState.speedMetersPerSecond, chosenModule.wheelCircumference);
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      mDriveMotor.setControl(driveVelocity);
    }
  }

  /**
   * gets the position of the drive motor in rotations
   *
   * @return position
   */
  public double getPosition() {
    return mDriveMotor.getPosition().getValueAsDouble();
  }

  /**
   * gets the velocity of the drive motor in rotations per second.
   *
   * @return velocity
   */
  public double getVelocity() {
    return mDriveMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Sets up the control mode for SYSID ONLY!
   *
   * @param volts
   */
  public void setControl(double volts) {
    VoltageOut sysidControl = new VoltageOut(0);
    sysidControl.withOutput(volts);
    mDriveMotor.setControl(sysidControl);
  }

  /** Gets motor Voltage of the given motor */
  public double getMotorVoltage() {
    return mDriveMotor.getMotorVoltage().getValueAsDouble();
  }

  /** Stops the motors properly. */
  public void stopMotor() {
    mDriveMotor.stopMotor();
  }
}
