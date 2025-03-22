package BobcatLib.Subsystems.Elevators.Modules.Motors;

import BobcatLib.Hardware.Motors.Utility.CTRE.MotionMagicWrapper;
import BobcatLib.Hardware.Motors.Utility.SoftwareLimitWrapper;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class KrakenElevatorMotor implements BaseElevatorMotor {
  private TalonFX mElevatorMotor;
  private TalonFXConfiguration swerveElevatorFXConfig = new TalonFXConfiguration();
  /* Elevator motor control requests */
  private final PositionVoltage ElevatorPosition = new PositionVoltage(0);

  public KrakenElevatorMotor(int id, String canivorename) {
    if (canivorename == "" || canivorename == "rio") {
      mElevatorMotor = new TalonFX(id);
    } else {
      mElevatorMotor = new TalonFX(id, canivorename);
    }

    configElevatorMotor();
  }
  /** Configures the angle motor with default settings. */
  public void configElevatorMotor() {
    /** Swerve Elevator Motor Configurations */
    /* Motor Inverts and Neutral Mode */
    swerveElevatorFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    swerveElevatorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    /* Gear Ratio and Wrapping Config */
    swerveElevatorFXConfig.Feedback.SensorToMechanismRatio = 12;
    swerveElevatorFXConfig.ClosedLoopGeneral.ContinuousWrap = false;
    /* Current Limiting */
    swerveElevatorFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    swerveElevatorFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
    /* PID Config */
    swerveElevatorFXConfig.Slot0.kP = 0.125;
    swerveElevatorFXConfig.Slot0.kI = 0;
    swerveElevatorFXConfig.Slot0.kD = 0;
    /* Open and Closed Loop Ramping */
    swerveElevatorFXConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.0);
    swerveElevatorFXConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(0.25);
    /* Apply Configuration */
    mElevatorMotor.getConfigurator().apply(swerveElevatorFXConfig);
  }
  /**
   * Applies Motion Magic configuration to the elevator motor.
   *
   * @param mmWrapper The Motion Magic configuration wrapper.
   * @return A new BaseElevatorMotor instance with Motion Magic settings applied.
   */
  public BaseElevatorMotor withMotionMagic(MotionMagicWrapper mmWrapper) {
    swerveElevatorFXConfig.MotionMagic = mmWrapper.getMMConfig();
    swerveElevatorFXConfig.Slot0.kG = mmWrapper.getPID_kG();
    swerveElevatorFXConfig.Slot0.kS = mmWrapper.getPID_kS();

    /* Apply Configuration */
    mElevatorMotor.getConfigurator().apply(swerveElevatorFXConfig);
    return this;
  }

  /**
   * Applies software limit settings to the elevator motor.
   *
   * @param limitsWrapper The software limit configuration wrapper.
   * @return A new BaseElevatorMotor instance with software limit settings applied.
   */
  public BaseElevatorMotor withSoftwareLimit(SoftwareLimitWrapper limitsWrapper) {
    SoftwareLimitSwitchConfigs internalConfigs = limitsWrapper.asCTRE();
    // Sets the Upper Range of the Elevator Position
    swerveElevatorFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        internalConfigs.ForwardSoftLimitEnable;
    swerveElevatorFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        internalConfigs.ForwardSoftLimitThreshold;
    // Sets the Lower Range of the Elevator Position
    swerveElevatorFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable =
        internalConfigs.ReverseSoftLimitEnable;
    swerveElevatorFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        internalConfigs.ReverseSoftLimitThreshold;
    /* Apply Configuration */
    mElevatorMotor.getConfigurator().apply(swerveElevatorFXConfig);
    return this;
  }

  /**
   * Sets the angle of the motor using the internal PID controller.
   *
   * @param rotations The desired angle in rotations.
   */
  public void setAngle(double rotations) {
    mElevatorMotor.setControl(ElevatorPosition.withPosition(rotations));
  }
  /**
   * Sets the motor control using a percentage output.
   *
   * @param percent The percentage of power to apply to the motor.
   */
  public void setControl(double percent) {
    mElevatorMotor.set(percent);
  }
  /**
   * Sets the motor control using a VoltageOut object.
   *
   * @param output The voltage output control for the motor.
   */
  public void setControl(VoltageOut output) {
    mElevatorMotor.setControl(output);
  }

  /**
   * Gets the current position of the Elevator motor.
   *
   * @return The position of the Elevator motor in rotations.
   */
  public double getPosition() {
    return mElevatorMotor.getPosition().getValueAsDouble();
  }

  /**
   * Gets the error in position of the steer motor.
   *
   * @return The position error in rotations.
   */
  public double getErrorPosition() {
    return mElevatorMotor.getClosedLoopError().getValueAsDouble();
  }

  /**
   * sets the position of the steer motor given the desired rotations.
   *
   * @param absolutePosition
   */
  public void setPosition(double absolutePosition) {
    mElevatorMotor.setPosition(absolutePosition);
  }

  /** Stops the motors properly. */
  public void stopMotor() {
    mElevatorMotor.stopMotor();
  }
}
