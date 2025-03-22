package BobcatLib.Subsystems.Elevators.Commands;

import BobcatLib.Subsystems.Elevators.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class LowerElevator extends Command {
  private final ElevatorSubsystem m_elevator;
  private final double LowerPercentOutput;
  private final double HoldOutput;

  /**
   * Rolls Algae into the intake.
   *
   * @param elevator The subsystem used by this command.
   */
  public LowerElevator(ElevatorSubsystem elevator) {
    this.m_elevator = elevator;
    this.LowerPercentOutput = -0.1;
    this.HoldOutput = 0.05;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_elevator);
  }
  /**
   * Rolls Algae into the intake.
   *
   * @param elevator The subsystem used by this command.
   * @param output The output of the motor used to drive the elevator
   * @param holdOutput the output of hte motor used to hold the position
   */
  public LowerElevator(ElevatorSubsystem elevator, double output, double holdOutput) {
    this.m_elevator = elevator;
    this.LowerPercentOutput = output;
    this.HoldOutput = holdOutput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.runElevator(LowerPercentOutput);
  }

  // Called once the command ends or is interrupted. This ensures the roller is
  // not running when not intented.
  @Override
  public void end(boolean interrupted) {
    m_elevator.holdPositionMechanically(HoldOutput);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
