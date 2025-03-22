package BobcatLib.Subsystems.Elevators.Commands;

import BobcatLib.Subsystems.Elevators.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToSetPoint extends Command {
  private final ElevatorSubsystem m_elevator;
  private final double currentSetPoint;

  /**
   * Moves the Elevator to the provided set point.
   *
   * @param elevator The subsystem used by this command.
   * @param setPoint The setpoint to set as the current SetPoint
   */
  public MoveToSetPoint(ElevatorSubsystem elevator, double setPoint) {
    this.m_elevator = elevator;
    this.currentSetPoint = setPoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.runElevator();
  }

  // Called once the command ends or is interrupted. This ensures the roller is
  // not running when not intented.
  @Override
  public void end(boolean interrupted) {
    m_elevator.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
