
package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;

public class TestPath2Systems extends CommandBase {

  BallSubsystem ballSubsystem;

  int counter = 0;

  public TestPath2Systems(BallSubsystem subsystem) {

    ballSubsystem = subsystem;
    addRequirements(subsystem);

  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter < 250) {
      ballSubsystem.openIntake();
    } else if (counter > 250) {
      ballSubsystem.closeIntake();
    }
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballSubsystem.stopShooter();
    ballSubsystem.closeIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter == 750;
  }
}
