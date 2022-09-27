

package frc.robot.commands.AutoCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TrajectoryFollowingCommand;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestPath2 extends ParallelCommandGroup{
    public TestPath2(BallSubsystem ballSubsystem, DrivetrainSubsystem drivetrainSubsystem, String bluePathName, String redPathName, double speedMultiplier) {
        addCommands(new TestPath2Systems(ballSubsystem));
        if (DriverStation.getAlliance() == Alliance.Blue){
            addCommands(sequence(new WaitCommand(2), new TrajectoryFollowingCommand(drivetrainSubsystem, bluePathName, speedMultiplier)));
          } else {
            addCommands(sequence(new WaitCommand(2), new TrajectoryFollowingCommand(drivetrainSubsystem, redPathName, speedMultiplier)));
          }
          
          addRequirements(ballSubsystem, drivetrainSubsystem);
    }
}
