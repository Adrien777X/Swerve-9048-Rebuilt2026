package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoForward extends SequentialCommandGroup {
    public AutoForward(SwerveSubsystem drivebase) {
        addCommands(
          new InstantCommand(()-> drivebase.resetOdometry(new Pose2d(0,0,drivebase.getHeading()))),
          new WaitCommand(0),
          new RunCommand(() -> drivebase.drive(new Translation2d(1.3,0), 0, false), drivebase).withTimeout(2),
          new InstantCommand(()->drivebase.drive(new Translation2d(0,0), 0,false)));
      }
}
