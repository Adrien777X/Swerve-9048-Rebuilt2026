package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDiagonalL2EndWithDrive extends SequentialCommandGroup {


    public AutoDiagonalL2EndWithDrive(
        SwerveSubsystem drivebase,
        boolean isRightScore,
        boolean rightStart
    ) {


        addCommands(

            // wait for safety
            new WaitCommand(0),

            // drive forward 1.3 m for 2.3 seconds
            new RunCommand(
                () -> drivebase.drive(new Translation2d(1.3, 0), 0, false),
                drivebase
            ).withTimeout(2.3),

            // stop
            new InstantCommand(() ->
                drivebase.drive(new Translation2d(0, 0), 0, false)
            ),

            // align to the reef
            new AlignToReefTagRelative(isRightScore, drivebase)
                .withTimeout(4),

            // SHOOT with correct constructor

            // sideways drive
            new RunCommand(
                () -> drivebase.drive(
                    new Translation2d(0, rightStart ? 2 : -2),
                    0,
                    false
                ),
                drivebase
            ).withTimeout(3)
        );
    }
}