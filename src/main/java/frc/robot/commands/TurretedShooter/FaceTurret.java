package frc.robot.commands.TurretedShooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.GoalConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret;

public class FaceTurret extends Command {
    private final Turret m_turret;
    private final SwerveSubsystem m_robotDrive;

    public FaceTurret(Turret turret, SwerveSubsystem drive) {
        m_turret = turret;
        m_robotDrive = drive;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_turret.trackTarget(false);
    }

    @Override
    public void execute() {
        m_turret.aimAtGoal(m_robotDrive.getPose(), GoalConstants.kGoalLocation, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }

}