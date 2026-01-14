package frc.robot.commands.swervedrive.drivebase;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAlignTurret extends Command {
    private final TurretSubsystem m_turret;
    // Tolerance is used for checking if we are "aligned enough" (e.g., for an isFinished() condition)
    private static final double ANGLE_TOLERANCE_DEGREES = 1.0; 

    // Store the last valid target angle calculation
    private double m_lastTargetAngle;

    public AutoAlignTurret(TurretSubsystem turret) {
        m_turret = turret;
        addRequirements(m_turret);
    }
    
    @Override
    public void initialize() {
        // Set the initial goal angle to whatever the turret is currently at when the command starts.
        // This prevents a sudden jump on activation.
        m_lastTargetAngle = m_turret.getAngle();
        m_turret.setGoalAngle(m_lastTargetAngle);
    }

    @Override
    public void execute() {
        // Check if Limelight sees any AprilTags (using the default pipeline 'limelight' name)
        if (LimelightHelpers.getTV("limelight")) {
            double tx = LimelightHelpers.getTX("limelight"); // Horizontal offset in degrees
            
            // Calculate the absolute target angle in robot space
            double currentGoal = m_turret.getAngle() + tx;
            
            // Update the last valid target and send it to the PID controller
            m_lastTargetAngle = currentGoal;
            m_turret.setGoalAngle(currentGoal);
        } else {
            // Improvement: If we lose the target, the turret stays aimed at the last 
            // known position using its internal PID, preventing sudden stops/jerkiness.
            // We just don't update the goal angle in this else block.
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure the motor stops when the command finishes or is interrupted
        // The default command will take over shortly after this and keep it stopped.
        m_turret.stop(); 
    }

    @Override
    public boolean isFinished() {
        // This command should run continuously as long as the button is held.
        return false; 
    }
}
