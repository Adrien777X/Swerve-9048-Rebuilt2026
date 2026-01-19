
package frc.robot.commands.TurretedShooter;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LinearInterpolationTable;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RunShooter extends Command {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final SwerveSubsystem m_drive;
    private final boolean m_updatePose;
    private final Timer m_timer = new Timer();
    private final XboxController m_driver;

    private static LinearInterpolationTable m_rpmTable = ShooterConstants.kRPMTable;

    public RunShooter(Shooter shooter, Turret turret, SwerveSubsystem drive, boolean updatePose, XboxController driver) {
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_updatePose = updatePose;
        m_driver = driver;
        addRequirements(shooter, turret);
    }

    public RunShooter(Shooter shooter, Turret turret, SwerveSubsystem drive, boolean updatePose) {
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_updatePose = updatePose;
        m_driver = new XboxController(1);
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        m_turret.trackTarget(true);
        m_timer.reset();
        m_timer.start();
        SmartDashboard.putNumber("SetHoodAdjust", 0.0);
        SmartDashboard.putNumber("SetShotAdjust", 0);
        SmartDashboard.putBoolean("Adjust Shot?", false);
    }

    @Override
    public void execute() {
        //double currentTime = m_timer.get();
        boolean useVision = true;

        Translation2d target = GoalConstants.kGoalLocation;


        SmartDashboard.putBoolean("Shooter Running", true);
        Translation2d robotToGoal = target.minus(m_drive.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d()) * 39.37;
        SmartDashboard.putNumber("Calculated (in)", dist);
        if (Limelight.valid() && useVision) {
            dist = Limelight.getDistance();
            SmartDashboard.putNumber("Limelight (in)", dist);
            if (SmartDashboard.getBoolean("Adjust Shot?", false)) {
                m_shooter.run(m_rpmTable.getOutput(dist) + SmartDashboard.getNumber("SetShotAdjust", 0));
            } else {
                m_shooter.run(m_rpmTable.getOutput(dist));
            }
        } else {

            if (SmartDashboard.getBoolean("Adjust Shot?", false)) {
                m_shooter.run(m_rpmTable.getOutput(dist) + SmartDashboard.getNumber("SetShotAdjust", 0));
            } else {
                m_shooter.run(m_rpmTable.getOutput(dist));
            }

        }
        m_turret.aimAtGoal(m_drive.getPose(), target, useVision);

        if (Limelight.valid() && m_turret.atDesiredAngle()) {
            double dL = Limelight.getDistance() * 0.0254;
            double tR = m_drive.getGyro().getRadians();
            double tT = m_turret.getAngle().in(Radians) - Math.PI;
            double tL = Math.toRadians(-Limelight.tx());

            Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, GoalConstants.kGoalLocation);

            if (m_updatePose) {
                m_drive.setPose(pose);
            }

        }

        if (m_turret.closeToDeadzone()) {
            m_driver.setRumble(RumbleType.kLeftRumble, 1.0);
            m_driver.setRumble(RumbleType.kRightRumble, 1.0);
        } else {
            m_driver.setRumble(RumbleType.kLeftRumble, 0.0);
            m_driver.setRumble(RumbleType.kRightRumble, 0.0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter Running", false);
        m_turret.trackTarget(false);
        m_turret.stop();
        m_shooter.stopCommand();
        m_timer.stop();
        m_driver.setRumble(RumbleType.kLeftRumble, 0.0);
        m_driver.setRumble(RumbleType.kRightRumble, 0.0);
    }

    private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal) {
        double tG = tR + tT + tL;
        double rX = goal.getX() - dL * Math.cos(tG);
        double rY = goal.getY() - dL * Math.sin(tG);

        return new Pose2d(rX, rY, new Rotation2d(-tR));
    }

}
