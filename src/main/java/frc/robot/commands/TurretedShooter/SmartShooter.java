package frc.robot.commands.TurretedShooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.FieldRelativeAccel;
import frc.robot.util.FieldRelativeSpeed;
import frc.robot.util.LinearInterpolationTable;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class SmartShooter extends Command {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final SwerveSubsystem m_drive;
    //private final boolean m_updatePose;
    private final XboxController m_driver;
    private final Timer m_timer = new Timer();

    private static LinearInterpolationTable m_timeTable = ShooterConstants.kTimeTable;
    private static LinearInterpolationTable m_rpmTable = ShooterConstants.kRPMTable;

    public SmartShooter(Shooter shooter, Turret turret, SwerveSubsystem drive, XboxController driver) {
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        //m_updatePose = updatePose;
        m_driver = driver;
        addRequirements(shooter, turret);
    }

    public SmartShooter(Shooter shooter, Turret turret, SwerveSubsystem drive) {
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        //m_updatePose = updatePose;
        m_driver = new XboxController(4);
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

        double currentTime = m_timer.get();

        SmartDashboard.putNumber("Current Time", currentTime);

        SmartDashboard.putBoolean("Shooter Running", true);

        FieldRelativeSpeed robotVel = m_drive.getFieldRelativeSpeed();
        FieldRelativeAccel robotAccel = m_drive.getFieldRelativeAccel();

        Translation2d target = GoalConstants.kGoalLocation;

        Translation2d robotToGoal = target.minus(m_drive.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d()) * 39.37;

        SmartDashboard.putNumber("Calculated (in)", dist);

        //double fixedShotTime = m_timeTable.getOutput(dist);
        double shotTime = m_timeTable.getOutput(dist);

        SmartDashboard.putNumber("Fixed Time", shotTime);

        Translation2d movingGoalLocation = target; //prev== Translation2d movingGoalLocation = new Translation2d();

        for(int i=0;i<5;i++){

            double virtualGoalX = target.getX()
                    - shotTime * (robotVel.vx + robotAccel.ax * ShooterConstants.kAccelCompFactor);
            double virtualGoalY = target.getY()
                    - shotTime * (robotVel.vy + robotAccel.ay * ShooterConstants.kAccelCompFactor);

            SmartDashboard.putNumber("Goal X", virtualGoalX);
            SmartDashboard.putNumber("Goal Y", virtualGoalY);

            Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

            Translation2d toTestGoal = testGoalLocation.minus(m_drive.getPose().getTranslation());

            double newShotTime = m_timeTable.getOutput(toTestGoal.getDistance(new Translation2d()) * 39.37);

            if(Math.abs(newShotTime-shotTime) <= 0.010){
                i=4;
            }
            
            if(i == 4){
                movingGoalLocation = testGoalLocation;
                SmartDashboard.putNumber("NewShotTime", newShotTime);
            }
            else{
                shotTime = newShotTime;
            }

        }

        double newDist = movingGoalLocation.minus(m_drive.getPose().getTranslation()).getDistance(new Translation2d()) * 39.37;

        SmartDashboard.putNumber("NewDist", newDist);

        m_turret.aimAtGoal(m_drive.getPose(), movingGoalLocation, false);

        if (m_turret.atDesiredAngle()) {
            m_shooter.run(m_rpmTable.getOutput(newDist) + SmartDashboard.getNumber("SetShotAdjust", 0));
        } else {
            m_shooter.run(m_rpmTable.getOutput(newDist));
        }

        /*if (currentTime > 0.250 && Limelight.valid() && Limelight.getDistance() >= 85.0) {
            double dL = m_drive.getDistanceToHub(); //Limelight.getDistance() * 0.0254
            double tR = m_drive.getGyro().getRadians();
            double tT = m_turret.getMeasurement() - Math.PI;
            double tL = -1.0 * Limelight.tx();

            Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, GoalConstants.kGoalLocation);

            if (m_updatePose) {
                m_drive.setPose(pose);
            }

        }*/

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

    /*private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal) {
        double tG = tR + tT + tL;
        double rX = goal.getX() - dL * Math.cos(tG);
        double rY = goal.getY() - dL * Math.sin(tG);

        return new Pose2d(rX, rY, new Rotation2d(-tR));
    }*/

}