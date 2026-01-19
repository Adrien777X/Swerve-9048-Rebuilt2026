// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import frc.robot.util.LinearInterpolationTable;

import java.awt.geom.Point2D;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = 90 * 0.453592;
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(11.2)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13;
  public static final double MAX_SPEED = Units.feetToMeters(15);

  public static final class GoalConstants {
    public static final Translation2d kGoalLocation = new Translation2d(4.115, 4.115);
    public static final Translation2d kWrongBallGoal = new Translation2d(3.00, 4.115);
    //public static final Translation2d kHangerLocation = new Translation2d(2.00, 6.00);

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT = 6;
  }

  public static class DriveConstants {
    public static final double kInnerDeadband = 0.10;
    public static final double kOuterDeadband = 0.98;

    public static final double kMaxSpeedMetersPerSecond = 3.25; // Maximum Sustainable Drivetrain Speed under Normal
                                                                // Conditions & Battery, Robot will not exceed this
                                                                // speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);
  }

  public static final class IntakeConstants {
    public static int kRollerMotorIdLeader = 30;
    public static int kRollerMotorIdFollower = 31;
    public static int kPivotMotorId = 32;
  }

  public static class HopperConstants {
    public static final int kHopperMotorId = 40;
  }

  public static class KickerConstants {
    public static final int kKickerMotorId = 50;
  }

  public static final class TurretConstants {
    public static final int kIDVortex = 31;
    public static final double kmaxminAngle = 120;
    public static final double kMotorToEncoderRatio = 3.0;
    public static final double kEncoderGearToTurretGearRatio = 6.296; //170.0 / 27.0
    public static final double TotalReduction = kMotorToEncoderRatio * kEncoderGearToTurretGearRatio;
    public static final double kTolerance = 2 * 0.0349; // allowable angle error in radians for the PIDSubsystem to
                                                        // report atSetpoint() to true
    public static final double kLow = 0.383; // Minimum angle in radians allowed (defines the turret deadzone)
    public static final double kHigh = 5.90; // Maximum angle in radians allowed (defines the turret deadzone)
    public static final double kAccel = 15000;
    public static final double kMaxVel = 5000;
    public static final double kMotionTol = 0.2;
    public static final double kRatio = 0.0171875;
    public static final double kStartingPositionDegrees = 180.0;
    public static final float kLowLimit = (float) (kLow / kRatio / 2.0 / Math.PI - 1.0);
    public static final float kHighLimit = (float) (kHigh / kRatio / 2.0 / Math.PI + 1.0);
    public static final double kNearDeadzone = 0.20;
  }

  public static final class ShooterConstants {
    public static final int[] kMotorIDs = { 12, 13 }; // CANID of the Motor Controller for the Sooter Motor
    public static final double kRPMTolerance = 200.0; // RPMs of error allowed before a ball can be fed into t he
                                                      // shooter
    public static final double[] kPID = { 0.0001, 0.0005, 0 }; // Defines PID values for the shooter 0.00045
    public static final double kIntRange = 0.015;
    public static final double kStatic = 0.018;
    public static final double kFF = 0.00016;
    public static final double kAccelCompFactor = 0.100; // in units of seconds
    public static final double kMaxRPM = 3600.0;
    public static final double kMaxNegPower = -0.30;

    public static final double kHangarRPM = 1200;
    private static final Point2D[] kRPMPoints = new Point2D.Double[] {
        // (ty-angle,distance)
        new Point2D.Double(35, 1500+10),
        new Point2D.Double(55, 1860+10),
        new Point2D.Double(80, 2000+10), //
        new Point2D.Double(105, 2100+10), //
        new Point2D.Double(130, 2170+20), //
        new Point2D.Double(155, 2245+30), //
        new Point2D.Double(165, 2380), //
        new Point2D.Double(180, 2465+30), //
        new Point2D.Double(205, 2670+30), //
        new Point2D.Double(230, 2840+35), //
        new Point2D.Double(255, 2980+40), //
        new Point2D.Double(270, 3300), //
        new Point2D.Double(280, 3350+60)

    };

    public static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPMPoints);

    private static final Point2D[] kShotTimes = new Point2D.Double[] {
        // (ty-angle,time)
        new Point2D.Double(80, 0.78),
        new Point2D.Double(130, 0.80),
        new Point2D.Double(190, 0.81),
        new Point2D.Double(240, 0.82),
        new Point2D.Double(280, 0.83)
    };

    public static final LinearInterpolationTable kTimeTable = new LinearInterpolationTable(kShotTimes);
  }
  // Auto constants
  public static final double MAX_ALIGN_TRANSLATION_SPEED = 0.8; // m/s
  public static final double MAX_ALIGN_ROTATION_SPEED = 1.5;    // rad/s

	public static final double X_REEF_ALIGNMENT_P = 2.7;
	public static final double Y_REEF_ALIGNMENT_P = 2.7;
	public static final double ROT_REEF_ALIGNMENT_P = 0.094;

	public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.7;
	public static final double X_SETPOINT_REEF_ALIGNMENT = 1.20;  // Vertical pose
	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
	public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;

  public static final class VisionConstants {
    public static final double kElevationOffset = 38.5; //degree offset of lens from horizontal due to camera mount
    public static final double kAzimuthalAngle = 0.0; //degree Azimuthal offset of limelight
    public static final double kTargetCenterHeightFromLens = 81.0; //Center Height of the target in inches above the lens

    public static final double kTrackTolerance = 0.02000; //Allowable limelight angle error in radians
  }
}
