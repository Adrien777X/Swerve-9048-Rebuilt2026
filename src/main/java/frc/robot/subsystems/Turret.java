package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.MathUtils;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import edu.wpi.first.units.measure.Angle;

public class Turret extends SubsystemBase {
  private final SparkFlex m_motor = new SparkFlex(TurretConstants.kIDVortex, MotorType.kBrushless);

  private final double MAX_ONE_DIR_FOV = TurretConstants.kmaxminAngle;
  private boolean m_trackTarget = false;
  private double m_desiredAngleRad = 0.0;

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(100, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(180))
    .withFeedforward(new ArmFeedforward(0, 0, 0.1))
    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 10)))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(0.1))
    .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new SparkWrapper(m_motor, DCMotor.getNeoVortex(1), smcConfig);

  private final PivotConfig turretConfig = new PivotConfig(smc)
      .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV + 5))
      .withStartingPosition(Degrees.of(0))
      .withMOI(0.05)
      .withTelemetry("Turret", TelemetryVerbosity.HIGH);

  private Pivot turret = new Pivot(turretConfig);

  public Turret() {
    SmartDashboard.putBoolean("EnableLimelight", false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Angle (deg)", turret.getAngle().in(Degrees));
    turret.updateTelemetry();
  }

  public void zeroTurret() {
    turret.setAngle(Degrees.of(0));
  }

  public void aimAtGoal(Pose2d robotPose, Translation2d goal, boolean aimAtVision) {
    Translation2d robotToGoal = goal.minus(robotPose.getTranslation());
    double angleRad = Math.atan2(robotToGoal.getY(), robotToGoal.getX()) - robotPose.getRotation().getRadians();
    
    angleRad = MathUtils.toUnitCircAngle(angleRad);

    if (m_trackTarget) {
      Limelight.enable();
    } else {
      Limelight.disable();
    }

    SmartDashboard.putNumber("Turret Set Angle", angleRad);

    if (aimAtVision && Limelight.valid()) {
      angleRad -= Limelight.tx();
    }

    angleRad = MathUtil.clamp(angleRad, TurretConstants.kLow, TurretConstants.kHigh);

    m_desiredAngleRad = angleRad;

    /*if (angleRad < TurretConstants.kLow) {
      angleRad = TurretConstants.kLow;
    } else if (angleRad > TurretConstants.kHigh) {
      angleRad = TurretConstants.kHigh;
    }*/

    turret.setAngle(Radians.of(angleRad));
    //m_controller.setReference(neoRevs, ControlType.kMAXMotionPositionControl);
  }

  /**
   * Function to set the track target boolean to either true or false
   * 
   * @param track is true when Limelight tracking is desired
   */
  public void trackTarget(boolean track) {
    m_trackTarget = track;
  }

  public boolean visionAligned() {
    if (Limelight.valid() && Math.abs(Limelight.tx()) < VisionConstants.kTrackTolerance) {
      return true;
    } else {
      return false;
    }
  }

  public boolean atDesiredAngle() {
    return Math.abs(m_desiredAngleRad - turret.getAngle().in(Radians)) <= TurretConstants.kTolerance;
  }

  public boolean closeToDeadzone() {
    return (m_desiredAngleRad >= TurretConstants.kHigh - TurretConstants.kNearDeadzone
        || m_desiredAngleRad <= TurretConstants.kLow + TurretConstants.kNearDeadzone);
  }

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Command center() {
    return turret.setAngle(Degrees.of(0));
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  public void setToStartPosition() {
    m_desiredAngleRad = TurretConstants.kStartingPositionDegrees;
    turret.setAngle(Degrees.of(TurretConstants.kStartingPositionDegrees));
  }

  public boolean readyToClimb() {
    return Math.abs(m_desiredAngleRad - turret.getAngle().in(Radians)) <= Math.PI/2.0;
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}