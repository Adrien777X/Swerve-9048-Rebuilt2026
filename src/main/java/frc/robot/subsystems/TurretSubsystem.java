package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
 
    private final SparkFlex m_motor = new SparkFlex(31, MotorType.kBrushless);

    //private final RelativeEncoder m_encoder;
    private final RelativeEncoder m_encoder; 

    private final double kMinAngle = -180.0;
    private final double kMaxAngle = 180.0;
    //private final double kGearRatio = 30.0;
    private final double kEncoderGearRatio = 1.0;
    private final double kAngleOffsetDegrees = 0.0;

    public TurretSubsystem() {
        //m_encoder = m_motor.getEncoder();
        m_encoder = m_motor.getExternalEncoder();
        
        SparkFlexConfig config = new SparkFlexConfig();
        //config.encoder.positionConversionFactor(360.0 / kGearRatio);
        config.absoluteEncoder.positionConversionFactor(360.0 / kEncoderGearRatio);
        config.absoluteEncoder.zeroOffset(kAngleOffsetDegrees);

        config.closedLoop.pid(0.01, 0, 0);

        config.softLimit.forwardSoftLimit(kMaxAngle).forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(kMinAngle).reverseSoftLimitEnabled(true);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    public void stop() {
        m_motor.set(0.0);
    }

    public void setGoalAngle(double targetAngle) {
        double currentAngle = getAngle();
        double error = targetAngle - currentAngle;

        if (error > 180) targetAngle -= 360;
        else if (error < -180) targetAngle += 360;

        targetAngle = MathUtil.clamp(targetAngle, kMinAngle, kMaxAngle);

        // Deprecated ->>> m_motor.getClosedLoopController().setReference(targetAngle, ControlType.kPosition);
        m_motor.getClosedLoopController().setSetpoint(targetAngle, ControlType.kPosition);
    }

    public void manualControl(double speed) {
        m_motor.set(speed * 0.5);
    }
}
