
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Map;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Shooter extends SubsystemBase {
    private final SparkMax m_motor1 = new SparkMax(33, MotorType.kBrushless);
    private final SparkMax m_motor2 = new SparkMax(34, MotorType.kBrushless);
    private final RelativeEncoder m_encoder1;
    private final RelativeEncoder m_encoder2;
    private final PIDController m_PID = new PIDController(ShooterConstants.kPID[0], ShooterConstants.kPID[1],
            ShooterConstants.kPID[2]);
    private SimpleMotorFeedforward m_FF = new SimpleMotorFeedforward(ShooterConstants.kStatic, ShooterConstants.kFF);

    private double temp_distance = 0.0;

    private double m_RPM = ShooterConstants.kMaxRPM;

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withFollowers(Pair.of(m_motor2, false))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(m_PID)
        .withFeedforward(m_FF)
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    private final SmartMotorController smc = new SparkWrapper(m_motor1, DCMotor.getNEO(2), smcConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(1))
        .withUpperSoftLimit(RotationsPerSecond.of(3600.0 / 60.0))
        .withLowerSoftLimit(RotationsPerSecond.of(0))
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    private final FlyWheel shooter = new FlyWheel(shooterConfig);

    public Shooter() {

        m_encoder1 = m_motor1.getEncoder();
        m_encoder2 = m_motor2.getEncoder();

        m_PID.setTolerance(ShooterConstants.kRPMTolerance);
        //m_PID.setIntegratorRange(0, 0);

        m_PID.setIntegratorRange(-ShooterConstants.kIntRange, ShooterConstants.kIntRange);

    }

    public void run(double rpm) {
        /*if (rpm >= ShooterConstants.kMaxRPM) {
            rpm = ShooterConstants.kMaxRPM;
        }*/
        m_RPM = rpm;
        /*double outputPID = m_PID.calculate(m_encoder1.getVelocity(), m_RPM);
        double outputFF = m_FF.calculate(m_RPM);
        double output = outputPID + outputFF;

        if (output <= ShooterConstants.kMaxNegPower) {
            output = ShooterConstants.kMaxNegPower;
        }*/
        rpm = Math.min(rpm, ShooterConstants.kMaxRPM);
        shooter.setSpeed(RotationsPerSecond.of(rpm / 60.0));
    }

    public Command shootAtDistance(double distanceMeters) {
        return run(() -> {
        temp_distance = SHOOTING_SPEED_BY_DISTANCE.get(distanceMeters);

        shooter.setSpeed(RotationsPerSecond.of(temp_distance));
        });
    }

    public Command stop() {
        return shooter.setSpeed(RotationsPerSecond.of(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM1", m_encoder1.getVelocity());
        SmartDashboard.putNumber("Shooter RPM2", m_encoder2.getVelocity());
        SmartDashboard.putNumber("Shooter Error", m_RPM-m_encoder1.getVelocity());

    }

    public boolean atSetpoint() {
        return m_PID.atSetpoint();
    }
    // m / rps
    private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(2.63, 0.55),
      Map.entry(3.4, 0.6),
      Map.entry(4.83, 0.69));
}
