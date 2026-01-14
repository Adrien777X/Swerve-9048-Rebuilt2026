package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;
import yams.motorcontrollers.local.SparkWrapper;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class Intake extends SubsystemBase {
    private final SparkMax m_rollerLeader  = new SparkMax(20, MotorType.kBrushless);

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withClosedLoopController(0.00016541, 0, 0, RPM.of(11000), RotationsPerSecondPerSecond.of(183.33))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotorOne", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withFollowers(Pair.of(new SparkMax(21, MotorType.kBrushless), true));

    private final SparkFlex m_intakePivotSparkFlex = new SparkFlex(22, MotorType.kBrushless);

    private final SmartMotorControllerConfig intakePivotConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(10, 0, 0, DegreesPerSecond.of(1080), DegreesPerSecondPerSecond.of(1080))
      .withFeedforward(new ArmFeedforward(0, 0, 1.3))
      .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0, 42)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(10))
      .withClosedLoopRampRate(Seconds.of(0.1));

    //private Arm intakePivot = new Arm(intakePivotConfig);
}