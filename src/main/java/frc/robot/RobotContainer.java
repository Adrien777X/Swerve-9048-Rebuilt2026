// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private boolean isAutoAlignEnabled = false;

  private static final Angle STOW_ANGLE = Degrees.of(0);
  private static final Angle DEPLOY_ANGLE = Degrees.of(148);
  private static final Angle HOLD_ANGLE = Degrees.of(115); // Example middle position
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final SendableChooser<Command> autoChooser;
  //public final SendableChooser<Alliance> allianceChooser = new SendableChooser<>();
  DoubleSupplier swerveSpeedScaleTranslation = () -> 1;
	DoubleSupplier swerveSpeedScaleRotation = () -> 1;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(10));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    /*m_chooser.setDefaultOption(null, driveFieldOrientedAngularVelocity);
    m_chooser.addOption("forward", new AutoForward(drivebase));
    m_chooser.addOption("forward", new AutoForward(drivebase));
    SmartDashboard.putData("Auto Chooser", m_chooser);
    
    allianceChooser.setDefaultOption("blue", Alliance.Blue);
    allianceChooser.addOption("red", Alliance.Red);
    SmartDashboard.putData("Alliance Chooser", allianceChooser);*/
    //resetar ODOMETriA Teste NãO AtIVaR
    //Controller1.back().onTrue(new InstantCommand(() -> drive.setPose(new Pose2d(1.30, 5.55, Rotation2d.fromDegrees(0.0)))));
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    //m_turret.setDefaultCommand(
    //    Commands.run(m_turret::stop, m_turret) // Para a torre (função alpha)
    //);
  }
  

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> m_driverController.getLeftY() * 0.4 * swerveSpeedScaleTranslation.getAsDouble(), //valor y do manche esquerdo do joystick prev era -1 antes 0.4
                                                                        () -> m_driverController.getLeftX() * 0.4 * swerveSpeedScaleTranslation.getAsDouble()) //valor x do manche esquerdo do joystick antes 0.4
                                                                        .withControllerRotationAxis(() -> m_driverController.getRightX() * -0.5 * swerveSpeedScaleRotation.getAsDouble()) //valor x do manche direita do joystick antes -0.5
                                                                        .deadband(OperatorConstants.DEADBAND)     
                                                                        .scaleTranslation(0.8)               
                                                                        .allianceRelativeControl(true);       
                                                                        
  SwerveInputStream driveDriectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX, 
                                                                                            m_driverController::getRightY)
                                                                                            .headingWhile(true);
  
  Command driveFieldOrientedDriectAngle = drivebase.driveFieldOriented(driveDriectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //teste heading maintain w/ Yagsl
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDriectAngle);
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);


    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    m_driverController.povUp().onTrue((Commands.runOnce(()->drivebase.zeroGyro(), drivebase)));

    m_driverController.leftTrigger().onTrue(new InstantCommand(() -> {
			swerveSpeedScaleTranslation = () -> 0.3;
			swerveSpeedScaleRotation = () -> 0.2;
		}))
				.onFalse(new InstantCommand(() -> {
					swerveSpeedScaleTranslation = () -> 1;
					swerveSpeedScaleRotation = () -> 1;
				}));

    new Trigger(
        () -> m_operatorController.getRightTriggerAxis() > 0.1 // Use a small threshold
    ).whileTrue(m_IntakeSubsystem.intakeCommand());

    new Trigger(
        () -> m_operatorController.getLeftTriggerAxis() > 0.1 // Use a small threshold
    ).whileTrue(m_IntakeSubsystem.ejectCommand());

    m_operatorController.povUp().onTrue(m_IntakeSubsystem.setPivotAngle(STOW_ANGLE)); 

    // D-Pad Down (180 degrees POV) moves the intake down (deployed)
    m_operatorController.povDown().onTrue(m_IntakeSubsystem.setPivotAngle(DEPLOY_ANGLE));
        
    // D-Pad Right for a "hold" position
    m_operatorController.povRight().onTrue(m_IntakeSubsystem.setPivotAngle(HOLD_ANGLE));

    // D-Pad Left to rezero the encoder
    m_operatorController.povLeft().onTrue(m_IntakeSubsystem.rezero());

    m_operatorController.a().onTrue(new InstantCommand(() -> {
            isAutoAlignEnabled = !isAutoAlignEnabled;
            System.out.println("Turret Auto Align Enabled: " + isAutoAlignEnabled);
        }));
  }

  public void resetEncoderPositions() {
		drivebase.zeroGyro();
	}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //return drivebase.getAutonomousCommand("9048");
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
  }
  public SwerveDrive getSwerveDrive() {
    return drivebase.getSwerveDrive();
  }

  public Pose2d getRobotPose() {
    return drivebase.getPose();
  }
}
