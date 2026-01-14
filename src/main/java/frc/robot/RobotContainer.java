// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.swervedrive.auto.AutoForward;
import frc.robot.commands.swervedrive.drivebase.AutoAlignTurret;
import swervelib.SwerveInputStream;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
  /*private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  public final SendableChooser<Alliance> allianceChooser = new SendableChooser<>();*/
  DoubleSupplier swerveSpeedScaleTranslation = () -> 1;
	DoubleSupplier swerveSpeedScaleRotation = () -> 1;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final Joystick m_operatorController =
      new Joystick(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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
    m_turret.setDefaultCommand(
        Commands.run(m_turret::stop, m_turret) // Para a torre (função alpha)
    );
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


    m_driverController.povRight().onTrue(new AlignToReefTagRelative(true, drivebase).withTimeout(5));
    m_driverController.povLeft().onTrue(new AlignToReefTagRelative(false, drivebase).withTimeout(5));

    m_driverController.leftTrigger().onTrue(new InstantCommand(() -> {
			swerveSpeedScaleTranslation = () -> 0.3;
			swerveSpeedScaleRotation = () -> 0.2;
		}))
				.onFalse(new InstantCommand(() -> {
					swerveSpeedScaleTranslation = () -> 1;
					swerveSpeedScaleRotation = () -> 1;
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
    //return m_chooser.getSelected();
    return drivebase.getAutonomousCommand("9048");
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
  }
}
