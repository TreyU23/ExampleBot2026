// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.constants.DriverConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(DriverConstants.kDriver);

  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem(()-> true);
  private final KickerSubsystem m_kicker = new KickerSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem(()-> -m_drivetrain.getState().Speeds.omegaRadiansPerSecond);
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();

  private final SmartShootByPose m_smartShoot = new SmartShootByPose(m_drivetrain, m_shooter, m_turret);
  private final IndexCommand m_indexCommand = new IndexCommand(m_turret, m_indexer, m_kicker, m_smartShoot);
  private final JumpIntakeCommand m_jumpIntake = new JumpIntakeCommand(m_intake, m_driverController);


  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
    configureNamedCommands();

    m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void configureDefaultCommands() {
    m_drivetrain.setDefaultCommand(
        DriveCommands.fieldOrientedDrive(m_drivetrain, 
            () -> m_driverController.getLeftY(), 
                () -> m_driverController.getLeftX(), 
                    () -> m_driverController.getRightX()));
  }

  private void configureBindings() {
    m_driverController.start().onTrue(DriveCommands.resetFieldOrientation(m_drivetrain));

    m_driverController.leftBumper()
        .whileTrue(m_intake.runIntakeCmd(()-> m_drivetrain.getState().Speeds.vxMetersPerSecond)
          .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    m_driverController.rightTrigger().whileTrue(m_indexCommand.alongWith(m_jumpIntake));

    m_driverController.leftTrigger().whileTrue(m_smartShoot);
  }

  private void configureNamedCommands() {
    new EventTrigger("SmartShoot").whileTrue(m_smartShoot.asProxy());

    new EventTrigger("Index").whileTrue(m_indexCommand);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}