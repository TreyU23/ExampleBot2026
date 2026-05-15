package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class JumpIntakeCommand extends Command {
    private final IntakeSubsystem m_intake;

    private final CommandXboxController m_driverController;

    private static boolean m_firstLoop = true;
    private static double m_downSetpoint = 0.0;

    private static final Timer m_time = new Timer();
    
    public JumpIntakeCommand(IntakeSubsystem intake, CommandXboxController driver) {
        m_intake = intake;
        m_driverController = driver;
        addRequirements(m_intake);
    }
    
    @Override
    public void initialize() {
        m_downSetpoint = 0.0;
        m_firstLoop = true;

        m_time.reset();
        m_time.start();
    }

    @Override
    public void execute() {
        double currentTime = m_time.get();
        boolean driver = m_driverController.y().getAsBoolean();

        if (driver) m_firstLoop = false;

        if (m_firstLoop) {
            m_intake.setPosition(m_downSetpoint);
        } else {
            m_intake.setVoltage(2);
            if (currentTime < 0.3) {
                m_intake.setPosition(m_downSetpoint);
            } else if (currentTime <= 0.6 && currentTime >= 0.3) {
                m_intake.setPosition(IntakeConstants.Arm.kRestingPosition);
            } else if (currentTime >= 0.6) {
                if (m_downSetpoint < IntakeConstants.Arm.kRestingPosition) {
                    m_downSetpoint -= 0.025;
                }
                m_time.reset();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_time.stop();
        m_intake.stopArm();
        m_intake.stopRoller();
        m_intake.setPosition(IntakeConstants.Arm.kRestingPosition);
    }
}
