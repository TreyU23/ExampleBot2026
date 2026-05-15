package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.KickerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class IndexCommand extends Command {
    private static TurretSubsystem m_turret;
    private static IndexerSubsystem m_indexer;
    private static KickerSubsystem m_kicker;

    private static SmartShootByPose m_smartShooter;

    private static boolean m_goodToIndex;

    public IndexCommand(TurretSubsystem turret, IndexerSubsystem indexer, 
                                KickerSubsystem kicker, SmartShootByPose smartShooter) {
        m_turret = turret;
        m_indexer = indexer;
        m_kicker = kicker;
        m_smartShooter = smartShooter;
        addRequirements(turret, indexer, kicker);
    }

    @Override
    public void execute() {
        m_goodToIndex = m_turret.atSetpoint() && m_smartShooter.goodToIndex();

        if (m_goodToIndex) {
            m_indexer.setVelocity(IndexerConstants.kDefaultVelocity);
            m_kicker.setVelocity(KickerConstants.kDefaultVelocity);
        } else {
            m_indexer.stop();
            m_kicker.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_indexer.stop();
        m_kicker.stop();
    }
}
