package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase { 
    private final TalonFX m_shooter;
    private final TalonFX m_hood;

    private final TalonFXConfigurator m_shooterConfig;
    private final TalonFXConfigurator m_hoodConfig;

    private final VelocityVoltage m_vvReq = new VelocityVoltage(0.0);
    private final PositionVoltage m_pvReq = new PositionVoltage(0.0);

    private double m_setPosition = 0.0;
    private double m_setVelocity = 0.0;

    private boolean m_isLeft;

    public ShooterSubsystem(boolean isLeft) {
        m_isLeft = isLeft;
        m_shooter = new TalonFX(isLeft ? 9 : 10);
        m_hood = new TalonFX(isLeft ? 11 : 12);
        m_shooterConfig = m_shooter.getConfigurator();
        m_hoodConfig = m_hood.getConfigurator();

        shooterConfigs();
        hoodConfigs();
    }

    public double getPosition() {
        return m_hood.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return m_shooter.getVelocity().getValueAsDouble();
    }

    public void setPosition(double pose) {
        m_setPosition = Units.degreesToRotations(pose);
        m_hood.setControl(m_pvReq.withPosition(m_setPosition));
    }

    public void setVelocity(double velocity) {
        m_setVelocity = velocity;
        m_shooter.setControl(m_vvReq.withVelocity(m_setVelocity));
    }

    public void stopShooter() {
        m_shooter.stopMotor();
    }

    public void stopHood() {
        m_hood.stopMotor();
    }

    public Command stopCmd() {
        return runOnce(()-> {
            stopShooter();
            stopHood();
        });
    }

    public Command setShooterCmd(double pose, double velocity) {
        return runEnd(()-> {
            setPosition(pose);
            setVelocity(velocity);
        }, ()-> {
            stopShooter();
            stopHood();
        });
    }

    public boolean isLeft() {
        return m_isLeft;
    }

    public boolean isAtPosition() {
        return Math.abs(getPosition() - m_setPosition) < 0.1;
    }

    private void shooterConfigs() {}

    private void hoodConfigs() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
