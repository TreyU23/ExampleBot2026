package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase { 
    private final TalonFX m_shooter;
    private final TalonFX m_hood;

    private final TalonFXConfigurator m_shooterConfig;
    private final TalonFXConfigurator m_hoodConfig;

    private final VelocityVoltage m_vvReq = new VelocityVoltage(0.0);
    private final PositionVoltage m_pvReq = new PositionVoltage(0.0);

    private double m_setPosition = 0.0;
    private double m_setVelocity = 0.0;

    private static InvertedValue m_invertedValue;

    private boolean m_isLeft;
    private String m_side;

    public ShooterSubsystem(boolean isLeft) {
        m_isLeft = isLeft;
        m_side = isLeft ? "Left" : "Right";

        m_shooter = new TalonFX(isLeft ? ShooterConstants.kLeftShooterID : ShooterConstants.kRightShooterID);
        m_hood = new TalonFX(isLeft ? ShooterConstants.kLeftHoodID : ShooterConstants.kRightHoodID);

        m_shooterConfig = m_shooter.getConfigurator();
        m_hoodConfig = m_hood.getConfigurator();

        m_invertedValue = isLeft ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        shooterConfigs();
        hoodConfigs();
    }

    public double getPosition() {
        return m_hood.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return m_shooter.getVelocity().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return m_shooter.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_shooter.getSupplyCurrent().getValueAsDouble();
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
        return Math.abs(getPosition() - m_setPosition) < ShooterConstants.kPostionTolerance;
    }

    private void shooterConfigs() {
        m_shooterConfig.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimitConstants.kShooterStatorLimit)
            .withSupplyCurrentLimit(CurrentLimitConstants.kShooterSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true));
        
        m_shooterConfig.apply(new Slot0Configs()
            .withKP(ShooterConstants.kP)
            .withKI(ShooterConstants.kI)
            .withKD(ShooterConstants.kD));

        m_shooterConfig.apply(new MotorOutputConfigs().withInverted(m_invertedValue));
    }

    private void hoodConfigs() {
        m_hoodConfig.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimitConstants.kShooterHoodStatorLimit)
            .withSupplyCurrentLimit(CurrentLimitConstants.kShooterHoodSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true));
        
        m_hoodConfig.apply(new Slot0Configs()
            .withKP(ShooterConstants.Hood.kP)
            .withKI(ShooterConstants.Hood.kI)
            .withKD(ShooterConstants.Hood.kD));

        m_hoodConfig.apply(new MotorOutputConfigs().withInverted(m_invertedValue));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(m_side + " Shooter Velocity", getVelocity());
        SmartDashboard.putNumber(m_side + " Hood Position", getPosition());
        SmartDashboard.putBoolean(m_side + " Hood At Position", isAtPosition());
        SmartDashboard.putNumber(m_side + " Shooter Set Velocity", m_setVelocity);
        SmartDashboard.putNumber(m_side + " Hood Set Position", m_setPosition);
        SmartDashboard.putNumber(m_side + " Shooter Stator Current", getStatorCurrent());
        SmartDashboard.putNumber(m_side + " Shooter Supply Current", getSupplyCurrent());
    }
}
