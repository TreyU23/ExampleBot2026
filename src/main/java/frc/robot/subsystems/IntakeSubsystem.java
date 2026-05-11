package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANBus m_canBus = new CANBus("*");

    private final TalonFX m_roller = new TalonFX(IntakeConstants.kRollerCanID, m_canBus);
    private final TalonFX m_arm = new TalonFX(IntakeConstants.kArmCanID, m_canBus);

    private final CANcoder m_armCANCoder = new CANcoder(IntakeConstants.kArmCANCoder);

    private final TalonFXConfigurator m_rollerConfig = m_roller.getConfigurator();
    private final TalonFXConfigurator m_armConfig = m_arm.getConfigurator();

    private final PositionVoltage m_pvReq = new PositionVoltage(0.0);

    private static double m_setVoltage = 0.0;
    private static double m_setPosition = 0.0;
    
    public IntakeSubsystem() {
        configureRoller();
        configureArm();
    }

    public double getVelocity() {
        return m_roller.getVelocity().getValueAsDouble();
    }

    public double getPosition() {
        return m_arm.getPosition().getValueAsDouble();
    }

    public void setVoltage(double volts) {
        m_setVoltage = volts;
        m_roller.setVoltage(m_setVoltage);
    }

    public void setPosition(double pose) {
        m_setPosition = pose; 

        if (m_setPosition > IntakeConstants.kArmMaxPosition) {
            m_setPosition = IntakeConstants.kArmMaxPosition;
        } else if (m_setPosition < IntakeConstants.kArmMinPosition) {
            m_setPosition = IntakeConstants.kArmMinPosition;
        }

        double rot = Units.degreesToRotations(m_setPosition);
        m_arm.setControl(m_pvReq.withPosition(rot));
    }

    public void stopRoller() {
        m_roller.stopMotor();
    }

    public void stopArm() {
        m_arm.stopMotor();
    }

    public Command setVoltageCmd(double volts) {
        return runOnce(()-> setVoltage(volts));
    }

    public Command setPositionCmd(double pose) {
        return runOnce(()-> setPosition(pose));
    }

    public Command stopCmd() {
        return runOnce(()-> {
            stopRoller();
            stopArm();
        });
    }

    public Command runIntakeCmd() {
        return runEnd(()-> {
            setVoltage(IntakeConstants.kRollerMaxVoltage);
            setPosition(IntakeConstants.kArmIntakePosition);
        }, ()-> {
            stopRoller();
            stopArm();
        });
    }

    private void configureRoller() {
        m_rollerConfig.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimitConstants.kIntakeRollerStatorLimit)
            .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeRollerSupplyLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        m_roller.setNeutralMode(NeutralModeValue.Coast);
    }

    private void configureArm() {
        m_armConfig.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimitConstants.kIntakeArmStatorLimit)
            .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeArmSupplyLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        m_armConfig.apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(Units.degreesToRotations(IntakeConstants.kArmMinPosition))
            .withReverseSoftLimitThreshold(Units.degreesToRotations(IntakeConstants.kArmMaxPosition))
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true));

        m_armConfig.apply(new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKP(IntakeConstants.kP)
            .withKI(IntakeConstants.kI)
            .withKD(IntakeConstants.kD)
            .withKS(IntakeConstants.kS)
            .withKG(IntakeConstants.kG));

        m_armConfig.apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(IntakeConstants.kArmGearRatio)
            .withRemoteCANcoder(m_armCANCoder));

        m_arm.setNeutralMode(NeutralModeValue.Brake);

        m_armCANCoder.getConfigurator()
            .apply(new MagnetSensorConfigs()
                .withMagnetOffset(0.225));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Roller Velocity", getVelocity());
        SmartDashboard.putNumber("Intake Roller Set Voltage", m_setVoltage);
        SmartDashboard.putNumber("Intake Roller Stator", m_roller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Roller Supply", m_roller.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Intake Arm Position", Units.rotationsToDegrees(getPosition()));
        SmartDashboard.putNumber("Intake Arm Setpoint", m_setPosition);
        SmartDashboard.putNumber("Intake Arm Stator", m_arm.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Arm Supply", m_arm.getSupplyCurrent().getValueAsDouble());
    }
}
