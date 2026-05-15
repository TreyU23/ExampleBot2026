package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SmartShooterConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SmartShootByPose extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final ShooterSubsystem m_shooter;
    private final TurretSubsystem m_turret; 

    private boolean m_goodToIndex = true;

    public SmartShootByPose(CommandSwerveDrivetrain drivetrain, 
                                ShooterSubsystem shooter, TurretSubsystem turret) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_turret = turret;
        addRequirements(m_drivetrain, m_shooter, m_turret);
    }

    @Override
    public void execute() {
        m_goodToIndex = true;

        Pose2d currentPose = m_drivetrain.getState().Pose;
        Translation2d target = SmartShooterConstants.Blue.kHub;

        Translation2d turretPose = currentPose.getTranslation().plus(
                                    TurretConstants.kTurretOffset.rotateBy(currentPose.getRotation()));

        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

        if (isBlue) {
            if (currentPose.getY() < SmartShooterConstants.Blue.kHub.getY()) {
                if (currentPose.getX() < SmartShooterConstants.Blue.kFeedDepo.getX()) {
                    target = SmartShooterConstants.Blue.kFeedDepo;
                } else {
                    target = SmartShooterConstants.Blue.kFeedOutpost;
                }
            }

            if (Math.abs(turretPose.getX() - SmartShooterConstants.Blue.kHub.getX()) < ((36 / 39.37)
                    / (Math.abs(turretPose.getY() - SmartShooterConstants.Blue.kHub.getY()) / 2.0)))
                        m_goodToIndex = false;

        } else {
            if (currentPose.getY() > SmartShooterConstants.Red.kHub.getY()) {
                if (currentPose.getX() < SmartShooterConstants.Red.kFeedDepo.getX()) {
                    target = SmartShooterConstants.Red.kFeedDepo;
                } else {
                    target = SmartShooterConstants.Red.kFeedOutpost;
                }
            }

            if (Math.abs(turretPose.getX() - SmartShooterConstants.Red.kHub.getX()) < ((36 / 39.37)
                    / (Math.abs(turretPose.getY() - SmartShooterConstants.Red.kHub.getY()) / 2.0)))
                        m_goodToIndex = false;

        }

        ChassisSpeeds chassisSpeeds = m_drivetrain.getState().Speeds;
        ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, currentPose.getRotation());

        currentPose.exp(new Twist2d(chassisSpeeds.vxMetersPerSecond * 0.030, chassisSpeeds.vyMetersPerSecond * 0.030,
                            chassisSpeeds.omegaRadiansPerSecond * 0.030));

        Translation2d turretSpeedVector = new Translation2d(turretPose.getNorm() * fieldRelSpeeds.omegaRadiansPerSecond,
                                            new Rotation2d(currentPose.getRotation().getRadians() + Math.PI / 2.0
                                                + turretPose.getAngle().getRadians()));

        ChassisSpeeds turretFieldSpeeds 
            = new ChassisSpeeds(turretSpeedVector.getX(), turretSpeedVector.getY(), 0).plus(fieldRelSpeeds);

        for (int i = 0; i < 10; i++) {
            Translation2d turretToGoal = target.minus(currentPose.getTranslation().plus(turretPose));

            double distance = turretToGoal.getNorm();
            double shotTime = ShooterConstants.kTimeTable.get(distance);

            double offsetX = -1.0 * turretFieldSpeeds.vxMetersPerSecond * shotTime;
            double offsetY = -1.0 * turretFieldSpeeds.vyMetersPerSecond * shotTime;

            target = target.plus(new Translation2d(offsetX, offsetY));
        };

        Translation2d turretToTarget = target.minus(turretPose);

        double distToTarget = turretToTarget.getNorm();
        double angle = turretToTarget.getAngle().getRadians();

        double shooterRollerSetpoint = ShooterConstants.kVelocityInterpolation.get(distToTarget);
        double shooterHoodSetpoint = ShooterConstants.kHoodInterpolation.get(distToTarget);

        m_shooter.setPosition(shooterHoodSetpoint);
        m_shooter.setVelocity(shooterRollerSetpoint);
        m_turret.setRadians(angle-currentPose.getRotation().getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooter();
        m_shooter.stopHood();
        m_turret.setPosition(0.0);
    }

    public boolean goodToIndex() {
        return m_goodToIndex;
    }
}