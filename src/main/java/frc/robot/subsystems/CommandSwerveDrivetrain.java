package frc.robot.subsystems;

import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;
    private final Field2d m_field = new Field2d();
    private boolean m_useMT2 = true;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        configureAutoBuilder();
        SmartDashboard.putData("field", m_field);
    }

    @SuppressWarnings("UseSpecificCatch")
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto

                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(6.5, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(6.5, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        updateWithVision();
        seedMegaTag2();

        m_field.setRobotPose(getState().Pose);

        double[] driveposeArray = {
                getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation().getRadians()};
                
        SmartDashboard.putNumberArray("Pose", driveposeArray);

        double abs_speed = Math.sqrt(Math.pow(getState().Speeds.vxMetersPerSecond, 2) + Math.pow(getState().Speeds.vyMetersPerSecond, 2));

        SmartDashboard.putNumber("Speed of Drive", abs_speed);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, 
            Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    public void seedMegaTag2() {
        if (getState().Speeds.omegaRadiansPerSecond <= Math.PI) {
            LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelight, 
                getState().Pose.getRotation().getDegrees(),
                     0, 0, 0, 0, 0);
        }
    }

    public void updateWithVision() {
        PoseEstimate megaTagOne = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelight);
        PoseEstimate megatagTwo = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelight);

        if (LimelightHelpers.validPoseEstimate(megaTagOne)
                && LimelightHelpers.validPoseEstimate(megatagTwo)
                    && getState().Speeds.omegaRadiansPerSecond <= Math.PI) {

            double tagArea = megaTagOne.avgTagArea;
            double timestamp = megaTagOne.timestampSeconds;
            double tagCount = megaTagOne.tagCount;

            SmartDashboard.putNumber("tagCount " + VisionConstants.kLimelight, tagCount);
            SmartDashboard.putNumber("TA " + VisionConstants.kLimelight, tagArea);

            if (m_useMT2) seedVisionPose(megaTagOne, megatagTwo, tagArea, timestamp, tagCount);
            else addVisionMeasurement(megaTagOne.pose, timestamp);
        }
    }

    private void seedVisionPose(PoseEstimate MT1Pose, PoseEstimate MT2Pose, 
                                    double tagArea, double timestamp, double tagCount) {

        if (tagArea > VisionConstants.kMaxTA && tagCount >= 2) {
            addVisionMeasurement(
                MT1Pose.pose, timestamp, 
                    VecBuilder.fill(
                        VisionConstants.kMultiTagXY.get(tagArea),
                            VisionConstants.kMultiTagXY.get(tagArea), 
                                VisionConstants.kMultiTagRot.get(tagArea)));

        } else if (tagArea <= VisionConstants.kMaxTA && 
                    tagCount >= 2 && tagArea >= VisionConstants.kMinTA) {
            addVisionMeasurement(MT2Pose.pose, timestamp, 
                VecBuilder.fill(
                    VisionConstants.kMegaTagTwo.get(tagArea),
                        VisionConstants.kMegaTagTwo.get(tagArea), 
                            VisionConstants.kDontTrust));

            addVisionMeasurement(MT1Pose.pose, timestamp,
                    VecBuilder.fill(
                        VisionConstants.kDontTrust, 
                            VisionConstants.kDontTrust, 
                                VisionConstants.kMultiTagRot.get(tagArea)));

        } else if (tagArea > VisionConstants.kMinTA && tagCount == 1) {
            addVisionMeasurement(MT2Pose.pose, timestamp, 
                VecBuilder.fill(
                    VisionConstants.kMegaTagTwo.get(tagArea),
                        VisionConstants.kMegaTagTwo.get(tagArea), 
                            VisionConstants.kDontTrust));
        }
    }

    public Command dontUseMT2(boolean dontUse) {
        return new ConditionalCommand(
            new InstantCommand(()-> m_useMT2 = false), 
                new InstantCommand(()-> m_useMT2 = true), 
                    ()-> dontUse);
    }
}