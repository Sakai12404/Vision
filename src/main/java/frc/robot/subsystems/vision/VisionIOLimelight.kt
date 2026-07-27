package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.Timer
import limelight.Limelight
import limelight.networktables.AngularVelocity3d
import limelight.networktables.LimelightPoseEstimator
import limelight.networktables.LimelightResults
import limelight.networktables.LimelightSettings
import limelight.networktables.Orientation3d
import limelight.networktables.PoseEstimate
import java.util.Optional
import java.util.function.Supplier

class VisionIOLimelight(
    name: String,
    cameraPosition: Pose3d,
    private val robotPosition: Supplier<Pose2d>,
    private val robotVelocity: Supplier<AngularVelocity3d>,
) : VisionIO {
    private val limelight = Limelight(name)
    private val poseObservation = PoseObservationType.MegaTag_2
    private val estimatorMode = LimelightPoseEstimator.EstimationMode.MEGATAG2
    private val poseEstimator = limelight.createPoseEstimator(estimatorMode)

    init {
        limelight.settings
            .withLimelightLEDMode(LimelightSettings.LEDMode.ForceBlink) // NOT SURE ABOUT LEDMode TYPE
            .withCameraOffset(cameraPosition)
        limelight.settings
            .withImuMode(LimelightSettings.ImuMode.ExternalImu)
            .save()
    }

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
        limelight.settings
            .withRobotOrientation(
                Orientation3d(
                    Rotation3d(robotPosition.get().rotation),
                    robotVelocity.get(),
                ),
            )

        val optionalPoseEstimator: Optional<PoseEstimate> = poseEstimator.poseEstimate
        val optionalLimelightResults: Optional<LimelightResults> = limelight.latestResults

        if (optionalPoseEstimator.isPresent) {
            val estimate = optionalPoseEstimator.get()

            inputs.connected = Timer.getFPGATimestamp() - estimate.timestampSeconds <= 0.25

            inputs.estimatedPoses =
                arrayOf(
                    PoseObservation(
                        estimate.pose,
                        estimate.timestampSeconds,
                        estimate.avgTagAmbiguity,
                        estimate.tagCount,
                        estimate.avgTagDist,
                        poseObservation,
                    ),
                )
        } else {
            inputs.connected = false
            inputs.estimatedPoses = emptyArray()
        }

        if (optionalLimelightResults.isPresent) {
            val results = optionalLimelightResults.get()

            if (results.targets_Fiducials.isNotEmpty()) {
                inputs.latestTargetObservation =
                    TargetObservation(
                        Rotation2d(results.targets_Fiducials[0].tx),
                        Rotation2d(results.targets_Fiducials[0].ty),
                    )
            } else {
                inputs.latestTargetObservation =
                    TargetObservation(
                        Rotation2d.kZero,
                        Rotation2d.kZero,
                    )
            }

            val tagIds = mutableSetOf<Int>()
            for (target in results.targets_Fiducials) {
                tagIds.add(target.fiducialID.toInt())
            }
            inputs.tagIds = tagIds.toIntArray()
        } else {
            inputs.latestTargetObservation =
                TargetObservation(
                    Rotation2d.kZero,
                    Rotation2d.kZero,
                )
            inputs.tagIds = intArrayOf(0)
        }
    }
}
