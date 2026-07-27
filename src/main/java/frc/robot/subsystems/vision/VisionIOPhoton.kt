package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.util.Units
import org.photonvision.PhotonCamera

open class VisionIOPhoton(
    name: String,
    private val cameraPosition: Transform3d,
) : VisionIO {
    protected val photonCamera: PhotonCamera = PhotonCamera(name)
    val aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)!!

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
        inputs.connected = photonCamera.isConnected

        val tagIds = mutableSetOf<Int>()
        val poseObservations = mutableSetOf<PoseObservation>()

        for (results in photonCamera.allUnreadResults) {
            if (results.hasTargets()) {
                inputs.latestTargetObservation =
                    TargetObservation(
                        Rotation2d(Units.radiansToDegrees(results.bestTarget.pitch)),
                        Rotation2d(Units.radiansToDegrees(results.bestTarget.yaw)),
                    )
            } else {
                inputs.latestTargetObservation =
                    TargetObservation(
                        Rotation2d.kZero,
                        Rotation2d.kZero,
                    )
            }

            if (results.multitagResult.isPresent) {
                val tags = results.multitagResult.get()
                val transform = tags.estimatedPose.best.plus(cameraPosition.inverse())
                val robotPose = Pose3d(transform.translation, transform.rotation)

                var totalTargetDistance = 0.0
                for (target in results.targets) {
                    tagIds.add(target.fiducialId)
                    totalTargetDistance += target.bestCameraToTarget.translation.norm
                }

                poseObservations.add(
                    PoseObservation(
                        robotPose,
                        results.timestampSeconds,
                        tags.estimatedPose.ambiguity,
                        tags.fiducialIDsUsed.size, // tagIds.size,
                        totalTargetDistance / results.targets.size,
                        PoseObservationType.PhotonVision,
                    ),
                )
            } else if (results.targets.isNotEmpty()) {
                val target = results.targets[0]

                val tagPose = aprilTagFieldLayout.getTagPose(target.fiducialId)
                if (tagPose.isPresent) {
                    val fieldToTarget = Transform3d(tagPose.get().translation, tagPose.get().rotation)
                    val cameraToTarget = target.bestCameraToTarget
                    val fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse())
                    val fieldToRobot = fieldToCamera.plus(cameraPosition)
                    val robotPose = Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

                    tagIds.add(target.fiducialId)

                    poseObservations.add(
                        PoseObservation(
                            robotPose,
                            results.timestampSeconds,
                            target.poseAmbiguity,
                            1,
                            cameraToTarget.translation.norm,
                            PoseObservationType.PhotonVision,
                        ),
                    )
                }
            }
        }

        inputs.estimatedPoses = poseObservations.toTypedArray()
        inputs.tagIds = tagIds.toIntArray()
    }
}
