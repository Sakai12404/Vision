package frc.robot.subsystems.vision

import com.ctre.phoenix6.Utils
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.VisionConstants
import org.littletonrobotics.junction.Logger
import kotlin.math.pow

class VisionSubsystem(
    private val visionMeasurementConsumer: (
        visionRobotPoseMeters: Pose2d,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3, N1>,
    ) -> Unit,
    private vararg val io: VisionIO,
) : SubsystemBase() {
    private val inputs: Array<VisionIOInputsAutoLogged> = Array(io.size) { VisionIOInputsAutoLogged() }

    private val camerasDisconnectAlert: Array<Alert> = Array(io.size) { i -> Alert("Camera $i Disconnected", AlertType.kWarning) }

    override fun periodic() {
        val allAcceptedRobotPoses = mutableListOf<Pose3d>()
        val allRejectedRobotPoses = mutableListOf<Pose3d>()

        for (cameraIndex in 0 until io.size) {
            io[cameraIndex].updateInputs(inputs[cameraIndex])

            Logger.processInputs("Vision/Camera$cameraIndex", inputs[cameraIndex])
            camerasDisconnectAlert[cameraIndex].set(!inputs[cameraIndex].connected)

            val acceptedRobotPoses = mutableListOf<Pose3d>()
            val rejectRobotPoses = mutableListOf<Pose3d>()

            val angularStdDevs = mutableListOf<Double>()
            val linearStdDevs = mutableListOf<Double>()

            for (poseObservation in inputs[cameraIndex].estimatedPoses) {
                if (
                    poseObservation.estimatedRobotPose.x < 16.54 && poseObservation.estimatedRobotPose.x > 0.0 &&
                    poseObservation.estimatedRobotPose.y < 8.07 && poseObservation.estimatedRobotPose.y > 0.0 &&
                    poseObservation.estimatedRobotPose.z <= 0.075 &&
                    poseObservation.ambiguity < 0.2 &&
                    poseObservation.targetCount >= 1
                ) {
                    acceptedRobotPoses.add(poseObservation.estimatedRobotPose)
                    val factorStdDev = poseObservation.averageTagDistance.pow(2) / poseObservation.targetCount
                    var linearStdDev = VisionConstants.LINEAR_STANDARD_DEVIATION * factorStdDev
                    var angularStdDev = VisionConstants.ANGULAR_STANDARD_DEVIATION * factorStdDev

                    if (poseObservation.poseObservation == PoseObservationType.MegaTag_2) {
                        linearStdDev *= VisionConstants.MT2_LINEAR_STANDARD_DEVIATION_FACTOR
                        angularStdDev *= VisionConstants.MT2_ANGULAR_STANDARD_DEVIATION_FACTOR
                    }

                    linearStdDev *= VisionConstants.CAMERA_STANDARD_DEVIATION_FACTOR[cameraIndex]
                    angularStdDev *= VisionConstants.CAMERA_STANDARD_DEVIATION_FACTOR[cameraIndex]

                    visionMeasurementConsumer(
                        poseObservation.estimatedRobotPose.toPose2d(),
                        Utils.fpgaToCurrentTime(Timer.getFPGATimestamp()),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
                    )

                    linearStdDevs.add(linearStdDev)
                    angularStdDevs.add(angularStdDev)
                } else {
                    rejectRobotPoses.add(poseObservation.estimatedRobotPose)
                }
            }
            Logger.recordOutput("Vision/Camera$cameraIndex/Linear Standard Deviation", linearStdDevs.toDoubleArray())
            Logger.recordOutput("Vision/Camera$cameraIndex/Angular Standard Deviation", angularStdDevs.toDoubleArray())

            Logger.recordOutput("Vision/Camera$cameraIndex/Accepted Robot Poses", *acceptedRobotPoses.toTypedArray())
            Logger.recordOutput("Vision/Camera$cameraIndex/Rejected Robot Poses", *rejectRobotPoses.toTypedArray())

            allAcceptedRobotPoses.addAll(acceptedRobotPoses)
            allRejectedRobotPoses.addAll(rejectRobotPoses)
        }

        Logger.recordOutput("Vision/All Accepted Robot Poses", *allAcceptedRobotPoses.toTypedArray())
        Logger.recordOutput("Vision/All Rejected Robot Poses", *allRejectedRobotPoses.toTypedArray())
    }
}
