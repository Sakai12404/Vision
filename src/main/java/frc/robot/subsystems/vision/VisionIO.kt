package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.struct.StructSerializable
import org.littletonrobotics.junction.AutoLog
import java.nio.ByteBuffer

interface VisionIO {
    @AutoLog
    open class VisionIOInputs {
        @JvmField var connected: Boolean = true

        @JvmField var tagIds: IntArray = IntArray(0)

        @JvmField var latestTargetObservation: TargetObservation = TargetObservation(Rotation2d.kZero, Rotation2d.kZero)

        @JvmField var estimatedPoses: Array<PoseObservation> = emptyArray()
    }

    fun updateInputs(inputs: VisionIOInputs) {}
}

data class PoseObservation(
    val estimatedRobotPose: Pose3d,
    val timeStamp: Double,
    val ambiguity: Double,
    val targetCount: Int,
    val averageTagDistance: Double,
    val poseObservation: PoseObservationType,
) : StructSerializable {
    companion object {
        @JvmField
        val struct =
            object : edu.wpi.first.util.struct.Struct<PoseObservation> {
                override fun getTypeClass(): Class<PoseObservation> = PoseObservation::class.java

                override fun getTypeName(): String = "PoseObservation"

                override fun getSize(): Int = 32 + Pose3d.struct.size // Pose3d size + double.size * 3 + int.size + enum.size

                override fun getSchema(): String =
                    "Pose3d estimatedRobotPose;double timeStamp;double ambiguity;int targetCount;double averageTagDistance;PoseObservationType poseObservation"

                override fun pack(
                    byte: ByteBuffer,
                    pose: PoseObservation,
                ) {
                    byte.putDouble(pose.timeStamp)
                    Pose3d.struct.pack(byte, pose.estimatedRobotPose)
                    byte.putDouble(pose.ambiguity)
                    byte.putInt(pose.targetCount)
                    byte.putDouble(pose.averageTagDistance)
                    byte.putInt(pose.poseObservation.ordinal)
                }

                override fun unpack(byte: ByteBuffer): PoseObservation {
                    val newEstimatedRobotPose = Pose3d.struct.unpack(byte)
                    val newTimeStamp = byte.double
                    val newAmbiguity = byte.double
                    val newTargetCount = byte.int
                    val newAverageTagDistance = byte.double
                    val newPoseObservationType = PoseObservationType.entries[byte.int]
                    return PoseObservation(
                        newEstimatedRobotPose,
                        newTimeStamp,
                        newAmbiguity,
                        newTargetCount,
                        newAverageTagDistance,
                        newPoseObservationType,
                    )
                }
            }
    }
}

data class TargetObservation(
    val tx: Rotation2d,
    val ty: Rotation2d,
) : StructSerializable {
    companion object {
        @JvmField
        val struct =
            object : edu.wpi.first.util.struct.Struct<TargetObservation> {
                override fun getTypeClass(): Class<TargetObservation> = TargetObservation::class.java

                override fun getTypeName(): String = "TargetObservation"

                override fun getSize(): Int = Rotation2d.struct.size * 2 // self explanatory

                override fun getSchema(): String = "Rotation2d tx;Rotation2d ty"

                override fun pack(
                    byte: ByteBuffer,
                    target: TargetObservation,
                ) {
                    Rotation2d.struct.pack(byte, target.tx)
                    Rotation2d.struct.pack(byte, target.ty)
                }

                override fun unpack(byte: ByteBuffer): TargetObservation {
                    val newTx = Rotation2d.struct.unpack(byte)
                    val newTy = Rotation2d.struct.unpack(byte)
                    return TargetObservation(newTx, newTy)
                }
            }
    }
}

enum class PoseObservationType {
    MegaTag_1,
    MegaTag_2,
    PhotonVision,
}
