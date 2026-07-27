package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import java.util.function.Supplier

class VisionIOPhotonSim(
    name: String,
    cameraPosition: Pose3d,
    private val poseSupplier: Supplier<Pose2d>,
) : VisionIOPhoton(
        name,
        cameraPosition.minus(Pose3d()),
    ) {
    private val cameraSim: PhotonCameraSim
    private val visionSim = VisionSystemSim("main")

    init {
        visionSim.addAprilTags(aprilTagFieldLayout)

        val cameraProperties = SimCameraProperties()
        cameraProperties.setCalibration(1280, 800, Rotation2d(99.41))
        cameraProperties.setCalibError(0.0, 0.0)
        cameraProperties.setFPS(120.0)
        cameraProperties.setAvgLatencyMs(15.0)
        cameraProperties.setLatencyStdDevMs(5.0)

        cameraSim = PhotonCameraSim(photonCamera, cameraProperties, aprilTagFieldLayout)
        visionSim.addCamera(cameraSim, cameraPosition.minus(Pose3d()))
    }

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
        visionSim.update(poseSupplier.get())
        super.updateInputs(inputs)
    }
}
