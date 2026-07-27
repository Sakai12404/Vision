package frc.robot

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import frc.robot.Constants.Mode
import frc.robot.subsystems.vision.VisionIO
import frc.robot.subsystems.vision.VisionIOLimelight
import frc.robot.subsystems.vision.VisionIOPhotonSim
import frc.robot.subsystems.vision.VisionSubsystem
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.drive.DriveIO
import frc.team449.subsystems.drive.DriveIOHardware
import frc.team449.subsystems.drive.DriveIOSim
import frc.team449.subsystems.drive.DriveSubsystem

object RobotContainer {
    val driveController = CommandPS4Controller(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT) // I have a ps4 controller
    val drive: DriveSubsystem =
        DriveSubsystem(
            when (Constants.CURRENT_MODE) {
                Mode.REAL -> {
                    DriveIOHardware(
                        TunerConstants.DrivetrainConstants,
                        arrayOf(
                            TunerConstants.FrontLeft,
                            TunerConstants.FrontRight,
                            TunerConstants.BackLeft,
                            TunerConstants.BackRight,
                        ),
                    )
                }

                else -> {
                    DriveIOSim(
                        TunerConstants.DrivetrainConstants,
                        arrayOf(
                            TunerConstants.FrontLeft,
                            TunerConstants.FrontRight,
                            TunerConstants.BackLeft,
                            TunerConstants.BackRight,
                        ),
                    )
                }

//                else -> {
//                    DriveIO {
//                        estimatedPose: Pose2d,
//                        timestamp: Double,
//                        stdDevs: Matrix<N3, N1>,
//                        ->
//                        Unit
//                    }
//                }
            },
        )

    val vision: VisionSubsystem =
        when (Constants.CURRENT_MODE) {
            Mode.REAL -> {
                VisionSubsystem(
                    drive::addVisionMeasurement,
                    VisionIOLimelight(
                        "Camera1",
                        Constants.VisionConstants.ROBOT_TO_CAMERA_LEFT,
                        { drive.pose },
                        { drive.angularVelocity },
                    ),
                    VisionIOLimelight(
                        "Camera2",
                        Constants.VisionConstants.ROBOT_TO_CAMERA_RIGHT,
                        { drive.pose },
                        { drive.angularVelocity },
                    ),
                )
            }

            else -> {
                VisionSubsystem(
                    drive::addVisionMeasurement,
                    VisionIOPhotonSim("camera1", Constants.VisionConstants.ROBOT_TO_CAMERA_LEFT) { drive.pose },
                    VisionIOPhotonSim("camera2", Constants.VisionConstants.ROBOT_TO_CAMERA_RIGHT) { drive.pose },
                )
            }

//            else -> {
//                VisionSubsystem(
//                    drive::addVisionMeasurement,
//                    object : VisionIO {},
//                    object : VisionIO {},
//                )
//            }
        }

    val bindings: Binding =
        Binding(this)
}
