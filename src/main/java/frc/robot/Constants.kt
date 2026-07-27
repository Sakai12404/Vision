package frc.robot

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.RobotBase
import kotlin.math.PI

object Constants {
    // --- OPERATIONAL MODES ---
    enum class Mode {
        REAL,
        SIM,
        REPLAY,
    }

    val CURRENT_MODE: Mode = if (RobotBase.isReal()) Mode.REAL else Mode.SIM
    const val TUNING_MODE: Boolean = false

    // --- SYSTEM TIMING ---
    const val LOOP_TIME = 0.02

    // --- PHYSICAL SPECS ---
    const val ROBOT_MASS_KG = 59.8
    const val ROBOT_WIDTH_INCHES = 35.0 // including bumpers (front to rear)
    const val ROBOT_LENGTH_INCHES = 34.125 // including bumpers (left to right)

    object DriveConstants {
        // --- LOOP TIMING ---
        const val ODOMETRY_LOOP_HZ = 100.0

        // --- PHYSICAL SPECS ---
        const val TRACKWIDTH_INCHES = 21.75 // front to rear
        const val WHEELBASE_INCHES = 21.75 // left to right
        const val WHEEL_COF = 1.4

        // --- SPEED LIMITS (STANDARD) ---
        const val MAX_LINEAR_SPEED_METERS_PER_SEC = 4.7244
        const val MAX_ANGULAR_SPEED_RADS_PER_SEC = 2 * PI

        // --- SPEED LIMITS (SLOW) ---
        const val SLOW_LINEAR_SPEED_METERS_PER_SEC = 1.5
        const val SLOW_ANGULAR_SPEED_RADS_PER_SEC = 0.5804

        // --- DEADBANDS & TOLERANCE ---
        const val TRANSLATION_DEADBAND = 0.1
        const val ANGULAR_DEADBAND = 0.1
        const val INTERRUPT_DEADBAND = 0.25
        const val MODULE_ALIGN_TOLERANCE_DEG = 5.0
    }

    object OperatorConstants {
        const val DRIVER_CONTROLLER_PORT = 0
    }

    object VisionConstants {
        const val LINEAR_STANDARD_DEVIATION = 0.02 // play with
        const val ANGULAR_STANDARD_DEVIATION = 0.06 // play with

        const val MT2_LINEAR_STANDARD_DEVIATION_FACTOR = 0.5
        const val MT2_ANGULAR_STANDARD_DEVIATION_FACTOR = Double.POSITIVE_INFINITY

        val CAMERA_STANDARD_DEVIATION_FACTOR = Array(2) { 1.0 }

        var ROBOT_TO_CAMERA_RIGHT: Pose3d = Pose3d(
            -0.013, 0.270, 0.539243,
            Rotation3d(0.0, 0.438377245469, -0.583128849696)
        )
        var ROBOT_TO_CAMERA_LEFT: Pose3d = Pose3d(-0.013, -0.270, 0.539243, Rotation3d(0.0, 0.438377245469, 0.583128849696))
    }
}
