package frc.robot

import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

class Robot : LoggedRobot() {
    private val robotContainer = RobotContainer

    init {
        Logger.addDataReceiver(NT4Publisher())
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)
        Logger.start()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun robotInit() {
        robotContainer.bindings.setDefaultCommands()
        robotContainer.bindings.bindControls()
    }
}
