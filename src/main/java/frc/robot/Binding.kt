package frc.robot

import frc.team449.commands.SwerveRequestCommand

class Binding(
    val robotContainer: RobotContainer,
) {
    val driver = robotContainer.driveController

    fun setDefaultCommands() {
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -driver.leftY },
                { -driver.leftX },
                { -driver.rightX },
            )
    }

    // simple control binds with arbitrary values
    fun bindControls() {
        driver
            .touchpad()
            .onTrue(robotContainer.drive.seedFieldCentric())
    }
}
