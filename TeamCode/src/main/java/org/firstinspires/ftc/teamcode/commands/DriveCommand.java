package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

public class DriveCommand extends CommandBase {
    private final Drive drive;
    private final GamepadEx gamepadEx;

    public DriveCommand(Drive drive, GamepadEx gamepadEx) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        addRequirements(drive);
    }

    @Override
    public void execute() {
    if (Math.abs(gamepadEx.getLeftX()) > 0.03 ||
                Math.abs(gamepadEx.getLeftY()) > 0.03 ||
                Math.abs(gamepadEx.getRightX()) > 0.03) {
            drive.setDriveState(Drive.DriveState.TELEOP);
            drive.moveRobotFieldRelative(
                    gamepadEx.getLeftY(),
                    gamepadEx.getLeftX(),
                    gamepadEx.getRightX()
            );
        }
        else {
            drive.setDriveState(Drive.DriveState.STOP);
            drive.stop();
        }
    }
}
