package org.firstinspires.ftc.teamcode.opmodes.teleops;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

@TeleOp(name = "TeleOp Solo Blue Static", group = "TeleOp")
public class BlueStatic extends TeleOpBaseStatic {
    @Override
    public Drive.Alliance getAlliance() {
        return Drive.Alliance.BLUE;
    }
}
