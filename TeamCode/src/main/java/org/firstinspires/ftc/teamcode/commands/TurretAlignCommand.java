package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.kD;
import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.kDMoving;
import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.kF;
import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.kFMoving;
import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.kI;
import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.kIMoving;
import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.kP;
import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.kPMoving;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.utils.PolarVector;
import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Util;

import java.util.function.BooleanSupplier;

public class TurretAlignCommand extends CommandBase {
    private Drive drive;
    private Turret turret;
    private Drive.Alliance alliance;
    private Boolean isAlign;
    private Vision vision;
    private BooleanSupplier killButton;
    private Follower follower;

    public TurretAlignCommand(Drive drive, Turret turret, Drive.Alliance alliance, Vision vision,
                              BooleanSupplier killButton) {
        this.drive = drive;
        this.turret = turret;
        this.alliance = alliance;
        this.isAlign = true;
        this.vision = vision;
        this.killButton = killButton;
        addRequirements(turret);
    }

    public TurretAlignCommand(Follower follower, Turret turret, Drive.Alliance alliance, Vision vision) {
        this.follower = follower;
        this.turret = turret;
        this.alliance = alliance;
        this.isAlign = true;
        this.vision = vision;
        this.killButton = () -> false;
    }

    @Override
    public void initialize() {
        turret.setTurretState(Turret.TurretState.ACTIVE);
    }

    @Override
    public void execute() {
        if (killButton.getAsBoolean()) isAlign = !isAlign;
        if (isAlign) {
            if (drive != null && drive.getAligned()) {
                PolarVector goalInTurretSys = Util.goalInTurretSys(drive.getExpectedPose(alliance), alliance);
                if (drive.isMoving()) {
                    turret.setPIDF(kPMoving, kIMoving, kDMoving, kFMoving);
                }
                else turret.setPIDF(kP, kI, kD, kF);
                turret.setTurret(goalInTurretSys.getHeading(DriveConstants.angleUnit));
            }
            else if (follower != null){
                turret.setPIDF(kP, kI, kD, kF);
                PolarVector goalInTurretSys = Util.goalInTurretSys(Util.PoseToPose2D(follower.getPose()), alliance);
                turret.setTurret(goalInTurretSys.getHeading(DriveConstants.angleUnit));
            }
        }
        else {
            turret.setTurretState(Turret.TurretState.INIT);
        }
    }
}
