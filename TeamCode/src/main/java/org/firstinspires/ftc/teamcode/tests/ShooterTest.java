package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kD;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kF;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kI;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kP;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.maxVelocity;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.fastVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Shooter Test")
public class ShooterTest extends OpMode {
    DcMotorEx rightMotor, leftMotor;
    PIDFController pidfController;
    FtcDashboard dashboard;
    
    public static double targetVelocity = 0;
    public static double p = 0, i = 0, d = 0, f = 0;

    @Override
    public void init() {
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");

        // 两个电机都用开环模式，我们自己做 PID
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // 使用 FTCLib 的 PIDFController
        pidfController = new PIDFController(p, i, d, f);
        
        // 获取 FTC Dashboard 实例
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        // 实时更新 PIDF 参数（方便通过 Dashboard 调参）
        pidfController.setPIDF(p, i, d, f);

        // 用右扳机控制目标速度 (0 ~ fastVelocity)

        // 读取当前速度（从 rightMotor 编码器）
        double currentVelocity = rightMotor.getVelocity();

        // 使用 calculate(currentValue, setpoint) 计算 PID 输出
        double output = pidfController.calculate(currentVelocity, targetVelocity);

        // 限制输出范围
        output = Range.clip(output, -1.0, 1.0);

        // 两个电机使用相同的 PID 输出（注意方向）
        rightMotor.setPower(output);
        leftMotor.setPower(output);

        // 发送数据到 FTC Dashboard 图表
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("targetVelocity", targetVelocity);
        packet.put("currentVelocity", currentVelocity);
        packet.put("error", targetVelocity - currentVelocity);
        packet.put("output", output);
        dashboard.sendTelemetryPacket(packet);

        // Driver Station telemetry
        telemetry.addData("Target", "%.1f", targetVelocity);
        telemetry.addData("Current", "%.1f", currentVelocity);
        telemetry.addData("Error", "%.1f", targetVelocity - currentVelocity);
        telemetry.addData("Output", "%.3f", output);
        telemetry.update();
    }

    @Override
    public void stop() {
        // 停止时确保电机停止
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}
