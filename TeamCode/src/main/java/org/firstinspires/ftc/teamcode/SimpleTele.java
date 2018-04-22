package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotModules.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@TeleOp(name = "Simple Tele Op")
public class SimpleTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveTrain mecanumDriveTrain = new MecanumDriveTrain(this, DcMotor.ZeroPowerBehavior.FLOAT, false);

        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            mecanumDriveTrain.updateAll();
        }
    }
}
