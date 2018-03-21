package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotModules.JewelSwatter;

@Disabled
@TeleOp(name = "Test Jewel Swatter")
public class JewelSwatterTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        JewelSwatter jewelSwatter = new JewelSwatter(this, true);
        waitForStart();
        while (opModeIsActive()) {
            jewelSwatter.updateAll();
            telemetry.update();
        }
    }
}
