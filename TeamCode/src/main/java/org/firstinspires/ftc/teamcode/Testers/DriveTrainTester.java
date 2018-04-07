package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Disabled
@TeleOp(name = "drive slow")
public class DriveTrainTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, false, false, true, DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        double max = .1;
        Controller game1 = new Controller(gamepad1);
        while(opModeIsActive()){
            game1.update();
            if(game1.dpadLeftOnce()){
                max += .01;
            } else if(game1.dpadRightOnce()){
                max -= .01;
            }
            robot.driveTrain.translateBy(Range.clip(game1.left_stick_y, -max, max),Range.clip(game1.left_stick_x, -max, max),Range.clip(game1.right_stick_x, -max, max));

            telemetry.addData("Max", max);
            telemetry.update();

        }
    }
}
