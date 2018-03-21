package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.RobotModules.MecanumDriveTrain;

@TeleOp(name = "Fowards Backwards")
@Disabled
public class ForwardsBackwards extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Controller lastGamepad1 = new Controller(gamepad1);

        MecanumDriveTrain drive = new MecanumDriveTrain(this, DcMotor.ZeroPowerBehavior.FLOAT);
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
        double forwardsPower, horizontalPower, turnPower;
        waitForStart();

        while (opModeIsActive()) {
            forwardsPower = -Math.pow(gamepad1.left_stick_y * Range.clip(1 - gamepad1.left_trigger, .3, 1), 3);
            drive.setLeft(forwardsPower); //Remove this for Mecanum
            drive.setRight(forwardsPower); //Remove this for Mecanum
            drive.updateTelemetry();
            lastGamepad1.update();
        }
    }

}
