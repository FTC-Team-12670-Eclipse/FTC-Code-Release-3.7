package org.firstinspires.ftc.teamcode.Autonomous.MultiGlyph;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Autonomous(name = "Tester A")
public class TesterAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        waitForStart();

        robot.driveTrain.moveToInches( 36, .25);
        robot.driveTrain.turnRelative(90,.25);
        robot.driveTrain.moveToInches( 36, .25);
        robot.driveTrain.turnRelative(90,.25);
        robot.driveTrain.moveToInches( 36, .25);
        robot.driveTrain.turnRelative(90,.25);
        robot.driveTrain.moveToInches( 36, .25);
        robot.driveTrain.park();
    }
}
