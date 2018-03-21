package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Disabled
@Autonomous(name = "Red Close Dunk")
public class AutonomousRedCloseDunk extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();

        robot.driveTrain.moveToInches(25, .25);
        robot.driveTrain.alignToWithinOf(65);
        if (!robot.vuforiaRelicRecoveryGetter.getOffset().foundValues) {
            robot.driveTrain.alignToWithinOf(45);
        }
        RelicRecoveryVuMark vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();

        double dist;
        switch (vuMark) {
            case LEFT:
                dist = 2;
                break;
            case CENTER:
                dist = 8;
                break;
            default:
                dist = 17;
        }
        if (dist != 0) {
            robot.driveTrain.alignToWithinOf(0);
            robot.driveTrain.moveToInches(dist, .25);
        }
        robot.driveTrain.alignToWithinOf(90);
        robot.driveTrain.moveToInches(-8.5, .25);
        sleep(500);
        robot.slamDunker.dunkSlow();
        sleep(1000);
        robot.driveTrain.moveToInches(5, .25);
        sleep(500);
        robot.slamDunker.retractDunkNoWait();
        robot.driveTrain.moveToInches(-6.5, .25);
        robot.driveTrain.moveToInches(3, .25);


    }
}
