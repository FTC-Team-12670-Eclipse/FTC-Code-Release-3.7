package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutonomousUtil;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

import java.util.Locale;

//@Disabled
@Autonomous(name = "Auto Tester")
public class AutonomousTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addLine("Ready to go!");
        telemetry.update();

        Controller game1 = new Controller(gamepad1);
        int mode = 1;

        String modeName;
        waitForStart();
        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        while (opModeIsActive()) {
            while (!gamepad1.a && opModeIsActive()) {
                game1.update();
                if (game1.leftBumperOnce()) {
                    mode = ((mode + 1) + 8) % 8;
                } else if (game1.rightBumperOnce()) {
                    mode = ((mode - 1) + 8) % 8;
                }
                switch (mode + 1) {
                    case 1:
                        modeName = "robot.driveTrain.moveToPositionInches(10, .1);";
                        break;
                    case 2:
                        modeName = "robot.driveTrain.moveToPositionInches(-10, -.1);";
                        break;
                    case 3:
                        modeName = "robot.driveTrain.gyroTurn(.125, 0);";
                        break;
                    case 4:
                        modeName = "robot.driveTrain.gyroTurn(.25, 0);";
                        break;
                    case 5:
                        modeName = "robot.strafeToJewelSensedDistance(.15, 10);";
                        break;
                    case 6:
                        modeName = "jewel to forwards";
                        break;
                    case 7:
                        modeName = "skip column go left";
                        break;
                    case 8:
                        modeName = "swat red jewel";
                        break;
                    default:
                        modeName = "no set mode";
                }
                telemetry.addData("Mode", modeName);
                telemetry.addData("Left Inch", robot.driveTrain.leftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("Right Inch", robot.driveTrain.rightDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("modes", "" + robot.driveTrain.leftFront.getMode() + " | " + robot.driveTrain.rightFront.getMode() + " | " + robot.driveTrain.leftBack.getMode() + " | " + robot.driveTrain.rightBack.getMode());
                telemetry.addData("zpb", "" + robot.driveTrain.leftFront.getZeroPowerBehavior() + " | " + robot.driveTrain.rightFront.getZeroPowerBehavior() + " | " + robot.driveTrain.leftBack.getZeroPowerBehavior() + " | " + robot.driveTrain.rightBack.getZeroPowerBehavior());
                telemetry.addData("pos", "" + robot.driveTrain.leftFront.getCurrentPosition() + " | " + robot.driveTrain.rightFront.getCurrentPosition() + " | " + robot.driveTrain.leftBack.getCurrentPosition() + " | " + robot.driveTrain.rightBack.getCurrentPosition());
                telemetry.addData("Distance (in)",
                        String.format(Locale.US, "%.02f", robot.jewelSwatter.sensorDistance.getDistance(DistanceUnit.CM)));
                robot.driveTrain.updateTelemetry();
                robot.vuforiaRelicRecoveryGetter.updateTelemetry();
                telemetry.update();
            }
            switch (mode + 1) {
                case 1:
                    robot.driveTrain.moveToPositionInches(10, 1);
                    break;
                case 2:
                    robot.driveTrain.moveToPositionInches(-10, -1);
                    break;
                case 3:
                    robot.driveTrain.strafeToPositionInches(10, -1);
                    break;
                case 4:
                    robot.driveTrain.strafeToPositionInches(10, 1);
                    break;
                case 5:
                    robot.strafeToJewelSensedDistance(-.15, 10);
                    break;
                case 6:
                    robot.jewelSwatter.moveJewelToForwards();
                    break;
                case 7:
                    robot.strafeToJewelSensedDistance(-.15, 10, false);
                    //false parameter gives early exit to function, and doesn't park
                    robot.jewelSwatter.moveJewelForwardsAway();
                    sleep(750);
                    robot.jewelSwatter.moveJewelToForwards();
                    robot.strafeToJewelSensedDistance(-.15, 10, true);
                    robot.strafeToJewelSensedDistance(-.15, 10, true);
                    robot.strafeToJewelSensedDistance(-.15, 10, true);
                    //true parameter gives late exit to function, error based exit to function with parking
                    break;
                case 8:
                    robot.jewelSwatter.removeJewel(AutonomousUtil.AllianceColor.Red);
                default:
                    break;
            }
        }
    }
}
