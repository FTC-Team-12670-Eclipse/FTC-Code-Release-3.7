package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Autonomous(name = "Red FAR Color")
public class AutonomousFarIntakeRedNew extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.AllianceColor color = AutonomousUtil.getColorToDislodge(this, AutonomousUtil.AllianceColor.Blue, robot);

        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.relicMecanism.swingElbowUp();
        robot.jewelSwatter.removeJewelOfColor(color);
        AutonomousUtil.driveRobotOffRamp(robot, AutonomousUtil.AllianceColor.Red);

        robot.intakeMecanism.deployFoldoutIntake();

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        double targetAngle = 0;

        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.jewelSwatter.elbowServo.setPosition(UniversalConstants.jewelElbowForwards);
        robot.driveTrain.assistedStrafe(-.5, 0);
        sleep(250);
        robot.driveTrain.park();
        robot.jewelSwatter.wristServo.setPosition(UniversalConstants.jewelWristForwards);
        robot.driveTrain.moveToPositionInches(-2.5, .2);
        double startTime;
        switch (vuMark) {
            case LEFT:
                robot.strafeToJewelSensedDistance(-.2, 5, targetAngle, true);
                //false parameter gives early exit to function, and doesn't park
                robot.jewelSwatter.zeroSwatter();
                startTime = getRuntime();
                while (opModeIsActive() && (getRuntime() - startTime < 1)) {
                    robot.driveTrain.assistedStrafe(-.2, 0);
                }
                robot.driveTrain.park();
                robot.jewelSwatter.moveJewelToForwards();
            case CENTER:
                robot.strafeToJewelSensedDistance(-.2, 5, targetAngle, true);
                //false parameter gives early exit to function, and doesn't park
                robot.jewelSwatter.zeroSwatter();
                startTime = getRuntime();
                while (opModeIsActive() && (getRuntime() - startTime < 1)) {
                    robot.driveTrain.assistedStrafe(-.2, 0);
                }
                robot.driveTrain.park();
                robot.jewelSwatter.moveJewelToForwards();
            case RIGHT:
            default:
                robot.strafeToJewelSensedDistance(-.2, 5, targetAngle, true);
                break;
        }
        robot.jewelSwatter.zeroSwatter();
        robot.driveTrain.gyroTurn(.1, 10);
        robot.intakeMecanism.deployFoldoutIntake();
        robot.intakeMecanism.intake();
        sleep(250);
        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.outtake();

        robot.driveTrain.moveToInches(5, .25);
        robot.driveTrain.moveToInches(-7, .5);
        robot.intakeMecanism.stopIntake();
        robot.driveTrain.gyroTurn(.25, 90);
        robot.driveTrain.gyroTurn(.07, 90);
        switch (vuMark) {
            case LEFT:
                robot.driveTrain.moveToPositionInches(5, 1);
                break;
            case CENTER:
                robot.driveTrain.moveToPositionInches(10, 1);
                break;
            default:
            case RIGHT:
                robot.driveTrain.moveToPositionInches(15, 1);
                break;
        }
        robot.driveTrain.gyroTurn(.25, 165);
        robot.intakeMecanism.intake();
        robot.driveTrain.moveToInches(30, 1);
        robot.driveTrain.moveToInches(-20, 1);
        robot.driveTrain.gyroTurn(.25, 0);
        robot.driveTrain.gyroTurn(.05, 0);
        startTime = getRuntime();
        while (opModeIsActive() && getRuntime() - startTime < 1) {
            robot.driveTrain.assistedStrafe(1, 0);
        }
        robot.driveTrain.park();

    }
}
