package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

        robot.driveTrain.moveToPositionInches(-2.5, .2);
        double startTime;
        // deploy color distance sensor
        double power = .2;
        double distance = 3;
        DistanceUnit unit = DistanceUnit.CM;

        switch (vuMark) {
            case LEFT:
                // strafe to the left at [some power] until distance is at [a critical value] <- parameter
                // A method strafeToDistance(double power, double distance, DistanceUnit unit)

                for(int i = 0; i < 3; i++) {
                    robot.driveTrain.strafeToDistance(power, distance, unit);
                    robot.driveTrain.swingColorDistanceUp();
                    sleep(300);
                    robot.driveTrain.swingColorDistanceDown();
                }

                robot.driveTrain.park();

                break;
            case CENTER:
                for(int i = 0; i < 2; i++) {
                    robot.driveTrain.strafeToDistance(power, distance, unit);
                    robot.driveTrain.swingColorDistanceUp();
                    sleep(300);
                    robot.driveTrain.swingColorDistanceDown();
                }
                robot.driveTrain.park();

                break;
            case RIGHT:
            default:
                for(int i = 0; i < 1; i++) {
                    robot.driveTrain.strafeToDistance(power, distance, unit);
                    robot.driveTrain.swingColorDistanceUp();
                    sleep(300);
                    robot.driveTrain.swingColorDistanceDown();
                }
                robot.driveTrain.park();
                break;
        }
        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.outtake();

        robot.driveTrain.moveToInches(5, .25);
        robot.driveTrain.moveToInches(-5, .25);
        robot.driveTrain.moveToInches(5, .25);
        robot.driveTrain.moveToInches(-10, .5);
        robot.intakeMecanism.stopIntake();
        robot.driveTrain.gyroTurn(.25, 90);
        robot.driveTrain.gyroTurn(.07, 90);
        switch (vuMark) {
            case LEFT:
                break;
            case CENTER:
                robot.driveTrain.moveToPositionInches(5, 1);
                break;
            default:
            case RIGHT:
                robot.driveTrain.moveToPositionInches(10, 1);
                break;
        }
        robot.driveTrain.gyroTurn(.25, 165);
        robot.intakeMecanism.intake();
        robot.driveTrain.moveToInches(30, 1);
        robot.driveTrain.moveToInches(-30, 1);
        robot.driveTrain.gyroTurn(.25, 0);
        robot.driveTrain.gyroTurn(.05, 0);
        startTime = getRuntime();
        while (opModeIsActive() && getRuntime() - startTime < .5) {
            robot.driveTrain.assistedStrafe(1, 0);
        }
        robot.driveTrain.park();

    }
}
