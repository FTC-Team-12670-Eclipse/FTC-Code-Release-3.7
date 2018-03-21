package org.firstinspires.ftc.teamcode.InitialMGAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutonomousUtil;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;
import org.firstinspires.ftc.teamcode.UniversalConstants;

@Disabled
@Autonomous(name = "Blue CLOSE Multiglyph")
public class AutonomousCloseDunkBlueDistanceSensor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.Settings settings = AutonomousUtil.getStartingSettings(this, AutonomousUtil.AllianceColor.Red, robot);
        double moveToPositionPower = .6;
        double strafeSpeed = .1;
        double expectedDistance = 14;

        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.intakeMecanism.deployFoldoutIntake();
        robot.relicMecanism.swingElbowUp();
        robot.jewelSwatter.removeJewelOfColor(settings.allianceColor);
        robot.intakeMecanism.setIntakePowers(.2);
        AutonomousUtil.driveRobotOffRamp(robot, AutonomousUtil.AllianceColor.Blue);

        robot.driveTrain.gyroTurn(.25, 0);
        robot.driveTrain.gyroTurn(.05, 0);

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        robot.jewelSwatter.wristServo.setPosition(UniversalConstants.jewelWristStored);

        double closestPosition = 6;
        double targetAngle = 90;

        switch (vuMark) {
            case LEFT:
                robot.driveTrain.moveToInchesSpeedUpSlowDown(closestPosition - 11, moveToPositionPower);
                break;
            default:
            case RIGHT:
            case CENTER:
                robot.driveTrain.moveToInchesSpeedUpSlowDown(closestPosition - 6, moveToPositionPower);
                break;
        }
        robot.driveTrain.gyroTurn(.20, targetAngle);
        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.driveTrain.moveToInches(-7.5, .35);
        robot.driveTrain.moveToInches(1.75, moveToPositionPower * 4 / 5);
        robot.driveTrain.gyroTurn(.05, targetAngle);

        switch (vuMark) {
            case LEFT:
            case CENTER:
                robot.driveTrain.strafeToPosition(-strafeSpeed, targetAngle, DistanceUnit.CM, expectedDistance);
                break;
            default:
            case RIGHT:
                robot.driveTrain.strafeToPosition(strafeSpeed, targetAngle, DistanceUnit.CM, expectedDistance);
                break;
        }
        robot.driveTrain.moveToInches(1.75, moveToPositionPower);

        robot.slamDunker.dunkSlowWithAutoStop();
        sleep(250);
        robot.intakeMecanism.stopIntake();
        robot.driveTrain.moveToInches(2.5, moveToPositionPower);
        robot.slamDunker.retractDunkNoWait();
        sleep(250);


        robot.driveTrain.gyroTurn(.25, targetAngle);
        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.intakeMecanism.intake();
        robot.slamDunker.dunkMotor.setPower(0);

        robot.driveTrain.moveToInchesSpeedUpSlowDown(24, moveToPositionPower);

        switch (vuMark) {
            case LEFT:
                robot.driveTrain.gyroTurn(.25, targetAngle + settings.leftAngle);
                break;
            case CENTER:
                robot.driveTrain.gyroTurn(.25, targetAngle + settings.centerAngle);
                break;
            default:
            case RIGHT:
                robot.driveTrain.gyroTurn(.25, targetAngle - settings.rightAngle);
                break;
        }

        robot.driveTrain.moveToInches(8, 1);
        robot.driveTrain.moveToInches(-8, 1);
        robot.driveTrain.gyroTurn(.05, targetAngle);
        robot.intakeMecanism.stopIntake();
        robot.driveTrain.moveToInches(-5, 1);

        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.intake();
        robot.driveTrain.moveToInches(2, moveToPositionPower);

        robot.driveTrain.moveToInches(-12, moveToPositionPower);

        robot.driveTrain.gyroTurn(.25, targetAngle);
        robot.driveTrain.gyroTurn(.05, targetAngle);

        if (settings.shouldDunk) {
            robot.driveTrain.moveToInches(-7.5, .35);
            robot.driveTrain.moveToInches(2, moveToPositionPower * 4 / 5);
            robot.driveTrain.gyroTurn(.05, targetAngle);

            switch (vuMark) {
                case CENTER:
                    robot.driveTrain.strafeToPosition(-strafeSpeed, targetAngle, DistanceUnit.CM, expectedDistance);
                    break;
                case LEFT:
                case RIGHT:
                default:
                    robot.driveTrain.strafeToPosition(strafeSpeed, targetAngle, DistanceUnit.CM, expectedDistance);
                    break;
            }
            robot.driveTrain.moveToInches(1.75, moveToPositionPower);

            robot.slamDunker.dunkSlowWithAutoStop();
            sleep(250);
            robot.intakeMecanism.stopIntake();
            robot.driveTrain.moveToInches(2.5, moveToPositionPower * 2 / 5);
            robot.slamDunker.retractDunkNoWait();
            sleep(250);
        } else {
            robot.driveTrain.moveToInches(-5, moveToPositionPower);
        }
        robot.relicMecanism.storeServos();
        robot.slamDunker.retract();
        robot.slamDunker.dunkMotor.setPower(0);

    }
}
