package org.firstinspires.ftc.teamcode.Autonomous.MultiGlyph;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutonomousUtil;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;
import org.firstinspires.ftc.teamcode.UniversalConstants;

@Autonomous(name = "Blue FAR Special")
public class AutonomousFarIntakeBlueMG extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.AllianceColor color = AutonomousUtil.getColorToDislodge(this, AutonomousUtil.AllianceColor.Red, robot);

        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.relicMecanism.swingAwayFromWall();
        robot.jewelSwatter.removeJewelOfColor(color);

        AutonomousUtil.driveRobotOffRamp(robot, AutonomousUtil.AllianceColor.Blue);
        robot.driveTrain.gyroTurn(.07, 180);

        robot.jewelSwatter.wristServo.setPosition(UniversalConstants.jewelWristStored);

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        double FAR_DISTANCE = -8.25;
        double MIDDLE_DISTANCE = -5.5;
        double CLOSE_DISTANCE = -3.75;

        double FAR_US_DISTANCE = 84;
        double MIDDLE_US_DISTANCE = 66;
        double CLOSE_US_DISTANCE = 48;

        double moveToPositionPower = .2;
        double targetAngle = 180;

        switch (vuMark) {
            case RIGHT:
                robot.driveTrain.encoderStrafeToInches(FAR_DISTANCE, moveToPositionPower, targetAngle);
                robot.driveTrain.autoLeftDistanceSensor(FAR_US_DISTANCE, .5 * moveToPositionPower, targetAngle, DistanceUnit.CM, 2);
                break;
            case CENTER:
                robot.driveTrain.encoderStrafeToInches(MIDDLE_DISTANCE, moveToPositionPower, targetAngle);
                robot.driveTrain.autoLeftDistanceSensor(MIDDLE_US_DISTANCE, .5 * moveToPositionPower, targetAngle, DistanceUnit.CM, 2);
                break;
            default:
            case LEFT:
                robot.driveTrain.encoderStrafeToInches(CLOSE_DISTANCE, moveToPositionPower, targetAngle);
                robot.driveTrain.autoLeftDistanceSensor(CLOSE_US_DISTANCE, .5 * moveToPositionPower, targetAngle, DistanceUnit.CM, 2);
                break;
        }
        robot.driveTrain.park();

        robot.driveTrain.gyroTurn(.25, targetAngle);
        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.intakeMecanism.deployFoldoutIntake();
        robot.intakeMecanism.intake();
        sleep(200);
        robot.intakeMecanism.outtakeFully();
        robot.intakeMecanism.setIntakePowers(.5, -.5);
        sleep(500);
        robot.intakeMecanism.setIntakePowers(-1);

        robot.driveTrain.moveToInches(4, .15);
        robot.driveTrain.moveToInches(-8, .20);
        robot.intakeMecanism.stopIntake();

        switch (vuMark) {
            case RIGHT:
                break;
            case CENTER:
                robot.driveTrain.encoderStrafeToInches(FAR_DISTANCE - MIDDLE_DISTANCE, .3, targetAngle);
                break;
            default:
            case LEFT:
                robot.driveTrain.encoderStrafeToInches(FAR_DISTANCE - CLOSE_DISTANCE, .3, targetAngle);
                break;
        }

        robot.driveTrain.autoLeftDistanceSensor(FAR_US_DISTANCE, .3, targetAngle, DistanceUnit.CM, 2);

        robot.driveTrain.park();

        targetAngle = 180 - 155;

        robot.driveTrain.gyroTurn(.1, targetAngle);
        robot.intakeMecanism.intake();
        robot.driveTrain.moveToInches(45, .65);
        robot.driveTrain.moveToInches(-25, .65);

        targetAngle = 180;
        robot.driveTrain.gyroTurn(.1, targetAngle);

        robot.driveTrain.autoWallDistanceSensor(40, .35, DistanceUnit.CM, 20);
        robot.driveTrain.autoWallDistanceSensor(40, .15, DistanceUnit.CM);

        robot.driveTrain.gyroTurn(.1, targetAngle);
        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.driveTrain.park();

        robot.driveTrain.autoLeftDistanceSensor(FAR_US_DISTANCE, .3, targetAngle, DistanceUnit.CM, 3);
        if (vuMark == RelicRecoveryVuMark.CENTER) {
            robot.driveTrain.gyroTurn(.05, targetAngle - 15);
        } else {
            robot.driveTrain.gyroTurn(.05, targetAngle + 15);
        }

        robot.driveTrain.moveToInches(4, .15);
        robot.intakeMecanism.outtakeFully();

        robot.slamDunker.dunkMotor.setPower(UniversalConstants.dunkGlyphsSpeed * 3);
        robot.slamDunker.dunkMotor.setTargetPosition(robot.slamDunker.dunkMotor.getTargetPosition() + 150);

        robot.intakeMecanism.setIntakePowers(.5, -.5);
        sleep(500);

        robot.slamDunker.dunkMotor.setTargetPosition(robot.slamDunker.dunkMotor.getTargetPosition() - 150);

        robot.intakeMecanism.setIntakePowers(-1);
        robot.slamDunker.dunkMotor.setPower(0);
        robot.driveTrain.moveToInches(4, .15);

        robot.driveTrain.moveToInches(-6, .20);
    }
}
