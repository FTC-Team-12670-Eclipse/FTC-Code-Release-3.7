package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutonomousUtil;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;
import org.firstinspires.ftc.teamcode.UniversalConstants;

@Disabled
@Autonomous(name = "Blue CLOSE MG")
public class AutonomousCloseDunkBlueMG extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousUtil.AllianceColor startingAllianceColor = AutonomousUtil.AllianceColor.Blue;
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.Settings settings = AutonomousUtil.getStartingSettings(this, AutonomousUtil.switchColor(startingAllianceColor), robot);
        double moveToPositionPower = .4;

        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.relicMecanism.swingElbowUp();
        robot.jewelSwatter.removeJewelOfColor(settings.allianceColor);
        AutonomousUtil.driveRobotOffRampFast(robot, startingAllianceColor, 1);

        robot.intakeMecanism.deployFoldoutIntake();

        robot.intakeMecanism.intake();
        sleep(250);
        robot.intakeMecanism.stopIntake();

        robot.driveTrain.gyroTurn(.05, 0);

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        robot.jewelSwatter.wristServo.setPosition(UniversalConstants.jewelWristStored);

        double closestPosition = -6;
        double targetAngle = 90;

        switch (vuMark) {
            case CENTER:
                robot.driveTrain.moveToPositionInches(closestPosition - 6, moveToPositionPower);
                break;
            case RIGHT:
                robot.driveTrain.moveToPositionInches(closestPosition - 12, moveToPositionPower);
                break;
            case LEFT:
            default:
                robot.driveTrain.moveToPositionInches(closestPosition, moveToPositionPower);
                break;
        }

        robot.driveTrain.gyroTurn(.1, targetAngle + 15);
        robot.driveTrain.gyroTurn(.05, targetAngle + 15);
        robot.driveTrain.moveToPositionInches(2, 1);

        robot.slamDunker.dunkSlowWithAutoStop();
        robot.driveTrain.moveToInches(2, .25);
        sleep(250);
        robot.slamDunker.retractDunkNoWait();


        robot.driveTrain.moveToInches(-5, moveToPositionPower);
        robot.driveTrain.moveToInches(5, moveToPositionPower);

        robot.driveTrain.gyroTurn(.05, targetAngle);
        robot.intakeMecanism.intake();
        robot.driveTrain.moveToPositionInches(35, 1);
        robot.slamDunker.retract();
        robot.driveTrain.moveToPositionInches(-5, 1);

        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            robot.driveTrain.gyroTurn(.05, targetAngle - 25);
        } else {
            robot.driveTrain.gyroTurn(.05, targetAngle + 25);
        }

        robot.driveTrain.moveToPositionInches(10, 1);
        robot.driveTrain.moveToPositionInches(-5, 1);

        switch (vuMark) {
            case CENTER:
                robot.driveTrain.gyroTurn(.05, targetAngle);
                break;
            case RIGHT:
                robot.driveTrain.gyroTurn(.05, targetAngle + 15);
                break;
            case LEFT:
            default:
                robot.driveTrain.gyroTurn(.05, targetAngle - 15);
                break;
        }

        robot.driveTrain.moveToPositionInches(-10, moveToPositionPower);
        robot.intakeMecanism.outtakeFully();
        robot.intakeMecanism.intake();

        robot.driveTrain.moveToPositionInches(7, moveToPositionPower);
        robot.driveTrain.moveToPositionInches(-7, moveToPositionPower);


        if (settings.shouldDunk) {
            //Score second Glyph
             if (vuMark == RelicRecoveryVuMark.CENTER) {
                robot.driveTrain.gyroTurn(.05, targetAngle + 25);
            } else {
                robot.driveTrain.gyroTurn(.05, targetAngle);
            }
            robot.driveTrain.moveToPositionInches(-3, moveToPositionPower);
            robot.slamDunker.dunkSlowWithAutoStop();
            robot.intakeMecanism.stopIntake();
            sleep(250);
            robot.driveTrain.moveToPositionInches(5, .25);
            robot.slamDunker.retractDunkNoWait();
            robot.driveTrain.moveToPositionInches(-5, .25);
            robot.slamDunker.retract();
        }

        //Push second glyph into box
        robot.driveTrain.moveToPositionInches(-10, 1);
        robot.slamDunker.dunkMotor.setPower(0);
        robot.relicMecanism.storeServos();
        robot.driveTrain.moveToPositionInches(5, 1);
    }
}
