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
@Autonomous(name = "Red CLOSE MG")
public class AutonomousCloseDunkRedMG extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousUtil.AllianceColor allianceColor = AutonomousUtil.AllianceColor.Red;
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.Settings settings = AutonomousUtil.getStartingSettings(this, AutonomousUtil.switchColor(allianceColor), robot);
        double moveToPositionPower = .2;

        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.relicMecanism.swingElbowUp();
        robot.jewelSwatter.removeJewelOfColor(settings.allianceColor);
        AutonomousUtil.driveRobotOffRamp(robot, allianceColor, .6);

        robot.intakeMecanism.deployFoldoutIntake();

        robot.intakeMecanism.intake();
        sleep(250);
        robot.intakeMecanism.stopIntake();

        robot.driveTrain.gyroTurn(.2, 0);
        robot.driveTrain.gyroTurn(.05, 0);

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        robot.jewelSwatter.wristServo.setPosition(UniversalConstants.jewelWristStored);

        double closestPosition = -7;
        double targetAngle = 90;

        switch (vuMark) {
            case CENTER:
                robot.driveTrain.moveToInches(closestPosition + 7, moveToPositionPower);
                break;
            case RIGHT:
                robot.driveTrain.moveToInches(closestPosition + 14, moveToPositionPower);
                break;
            case LEFT:
            default:
                robot.driveTrain.moveToInches(closestPosition, moveToPositionPower);
                break;
        }

        robot.driveTrain.gyroTurn(.25, targetAngle + 5);
        robot.driveTrain.gyroTurn(.05, targetAngle + 5);

        robot.slamDunker.dunkSlowWithAutoStop();
        sleep(250);
        robot.slamDunker.retractDunkNoWait();


        robot.driveTrain.moveToInches(-5, .75);
        robot.intakeMecanism.intake();


        robot.driveTrain.moveToPositionInches(25, moveToPositionPower);
        robot.driveTrain.gyroTurn(.25, targetAngle + 25);
        robot.driveTrain.gyroTurn(.05, targetAngle + 25);
        robot.driveTrain.moveToPositionInches(10, moveToPositionPower);
        robot.driveTrain.moveToPositionInches(-10, moveToPositionPower);
        robot.driveTrain.gyroTurn(.25, targetAngle);
        robot.driveTrain.gyroTurn(.05, targetAngle);
        robot.driveTrain.moveToPositionInches(-25, moveToPositionPower);

        if(settings.shouldDunk) {
            //Score second Glyph
            robot.slamDunker.dunkSlowWithAutoStop();
            sleep(250);
            robot.slamDunker.retractDunkNoWait();
        }

        //Push second glyph into box
        robot.driveTrain.moveToPositionInches(-10, 1);
        robot.slamDunker.dunkMotor.setPower(0);
        robot.relicMecanism.storeServos();
        robot.driveTrain.moveToPositionInches(5, 1);
    }
}
