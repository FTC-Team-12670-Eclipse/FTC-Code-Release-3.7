package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Disabled
@Autonomous(name = "Red FAR MG")
public class AutonomousFarDunkRedMG extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousUtil.AllianceColor startingColor = AutonomousUtil.AllianceColor.Red;
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.Settings settings = AutonomousUtil.getStartingSettings(this, AutonomousUtil.switchColor(startingColor), robot);
        double moveToPositionPower = .4;
        double turnSpeed = .15;
        double strafePower = .15;

        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.relicMecanism.swingElbowUp();
        robot.jewelSwatter.removeJewelOfColor(settings.allianceColor);
        AutonomousUtil.driveRobotOffRampFast(robot, startingColor, .75);

        robot.driveTrain.gyroTurn(turnSpeed, 180);

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        robot.jewelSwatter.wristServo.setPosition(UniversalConstants.jewelWristStored);
        robot.intakeMecanism.deployFoldoutIntake();

        double closestPosition = 20;
        double targetAngle = 180;
        robot.driveTrain.gyroTurn(turnSpeed, targetAngle);

        switch (vuMark) {
            case CENTER:
                robot.driveTrain.strafeToLeftPosition(strafePower, targetAngle, DistanceUnit.INCH, closestPosition + 7.5);
                break;
            case RIGHT:
                robot.driveTrain.strafeToLeftPosition(strafePower, targetAngle, DistanceUnit.INCH, closestPosition + 15);
                break;
            default:
            case LEFT:
                robot.driveTrain.strafeToLeftPosition(strafePower, targetAngle, DistanceUnit.INCH, closestPosition);
                break;
        }

        //Scores first glyph
        robot.driveTrain.gyroTurn(turnSpeed, targetAngle - 10);
        robot.driveTrain.moveToPositionInches(-1.5, moveToPositionPower);
        robot.slamDunker.dunkSlowWithAutoStop();
        sleep(250);
        robot.driveTrain.moveToPositionInches(4, moveToPositionPower);
        robot.slamDunker.retractDunkNoWait();

        //Pushes Glyph into box
        robot.driveTrain.gyroTurn(turnSpeed, targetAngle);
        robot.driveTrain.moveToPositionInches(-5, moveToPositionPower);
        robot.slamDunker.dunkMotor.setPower(0);
        robot.driveTrain.moveToPositionInches(5, moveToPositionPower);
        robot.driveTrain.gyroTurn(turnSpeed, targetAngle);

        //Strafes to column closest to Glyph pit, no matter which column we scored in
        robot.driveTrain.strafeToRightPosition(strafePower, targetAngle, DistanceUnit.INCH, closestPosition + 18);
        robot.driveTrain.moveToPositionInches(25, 1);

        //Goes into glyph pit to get glyphs
        robot.driveTrain.gyroTurn(turnSpeed, targetAngle - 45);
        robot.intakeMecanism.intake();
        robot.driveTrain.moveToPositionInches(15, 1);
        robot.driveTrain.moveToPositionInches(-5, 1);

        //Once in the glyph pit, try to get more glyphs
        robot.driveTrain.gyroTurn(turnSpeed, targetAngle - 25);
        robot.driveTrain.moveToPositionInches(10, 1);
        robot.driveTrain.moveToPositionInches(-10, 1);

        robot.driveTrain.gyroTurn(turnSpeed, targetAngle - 45);

        //Fix Glyphs
        robot.driveTrain.moveToPositionInches(-5, 1);
        robot.driveTrain.gyroTurn(turnSpeed, targetAngle);
        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.intake();
        robot.driveTrain.moveToPositionInches(4, 1);

        //Goes back to the Cryptobox
        robot.driveTrain.moveToPositionInches(-10, 1);
        robot.driveTrain.strafeToRightPosition(strafePower, targetAngle, DistanceUnit.INCH, closestPosition + 15);

        robot.driveTrain.moveToPositionInches(-6, 1);
        robot.driveTrain.gyroTurn(turnSpeed, targetAngle - 5);

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
