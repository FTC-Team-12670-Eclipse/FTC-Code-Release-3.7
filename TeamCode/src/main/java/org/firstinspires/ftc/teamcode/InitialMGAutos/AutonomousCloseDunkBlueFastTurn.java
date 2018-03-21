package org.firstinspires.ftc.teamcode.InitialMGAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutonomousUtil;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;
import org.firstinspires.ftc.teamcode.UniversalConstants;

@Disabled
@Autonomous(name = "Blue CLOSE Dunk/MGlyph")
public class AutonomousCloseDunkBlueFastTurn extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.AllianceColor color = AutonomousUtil.getColorToDislodge(this, AutonomousUtil.AllianceColor.Red, robot);
        double moveToPositionPower = .2;

        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.relicMecanism.swingElbowUp();
        robot.jewelSwatter.removeJewelOfColor(color);
        AutonomousUtil.driveRobotOffRamp(robot, AutonomousUtil.AllianceColor.Blue);

        robot.driveTrain.gyroTurn(.2, 0);
        robot.driveTrain.gyroTurn(.05, 0);

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        robot.jewelSwatter.wristServo.setPosition(UniversalConstants.jewelWristStored);
        robot.intakeMecanism.deployFoldoutIntake();

        double closestPosition = .5;
        double targetAngle = -90;

        switch (vuMark) {
            case CENTER:
                robot.driveTrain.moveToInches(closestPosition + 5.5, moveToPositionPower);
                break;
            case RIGHT:
                robot.driveTrain.moveToInches(closestPosition + 11, moveToPositionPower);
                break;
            default:
            case LEFT:
                robot.driveTrain.moveToInches(closestPosition, moveToPositionPower);
                break;
        }
        robot.driveTrain.gyroTurn(.25, targetAngle + 15);
        robot.driveTrain.gyroTurn(.05, targetAngle + 15);

        robot.driveTrain.moveToInches(-2.5, moveToPositionPower);

        robot.slamDunker.dunkSlowWithAutoStop();
        sleep(250);
        robot.driveTrain.moveToInches(2.5, moveToPositionPower);
        robot.slamDunker.retractDunkNoWait();

        robot.driveTrain.gyroTurn(.2, targetAngle);
        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.intakeMecanism.intake();
        robot.driveTrain.moveToInches(22, .25);
        robot.driveTrain.moveToInches(-2, .25);
        robot.driveTrain.alignToWithinOf(targetAngle + 25, .35, 10);
        robot.driveTrain.moveToInches(7, .25);
        robot.driveTrain.moveToInches(-7, .25);
        robot.driveTrain.gyroTurn(.05, targetAngle - 10);
        robot.intakeMecanism.stopIntake();
        robot.driveTrain.moveToInches(-20, .2);
        robot.slamDunker.dunkSlowWithAutoStop();
        sleep(250);
        robot.driveTrain.moveToInches(5, .5);
        robot.slamDunker.retractDunkNoWait();
        robot.driveTrain.moveToInches(-10, .25);
        robot.driveTrain.moveToInches(5, .25);


    }
}
