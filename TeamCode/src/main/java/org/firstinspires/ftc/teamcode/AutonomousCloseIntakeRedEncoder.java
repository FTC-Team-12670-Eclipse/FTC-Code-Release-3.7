package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Autonomous(name = "Red CLOSE")
public class AutonomousCloseIntakeRedEncoder extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.AllianceColor color = AutonomousUtil.getColorToDislodge(this, AutonomousUtil.AllianceColor.Blue, robot);
        double moveToPositionPower = .2;

        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.relicMecanism.swingElbowUp();
        robot.jewelSwatter.removeJewelOfColor(color);
        AutonomousUtil.driveRobotOffRamp(robot, AutonomousUtil.AllianceColor.Red);

        robot.intakeMecanism.deployFoldoutIntake();

        robot.intakeMecanism.intake();

        robot.driveTrain.gyroTurn(.2, 0);
        robot.intakeMecanism.stopIntake();
        robot.driveTrain.gyroTurn(.05, 0);

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        robot.jewelSwatter.wristServo.setPosition(UniversalConstants.jewelWristStored);

        double closestPosition = 1.25;
        double targetAngle = -90;

        switch (vuMark) {
            case CENTER:
                robot.driveTrain.moveToPositionInches(closestPosition + 6.5, .75);
                break;
            case LEFT:
                robot.driveTrain.moveToPositionInches(closestPosition + 13.5, .75);
                break;
            case RIGHT:
            default:
                robot.driveTrain.moveToPositionInches(closestPosition, .75);
                break;
        }

        robot.driveTrain.gyroTurn(.05, targetAngle);
        robot.driveTrain.gyroTurn(.05, targetAngle);
        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.intakeMecanism.deployFoldoutIntake();
        robot.intakeMecanism.intake();
        sleep(250);
        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.outtake();

        robot.driveTrain.moveToInches(3, moveToPositionPower);
        robot.driveTrain.moveToInches(-5, moveToPositionPower);
        robot.driveTrain.moveToInches(5, moveToPositionPower);
        robot.driveTrain.moveToInches(-9, moveToPositionPower);
        robot.driveTrain.moveToInches(8, moveToPositionPower);
        robot.driveTrain.moveToInches(-3, moveToPositionPower);
    }
}
