package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Autonomous(name = "Blue CLOSE Special")
public class AutonomousCloseIntakeBlueRangeMG extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double TURN_SPEED_MODIFIER = 1;
        double STRAFE_SPEED_MODIFIER = 1;
        double FORWARDS_SPEED_MODIFIER = 1;

        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.AllianceColor color = AutonomousUtil.getColorToDislodge(this, AutonomousUtil.AllianceColor.Red, robot);

        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.relicMecanism.swingElbowUp();
        robot.intakeMecanism.deployFoldoutIntake();
        robot.jewelSwatter.removeJewelOfColor(color);
        AutonomousUtil.driveRobotOffRamp(robot, AutonomousUtil.AllianceColor.Blue);

        robot.relicMecanism.swingAwayFromWall();

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        double targetAngle = 90;
        robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);

        switch (vuMark) {
            case RIGHT:
                robot.driveTrain.encoderStrafeToInches(-4.5, .25);
                break;
            case CENTER:
                robot.driveTrain.encoderStrafeToInches(-4.5 / 2, .25);
                break;
        }
        robot.driveTrain.park();

        robot.driveTrain.moveToInches(2.5, .1 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.deployFoldoutIntake();
        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.setIntakePowers(.35, -.35);
        sleep(500);
        robot.intakeMecanism.setIntakePowersOverride(-.25);
        robot.driveTrain.moveToInches(8, .2 * FORWARDS_SPEED_MODIFIER);

        robot.driveTrain.moveToInches(-10, .25 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.stopIntake();
        robot.relicMecanism.storeServos();

        targetAngle = -90;
        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.intakeMecanism.intake();
        robot.driveTrain.moveToInches(25, .25);
        robot.driveTrain.moveToInches(-15, .25);

        targetAngle = 90;
        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.driveTrain.park();

        switch (vuMark) {
            case CENTER:
                robot.driveTrain.encoderStrafeToInches(5.5 / 2, .25);
                robot.driveTrain.moveToInches(4, .25);
                break;
            case LEFT:
                robot.driveTrain.gyroTurn(.05, targetAngle - 20);
                break;
            case RIGHT:
            default:
                robot.driveTrain.gyroTurn(.05, targetAngle + 20);
                break;
        }

        robot.driveTrain.moveToInches(12, .2 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.setIntakePowers(.35, -.35);
        sleep(500);
        robot.intakeMecanism.setIntakePowersOverride(-.25);

        robot.driveTrain.moveToInches(8, .2 * FORWARDS_SPEED_MODIFIER);

        robot.driveTrain.moveToInches(-10, .25 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.stopIntake();
        robot.relicMecanism.storeServos();


    }
}
