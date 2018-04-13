package org.firstinspires.ftc.teamcode.Autonomous.MultiGlyph;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutonomousUtil;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Autonomous(name = "Blue CLOSE Special")
public class AutonomousCloseIntakeBlueMG extends LinearOpMode {

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

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        double targetAngle = -90;

        double closestPosition = 27;
        switch (vuMark) {
            case CENTER:
                robot.driveTrain.moveToInches(closestPosition + 7, .25);
                break;
            case LEFT:
                robot.driveTrain.moveToInches(closestPosition + 14, .25);
            case RIGHT:
            default:
                robot.driveTrain.moveToInches(closestPosition, .25);
                break;
        }

        robot.relicMecanism.swingAwayFromWall();

        robot.driveTrain.park();

        robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);

        robot.driveTrain.moveToInches(7, .25 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.deployFoldoutIntake();
        robot.intakeMecanism.outtakeFully();
        robot.intakeMecanism.setIntakePowers(.35, -.35);
        sleep(500);
        robot.intakeMecanism.setIntakePowers(-1);
        robot.driveTrain.moveToInches(7, .15 * FORWARDS_SPEED_MODIFIER);

        robot.driveTrain.moveToInches(-10, .15 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.stopIntake();
        robot.relicMecanism.storeServos();

        targetAngle = 90;
        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.intakeMecanism.intake();
        robot.driveTrain.moveToInches(25, .35);
        robot.driveTrain.moveToInches(-15, .25);

        targetAngle = -90;
        robot.driveTrain.gyroTurn(.05, targetAngle);

        robot.driveTrain.park();

        switch (vuMark) {
            case CENTER:
                robot.driveTrain.gyroTurn(.05, targetAngle - 15);
                break;
            case LEFT:
                robot.driveTrain.gyroTurn(.05, targetAngle - 25);
                break;
            case RIGHT:
            default:
                robot.driveTrain.gyroTurn(.05, targetAngle + 25);
                break;
        }

        robot.driveTrain.moveToInches(15, .25 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.outtakeFully();
        robot.intakeMecanism.setIntakePowers(.35, -.35);
        sleep(500);
        robot.intakeMecanism.setIntakePowersOverride(-.25);

        robot.driveTrain.moveToInches(6, .15 * FORWARDS_SPEED_MODIFIER);

        robot.driveTrain.moveToInches(-8, .15 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.stopIntake();
        robot.relicMecanism.storeServos();


    }
}
