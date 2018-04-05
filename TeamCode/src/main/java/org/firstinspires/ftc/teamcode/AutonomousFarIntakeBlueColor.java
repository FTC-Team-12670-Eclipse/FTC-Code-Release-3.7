package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Autonomous(name = "Blue FAR Color")
public class AutonomousFarIntakeBlueColor extends LinearOpMode {

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
        robot.jewelSwatter.removeJewelOfColor(color);
        AutonomousUtil.driveRobotOffRamp(robot, AutonomousUtil.AllianceColor.Blue);

        robot.relicMecanism.swingAwayFromWall();

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }
        robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, 90);
        robot.driveTrain.moveToInches(3, .25 * FORWARDS_SPEED_MODIFIER);

        double targetAngle = 180;
        double power = .1;
        double distance = 7;
        DistanceUnit unit = DistanceUnit.CM;

        robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);
        robot.driveTrain.park();
        robot.driveTrain.swingColorDistanceDown();
        robot.driveTrain.moveToInches(3, .2 * FORWARDS_SPEED_MODIFIER);

        switch (vuMark) {
            case RIGHT:
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceRightCoast(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.translateBy(0, power * STRAFE_SPEED_MODIFIER, 0);
                robot.driveTrain.swingColorDistanceUp();
                robot.driveTrain.encoderStrafeToInches(-1.5, power * STRAFE_SPEED_MODIFIER);
                robot.driveTrain.swingColorDistanceDown();
            case CENTER:
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceRightCoast(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.translateBy(0, power * STRAFE_SPEED_MODIFIER, 0);
                robot.driveTrain.swingColorDistanceUp();
                robot.driveTrain.encoderStrafeToInches(-1.5, power * STRAFE_SPEED_MODIFIER);
                robot.driveTrain.swingColorDistanceDown();
            case LEFT:
            default:
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceRightCoast(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.translateBy(0, power * STRAFE_SPEED_MODIFIER, 0);
                robot.driveTrain.swingColorDistanceUp();
                robot.driveTrain.encoderStrafeToInches(-1.5, power * STRAFE_SPEED_MODIFIER);
                robot.driveTrain.swingColorDistanceDown();

                robot.driveTrain.park();
                robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceLeft(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.park();
                break;
        }

        robot.driveTrain.storeColorDistance();

        robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);
        robot.intakeMecanism.deployFoldoutIntake();
        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.setIntakePowers(.5, -.5);
        sleep(500);
        robot.intakeMecanism.setIntakePowers(-.75);
        robot.driveTrain.moveToInches(3, .25 * FORWARDS_SPEED_MODIFIER);

        robot.driveTrain.moveToInches(-7, .35 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.stopIntake();
        robot.relicMecanism.storeServos();

        stop();


    }
}