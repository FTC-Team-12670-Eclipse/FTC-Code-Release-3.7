package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Autonomous(name = "Red FAR Color")
public class AutonomousFarIntakeRedColor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double TURN_SPEED_MODIFIER = 1;
        double STRAFE_SPEED_MODIFIER = 1;
        double FORWARDS_SPEED_MODIFIER = 1;

        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.AllianceColor color = AutonomousUtil.getColorToDislodge(this, AutonomousUtil.AllianceColor.Blue, robot);

        waitForStart();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.relicMecanism.swingElbowUp();
        robot.driveTrain.swingColorDistanceDown();
        robot.jewelSwatter.removeJewelOfColor(color);
        AutonomousUtil.driveRobotOffRamp(robot, AutonomousUtil.AllianceColor.Red);


        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        double targetAngle = 0;

        robot.driveTrain.gyroTurn(.05*TURN_SPEED_MODIFIER, targetAngle);

        double power = .1;
        double distance = 7;
        DistanceUnit unit = DistanceUnit.CM;


        switch (vuMark) {
            case LEFT:
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceLeftCoast(power*STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.translateBy(0, -power*STRAFE_SPEED_MODIFIER, 0);
                robot.driveTrain.swingColorDistanceUp();
                robot.driveTrain.encoderStrafeToInches(1.5*STRAFE_SPEED_MODIFIER, power);
                robot.driveTrain.swingColorDistanceDown();
                break;
            case CENTER:
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceLeftCoast(power*STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.translateBy(0, -power*STRAFE_SPEED_MODIFIER, 0);
                robot.driveTrain.swingColorDistanceUp();
                robot.driveTrain.encoderStrafeToInches(1.5, power*STRAFE_SPEED_MODIFIER);
                robot.driveTrain.swingColorDistanceDown();
            case RIGHT:
            default:
                robot.driveTrain.park();
                robot.driveTrain.gyroTurn(.05*TURN_SPEED_MODIFIER, targetAngle);
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceLeft(power*STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.park();
                break;
        }

        robot.driveTrain.storeColorDistance();

        robot.driveTrain.moveToInches(3, .25 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.deployFoldoutIntake();
        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.setIntakePowers(-.75);

        robot.driveTrain.moveToInches(-7, .35 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.stopIntake();
        robot.relicMecanism.storeServos();

        stop();

        /*
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            robot.driveTrain.moveToInches(-15, 1);
            robot.driveTrain.gyroTurn(.25, 145);
            robot.intakeMecanism.intake();
            robot.driveTrain.moveToInches(30, .75);
            robot.driveTrain.moveToInches(-15, .5);
            robot.driveTrain.gyroTurn(.25, -45);
            robot.driveTrain.moveToInches(25, .6);
            robot.driveTrain.swingColorDistanceUp();
            robot.driveTrain.gyroTurn(.25, 0);
            robot.intakeMecanism.outtake();
        }
        */

    }
}
