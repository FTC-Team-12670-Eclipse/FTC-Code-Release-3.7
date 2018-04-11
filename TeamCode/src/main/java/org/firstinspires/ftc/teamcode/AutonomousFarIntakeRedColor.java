package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Disabled
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
        robot.driveTrain.autoWallDistanceSensor(26, .15 * FORWARDS_SPEED_MODIFIER, DistanceUnit.CM);

        robot.relicMecanism.swingAwayFromWall();

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }

        double targetAngle = 0;

        robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);

        double power = .1;
        double distance = UniversalConstants.distanceRedODS;
        DistanceUnit unit = DistanceUnit.CM;


        switch (vuMark) {
            case LEFT:
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceLeftCoast(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.translateBy(0, -power * STRAFE_SPEED_MODIFIER, 0);
                robot.driveTrain.swingColorDistanceUp();
                robot.driveTrain.encoderStrafeToInches(1.5, power * STRAFE_SPEED_MODIFIER, targetAngle);
                robot.driveTrain.swingColorDistanceDown();
            case CENTER:
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceLeftCoast(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.translateBy(0, -power * STRAFE_SPEED_MODIFIER, 0);
                robot.driveTrain.swingColorDistanceUp();
                robot.driveTrain.encoderStrafeToInches(1.5, power * STRAFE_SPEED_MODIFIER, targetAngle);
                robot.driveTrain.swingColorDistanceDown();
            case RIGHT:
            default:
                robot.driveTrain.park();
                robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceLeft(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.park();
                break;
        }

        robot.driveTrain.storeColorDistance();


        robot.intakeMecanism.deployFoldoutIntake();
        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.setIntakePowers(.35, -.35);
        sleep(500);
        robot.intakeMecanism.setIntakePowersOverride(-.25);
        robot.driveTrain.moveToInches(3, .15 * FORWARDS_SPEED_MODIFIER);

        robot.driveTrain.moveToInches(-9, .15 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.stopIntake();
        robot.relicMecanism.storeServos();

        stop();

    }
}
