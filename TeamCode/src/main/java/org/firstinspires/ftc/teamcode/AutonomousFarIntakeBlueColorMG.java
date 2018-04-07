package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Autonomous(name = "Blue FAR Color MG")
public class AutonomousFarIntakeBlueColorMG extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double TURN_SPEED_MODIFIER = .8;
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
        robot.driveTrain.swingColorDistanceDown();
        robot.jewelSwatter.removeJewelOfColor(color);
        robot.relicMecanism.swingAwayFromWall();
        robot.intakeMecanism.deployFoldoutIntake();
        robot.driveTrain.encoderStrafeToInches(10, .25 * STRAFE_SPEED_MODIFIER);
        robot.driveTrain.park();


        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }
        robot.intakeMecanism.setIntakePowers(.5);
        robot.driveTrain.turnWithOneSideOnly(45, 5, 0, .25);
        robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, 45);
        robot.driveTrain.moveToInches(4, .35 * FORWARDS_SPEED_MODIFIER);
        robot.intakeMecanism.outtake();
        sleep(200);
        robot.intakeMecanism.setIntakePowers(.5);
        robot.driveTrain.moveToInches(1.5, .35 * FORWARDS_SPEED_MODIFIER);
        robot.driveTrain.moveToInches(-5, .25 * FORWARDS_SPEED_MODIFIER);

        double targetAngle = 180;

        robot.intakeMecanism.stopIntake();
        robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);

        //Add stuff here about forwards wall sensor
        //new stuff added starts here
        robot.driveTrain.autoWallDistanceSensor(3, 1 * FORWARDS_SPEED_MODIFIER, DistanceUnit.CM);

        //end of new stuff added

        //og code was this:
        //robot.driveTrain.moveToInches(37 , .2 * FORWARDS_SPEED_MODIFIER);
        robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);

        robot.driveTrain.swingColorDistanceDown();

        double power = .1;
        double distance = 12;
        DistanceUnit unit = DistanceUnit.CM;

        switch (vuMark) {
            case LEFT:
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceLeftCoast(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.swingColorDistanceUp();
                robot.driveTrain.encoderStrafeToInches(1.5, power * STRAFE_SPEED_MODIFIER);
                robot.driveTrain.swingColorDistanceDown();
            case CENTER:
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceLeftCoast(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.swingColorDistanceUp();
                robot.driveTrain.encoderStrafeToInches(1.5, power * STRAFE_SPEED_MODIFIER);
                robot.driveTrain.swingColorDistanceDown();
            case RIGHT:
            default:
                robot.driveTrain.park();
                robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);
                robot.driveTrain.swingColorDistanceDown();
                robot.driveTrain.strafeToDistanceLeft(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.strafeToDistanceLeft(power * STRAFE_SPEED_MODIFIER, distance, targetAngle, unit);
                robot.driveTrain.park();
                break;
        }

        robot.driveTrain.swingColorDistanceUp();
        robot.intakeMecanism.deployFoldoutIntake();
        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.setIntakePowers(.5, -.5);
        sleep(500);
        robot.intakeMecanism.setIntakePowersOverride(-1);
        robot.driveTrain.moveToInches(4, .25 * FORWARDS_SPEED_MODIFIER);
        robot.driveTrain.moveToInches(-4, .25 * FORWARDS_SPEED_MODIFIER);
        robot.intakeMecanism.intake();
        sleep(200);
        robot.intakeMecanism.stopIntake();
        robot.driveTrain.moveToInches(-4, .25 * FORWARDS_SPEED_MODIFIER);
        switch (vuMark) {
            case CENTER:
            case RIGHT:
            case UNKNOWN:
                robot.driveTrain.encoderStrafeToInches(4, power * STRAFE_SPEED_MODIFIER);
                robot.driveTrain.park();
                break;
            case LEFT:
            default:
                robot.driveTrain.encoderStrafeToInches(-4, power * STRAFE_SPEED_MODIFIER);
                robot.driveTrain.park();
        }

        robot.driveTrain.gyroTurn(.05 * TURN_SPEED_MODIFIER, targetAngle);
        robot.driveTrain.moveToInches(4, .25 * FORWARDS_SPEED_MODIFIER);
        robot.intakeMecanism.outtakeSlowly();
        robot.intakeMecanism.setIntakePowers(.5, -.5);
        sleep(500);
        robot.intakeMecanism.setIntakePowers(-.75);
        robot.driveTrain.moveToInches(3, .25 * FORWARDS_SPEED_MODIFIER);
        robot.driveTrain.moveToInches(-5, .35 * FORWARDS_SPEED_MODIFIER);

        robot.intakeMecanism.stopIntake();
        robot.relicMecanism.storeServos();

    }
}
