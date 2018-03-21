package org.firstinspires.ftc.teamcode.AutonomousIntakeNotFast;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutonomousUtil;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;
import org.firstinspires.ftc.teamcode.UniversalConstants;

@Disabled
@Autonomous(name = "Blue CLOSE")
public class AutonomousCloseIntakeBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        RelicRecoveryVuMark vuMark;
        AutonomousUtil.AllianceColor color = AutonomousUtil.AllianceColor.Red;
        while (!isStarted()) {
            if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad2.left_bumper && gamepad2.right_bumper) {
                color = AutonomousUtil.switchColor(color);
            }
            telemetry.addLine("Ready to Go!");
            telemetry.addData("Color to Dislodge", color.name());
            telemetry.addLine("Remember, the alliance you dislodge does NOT earn the points");
            telemetry.addLine("Press all 4 bumpers to switch the color");
            telemetry.update();
        }

        waitForStart();
        double startTime = getRuntime();

        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        robot.jewelSwatter.removeJewelOfColor(AutonomousUtil.AllianceColor.Red);
        robot.driveTrain.moveToInches(27, .25);
        robot.driveTrain.alignToWithinOf(0);

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        }
        robot.jewelSwatter.wristServo.setPosition(UniversalConstants.jewelWristStored);
        switch (vuMark) {
            case CENTER:
                robot.driveTrain.moveToInches(11.5, .35);
                break;
            case RIGHT:
                robot.driveTrain.moveToInches(18, .35);
                break;
            default:
            case LEFT:
                robot.driveTrain.moveToInches(5, .35);
                break;
        }
        robot.driveTrain.turnExactForcePower(105, .5, 35);
        robot.driveTrain.alignToWithinOf(105);
        robot.scoreBlock();

        if (startTime - getRuntime() < 25) {
            robot.intakeMecanism.outtake();
            robot.driveTrain.moveToInches(5, .25);
            robot.driveTrain.moveToInches(-5, .25);
        }
    }
}
