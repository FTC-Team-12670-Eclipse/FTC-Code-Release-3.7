package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Autonomous(name = "Vuforia Trial Demo")
public class VuforiaTrialTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("RTG");
        waitForStart();
        RelicRecoveryVuMark vuMark;
        while (opModeIsActive()) {
            robot.vuforiaRelicRecoveryGetter.activateTrackables();
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                while (robot.vuforiaRelicRecoveryGetter.getPattern() == RelicRecoveryVuMark.LEFT) {
                    robot.driveTrain.setAll(.05);
                    if (robot.vuforiaRelicRecoveryGetter.getPattern() != RelicRecoveryVuMark.LEFT) {
                        robot.driveTrain.park();
                        break;
                    }
                }
                robot.driveTrain.park();
            }
            vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                while (robot.vuforiaRelicRecoveryGetter.getPattern() == RelicRecoveryVuMark.RIGHT) {
                    robot.driveTrain.setAll(-.05);
                    if (robot.vuforiaRelicRecoveryGetter.getPattern() != RelicRecoveryVuMark.RIGHT) {
                        robot.driveTrain.park();
                        break;
                    }
                }
                robot.driveTrain.park();
            }
        }
    }
}