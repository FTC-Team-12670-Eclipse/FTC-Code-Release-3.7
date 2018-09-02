package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Autonomous (name = "Vuforia Trial Demo")
public class VuforiaTrialTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("RTG");
        RelicRecoveryVuMark vuMark;
        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();

        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        if (vuMark == RelicRecoveryVuMark.LEFT){
            while(vuMark == RelicRecoveryVuMark.LEFT){
                robot.driveTrain.setAll(.5);
            }
            robot.driveTrain.setAll(0);
        }
        vuMark = robot.vuforiaRelicRecoveryGetter.getPattern();
        if (vuMark == RelicRecoveryVuMark.RIGHT){
            while(vuMark == RelicRecoveryVuMark.RIGHT){
                robot.driveTrain.setAll(-.5);
            }
            robot.driveTrain.setAll(0);
        }
    }
}
