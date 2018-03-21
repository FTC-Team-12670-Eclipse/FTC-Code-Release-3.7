package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotModules.Robot;
import org.firstinspires.ftc.teamcode.RobotModules.VuforiaRelicRecoveryGetter;
import org.firstinspires.ftc.teamcode.UniversalConstants;

public class AutonomousBlueEasy extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        double angleToAlignWithCryptoBox = 90;
        waitForStart();

        robot.driveTrain.moveToMillimeters(750, 1);
        robot.driveTrain.turnRelative(.15, angleToAlignWithCryptoBox);

        VuforiaRelicRecoveryGetter.DistanceOffsets distanceOffsets = robot.vuforiaRelicRecoveryGetter.getOffset();
        while (!distanceOffsets.foundValues) {
            robot.driveTrain.translateBy(-1, 0, 0);
            distanceOffsets = robot.vuforiaRelicRecoveryGetter.getOffset();
        }

        robot.driveTrain.park();

    }
}
