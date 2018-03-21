package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotModules.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotModules.VuforiaRelicRecoveryGetter;
import org.firstinspires.ftc.teamcode.UniversalConstants;

@Disabled
@Autonomous(name = "Move Tester")
public class SimpleMoveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveTrain mecanumDriveTrain = new MecanumDriveTrain(this, DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while(opModeIsActive()){
            while(opModeIsActive() && !gamepad1.a){
                idle();
            }
            int startPos = mecanumDriveTrain.leftBack.getCurrentPosition() + mecanumDriveTrain.rightBack.getCurrentPosition();
            int cPos = mecanumDriveTrain.leftBack.getCurrentPosition() + mecanumDriveTrain.rightBack.getCurrentPosition();
            while(opModeIsActive() && (cPos-startPos) < UniversalConstants.ticksPerInch*10){
                cPos = mecanumDriveTrain.leftBack.getCurrentPosition() + mecanumDriveTrain.rightBack.getCurrentPosition();
                mecanumDriveTrain.setAll(1);
            }
            mecanumDriveTrain.park();
        }
    }
}
