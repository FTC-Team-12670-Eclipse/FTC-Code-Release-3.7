package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotModules.SlamDunker;

@TeleOp(name = "Drop the Dunk!")
public class DropTheDunk extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SlamDunker slamDunker = new SlamDunker(this);
        while (!isStarted()) {
            telemetry.addLine("Ready");
            telemetry.update();
        }
        slamDunker.dropTheDunk();
        while(opModeIsActive()) {
            if(!slamDunker.dunkMotor.isBusy()){
                slamDunker.dunkMotor.setPower(0);
            }
        }
    }
}
