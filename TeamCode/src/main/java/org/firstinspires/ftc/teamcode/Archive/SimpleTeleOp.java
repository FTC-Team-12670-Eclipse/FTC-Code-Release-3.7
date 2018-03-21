package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotModules.IntakeMecanism;
import org.firstinspires.ftc.teamcode.RobotModules.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotModules.SlamDunker;

@Disabled
@TeleOp(name = "Simple Tele Op")
public class SimpleTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDriveTrain drive = new MecanumDriveTrain(this, DcMotor.ZeroPowerBehavior.FLOAT);
        IntakeMecanism intakeMecanism = new IntakeMecanism(this);
        SlamDunker slamDunker = new SlamDunker(this);

        waitForStart();

        while (opModeIsActive()) {
            drive.updateByGamepad();
            drive.updateTelemetry();

            intakeMecanism.updateByGamepad();
            intakeMecanism.updateTelemetry();

            slamDunker.updateByGamepad();
            slamDunker.updateTelemetry();

            telemetry.update();
        }
    }

}
