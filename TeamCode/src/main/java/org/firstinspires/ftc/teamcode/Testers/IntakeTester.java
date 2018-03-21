package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotModules.IntakeMecanism;

@Disabled
@TeleOp(name = "Intake Tester")
public class IntakeTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IntakeMecanism intakeMecanism = new IntakeMecanism(this);

        waitForStart();

        while (opModeIsActive()) {
            intakeMecanism.updateByGamepad();
            intakeMecanism.updateTelemetry();
            telemetry.update();
        }
    }
}
