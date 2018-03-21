package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotModules.Robot;

@Disabled
@Autonomous(name = "Just Jewel, Red Side")
public class JustJewelParkRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.addAndUpdateTelemetry("Ready to go!");
        AutonomousUtil.AllianceColor color = AutonomousUtil.getColorToDislodge(this, AutonomousUtil.AllianceColor.Blue, robot);
        robot.jewelSwatter.removeJewelOfColor(color);
        AutonomousUtil.driveRobotOffRamp(robot, AutonomousUtil.AllianceColor.Red);
    }
}
