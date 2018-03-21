package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotModules.SmartRangeMR;
import org.firstinspires.ftc.teamcode.UniversalConstants;

@Disabled
@Autonomous(name = "Distance Sensor Tester")
public class DistanceSensorTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ModernRoboticsI2cRangeSensor leftDistanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, UniversalConstants.distanceSensorLeft);
        ModernRoboticsI2cRangeSensor rightDistanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, UniversalConstants.distanceSensorRight);
        while(!isStarted()) {
            telemetry.addData("Left Centimeters", leftDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Inches", leftDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Centimeters", rightDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Inches", rightDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        while (opModeIsActive()) {
            telemetry.addData("Left Centimeters", leftDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Inches", leftDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Centimeters", rightDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Inches", rightDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
