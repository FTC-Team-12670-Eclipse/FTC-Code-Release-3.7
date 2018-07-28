package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UniversalConstants;

public class DrivetrainY {
    private DcMotor LR, RR, LF, RF;
    private Telemetry telem;
    private LinearOpMode linearOPMode;

    private DriveMode drivemode;

    enum DriveMode {
        NORMAL_SPEED, SLOW_MODE, RELIC_SLOW
    }

    public DrivetrainY(LinearOpMode l) {
        drivemode = DriveMode.NORMAL_SPEED;
        telem = l.telemetry;
        linearOPMode = l;
        HardwareMap hardwareMap = l.hardwareMap;

        LR = hardwareMap.dcMotor.get(UniversalConstants.leftBackDrive);
        RR = hardwareMap.dcMotor.get(UniversalConstants.rightBackDrive);
        LF = hardwareMap.dcMotor.get(UniversalConstants.leftFrontDrive);
        RF = hardwareMap.dcMotor.get(UniversalConstants.rightFrontDrive);

        LR.hardwareMap


    }

}
