package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.UniversalConstants;

@TeleOp(name = "Everything Tester", group = "TeleOp")

public class EverythingTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        /*  4 motors to test for drivetrain
            1 motor for slam dunk
            1 motor for relic
            4 servos for intake (controlled by power) (continuous)
            2 servos for jewel swatter (controlled by position) (normal)
            2 servos for the relic (controlled by position) (normal)
            1 servo for slam dunk, it pinches stuff (normal)
            for the two days, no color sensor stuff
        */
        /* controller stuff:
            gamepad 1 for motors
            --D pad (top left plus sign) is used to "scroll"
                every time you scroll, the motor that you "selected" is going to show up on the
                phone using telemetry
            -- Joystick (bottom left) control motor power
            gamepad 2 for servos
            --D pad (top left plus sign) is used to "scroll"
                every time you scroll, the servo that you "selected" is going to show up on the
                phone using telemetry

         */
        //motors initial stuff
        Controller controller1 = new Controller(gamepad1);
        DcMotor[] motors = {hardwareMap.dcMotor.get(UniversalConstants.leftFrontDrive),
                hardwareMap.dcMotor.get(UniversalConstants.rightFrontDrive),
                hardwareMap.dcMotor.get(UniversalConstants.leftBackDrive),
                hardwareMap.dcMotor.get(UniversalConstants.rightBackDrive),
                hardwareMap.dcMotor.get(UniversalConstants.slamDunker),
                hardwareMap.dcMotor.get(UniversalConstants.relicSpoolMotor),
                hardwareMap.dcMotor.get(UniversalConstants.intakeMotorLeft),
                hardwareMap.dcMotor.get(UniversalConstants.intakeMotorRight)
        };
        String[] motorName = {"Left Front", "Right Front", "Left Back", "Right Back",
                "Slam Dunk", "Relic Motor", "Intake Left", "Intake Right"};

        int motorArrayPosition = 0;

        //servos initial stuff

        Controller controller2 = new Controller(gamepad2);
        //normal servo array
        Servo[] normalServos = {
                hardwareMap.servo.get(UniversalConstants.jewelGimbleElbow),
                hardwareMap.servo.get(UniversalConstants.jewelGimbleWrist),
                hardwareMap.servo.get(UniversalConstants.relicElbow),
                hardwareMap.servo.get(UniversalConstants.relicPincher),
                hardwareMap.servo.get(UniversalConstants.dunkerServo),
                hardwareMap.servo.get(UniversalConstants.intakeFoldOutLeft),
                hardwareMap.servo.get(UniversalConstants.intakeFoldOutRight)
        };
        //normal servo name array
        String[] normalServoName = {"Jewel Gimble Elbow", "Jewel Gimble Wrist",
                "Relic Elbow", "Relic Pincher", "Slam Dunker", "Intake Fold Out Left", "Intake Fold Out Right"};

        int normalServoArrayPosition = 0;
        int continuousServoArrayPosition = 0;

        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();
            //code for motors

            if (controller1.dpadLeftOnce()) {
                motorArrayPosition -= 1;
            } else if (controller1.dpadRightOnce()) {
                motorArrayPosition += 1;
            }
            motorArrayPosition = (motorArrayPosition + motors.length) % motors.length;
            //motor position will always be between 0 and 5

            telemetry.addData("Motor Selected", motorName[motorArrayPosition]);
            motors[motorArrayPosition].setPower(controller1.left_stick_y);
            //y is for up and down
            //x is for right and left of the bottom left joystick
            telemetry.addData("Motor Selected Power", motors[motorArrayPosition].getPower());
            telemetry.addData("Motor Encoder Position", motors[motorArrayPosition].getCurrentPosition());

            //code for Normal Servos; This will the Left and Right for the D pad; POSITION

            if (controller2.dpadLeftOnce()) {
                normalServoArrayPosition -= 1;
            } else if (controller2.dpadRightOnce()) {
                normalServoArrayPosition += 1;
            }
            normalServoArrayPosition = (normalServoArrayPosition + normalServos.length) % normalServos.length;
            telemetry.addData("Normal Servo Selected", normalServoName[normalServoArrayPosition]);
            //if y is pressed, add .01 to the position
            //if a is pressed, subtract .01 to the position
            if (controller2.YOnce()) {
                normalServos[normalServoArrayPosition].setPosition(normalServos[normalServoArrayPosition].getPosition() + .01);
            } else if (controller2.AOnce()) {
                normalServos[normalServoArrayPosition].setPosition(normalServos[normalServoArrayPosition].getPosition() - .01);
            }

            telemetry.addData("Normal Servo Selected Position", normalServos[normalServoArrayPosition].getPosition());

            /*
            //code for Continuous Servos; This will be Up and Down for for D pad; down is -1, up is +1; POWER

            if (controller2.dpadDownOnce()) {
                continuousServoArrayPosition -= 1;
            } else if (controller2.dpadUpOnce()) {
                continuousServoArrayPosition += 1;
            }
            continuousServoArrayPosition = ((continuousServoArrayPosition + continuousServos.length) % continuousServos.length);
            telemetry.addData("Continuous Servo Selected", continuousServoName[continuousServoArrayPosition]);
            continuousServos[continuousServoArrayPosition].setPower(controller2.left_stick_y / 2);
            telemetry.addData("Continuous Servo Selected Power", continuousServos[continuousServoArrayPosition].getPower());
            */

            telemetry.update();
        }

    }
}