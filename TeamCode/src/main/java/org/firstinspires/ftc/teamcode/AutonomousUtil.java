package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

public class AutonomousUtil {

    public static void driveRobotOffRamp(Robot robot, AllianceColor alliance) {
        driveRobotOffRamp(robot, alliance, .15);
    }

    public static void driveRobotOffRamp(Robot robot, AllianceColor alliance, double power) {
        double distance = 27;
        if (alliance == AllianceColor.Blue) {
            robot.driveTrain.moveToInches(-distance, power);
        } else {
            robot.driveTrain.moveToInches(distance, power);
        }
    }

    public static void driveRobotOffRampFast(Robot robot, AllianceColor alliance, double power) {
        double distance = 23;
        if (alliance == AllianceColor.Blue) {
            robot.driveTrain.moveToPositionInches(-distance, power);
        } else {
            robot.driveTrain.moveToPositionInches(distance, power);
        }
    }


    public enum AllianceColor {
        Red, Blue
    }

    public static AllianceColor getColorToDislodge(LinearOpMode l, AllianceColor color, Robot robot) {
        while (!l.isStarted()) {
            if (l.gamepad1.left_bumper && l.gamepad1.right_bumper) {
                color = AllianceColor.Blue;
            } else if (l.gamepad2.left_bumper && l.gamepad2.right_bumper) {
                color = AllianceColor.Red;
            }
            if (l.gamepad1.left_trigger > .5) {
                robot.intakeMecanism.foldoutHolderLeft.setPosition(UniversalConstants.foldoutHolderLeftOpen);
            } else if (l.gamepad1.right_trigger > .5) {
                robot.intakeMecanism.foldoutHolderLeft.setPosition(UniversalConstants.foldoutHolderLeftStored);
            }
            if (l.gamepad2.left_trigger > .5) {
                robot.intakeMecanism.foldoutHolderRight.setPosition(UniversalConstants.foldoutHolderRightOpen);
            } else if (l.gamepad2.right_trigger > .5) {
                robot.intakeMecanism.foldoutHolderRight.setPosition(UniversalConstants.foldoutHolderRightStored);
            }

            l.telemetry.addLine("Ready to Go!");
            l.telemetry.addData("Color to Dislodge", color.name());
            l.telemetry.addLine("The triggers on Game pad 1 handle the left servo");
            l.telemetry.addLine("The triggers on Game pad 2 handle the left servo");
            l.telemetry.addLine("Use the left trigger to open the servo, and the right trigger to close the servo");
            l.telemetry.addLine("Press two bumpers to switch color");
            l.telemetry.addLine("Dislodge Blue: Game pad 1");
            l.telemetry.addLine("Dislodge Red: Game pad 2");
            l.telemetry.addLine("Hold both triggers to switch color");
            l.telemetry.addData("Left Side", robot.driveTrain.leftFacingDistanceSensor.getDistance(DistanceUnit.CM));
            l.telemetry.addData("Right Side", robot.driveTrain.rightFacingDistanceSensor.getDistance(DistanceUnit.CM));
            l.telemetry.update();
        }
        return color;
    }

    public static class Settings {
        public AllianceColor allianceColor;
        public double leftAngle = 0, rightAngle = 0, centerAngle = 0;
        public boolean shouldDunk;

        public Settings(AllianceColor a, double l, double r, double c, boolean s) {
            allianceColor = a;
            leftAngle = l;
            rightAngle = r;
            centerAngle = c;
            shouldDunk = s;
        }
    }

    public static Settings getStartingSettings(LinearOpMode l, AllianceColor color, Robot robot) {
        double leftAng, centerAng, rightAng;
        boolean shouldDunk = true;
        if (color == AllianceColor.Blue) {
            leftAng = 20;
            rightAng = -20;
            centerAng = 0;
        } else {
            leftAng = -20;
            rightAng = 20;
            centerAng = 0;
        }
        Controller controller1 = new Controller(l.gamepad1);
        Controller controller2 = new Controller(l.gamepad2);
        while (!l.isStarted()) {
            controller1.update();
            controller2.update();
            if (controller1.rightBumper() && controller1.leftBumper()) {
                color = AllianceColor.Blue;
            } else if (controller2.rightBumper() && controller2.leftBumper()) {
                color = AllianceColor.Red;
            }

            if (controller1.dpadLeftOnce()) {
                leftAng += 5;
            } else if (controller1.dpadRightOnce()) {
                leftAng -= 5;
            }

            if (controller1.dpadUpOnce()) {
                centerAng += 5;
            } else if (controller1.dpadDownOnce()) {
                centerAng -= 5;
            }

            if (controller2.dpadLeftOnce()) {
                rightAng += 5;
            } else if (controller2.dpadRightOnce()) {
                rightAng -= 5;
            }

            if (controller2.dpadUpOnce()) {
                shouldDunk = true;
            } else if (controller2.dpadDownOnce()) {
                shouldDunk = false;
            }

            if (l.gamepad1.left_trigger > .5) {
                robot.intakeMecanism.foldoutHolderLeft.setPosition(UniversalConstants.foldoutHolderLeftOpen);
            } else if (l.gamepad1.right_trigger > .5) {
                robot.intakeMecanism.foldoutHolderLeft.setPosition(UniversalConstants.foldoutHolderLeftStored);
            }
            if (l.gamepad2.left_trigger > .5) {
                robot.intakeMecanism.foldoutHolderRight.setPosition(UniversalConstants.foldoutHolderRightOpen);
            } else if (l.gamepad2.right_trigger > .5) {
                robot.intakeMecanism.foldoutHolderRight.setPosition(UniversalConstants.foldoutHolderRightStored);
            }

            l.telemetry.addLine("Ready to Go!");
            l.telemetry.addLine(shouldDunk ? "Will Extra Dunk" : "!!!!WILL NOT EXTRA DUNK!!!!");
            l.telemetry.addLine("Positive means turn Left");
            l.telemetry.addData("Left Angle (c1 l/r", leftAng);
            l.telemetry.addData("Center Angle (c1 u/d)", centerAng);
            l.telemetry.addData("Right Angle (c2 l/r", rightAng);
            l.telemetry.addData("Color to Dislodge", color.name());
            l.telemetry.addLine("Press two bumpers to switch color");
            l.telemetry.addLine("Dislodge Blue: Game pad 1");
            l.telemetry.addLine("Dislodge Red: Game pad 2");
            l.telemetry.addLine("Hold both triggers to switch color");
            l.telemetry.update();
        }
        return new Settings(color, leftAng, centerAng, rightAng, shouldDunk);
    }

    public static AllianceColor switchColor(AllianceColor color) {
        return color == AllianceColor.Red ? AllianceColor.Blue : AllianceColor.Red;
    }


}
