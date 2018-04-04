package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutonomousUtil;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.RobotModules.Robot;

//@Disabled
@Autonomous(name = "Auto Tester New")
public class AutonomousTesterNew extends LinearOpMode {

    public class ActionType {
        public String name;
        public Runnable action;

        public ActionType(String actionName, Runnable actionAction) {
            name = actionName;
            action = actionAction;
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        final Robot robot = new Robot(this, true, true, true, DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addLine("Ready to go!");
        telemetry.update();

        Controller game1 = new Controller(gamepad1);
        int position = 0;
        ActionType[] actions = new ActionType[]{
                new ActionType("Forwards 10 Inches", new Runnable() {
                    @Override
                    public void run() {
                        robot.driveTrain.moveToInches(10, 1);
                    }
                }),
                new ActionType("Backwards 10 Inches", new Runnable() {
                    @Override
                    public void run() {
                        robot.driveTrain.moveToInches(-10, 1);
                    }
                }),
                new ActionType("Color Distance Up", new Runnable() {
                    @Override
                    public void run() {
                        robot.driveTrain.swingColorDistanceUp();
                    }
                }),
                new ActionType("Color Distance Down", new Runnable() {
                    @Override
                    public void run() {
                        robot.driveTrain.swingColorDistanceDown();
                    }
                }),
                new ActionType("Strafe to 10cm", new Runnable() {
                    @Override
                    public void run() {
                        double distance = 10;
                        robot.driveTrain.strafeToDistanceLeft(.2, distance, DistanceUnit.CM);
                    }
                }),
                new ActionType("Strafe to 6cm", new Runnable() {
                    @Override
                    public void run() {
                        double distance = 6;
                        robot.driveTrain.strafeToDistanceLeft(.2, distance, DistanceUnit.CM);
                    }
                }),
                new ActionType("Swat Red Jewel", new Runnable() {
                    @Override
                    public void run() {
                        try {
                            robot.jewelSwatter.removeJewel(AutonomousUtil.AllianceColor.Red);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }),
                new ActionType("Turn to 90 Degrees", new Runnable() {
                    @Override
                    public void run() {
                        robot.driveTrain.gyroTurn(.1, 90);
                    }
                })
        };


        waitForStart();
        robot.vuforiaRelicRecoveryGetter.activateTrackables();
        while (opModeIsActive()) {
            while (!gamepad1.a && opModeIsActive()) {
                game1.update();
                if (game1.dpadLeftOnce()) {
                    position -= 1;
                } else if (game1.dpadRightOnce()) {
                    position += 1;
                }
                position = (position + actions.length) % actions.length;
                telemetry.addData("Color Distance Reading Left (cm)", robot.driveTrain.leftSensorDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("Color Distance Reading Right (cm)", robot.driveTrain.rightSensorDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("Chosen Task", actions[position].name);
                telemetry.update();
            }

            actions[position].action.run();

        }
    }
}
