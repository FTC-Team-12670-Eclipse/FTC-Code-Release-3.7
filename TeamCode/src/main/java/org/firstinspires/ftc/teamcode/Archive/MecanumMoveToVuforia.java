package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotModules.VuforiaRelicRecoveryGetter;
import org.firstinspires.ftc.teamcode.UniversalConstants;

@Deprecated
//DON'T USE THIS CLASS. I JUST WANTED TO SAVE MY VUFORIA NAVIAGATION CODE AND HAVE IT COMPILE
public class MecanumMoveToVuforia {
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private BNO055IMU imu;

    public DcMotor leftFront, leftBack, rightFront, rightBack;

    private VoltageSensor voltageSensor;

    public void init(LinearOpMode l, DcMotor.ZeroPowerBehavior zeroPowerBehavior, boolean useImu) {
        telemetry = l.telemetry;
        linearOpMode = l;
        HardwareMap hardwareMap = l.hardwareMap;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront = hardwareMap.dcMotor.get(UniversalConstants.leftFrontDrive);
        leftBack = hardwareMap.dcMotor.get(UniversalConstants.leftBackDrive);
        rightFront = hardwareMap.dcMotor.get(UniversalConstants.rightFrontDrive);
        rightBack = hardwareMap.dcMotor.get(UniversalConstants.rightBackDrive);

        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        if (useImu) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
    }

    double vuforiaMax = .20;
    double angleThreshold = 1.5;
    double threshold = 15;

    public double getVoltage() {
        return 14;
    }

    public void park(){
    }

    public double getTurnPower() {
        return 1.1 - (getVoltage() / 14);
        // https://www.desmos.com/calculator/phdfnhkvgf
        // return .5 - (getVoltage() / 22);
        // tune this equation
    }

    public double getPowerStrafe(double diff) {
        return getPower(diff) * 4;
        // we need double power so our wheels can fight each other
    }

    public double getPower(double diff) {
        if (diff > threshold) {
            return getTurnPower();
        } else if (diff < threshold) {
            return -getTurnPower();
        }
        return 0;
    }

    public void translateBy(double x, double y, double c) {

    }

    public enum MovementDirection {
        Forwards, HORIZONTAL
    }

    public void singleDirectionToVuforia(VuforiaRelicRecoveryGetter vuforiaRelicRecoveryGetter, double distance, MovementDirection direction) {
        VuforiaRelicRecoveryGetter.DistanceOffsets distanceOffsets = vuforiaRelicRecoveryGetter.getOffset();
        if (!distanceOffsets.foundValues) {
            return;
        }
        if (direction == MovementDirection.HORIZONTAL) {
            horizontalVuforia(vuforiaRelicRecoveryGetter, distance);
        } else {
            forwardsVuforia(vuforiaRelicRecoveryGetter, distance);
        }
    }

    public void horizontalVuforia(VuforiaRelicRecoveryGetter vuforiaRelicRecoveryGetter, double distance) {
        VuforiaRelicRecoveryGetter.DistanceOffsets distanceOffsets = vuforiaRelicRecoveryGetter.getOffset();
        double horizontalError = Math.abs(distance - distanceOffsets.horizontal);
        while (opModeIsActive() && Math.abs(horizontalError) > threshold) {
            horizontalError = distance - vuforiaRelicRecoveryGetter.getOffset().horizontal;
            translateBy(0, getPowerStrafe(horizontalError), 0);
            updateTelemetry();
            vuforiaRelicRecoveryGetter.updateTelemetry();
            telemetry.update();
        }
        park();
    }

    public void forwardsVuforia(VuforiaRelicRecoveryGetter vuforiaRelicRecoveryGetter, double distance) {
        VuforiaRelicRecoveryGetter.DistanceOffsets distanceOffsets = vuforiaRelicRecoveryGetter.getOffset();
        double forwardsError = Math.abs(distance - distanceOffsets.distance);
        while (opModeIsActive() && Math.abs(forwardsError) > threshold) {
            forwardsError = distance - vuforiaRelicRecoveryGetter.getOffset().distance;
            translateBy(getPower(forwardsError), 0, 0);
            updateTelemetry();
            vuforiaRelicRecoveryGetter.updateTelemetry();
            telemetry.update();
        }
        park();

    }

    public void simpleToVuforia(VuforiaRelicRecoveryGetter vuforiaRelicRecoveryGetter, double xOff, double yOff, double angleGoal) {
        double horizontalError;
        double forwardsError;
        double angleError, anglePower;
        VuforiaRelicRecoveryGetter.DistanceOffsets distanceOffsets = vuforiaRelicRecoveryGetter.getOffset();
        if (!distanceOffsets.foundValues) {
            return;
        }
        horizontalError = Math.abs(xOff - distanceOffsets.horizontal);
        forwardsError = Math.abs(yOff - distanceOffsets.distance);
        angleError = Math.abs(angleGoal - getHeading());
        double threshold = 15;
        while ((horizontalError > threshold || forwardsError > threshold || angleError > angleThreshold) && opModeIsActive()) {

            distanceOffsets = vuforiaRelicRecoveryGetter.getOffset();
            horizontalError = xOff - distanceOffsets.horizontal;
            forwardsError = yOff - distanceOffsets.distance;
            angleError = angleGoal - getHeading();
            anglePower = 0;

            translateBy(getPower(forwardsError), getPower(horizontalError), anglePower);

            telemetry.addData("Angle Error", angleError);
            updateTelemetry();
            vuforiaRelicRecoveryGetter.updateTelemetry();
            telemetry.update();
        }
    }

    public void moveToVuforia(VuforiaRelicRecoveryGetter vuforiaRelicRecoveryGetter, double xOffset, double yOffset, double gyroGoal) {
        double horizontalError, horizontalPower;
        double forwardsError, forwardsPower;
        double angleError, anglePower;
        VuforiaRelicRecoveryGetter.DistanceOffsets distanceOffsets = vuforiaRelicRecoveryGetter.getOffset();
        if (!distanceOffsets.foundValues) {
            return;
        }
        double startHorizontalOffset = distanceOffsets.horizontal, startForwardsOffset = distanceOffsets.horizontal;

        do {
            distanceOffsets = vuforiaRelicRecoveryGetter.getOffset();
            horizontalError = Math.abs(xOffset - distanceOffsets.horizontal);
            forwardsError = Math.abs(yOffset - distanceOffsets.distance);
            angleError = Math.abs(gyroGoal - getHeading());
            horizontalPower = distanceOffsets.horizontal / startHorizontalOffset;
            forwardsPower = distanceOffsets.distance / startForwardsOffset;
            anglePower = angleError;
            if (Math.abs(horizontalPower) < .1) {
                horizontalPower = .1 * horizontalPower / Math.abs(horizontalPower);
            }
            if (Math.abs(forwardsPower) < .1) {
                forwardsPower = .1 * forwardsPower / Math.abs(forwardsPower);
            }
            forwardsPower = Range.clip(forwardsPower, -vuforiaMax, vuforiaMax);
            horizontalPower = Range.clip(horizontalPower, -vuforiaMax, vuforiaMax);
            anglePower = Range.clip(anglePower, -vuforiaMax, vuforiaMax);
            translateBy(forwardsPower, horizontalPower, anglePower);
            updateTelemetry();
            vuforiaRelicRecoveryGetter.updateTelemetry();
            telemetry.update();
        }
        while ((horizontalError > 25 || forwardsError > 25 || angleError > 2.5) && opModeIsActive());
        park();
    }

    public void updateTelemetry() {
        telemetry.addData("g1 y", linearOpMode.gamepad1.left_stick_y);
        telemetry.addData("g1 x", linearOpMode.gamepad1.left_stick_x);
        telemetry.addData("g1 frontWheelHowMuchFaster", linearOpMode.gamepad1.right_stick_x);
        telemetry.addData("lf", leftFront.getPower());
        telemetry.addData("rf", rightFront.getPower());
        telemetry.addData("lb", leftBack.getPower());
        telemetry.addData("rb", rightBack.getPower());

        telemetry.addData("lf pos", leftFront.getCurrentPosition());
        telemetry.addData("rf pos", rightFront.getCurrentPosition());
        telemetry.addData("lb pos", leftBack.getCurrentPosition());
        telemetry.addData("rb pos", rightBack.getCurrentPosition());

        telemetry.addData("voltage", getVoltage());
    }

    private boolean opModeIsActive() {
        return this.linearOpMode.opModeIsActive();
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getPitch() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.secondAngle;
    }

    public double getRoll() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    private void idle() {
        linearOpMode.idle();
    }

}

