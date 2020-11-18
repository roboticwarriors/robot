// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right stickLeftRight.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Blue FOUNDATION", group="Exercises")
//@Disabled
public class AutoGarry_BlueSide_Foundation extends LinearOpMode
{
    DcMotor                 leftMotor, rightMotor;
    TouchSensor             touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = 1.0, correction;
    boolean                 aButton, bButton, touched;
    HardwareGarry r = new HardwareGarry();
    private ElapsedTime runtime = new ElapsedTime();

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        r.init(hardwareMap);

        // get a reference to touch sensor.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        r.capRotate.setPosition(0.4);

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);
        runtime.reset();

        // drive until end of period.

        while (opModeIsActive())
        {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            //Step 1: Raise Main Arm
            if(runtime.seconds() < 0.3) {
                r.capRotate.setPosition(0.5);
                r.mainArm.setPower(1);
            }
            if(runtime.seconds() > 0.3 && runtime.seconds() < 0.4) r.mainArm.setPower(0);

            //Step 2: Drive Sideways to Build Site
            if(runtime.seconds() > 0.3 && runtime.seconds() < 0.8) {
                r.leftDrive.setPower(power - correction);
                r.rightDrive.setPower(-power + correction);
                r.leftDrive2.setPower(-power - correction);
                r.rightDrive2.setPower(power + correction);
            }

            //Step 2:  Drive forward for 1.3 seconds
            if(runtime.seconds() > 0.8 && runtime.seconds() < 2.2) {
                r.leftDrive.setPower(power - correction);
                r.leftDrive2.setPower(power - correction);
                r.rightDrive.setPower(power + correction);
                r.rightDrive2.setPower(power + correction);
            }

            //Step 3: Lower Grabber
            if(runtime.seconds() > 2.2 && runtime.seconds() < 3.5) {
                r.leftDrive.setPower(0);
                r.leftDrive2.setPower(0);
                r.rightDrive.setPower(0);
                r.rightDrive2.setPower(0);
                r.leftGrab.setPosition(0.2);
                r.rightGrab.setPosition(0.8);
            }

            //Step 4: Drive Backward
            if(runtime.seconds() > 3.5 && runtime.seconds() < 5.5) {
                r.leftDrive.setPower(-power - correction);
                r.leftDrive2.setPower(-power - correction);
                r.rightDrive.setPower(-power + correction);
                r.rightDrive2.setPower(-power + correction);
            }

            //Step 5: Let Go
            if(runtime.seconds() > 5.5 && runtime.seconds() < 6.3) {
                r.leftDrive.setPower(0);
                r.leftDrive2.setPower(0);
                r.rightDrive.setPower(0);
                r.rightDrive2.setPower(0);
                r.leftGrab.setPosition(0.8);
                r.rightGrab.setPosition(0.2);
            }

            //Step 6: Drive Sideways
            if(runtime.seconds() > 6.3 && runtime.seconds() < 7.8) {
                r.leftDrive.setPower(-power - correction);
                r.rightDrive.setPower(power + correction);
                r.leftDrive2.setPower(power - correction);
                r.rightDrive2.setPower(-power + correction);
            }

            //Step 7: Lower Main Arm
            if(runtime.seconds() > 7.8 && runtime.seconds() < 8.1) {
                r.leftDrive.setPower(0);
                r.leftDrive2.setPower(0);
                r.rightDrive.setPower(0);
                r.rightDrive2.setPower(0);
                r.mainArm.setPower(-1);
            }
            if(runtime.seconds() > 8.1 && runtime.seconds() < 8.3) r.mainArm.setPower(0);

            //Step 8: Move to Bridge Tape
            if(runtime.seconds() > 8.3 && runtime.seconds() < 9.0) {
                r.leftDrive.setPower(-power - correction);
                r.rightDrive.setPower(power + correction);
                r.leftDrive2.setPower(power - correction);
                r.rightDrive2.setPower(-power + correction);
            }
            if(runtime.seconds() > 9.0 && runtime.seconds() < 10.4) {
                r.leftDrive.setPower(0);
                r.leftDrive2.setPower(0);
                r.rightDrive.setPower(0);
                r.rightDrive2.setPower(0);
            }
            if(runtime.seconds() > 10.4 && runtime.seconds() < 11.0) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("Path", "Complete");
                telemetry.update();
            }


            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

//            aButton = gamepad1.a;
//            bButton = gamepad1.b;
//            touched = gamepad1.x;

            if (touched || aButton || bButton)
            {
                // backup.
                r.leftDrive.setPower(-power);
                r.rightDrive.setPower(-power);
                r.leftDrive2.setPower(-power);
                r.rightDrive2.setPower(-power);

                sleep(500);

                // stop.
                r.leftDrive.setPower(0);
                r.rightDrive.setPower(0);
                r.leftDrive2.setPower(0);
                r.rightDrive2.setPower(0);

                // stickLeftRight 90 degrees right.
                if (touched || aButton) rotate(-90, power);

                // stickLeftRight 90 degrees left.
                if (bButton) rotate(90, power);
            }
        }

/*        // stickLeftRight the motors off.
        r.leftDrive.setPower(0);
        r.rightDrive.setPower(0);
        r.leftDrive2.setPower(0);
        r.rightDrive2.setPower(0);
 */
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to stickLeftRight, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // stickLeftRight right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // stickLeftRight left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        r.leftDrive.setPower(leftPower);
        r.rightDrive.setPower(rightPower);
        r.leftDrive2.setPower(leftPower);
        r.rightDrive2.setPower(rightPower);

        // rotate until stickLeftRight is completed.
        if (degrees < 0)
        {
            // On right stickLeftRight we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left stickLeftRight.
            while (opModeIsActive() && getAngle() < degrees) {}

        // stickLeftRight the motors off.
        r.leftDrive.setPower(0);
        r.rightDrive.setPower(0);
        r.leftDrive2.setPower(0);
        r.rightDrive2.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}
