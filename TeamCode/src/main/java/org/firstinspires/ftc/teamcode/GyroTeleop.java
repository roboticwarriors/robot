// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right stickLeftRight.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="GyroTeleop", group="Exercises")
//@Disabled
public class GyroTeleop extends LinearOpMode
{
    DcMotor                 leftMotor, rightMotor;
    TouchSensor             touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;
    double turn;
    double armMax, safeArmMax = 0;
    double ForBack, LeftRight;
    double FL, FR, RL, RR;
    double maxMec1,maxMec2, maxMecSafe;
    double armInOut;
    double speed = 0;
    HardwareGarry r = new HardwareGarry();

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

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.

        while (opModeIsActive())
        {
            // Use gyro to drive in a straight line.
            if(turn == 0)correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            //Mecanum Wheels
            {
                ForBack = -gamepad1.left_stick_y;
                LeftRight = -gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x/1.5;

                FL = ForBack + LeftRight + turn - correction;
                FR = ForBack - LeftRight - turn + correction;
                RL = ForBack - LeftRight + turn - correction;
                RR = ForBack + LeftRight - turn + correction;
                maxMec1 = Math.max(Math.abs(FL), Math.abs(FR));
                maxMec2 = Math.max(Math.abs(RR), Math.abs(RL));
                maxMecSafe = Math.max(Math.abs(maxMec1), Math.abs(maxMec2));
                if (maxMecSafe > 1) {
                    FL /= maxMecSafe;
                    FR /= maxMecSafe;
                    RL /= maxMecSafe;
                    RR /= maxMecSafe;
                }
                if (gamepad1.left_bumper){
                    speed = 1;
                }
                else if (gamepad1.right_bumper){
                    speed = 0;
                }
                if (speed==0){
                    r.leftDrive.setPower(FL);
                    r.rightDrive.setPower(FR);
                    r.leftDrive2.setPower(RL);
                    r.rightDrive2.setPower(RR);
                }
                else if (speed == 1 && turn == 0) {
                    r.leftDrive.setPower(FL / 3);
                    r.rightDrive.setPower(FR / 3);
                    r.leftDrive2.setPower(RL / 3);
                    r.rightDrive2.setPower(RR / 3);
                }
                else if (speed == 1 && turn > 0 || turn < 0){
                    r.leftDrive.setPower(FL / 1.5);
                    r.rightDrive.setPower(FR / 1.5);
                    r.leftDrive2.setPower(RL / 1.5);
                    r.rightDrive2.setPower(RR / 1.5);
            }
            }
            if(turn > 0) resetAngle(); correction = 0;
            if(turn < 0) resetAngle(); correction = 0;

            //Grab
            {
                //up
                if (gamepad1.left_trigger > 0) {
                    r.leftGrab.setPosition(0.2);
                    r.rightGrab.setPosition(0.8);
                }
                //down
                else if (gamepad1.right_trigger > 0) {
                    r.leftGrab.setPosition(0.8);
                    r.rightGrab.setPosition(0.2);
                }
            }

            //Cap Stone
            {
                //sets position of claw rotation servo
                if (gamepad1.x) {
                    r.capRotate.setPosition(0.5);
                } else if (gamepad1.y) {
                    r.capRotate.setPosition(1);
                } else if (gamepad1.b){
                    r.capRotate.setPosition(0.4);
                }
            }

            //Arm/Claw/GamePad2
            {
                // Main arm gear
                {
                    armMax = -gamepad2.left_stick_y;

                    // Normalize the values so neither exceed +/- 0.5
                    if (armMax > 0.9) {
                        safeArmMax = 0.9;
                    } else if (armMax < -0.9) {
                        safeArmMax = -0.9;
                    } else if (armMax < 0.9 && armMax > -0.9) {
                        safeArmMax = armMax;
                    }
                    // Output the safe values to the motor drives.
                    r.mainArm.setPower(safeArmMax);
                }

                //Second arm power and claw
                {
                    //Sets power of second arm servo
                    armInOut = gamepad2.right_stick_y;
                    if (armInOut < 0) {
                        r.sArm.setPower(-1);
                    } else if (armInOut > 0) {
                        r.sArm.setPower(1);
                    } else {
                        r.sArm.setPower(0);
                    }

                    //sets position of claw servo for grabbing and letting go of sky stone
                    if (gamepad2.right_bumper) {
                        r.claw.setPosition(.5);
                    } else if (gamepad2.left_bumper) {
                        r.claw.setPosition(1);
                    }
                }
            }
            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

           // aButton = gamepad1.a;
           // bButton = gamepad1.b;
           // touched = gamepad1.x;

            if (touched || aButton || bButton)
            {
                // backup.
                r.leftDrive.setPower(power);
                r.rightDrive.setPower(power);
                r.leftDrive2.setPower(power);
                r.rightDrive2.setPower(power);

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

        // stickLeftRight the motors off.
        r.leftDrive.setPower(0);
        r.rightDrive.setPower(0);
        r.leftDrive2.setPower(0);
        r.rightDrive2.setPower(0);
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
        double correction, angle, gain = .03;

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
    private void rotate(double degrees, double power)
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


        // reset angle tracking on new heading.
        resetAngle();
    }
}