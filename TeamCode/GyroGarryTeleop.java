package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Teleop: GyroGarry", group="Garry")
public class GyroGarryTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareGarry   r           = new HardwareGarry();

    //Define IMU
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    @Override
    public void runOpMode() {
        double turn;
        double armMax, safeArmMax = 0;
        double ForBack, LeftRight;
        double FL, FR, RL, RR;
        double maxMec1,maxMec2, maxMecSafe;
        double armIn, armOut;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        r.init(hardwareMap);

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



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * Resets the cumulative angle tracking to zero.
             */
            public void resetAngle()
            {
                lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                globalAngle = 0;
            }

            /**
             * Get current cumulative angle rotation from last reset.
             * @return Angle in degrees. + = left, - = right.
             */
            public double getAngle()
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
            public double checkDirection()
            {
                // The gain value determines how sensitive the correction is to direction changes.
                // You will have to experiment with your robot to get small smooth direction changes
                // to stay on a straight line.
                double correction, angle, gain = .10;

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
             * @param degrees Degrees to turn, + is left - is right
             */
            public void rotate(int degrees, double power)
            {
                double  leftPower, rightPower;

                // restart imu movement tracking.
                resetAngle();

                // getAngle() returns + when rotating counter clockwise (left) and - when rotating
                // clockwise (right).

                if (degrees < 0)
                {   // turn right.
                    leftPower = power;
                    rightPower = -power;
                }
                else if (degrees > 0)
                {   // turn left.
                    leftPower = -power;
                    rightPower = power;
                }
                else return;

                // set power to rotate.
                leftMotor.setPower(leftPower);
                rightMotor.setPower(rightPower);

                // rotate until turn is completed.
                if (degrees < 0)
                {
                    // On right turn we have to get off zero first.
                    while (opModeIsActive() && getAngle() == 0) {}

                    while (opModeIsActive() && getAngle() > degrees) {}
                }
                else    // left turn.
                    while (opModeIsActive() && getAngle() < degrees) {}

                // turn the motors off.
                rightMotor.setPower(0);
                leftMotor.setPower(0);

                // wait for rotation to stop.
                sleep(1000);

                // reset angle tracking on new heading.
                resetAngle();
            }

            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            leftMotor.setPower(power - correction);
            rightMotor.setPower(power + correction);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            aButton = gamepad1.a;
            bButton = gamepad1.b;
            touched = touch.isPressed();

            if (touched || aButton || bButton)
            {
                // backup.
                leftMotor.setPower(power);
                rightMotor.setPower(power);

                sleep(500);

                // stop.
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                // turn 90 degrees right.
                if (touched || aButton) rotate(-90, power);

                // turn 90 degrees left.
                if (bButton) rotate(90, power);
            }
        }

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

            /*
            //Drive/Filter/Capstone
            {
                //Mecanum Wheels
                {
                    ForBack = -gamepad1.left_stick_y;
                    LeftRight = -gamepad1.left_stick_x;
                    turn = gamepad1.right_stick_x;

                    FL = ForBack + LeftRight + turn;
                    FR = ForBack - LeftRight - turn;
                    RL = ForBack - LeftRight + turn;
                    RR = ForBack + LeftRight - turn;
                    maxMec1 = Math.max(Math.abs(FL), Math.abs(FR));
                    maxMec2 = Math.max(Math.abs(RR), Math.abs(RL));
                    maxMecSafe = Math.max(Math.abs(maxMec1), Math.abs(maxMec2));
                    if (maxMecSafe > 1) {
                        FL /= maxMecSafe;
                        FR /= maxMecSafe;
                        RL /= maxMecSafe;
                        RR /= maxMecSafe;
                    }
                    r.leftDrive.setPower(FL);
                    r.rightDrive.setPower(FR);
                    r.leftDrive2.setPower(RL);
                    r.rightDrive2.setPower(RR);
                }

                //Filter
                {
                    if (gamepad1.left_bumper) {
                        r.leftFilter.setPosition(1);
                        r.rightFilter.setPosition(0);
                    } else if (gamepad1.right_bumper) {
                        r.leftFilter.setPosition(0.9);
                        r.rightFilter.setPosition(0.15);
                    } else if (gamepad1.left_trigger > 0) {
                        r.leftFilter.setPosition(0.3);
                        r.rightFilter.setPosition(0.7);
                    } else if (gamepad1.right_trigger > 0) {
                        r.leftFilter.setPosition(0.55);
                        r.rightFilter.setPosition(0.45);
                    }
                }

                //Cap Stone
                {
                    //Holds the cap stone
                    if (gamepad1.a) {
                        r.capClaw.setPosition(1);
                    } else if (gamepad1.b) {
                        r.capClaw.setPosition(-1);
                    }

                    //sets position of claw rotation servo
                    if (gamepad1.x) {
                        r.capRotate.setPosition(-0.1);
                    } else if (gamepad1.y) {
                        r.capRotate.setPosition(0.5);
                    }
                }
            }
            //Arm/Claw/GamePad2
            {
                // Main arm speed
                {
                    armMax = -gamepad2.left_stick_y;

                    // Normalize the values so neither exceed +/- 0.5
                    if (armMax > 0.5) {
                        safeArmMax = 0.5;
                    } else if (armMax < -0.5) {
                        safeArmMax = -0.5;
                    } else if (armMax < 0.5 && armMax > -0.5) {
                        safeArmMax = armMax;
                    }
                    // Output the safe values to the motor drives.
                    r.mainArm.setPower(safeArmMax);
                }

                //Second arm power and claw
                {
                    //Sets power of second arm servo
                    armIn = gamepad2.right_trigger;
                    armOut = gamepad2.left_trigger;
                    if (armIn > 0) {
                        r.secondArm.setPower(-1);
                    } else if (armOut > 0) {
                        r.secondArm.setPower(1);
                    } else {
                        r.secondArm.setPower(0);
                    }

                    //sets position of claw servo for grabbing and letting go of sky stone
                    if (gamepad2.right_bumper) {
                        r.claw.setPosition(0);
                    } else if (gamepad2.left_bumper) {
                        r.claw.setPosition(1);
                    }
                }
            }



            // Show joystick information as some other illustrative data
            telemetry.addLine("left joystick | ")
                    .addData("x", gamepad1.left_stick_x)
                    .addData("y", -gamepad1.left_stick_y);
            telemetry.addLine("right joystick | ")
                    .addData("x", gamepad1.right_stick_x)
                    .addData("y", gamepad1.right_stick_y);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
             */
        }
    }

