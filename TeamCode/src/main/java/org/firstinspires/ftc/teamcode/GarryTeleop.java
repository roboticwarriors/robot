package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Teleop: Garry", group="Garry")
public class GarryTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareGarry   r           = new HardwareGarry();

    @Override
    public void runOpMode() {
        double turn;
        double armMax, safeArmMax = 0;
        double ForBack, LeftRight;
        double FL, FR, RL, RR;
        double maxMec1,maxMec2, maxMecSafe;
        double armInOut;
        double speed = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        r.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive/Grab/Capstone
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
                    else if (speed==1) {
                        r.leftDrive.setPower(FL / 3);
                        r.rightDrive.setPower(FR / 3);
                        r.leftDrive2.setPower(RL / 3);
                        r.rightDrive2.setPower(RR / 3);
                    }
                }

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
                    //Holds the cap stone
                    if (gamepad1.a) {
                        r.capClaw.setPosition(1);
                    } else if (gamepad1.b) {
                        r.capClaw.setPosition(-1);
                    }

                    //sets position of claw rotation servo
                    if (gamepad1.x) {
                        r.capRotate.setPosition(0.5);
                    } else if (gamepad1.y) {
                        r.capRotate.setPosition(1);
                    }
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


            // Show joystick information as some other illustrative data
            telemetry.addLine("left joystick | ")
                    .addData("x", gamepad1.left_stick_x)
                    .addData("y", -gamepad1.left_stick_y);
            telemetry.addLine("right joystick | ")
                    .addData("x", gamepad1.right_stick_x)
                    .addData("y", gamepad1.right_stick_y);
            telemetry.addLine("Right Motors")
                    .addData("Front", r.rightDrive.getPower())
                    .addData("Back", r.rightDrive2.getPower());
            telemetry.addLine("Left Motors")
                    .addData("Front", r.leftDrive.getPower())
                    .addData("Back", r.leftDrive2.getPower());
            telemetry.update();

            // Pace this loop so jaw action is reasonable gear.
            sleep(50);
        }
    }
}

