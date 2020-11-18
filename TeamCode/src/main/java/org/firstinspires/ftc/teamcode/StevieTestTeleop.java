
        package org.firstinspires.ftc.teamcode;


        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

        @TeleOp(name="Teleop: StevieTest", group="Linear Opmode")
        @Disabled
public class StevieTestTeleop extends LinearOpMode {
    HardwareGarry robot           = new HardwareGarry();     // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        telemetry.addData("Hello", "I am alive");
        telemetry.update();

        double Righttrigger;
        double Leftstick;
        double Lefttrigger;



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)

        Righttrigger = gamepad1.right_trigger;
        Leftstick = gamepad1.left_stick_x;
        Lefttrigger = gamepad1.left_trigger;

        while (opModeIsActive()) {

                if (gamepad1.right_trigger > 0 && gamepad1.left_stick_x > 0) {
                    robot.leftDrive.setPower(Righttrigger);
                    robot.leftDrive2.setPower(-Righttrigger);
                    robot.rightDrive.setPower(-Righttrigger - (-Leftstick / 2));
                    robot.rightDrive2.setPower(-Righttrigger - (-Leftstick / 2));
                } else if (Righttrigger > 0 && Leftstick < 0) {
                    robot.leftDrive.setPower(Righttrigger + (Leftstick / 2));
                    robot.leftDrive2.setPower(-Righttrigger + (Leftstick / 2));
                    robot.rightDrive.setPower(-Righttrigger);
                    robot.rightDrive2.setPower(-Righttrigger);
                } else if (Righttrigger > 0 && Leftstick == 0) {
                    robot.leftDrive.setPower(Righttrigger);
                    robot.leftDrive2.setPower(-Righttrigger);
                    robot.rightDrive.setPower(-Righttrigger);
                    robot.rightDrive2.setPower(-Righttrigger);

                } else if (Righttrigger == 0) {
                    robot.leftDrive.setPower(Righttrigger);
                    robot.leftDrive2.setPower(-Righttrigger);
                    robot.rightDrive.setPower(-Righttrigger);
                    robot.rightDrive2.setPower(-Righttrigger);
                }


                if (Lefttrigger > 0 && Leftstick == 0) {
                    robot.leftDrive.setPower(-Lefttrigger);
                    robot.leftDrive2.setPower(Lefttrigger);
                    robot.rightDrive.setPower(Lefttrigger);
                    robot.rightDrive2.setPower(Lefttrigger);
                }
                else if (Lefttrigger > 0 && Leftstick < 0) {
                    robot.leftDrive.setPower(-Lefttrigger + (-Leftstick / 2));
                    robot.leftDrive2.setPower(Lefttrigger + (-Leftstick / 2));
                    robot.rightDrive.setPower(Lefttrigger);
                    robot.rightDrive2.setPower(Lefttrigger);
                }

                else if (Lefttrigger > 0 && Leftstick > 0){
                    robot.leftDrive.setPower(-Lefttrigger);
                    robot.leftDrive2.setPower(Lefttrigger);
                    robot.rightDrive.setPower(Lefttrigger + (-Leftstick / 2));
                    robot.rightDrive2.setPower(Lefttrigger + (-Leftstick / 2));
                }
                else if (Lefttrigger == 0) {
                robot.leftDrive.setPower(-Lefttrigger);
                robot.leftDrive2.setPower(Lefttrigger);
                robot.rightDrive.setPower(Lefttrigger);
                robot.rightDrive2.setPower(Lefttrigger);
            }


            }
        }

    }


