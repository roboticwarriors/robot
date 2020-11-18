package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test: GamePad", group="Test")
public class GamePadTest extends LinearOpMode {

    HardwareGarry   robot           = new HardwareGarry();

    @Override
    public void runOpMode() {


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("left joystick | ")
                    .addData("x", gamepad1.left_stick_x)
                    .addData("y", gamepad1.left_stick_y);
            telemetry.addLine("right joystick | ")
                    .addData("x", gamepad1.right_stick_x)
                    .addData("y", gamepad1.right_stick_y);
            telemetry.addLine("Triggers | ")
                    .addData("LeftTrigger", gamepad1.left_trigger)
                    .addData("RightTrigger", gamepad1.right_trigger);
            telemetry.addLine("Bumpers | ")
                    .addData("LeftBumper", gamepad1.left_bumper)
                    .addData("RightBumper", gamepad1.right_bumper);
            telemetry.addLine("D-Pad | ")
                    .addData("up", gamepad1.dpad_up)
                    .addData("down", gamepad1.dpad_down)
                    .addData("left", gamepad1.dpad_left)
                    .addData("right", gamepad1.dpad_right);
            telemetry.addLine("Buttons | ")
                    .addData("a", gamepad1.a)
                    .addData("b", gamepad1.b)
                    .addData("x", gamepad1.x)
                    .addData("y", gamepad1.y);
            telemetry.update();
            sleep(50);
        }
    }
}
