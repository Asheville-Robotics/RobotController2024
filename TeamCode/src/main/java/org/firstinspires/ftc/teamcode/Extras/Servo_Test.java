package org.firstinspires.ftc.teamcode.Extras;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Simple Servo Control", group = "TeleOp")
public class Servo_Test extends OpMode {

    // Define the servos
    private Servo rotateServo;
    private Servo clawServo;
    private Servo wristServo;

    double wrist_down_position = 0.7; //Square
    double rotate_down_position = 0.75; // D-Pad Left
    double claw_down_position = 0.23; // D-Pad Down
    double wrist_up_position = 0.0; //Circle
    double rotate_up_position = 0.4; // D-Pad Right
    double claw_up_position = 0.0; // D-Pad Up

    @Override
    public void init() {
        // Initialize the servos
        rotateServo = hardwareMap.get(Servo.class, "rotate");
        clawServo = hardwareMap.get(Servo.class, "claw");
        wristServo = hardwareMap.get(Servo.class, "wrist");

        // Set initial positions if needed
        rotateServo.setPosition(0.5); // Default to mid position
        clawServo.setPosition(0.5);   // Default to mid position
        wristServo.setPosition(0.5);  // Default to mid position

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Control the rotate servo with the left and right D-pad
        if (gamepad1.dpad_left) {
            rotateServo.setPosition(rotate_down_position); // Fully open
            telemetry.addData("Rotate Servo", "Position: 0.5");
        } else if (gamepad1.dpad_right) {
            rotateServo.setPosition(rotate_up_position); // Fully closed
            telemetry.addData("Rotate Servo", "Position: 0.0");
        }

        // Control the claw servo with the top and bottom D-pad
        if (gamepad1.dpad_up) {
            clawServo.setPosition(claw_up_position); // Fully open
            telemetry.addData("Claw Servo", "Position: 0.7");
        } else if (gamepad1.dpad_down) {
            clawServo.setPosition(claw_down_position); // Fully closed
            telemetry.addData("Claw Servo", "Position: 0.0");
        }

        // Control the wrist servo with the circle and square buttons
        if (gamepad1.circle) {
            wristServo.setPosition(wrist_up_position); // Fully open
            telemetry.addData("Wrist Servo", "Position: 1.0");
        } else if (gamepad1.square) {
            wristServo.setPosition(wrist_down_position); // Fully closed
            telemetry.addData("Wrist Servo", "Position: 0.0");
        }

        // Update telemetry for debugging
        telemetry.update();
    }
}
