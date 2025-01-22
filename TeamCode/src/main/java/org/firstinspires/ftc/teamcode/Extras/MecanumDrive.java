package org.firstinspires.ftc.teamcode.Extras;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Drivetrain", group = "examples")
public class MecanumDrive extends LinearOpMode {
    public DcMotor frmotor;
    public DcMotor flmotor;
    public DcMotor brmotor;
    public DcMotor blmotor;

    @Override
    public void runOpMode() throws InterruptedException{

        flmotor = hardwareMap.dcMotor.get("fL");
        frmotor = hardwareMap.dcMotor.get("fR");
        brmotor = hardwareMap.dcMotor.get("bR");
        blmotor = hardwareMap.dcMotor.get("bL");

        waitForStart();

        while (opModeIsActive()) {

            // double = decimal [0.21 is a double, 5.00 is a double]

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double turn = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

            frmotor.setPower( (y + x + turn) / denominator);
            brmotor.setPower( (y - x + turn) / denominator);
            flmotor.setPower( (y - x - turn) / denominator);
            blmotor.setPower( (y + x - turn) / denominator);

            flmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            blmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        }


    }

}