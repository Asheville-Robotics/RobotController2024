package org.firstinspires.ftc.teamcode.Extras;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Example")
public class Example extends OpMode
{

    DcMotor frontLeft;
    DcMotor frontRight;

    @Override
    public void init()
    {

        frontLeft = hardwareMap.dcMotor.get("fL");
        frontRight = hardwareMap.dcMotor.get("fR");

        // front left motor is configured the "fL"
        // drivetrain motors are configured to "[lowercase front or back] + [uppercase left or right]

    }

    @Override
    public void loop()
    {

        frontLeft.setPower(1);
        // Power is on a scale of -1 to 1. -1 is full reverse, 0 is not moving, and 1 is full forward

    }
}
