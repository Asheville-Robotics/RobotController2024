package org.firstinspires.ftc.teamcode.Extras;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class PIDF_Arm_And_Lift extends OpMode {

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT,
        LIFT_HOME,
        LIFT_PIVOT



    };

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    PIDF_Arm_And_Lift.LiftState liftState = PIDF_Arm_And_Lift.LiftState.LIFT_START;
    private PIDController arm_controller;
    private PIDController slide_controller;

    public static double arm_p = 0.008, arm_i = 0.0006, arm_d = 0.0008, arm_f = -0.12;
    public static int arm_target = 0;

    public static double slide_p = 0.012, slide_i = 0, slide_d = 0, slide_f = 0;
    public static int slide_target = 250;

    private final double arm_ticks_in_degree = 537.6 / 180;
    private final double slide_ticks_per_unit = 537.6 / 180; // Replace with the actual ticks per linear unit (e.g., mm)

    private DcMotorEx arm_motor;
    private DcMotorEx slide_motor;

    public Servo wrist_servo;

    // Deadband thresholds (tweak these values as needed)
    public static double arm_deadband = 0; // Example: ±5 ticks
    public static double slide_deadband = 0;

    final int LIFT_LOW = 250; // the low encoder position for the lift
    final int LIFT_MEDIUM = 900;

    final int LIFT_HIGH = 1800;

    final int ARM_LOW = 0;

    final int ARM_HIGH = 310;

    final int DUMP_TIME = 2;

    final double WRIST_DEPOSIT = 0;

    final double WRIST_IDLE = 0.8;

    ElapsedTime liftTimer = new ElapsedTime();
    @Override
    public void init() {
        liftTimer.reset();
        arm_controller = new PIDController(arm_p, arm_i, arm_d);
        slide_controller = new PIDController(slide_p, slide_i, slide_d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "pivot");
        slide_motor = hardwareMap.get(DcMotorEx.class, "lift");
        wrist_servo = hardwareMap.get(Servo.class, "wrist");

        arm_target = ARM_LOW;
        slide_target = LIFT_LOW;
    }

    @Override
    public void loop() {

        // Arm PIDF Controller
        arm_controller.setPID(arm_p, arm_i, arm_d);
        int armPos = arm_motor.getCurrentPosition();
        double arm_error = arm_target - armPos; // Calculate the error

        double arm_pid = arm_controller.calculate(armPos, arm_target);
        double arm_ff = Math.cos(Math.toRadians(arm_target / arm_ticks_in_degree)) * arm_f;

        double arm_power;
        if (Math.abs(arm_error) < arm_deadband) {
            arm_power = arm_ff; // Apply only feedforward within deadband
        } else {
            arm_power = arm_pid + arm_ff; // Apply PID + Feedforward outside deadband
        }
        arm_motor.setPower(arm_power);

        // Linear Slide PIDF Controller
        slide_controller.setPID(slide_p, slide_i, slide_d);
        int slidePos = slide_motor.getCurrentPosition();
        double slide_error = slide_target - slidePos; // Calculate the error

        double slide_pid = slide_controller.calculate(slidePos, slide_target);
        double slide_ff = slide_target * slide_f / slide_ticks_per_unit;

        double slide_power;
        if (Math.abs(slide_error) < slide_deadband) {
            slide_power = slide_ff; // Apply only feedforward within deadband
        } else {
            slide_power = slide_pid + slide_ff; // Apply PID + Feedforward outside deadband
        }
        slide_motor.setPower(slide_power);

        // Control arm and slide position

        arm_motor.setPower(arm_power);

        switch (liftState) {
            case LIFT_START:
                // Waiting for some input
                if (gamepad1.square) {
                    // x is pressed, start extending
                    slide_target = LIFT_LOW;
                    liftState = PIDF_Arm_And_Lift.LiftState.LIFT_PIVOT;
                }
                break;
            case LIFT_PIVOT:
                // check if the lift has finished extending,
                // otherwise do nothing.
                if (Math.abs(slide_motor.getCurrentPosition() - LIFT_LOW) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    arm_target = ARM_HIGH;

                    liftTimer.reset();
                    liftState = PIDF_Arm_And_Lift.LiftState.LIFT_HOME;
                }
                break;
            case LIFT_HOME:
                // check if the lift has finished extending,
                // otherwise do nothing.
                if (Math.abs(arm_motor.getCurrentPosition() - ARM_HIGH) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    slide_target = LIFT_HIGH;

                    liftTimer.reset();
                    liftState = PIDF_Arm_And_Lift.LiftState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                // check if the lift has finished extending,
                // otherwise do nothing.
                if (Math.abs(slide_motor.getCurrentPosition() - LIFT_HIGH) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    wrist_servo.setPosition(WRIST_DEPOSIT);

                    liftTimer.reset();
                    liftState = PIDF_Arm_And_Lift.LiftState.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                if (liftTimer.seconds() >= DUMP_TIME) {
                    // The robot waited long enough, time to start
                    // retracting the lift
                    wrist_servo.setPosition(WRIST_IDLE);
                    slide_target = LIFT_LOW;
                    liftState = PIDF_Arm_And_Lift.LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                if (Math.abs(slide_motor.getCurrentPosition() - LIFT_LOW) < 10) {

                    //move arm back to pick-up position
                    arm_target = ARM_LOW;
                    liftState = PIDF_Arm_And_Lift.LiftState.LIFT_START;
                }
                break;
            default:
                // should never be reached, as liftState should never be null
                liftState = PIDF_Arm_And_Lift.LiftState.LIFT_START;
        }

        // small optimization, instead of repeating ourselves in each
        // lift state case besides LIFT_START for the cancel action,
        // it's just handled here
        if (gamepad1.circle && (liftState != PIDF_Arm_And_Lift.LiftState.LIFT_START)) {
            liftState = PIDF_Arm_And_Lift.LiftState.LIFT_START;
        }


        // Telemetry
        telemetry.addData("Arm Pos", armPos);
        telemetry.addData("Arm Target", arm_target);
        telemetry.addData("Arm Power", arm_power);
        telemetry.addData("Slide Pos", slidePos);
        telemetry.addData("Slide Target", slide_target);
        telemetry.addData("Slide Power", slide_power);
        telemetry.update();
    }
}
