package org.firstinspires.ftc.teamcode.Extras;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Some declarations that are boilerplate are
 * skipped for the sake of brevity.
 * Since there are no real values to use, named constants will be used.
 */

@Config

@TeleOp(name="FSM Example")
public class FSMExample extends OpMode {
    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)
    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    };

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_START;

    // Some hardware access boilerplate; these would be initialized in init()
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
    // the lift motor, it's in RUN_TO_POSITION mode
    public DcMotorEx liftMotor;

    // the dump servo
    public Servo liftDump;
    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();

    final double DUMP_IDLE = 0.8; // the idle position for the dump servo
    final double DUMP_DEPOSIT = 0.2; // the dumping position for the dump servo

    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME = 2;

    final int LIFT_LOW = 530; // the low encoder position for the lift
    final int LIFT_HIGH = 0; // the high encoder position for the lift

    public double arm_deadband = 0;

    public void init() {
        liftTimer.reset();

        // hardware initialization code goes here


        // this needs to correspond with the configuration used
        liftMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        liftDump = hardwareMap.get(Servo.class, "wrist");

        liftMotor.setTargetPosition(LIFT_LOW);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loop() {


            // Arm PIDF Controller
            arm_controller.setPID(arm_p, arm_i, arm_d);
            int armPos = arm_motor.getCurrentPosition();
            double arm_error = arm_target - armPos;

            double arm_pid = arm_controller.calculate(armPos, arm_target);
            double arm_ff = Math.cos(Math.toRadians(arm_target / arm_ticks_in_degree)) * arm_f;

            double arm_power;
            if (Math.abs(arm_error) < arm_deadband) {
                arm_power = arm_ff; // Apply only feedforward within deadband
            } else {
                arm_power = arm_pid + arm_ff; // Apply PID + Feedforward outside deadband\
            }
            arm_motor.setPower(arm_power);

        switch (liftState) {
            case LIFT_START:
                // Waiting for some input
                if (gamepad1.square) {
                    // x is pressed, start extending
                    liftMotor.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                // check if the lift has finished extending,
                // otherwise do nothing.
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_HIGH) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    liftDump.setPosition(DUMP_DEPOSIT);

                    liftTimer.reset();
                    liftState = LiftState.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                if (liftTimer.seconds() >= DUMP_TIME) {
                    // The robot waited long enough, time to start
                    // retracting the lift
                    liftDump.setPosition(DUMP_IDLE);
                    liftMotor.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_LOW) < 10) {
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                // should never be reached, as liftState should never be null
                liftState = LiftState.LIFT_START;
        }

        // small optimization, instead of repeating ourselves in each
        // lift state case besides LIFT_START for the cancel action,
        // it's just handled here
        if (gamepad1.circle && (liftState != LiftState.LIFT_START)) {
            liftState = LiftState.LIFT_START;
        }

        // mecanum drive code goes here
        // But since none of the stuff in the switch case stops
        // the robot, this will always run
    }
}