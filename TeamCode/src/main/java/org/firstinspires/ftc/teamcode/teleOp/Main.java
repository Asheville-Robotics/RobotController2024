package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleOp.Main;

@Config
@TeleOp
public class Main extends LinearOpMode {

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_DUMP_IDLE,
        LIFT_RETRACT,
        LIFT_HOME,
        LIFT_PIVOT,
        SAMPLE_SCAN,
        PICK_UP,
        SHOW_SAMPLE,



    };

    Main.LiftState liftState = Main.LiftState.LIFT_START;

    //definte PID Controllers
    private PIDController arm_controller;
    private PIDController slide_controller;

    public static double arm_p = 0.012, arm_i = 0.0006, arm_d = 0.0006, arm_f = 0.17;
    public static int arm_target = 0;

    public static double slide_p = 0.015, slide_i = 0, slide_d = 0, slide_f = 0;
    public static int slide_target = 0;

    //define motor ticks per degree

    private final double arm_ticks_in_degree = 537.6 / 180;
    private final double slide_ticks_per_unit = 537.6 / 180; // Replace with the actual ticks per linear unit (e.g., mm)

    //define arm motors

    private DcMotorEx arm_motor;
    private DcMotorEx slide_motor1;
    private DcMotorEx slide_motor2;
    private DcMotorEx slide_motor3;

    // Deadband thresholds (tweak these values as needed)
    public static double arm_deadband = 0; // Example: ±5 ticks
    public static double slide_deadband = 0;

    // Define servos
    private Servo rotateServo;
    private Servo clawServo;
    private Servo wristServo;

    //define servo positions

    double rotate_down_position = 0.75; // D-Pad Left
    double claw_down_position = 0.195; // D-Pad Down
    double rotate_up_position = 0.4; // D-Pad Right
    double claw_up_position = 0.0; // D-Pad Up

    final int LIFT_LOW = 0; // the low encoder position for the lift
    final int LIFT_MEDIUM = 600;

    final int LIFT_HIGH = 1600;

    final int ARM_LOW = 0;

    final int ARM_HIGH = 600;

    final int DUMP_TIME = 2;

    final double WRIST_DEPOSIT = 0;

    final double WRIST_IDLE = 0.8;


    //define drivetrain motors

    public DcMotor frmotor;
    public DcMotor flmotor;
    public DcMotor brmotor;
    public DcMotor blmotor;

    ElapsedTime liftTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        flmotor = hardwareMap.dcMotor.get("fL");
        frmotor = hardwareMap.dcMotor.get("fR");
        brmotor = hardwareMap.dcMotor.get("bR");
        blmotor = hardwareMap.dcMotor.get("bL");

        arm_controller = new PIDController(arm_p, arm_i, arm_d);
        slide_controller = new PIDController(slide_p, slide_i, slide_d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "pivot");
        slide_motor1 = hardwareMap.get(DcMotorEx.class, "lift1");
        slide_motor2 = hardwareMap.get(DcMotorEx.class, "lift2");
        slide_motor3 = hardwareMap.get(DcMotorEx.class, "lift3");

        // Initialize the servos
        rotateServo = hardwareMap.get(Servo.class, "rotate");
        clawServo = hardwareMap.get(Servo.class, "claw");
        wristServo = hardwareMap.get(Servo.class, "wrist");

        // Set initial positions if needed
        rotateServo.setPosition(rotate_down_position); // Default to mid position
        clawServo.setPosition(claw_up_position);   // Default to mid position
        wristServo.setPosition(WRIST_IDLE);  // Default to mid position

        arm_target = ARM_LOW;
        slide_target = LIFT_LOW;

        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {

            waitForStart();

            while (opModeIsActive()) {
                // Drivetrain control (unchanged)...

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
                int slidePos = slide_motor1.getCurrentPosition();
                double slide_error = slide_target - slidePos; // Calculate the error

                double slide_pid = slide_controller.calculate(slidePos, slide_target);
                double slide_ff = slide_target * slide_f / slide_ticks_per_unit;

                double slide_power;
                if (Math.abs(slide_error) < slide_deadband) {
                    slide_power = slide_ff; // Apply only feedforward within deadband
                } else {
                    slide_power = slide_pid + slide_ff; // Apply PID + Feedforward outside deadband
                }
                slide_motor1.setPower(slide_power);
                slide_motor2.setPower(slide_power);
                slide_motor3.setPower(slide_power);

                // Control arm and slide position

                arm_motor.setPower(arm_power);

                switch (liftState) {
                    case LIFT_START:
                        // Waiting for some input
                        if (gamepad1.square) {
                            // x is pressed, start extending
                            slide_target = LIFT_LOW;
                            liftState = Main.LiftState.LIFT_PIVOT;
                        }
                        break;
                    case LIFT_PIVOT:
                        // check if the lift has finished extending,
                        // otherwise do nothing.
                        if (Math.abs(slide_motor1.getCurrentPosition() - LIFT_LOW) < 10) {
                            // our threshold is within
                            // 10 encoder ticks of our target.
                            // this is pretty arbitrary, and would have to be
                            // tweaked for each robot.

                            // set the lift dump to dump
                            arm_target = ARM_HIGH;

                            liftTimer.reset();
                            liftState = Main.LiftState.LIFT_HOME;
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
                            liftState = Main.LiftState.LIFT_EXTEND;
                        }
                        break;
                    case LIFT_EXTEND:
                        // check if the lift has finished extending,
                        // otherwise do nothing.
                        if (Math.abs(slide_motor1.getCurrentPosition() - LIFT_HIGH) < 25) {
                            // our threshold is within
                            // 10 encoder ticks of our target.
                            // this is pretty arbitrary, and would have to be
                            // tweaked for each robot.

                            // set the lift dump to dump
                            wristServo.setPosition(WRIST_DEPOSIT);

                            liftTimer.reset();
                            liftState = Main.LiftState.LIFT_DUMP;
                        }
                        break;
                    case LIFT_DUMP:

                        if (liftTimer.seconds() >= DUMP_TIME) {
                            // The robot waited long enough, time to start
                            // retracting the lift
                            clawServo.setPosition(claw_up_position);
                        }
                        liftState = Main.LiftState.LIFT_DUMP_IDLE;
                        break;
                    case LIFT_DUMP_IDLE:
                         if (clawServo.getPosition() == claw_up_position) {
                             slide_target = LIFT_LOW;
                             wristServo.setPosition(WRIST_IDLE);
                         }
                            liftState = Main.LiftState.LIFT_RETRACT;
                        break;
                    case LIFT_RETRACT:
                        if (Math.abs(slide_motor1.getCurrentPosition() - LIFT_LOW) < 10) {

                            //move arm back to pick-up position
                            arm_target = ARM_LOW;
                            liftState = Main.LiftState.LIFT_START;
                        }
                        break;
                    default:
                        // should never be reached, as liftState should never be null
                        liftState = Main.LiftState.LIFT_START;
                }

                // small optimization, instead of repeating ourselves in each
                // lift state case besides LIFT_START for the cancel action,
                // it's just handled here
                if (gamepad1.circle && (liftState != Main.LiftState.LIFT_START)) {
                    liftState = Main.LiftState.LIFT_START;
                }

                // double = decimal [0.21 is a double, 5.00 is a double]

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;

                double turn = gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

                frmotor.setPower((y + x - turn) / denominator);
                brmotor.setPower((y - x - turn) / denominator);
                flmotor.setPower((y - x + turn) / denominator);
                blmotor.setPower((y + x + turn) / denominator);

                flmotor.setDirection(DcMotorSimple.Direction.REVERSE);
                blmotor.setDirection(DcMotorSimple.Direction.REVERSE);

                // Rotate servo control
                if (gamepad1.dpad_left) {
                    rotateServo.setPosition(rotate_down_position); // Rotate down
                    telemetry.addData("Rotate Servo", "Position: Down");
                } else if (gamepad1.dpad_right) {
                    rotateServo.setPosition(rotate_up_position); // Rotate up
                    telemetry.addData("Rotate Servo", "Position: Up");
                }

                // Claw servo control
                if (gamepad1.dpad_up) {
                    clawServo.setPosition(claw_up_position); // Fully open
                    telemetry.addData("Claw Servo", "Position: Open");
                } else if (gamepad1.dpad_down) {
                    clawServo.setPosition(claw_down_position); // Fully closed
                    telemetry.addData("Claw Servo", "Position: Closed");
                }

                if (gamepad1.right_bumper) {
                    slide_target = LIFT_MEDIUM;
                } else if (gamepad1.left_bumper) {
                    slide_target = LIFT_LOW;
                }
                // Telemetry for debugging
                telemetry.addData("Arm Pos", arm_motor.getCurrentPosition());
                telemetry.addData("Arm Target", arm_target);
                telemetry.addData("Slide Pos", slide_motor1.getCurrentPosition());
                telemetry.addData("Slide Pos", slide_motor2.getCurrentPosition());
                telemetry.addData("Slide Pos", slide_motor3.getCurrentPosition());
                telemetry.addData("Slide Target", slide_target);
                telemetry.update();

                // Update telemetry for debugging
                telemetry.update();

            }
        }

    }
}
