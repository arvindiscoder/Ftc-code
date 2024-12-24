
package org.firstinspires.ftc.teamcode;
// import android.graphics.`Camera`;

import android.graphics.Camera;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ANTSMAINCODE", group = "Linear OpMode")

public class ANTSMAINCODE extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backleft = null;
    private DcMotor frontleft = null;
    private DcMotor backright = null;
    private DcMotor frontright = null;
    private DcMotor rightintakeslide = null;
    private DcMotor leftintakeslide = null;


    private DcMotor specimenslide = null;

    private CRServo rightintake = null;

    private CRServo leftintake = null;
    private Servo heightservo = null;

    private Servo claw = null;


    private Camera camera;


    //   private Camera camera;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond

        // to the names assigned during the robot configuration step on the DS or RC devices.
        telemetry.addData("version ", "14");

        backright = hardwareMap.get(DcMotor.class, "frontright");
        frontright = hardwareMap.get(DcMotor.class, "backright");
        backleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontleft = hardwareMap.get(DcMotor.class, "backleft");

        specimenslide = hardwareMap.get(DcMotor.class, "specimenslide");
        rightintakeslide = hardwareMap.get(DcMotor.class, "rightslideintake");
        leftintakeslide = hardwareMap.get(DcMotor.class, "leftslideintake");
        rightintake = hardwareMap.get(CRServo.class, "rightintake");
        leftintake = hardwareMap.get(CRServo.class, "leftintake");
        heightservo = hardwareMap.get(Servo.class, "heightservo");
        claw = hardwareMap.get(Servo.class,"claw");


        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);

        telemetry.update();

        waitForStart();
        runtime.reset();

        long initialTime = System.currentTimeMillis();
        long endTime = System.currentTimeMillis();


        specimenslide.setDirection(DcMotorSimple.Direction.REVERSE);
//        specimenslide.setTargetPosition(-1000);
//        specimenslide.setPower(0.4);
//        while (specimenslide.isBusy()){
//        }
//        specimenslide.setPower(0);


        // initilization of the 0 position of the specimenslide
//        while (true) {
//            endTime = System.currentTimeMillis();
//
//            if (endTime - initialTime < 20) {
//                specimenslide.setPower(1);
//            } else {
//                specimenslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                break;
//            }
//        }
        // creation of tuning variables

        double specimenslideminimum = 100;
        double specimenslidemaximum = 1000;
        double controllersensitivity = 30;
        double powersensitivy = 0.01;

        // creation of non tuning variables

        double targetposition = specimenslideminimum;
        double specimenslidepower = 0;


        while (opModeIsActive()) {

            if(gamepad2.a){
                claw.setPosition(0.73);
            } else if (gamepad2.b) {
                claw.setPosition(0.8);
            }


            // main movement code of the specimen slide
            targetposition += (gamepad2.right_stick_y) * controllersensitivity;
//            if (targetposition>specimenslidemaximum){
//                targetposition = specimenslidemaximum;
//            }
//            if (targetposition < specimenslideminimum){
//                targetposition = specimenslideminimum;
//            }


            specimenslidepower = powersensitivy * (targetposition - specimenslide.getCurrentPosition());


            if (specimenslidepower > 1) {
                specimenslidepower = 1;
            } else if (specimenslidepower < -1) {
                specimenslidepower = -1;
            }
            specimenslide.setPower(specimenslidepower);
            telemetry.addData(Double.toString(specimenslidepower), "specimentSlidePower");
            telemetry.addData(Double.toString(targetposition), "targetposition");
            telemetry.addData(Double.toString(specimenslide.getCurrentPosition()), "currentposition");
            telemetry.addData(Double.toString(targetposition - specimenslide.getCurrentPosition()), "calculateddifference");
            telemetry.update();

            //specimenslide.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
//            telemetry.addData(Integer.toString(specimenslide.getCurrentPosition()),"specimentSlicePosition" );
//            telemetry.update();


            double intakeslidepower = gamepad2.right_trigger - gamepad2.left_trigger;

            rightintakeslide.setPower(intakeslidepower);
            leftintakeslide.setPower(-intakeslidepower);
            if (gamepad2.left_bumper) {
                rightintake.setPower(1);
                leftintake.setPower(-1);
            } else if (gamepad2.right_bumper) {
                rightintake.setPower(-1);
                leftintake.setPower(1);
            } else {
                rightintake.setPower(0);
                leftintake.setPower(0);
            }


            double axial = -gamepad1.left_stick_y;
            //axial is forward/backward
            double lateral = -gamepad1.left_stick_x;
            //lateral is left to right
            double yaw = 0.5 * gamepad1.right_stick_x;
            //yaw is angle


            double leftFrontPower = axial - lateral + yaw;
            double rightFrontPower = axial + lateral - yaw;
            double leftBackPower = axial + lateral + yaw;
            double rightBackPower = axial - lateral - yaw;


            backright.setPower(rightBackPower);
            backleft.setPower(leftBackPower);
            frontright.setPower(rightFrontPower);
            frontleft.setPower(leftFrontPower);


        }


    }
}
