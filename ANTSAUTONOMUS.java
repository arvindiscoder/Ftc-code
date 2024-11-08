package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="hopeforthebest")

public abstract class prototype extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private CRServo rightintake;
    private CRServo leftintake;
    private DcMotor extension = null;
    private DcMotor angle = null;

    private RevTouchSensor nehal = null;
    public void runOpMode(){
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fleft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
        rightintake = hardwareMap.get(CRServo.class,"rightintake");
        leftintake = hardwareMap.get(CRServo.class,"leftintake");
        angle = hardwareMap.get(DcMotor.class,"angle");
        extension = hardwareMap.get(DcMotor.class,"extension");
        nehal = hardwareMap.get(RevTouchSensor.class, "nehal");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        init();
        while (opModeIsActive()) {
            leftFrontDrive.setPower(0.5);
            sleep(5000);
            leftFrontDrive.setPower(0);

        }
    }}








 //   @Override
//    public void init() {
//
//
//
//        waitForStart();
//        runtime.reset();
//
//        motor = hardwareMap.dcMotor.get("fleft");
//
//
//    }
//
//    public void loop() {
//
//        while (opModeIsActive()) {
//            telemetry.addData("Roan says ", "I wanna molest atremus velasco because hes mean to me unghhhhhhhhh");
//
//
//            motor.setPower(1);
//
//    }
//}}
