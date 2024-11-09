package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="autnomus ig")
public class prototype extends LinearOpMode {

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
    private DcMotor hangingAngle = null;
    private DcMotor hangingExtension = null;

    private RevTouchSensor nehal = null;

    @Override
    public void runOpMode() {

        waitForStart();
        runtime.reset();

        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fleft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bleft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
//        int x = 0;
//        x = x+1;
//
//        if(x == 2){
//
//            rightFrontDrive.setPower(1);
        while(opModeIsActive()){
            //forward auto
            rightFrontDrive.setPower(0.5);
            leftFrontDrive.setPower(0.5);
            rightBackDrive.setPower(-0.5);
            leftBackDrive.setPower(0.5);
            sleep(5000);

            //back auto
            rightFrontDrive.setPower(-0.5);
            leftFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(0.5);
            leftBackDrive.setPower(-0.5);
            sleep(5000);

            rightFrontDrive.setPower(-0.5);
            leftFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            leftFrontDrive.setPower(0.5);


        }


        }





    }