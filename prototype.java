package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.



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
        int x = 0;
        x = x+1;

 //       if(x == 2){

            rightFrontDrive.setPower(1);
        while(opModeIsActive()){
            double max;


            double axial   =  0;
            double lateral =  0;
            double yaw     =  0;

            axial = 20;

            sleep(5000);
           // axial = -20;
            // axial is forward to backward
            // lateral is side to side
            // yaw is angle
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;



            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            max = Math.max(max, Math.abs((gamepad1.right_trigger - gamepad1.left_trigger)*0.5));
            if (max > 1.0) {
                leftFrontPower  /= max; //leftFrontPower = leftFrontPower / max
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;


            }


            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            if(x == 2){

                rightintake.setPower(0.2);
                leftintake.setPower(-0.2);

            }




            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       //     OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        }


        }





    }
