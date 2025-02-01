package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@TeleOp(name = "arvauto", group = "auto")
public class ArvindTelop extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor specimenslide = null;
    double static int motorpower=0;
    double static int targetpositon = 0;
    double target2;
    


    publie void runOpMode{
        leftFrontDrive = hardwareMap.get(DcMotor.class, "backleft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "frontleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class,"backright");
        specimenslide = hardwareMap.get(DcMotor.class,"specimenslide");

        specimenslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        runtime.reset();
        //left front and back wheels reversed
        x=0;

        while(opModeIsActive){
            telementry.addData("motorpower", Double.toString(motorpower));
            telemetry.addData("Targetposition", double.toString(targetposition));
            speciposition = specimenslide.getCurrentPosition();
            specimenslide.setPower(targetpositon-specipsition/100);
            telemetry.update();
            if(x==0){
                leftFrontDrive.setPower(-motorpower);
                leftBackDrive.setPower(-motorpower);
                rightFrontDrive.setPower(motorpower);
                rightBackDrive.setPower(motorpower);
                sleep(1500);
                
            }
        }
    }
}
            
            







