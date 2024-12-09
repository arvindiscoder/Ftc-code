package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;
import android.util.Size;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
@Autonomous(name = "BLUE detector")
public class colorporcessor extends LinearOpMode {



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
    public void runOpMode(){




        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fleft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
        rightintake = hardwareMap.get(CRServo.class,"rightintake");
        leftintake = hardwareMap.get(CRServo.class,"leftintake");
        angle = hardwareMap.get(DcMotor.class,"angle");
        extension = hardwareMap.get(DcMotor.class,"extension");

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {

            telemetry.addData("preview on/off", "... Camera Stream\n");


            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.
            telemetry.addLine(" Area Density Aspect  Center");
            for(ColorBlobLocatorProcessor.Blob b : blobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));

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

                    if(b.getDensity() >= 0.90){


                    }













                }
            int power = (int) 0.2;


            while (opModeIsActive()){




                }



            telemetry.update();
            sleep(50);
        }
    }





        }}




