
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
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name = "MAINSCODE", group = "Linear OpMode")

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

    private DcMotor intakespinner = null;


    private Servo heightservo = null;

    private Servo claw = null;

    GoBildaPinpointDriver odo;
    double oldTime = 0;



    private Camera camera;


    //   private Camera camera;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond

        // to the names assigned during the robot configuration step on the DS or RC devices.
        telemetry.addData("version ", "17");
        






        // WHEELS
        frontleft = hardwareMap.get(DcMotor.class, "backleft");
        backleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "backright");
        backright= hardwareMap.get(DcMotor.class, "frontright");


        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);



        // SPECIMENSLIDE
        specimenslide = hardwareMap.get(DcMotor.class, "specimenslide");
        claw = hardwareMap.get(Servo.class, "claw");

        // other
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odocomp");
        // Drive wheel rotation correction(positive powers make the wheels rotate forward
        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);

        specimenslide = hardwareMap.get(DcMotor.class, "specimenslide");
        rightintakeslide = hardwareMap.get(DcMotor.class, "rightslideintake");
        leftintakeslide = hardwareMap.get(DcMotor.class, "leftslideintake");

        heightservo = hardwareMap.get(Servo.class, "updownservo");
        intakespinner = hardwareMap.get(DcMotor.class,"intake");
        claw = hardwareMap.get(Servo.class,"claw");


        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);

        specimenslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();

        waitForStart();
        runtime.reset();

        long initialTime = System.currentTimeMillis();
        long endTime = System.currentTimeMillis();



        specimenslide.setDirection(DcMotorSimple.Direction.REVERSE);
        double specimenslideminimum = 100;
        double specimenslidemaximum = 1500;
        double controllersensitivity = 9;
        double powersensitivy = 0.01;
        double specimenslideminmaxoffset = 0;

        // creation of non tuning variables

        double targetposition = specimenslideminimum;
        double specimenslidepower = 0;


        while (opModeIsActive()) {
            if (gamepad2.right_bumper){
                specimenslideminmaxoffset += 2.5;
            } else if (gamepad2.left_bumper) {
                specimenslideminmaxoffset -= 2.5;

            }
            specimenslidemaximum += specimenslideminmaxoffset;
            specimenslideminimum += specimenslideminmaxoffset;

            if(gamepad2.x){
                heightservo.setPosition(1);
            }else if (gamepad2.y){
                heightservo.setPosition(0);
            }

            if(gamepad2.a){
                claw.setPosition(0.73);
            } else if (gamepad2.b) {
                claw.setPosition(0.8);
            }







            // main movement code of the specimen slide

            targetposition = targetposition + ( (-gamepad2.right_stick_y) * controllersensitivity);

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
//            telemetry.addData(Double.toString(specimenslidepower), "specimentSlidePower");
//            telemetry.addData(Double.toString(targetposition), "targetposition");
//            telemetry.addData(Double.toString(specimenslide.getCurrentPosition()), "currentposition");
//            telemetry.addData(Double.toString(targetposition - specimenslide.getCurrentPosition()), "calculateddifference");
//            telemetry.update();

            //specimenslide.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
//            telemetry.addData(Integer.toString(specimenslide.getCurrentPosition()),"specimentSlicePosition" );
//            telemetry.update();


            double intakeslidepower = gamepad2.right_trigger - gamepad2.left_trigger;

            rightintakeslide.setPower(intakeslidepower);
            leftintakeslide.setPower(-intakeslidepower);




            fieldCentricMovement();
            intakespinner.setPower(gamepad2.right_stick_y);

        }


    }

    private void fieldCentricMovement() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        //telemetry.addData("heading",Double.toString(pos.getHeading(AngleUnit.DEGREES)));

        double theta = (Math.atan2(y, x) * 180 / Math.PI) ; // aka angle
        telemetry.addData("theta", Double.toString(theta));
        telemetry.addData("heading",Double.toString(pos.getHeading(AngleUnit.DEGREES)));

//        telemetry.addData(Double.toStg(theta), "theta");
//        telemetry.update();
        double power = Math.hypot(x, y);

        double sin = Math.sin((theta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((theta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontpower = (power * cos / maxSinCos + turn);
        double rightFrontpower = (power * sin / maxSinCos - turn);
        double leftBackpower = (power * sin / maxSinCos + turn);
        double backrightpower = (power * cos / maxSinCos - turn);

        telemetry.addData("power", Double.toString(power));
        telemetry.addData("maxSinCos", Double.toString(maxSinCos));


        if ((power + Math.abs(turn)) > 1) {
            leftFrontpower /= power + turn;
            rightFrontpower /= power - turn;
            leftBackpower /= power + turn;
            backrightpower /= power - turn;
        }

        frontleft.setPower(leftFrontpower);
        backleft.setPower(leftBackpower);
        frontright.setPower(rightFrontpower);
        backright.setPower(backrightpower);
    }

    private void odometryrun () {
        /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
        odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
        //odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);




            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = 1 / loopTime;
        oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Velocity", velocity);
//
//
//            telemetry.addData("X offset", odo.getXOffset());



            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in

            */
//            telemetry.addData("Status", odo.getDeviceStatus());
//
//            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
//
//
//            telemetry.addData("targetpos", Double.toString(targetXposition));
//
//            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
//            telemetry.update();
    }
}



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
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name = "MAINSCODE", group = "Linear OpMode")

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

    private DcMotor intakespinner = null;


    private Servo heightservo = null;

    private Servo claw = null;

    GoBildaPinpointDriver odo;
    double oldTime = 0;



    private Camera camera;


    //   private Camera camera;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond

        // to the names assigned during the robot configuration step on the DS or RC devices.
        telemetry.addData("version ", "17");
        






        // WHEELS
        frontleft = hardwareMap.get(DcMotor.class, "backleft");
        backleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "backright");
        backright= hardwareMap.get(DcMotor.class, "frontright");


        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);



        // SPECIMENSLIDE
        specimenslide = hardwareMap.get(DcMotor.class, "specimenslide");
        claw = hardwareMap.get(Servo.class, "claw");

        // other
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odocomp");
        // Drive wheel rotation correction(positive powers make the wheels rotate forward
        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);

        specimenslide = hardwareMap.get(DcMotor.class, "specimenslide");
        rightintakeslide = hardwareMap.get(DcMotor.class, "rightslideintake");
        leftintakeslide = hardwareMap.get(DcMotor.class, "leftslideintake");

        heightservo = hardwareMap.get(Servo.class, "updownservo");
        intakespinner = hardwareMap.get(DcMotor.class,"intake");
        claw = hardwareMap.get(Servo.class,"claw");


        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);

        specimenslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();

        waitForStart();
        runtime.reset();

        long initialTime = System.currentTimeMillis();
        long endTime = System.currentTimeMillis();



        specimenslide.setDirection(DcMotorSimple.Direction.REVERSE);
        double specimenslideminimum = 100;
        double specimenslidemaximum = 1500;
        double controllersensitivity = 9;
        double powersensitivy = 0.01;
        double specimenslideminmaxoffset = 0;

        // creation of non tuning variables

        double targetposition = specimenslideminimum;
        double specimenslidepower = 0;


        while (opModeIsActive()) {
            if (gamepad2.right_bumper){
                specimenslideminmaxoffset += 2.5;
            } else if (gamepad2.left_bumper) {
                specimenslideminmaxoffset -= 2.5;

            }
            specimenslidemaximum += specimenslideminmaxoffset;
            specimenslideminimum += specimenslideminmaxoffset;

            if(gamepad2.x){
                heightservo.setPosition(1);
            }else if (gamepad2.y){
                heightservo.setPosition(0);
            }

            if(gamepad2.a){
                claw.setPosition(0.73);
            } else if (gamepad2.b) {
                claw.setPosition(0.8);
            }







            // main movement code of the specimen slide

            targetposition = targetposition + ( (-gamepad2.right_stick_y) * controllersensitivity);

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
//            telemetry.addData(Double.toString(specimenslidepower), "specimentSlidePower");
//            telemetry.addData(Double.toString(targetposition), "targetposition");
//            telemetry.addData(Double.toString(specimenslide.getCurrentPosition()), "currentposition");
//            telemetry.addData(Double.toString(targetposition - specimenslide.getCurrentPosition()), "calculateddifference");
//            telemetry.update();

            //specimenslide.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
//            telemetry.addData(Integer.toString(specimenslide.getCurrentPosition()),"specimentSlicePosition" );
//            telemetry.update();


            double intakeslidepower = gamepad2.right_trigger - gamepad2.left_trigger;

            rightintakeslide.setPower(intakeslidepower);
            leftintakeslide.setPower(-intakeslidepower);




            fieldCentricMovement();
            intakespinner.setPower(gamepad2.right_stick_y);

        }


    }

    private void fieldCentricMovement() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        //telemetry.addData("heading",Double.toString(pos.getHeading(AngleUnit.DEGREES)));

        double theta = (Math.atan2(y, x) * 180 / Math.PI) ; // aka angle
        telemetry.addData("theta", Double.toString(theta));
        telemetry.addData("heading",Double.toString(pos.getHeading(AngleUnit.DEGREES)));

//        telemetry.addData(Double.toStg(theta), "theta");
//        telemetry.update();
        double power = Math.hypot(x, y);

        double sin = Math.sin((theta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((theta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontpower = (power * cos / maxSinCos + turn);
        double rightFrontpower = (power * sin / maxSinCos - turn);
        double leftBackpower = (power * sin / maxSinCos + turn);
        double backrightpower = (power * cos / maxSinCos - turn);

        telemetry.addData("power", Double.toString(power));
        telemetry.addData("maxSinCos", Double.toString(maxSinCos));


        if ((power + Math.abs(turn)) > 1) {
            leftFrontpower /= power + turn;
            rightFrontpower /= power - turn;
            leftBackpower /= power + turn;
            backrightpower /= power - turn;
        }

        frontleft.setPower(leftFrontpower);
        backleft.setPower(leftBackpower);
        frontright.setPower(rightFrontpower);
        backright.setPower(backrightpower);
    }

    private void odometryrun () {
        /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
        odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
        //odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);




            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = 1 / loopTime;
        oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Velocity", velocity);
//
//
//            telemetry.addData("X offset", odo.getXOffset());



            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in

            */
//            telemetry.addData("Status", odo.getDeviceStatus());
//
//            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
//
//
//            telemetry.addData("targetpos", Double.toString(targetXposition));
//
//            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
//            telemetry.update();
    }
}


