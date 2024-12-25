package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@TeleOp(name = "VECTORADDITIONDRIVE", group = "Linear OpMode")
public class VectorTeleOp extends LinearOpMode {

    private DcMotor backleft = null;
    private DcMotor frontleft = null;
    private DcMotor backright = null;
    private DcMotor frontright = null;
    private DcMotor specimenslide = null;
    private Servo claw = null;
    private double targetposition = 0;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    @Override
    public void runOpMode() throws InterruptedException {

        configureWheelsSlideAndOdometer();

        // creation of tuning variables



        waitForStart();


        double specimenslide0 = specimenslide.getCurrentPosition();
        double specimenSlideMinimum = specimenslide0 + 100;
        double specimenSlideMaximum = specimenslide0 + 1600;
        double controllerSensitivity = 30;
        double powerSensitivity = 0.01;
        targetposition = specimenSlideMinimum;

        while (opModeIsActive()) {
            odo.update();
            logicForSpecimenSlide( controllerSensitivity, specimenSlideMaximum, specimenSlideMinimum, powerSensitivity, specimenslide0);
            fieldCentricMovement();
        }


    }

    private void fieldCentricMovement() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double theta = (Math.atan2(y, x) * 180 / Math.PI) - pos.getHeading(AngleUnit.DEGREES); // aka angle
//        telemetry.addData(Double.toString(theta), "theta");
//        telemetry.update();
        double power = Math.hypot(x, y);

        double sin = Math.sin((theta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((theta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFront = (power * cos / maxSinCos + turn);
        double rightFront = (power * sin / maxSinCos - turn);
        double leftBack = (power * sin / maxSinCos + turn);
        double rightBack = (power * cos / maxSinCos - turn);


        if ((power + Math.abs(turn)) > 1) {
            leftFront /= power + turn;
            rightFront /= power - turn;
            leftBack /= power + turn;
            rightBack /= power - turn;
        }

        frontleft.setPower(leftFront);
        backleft.setPower(leftBack);
        frontright.setPower(rightFront);
        backright.setPower(rightBack);
    }

    private void logicForSpecimenSlide(double controllersensitivity, double specimenslidemaximum, double specimenslideminimum, double powersensitivy, double specimenslide0) {
        double specimenslidepower;


        // main movement code of the specimen slide
        targetposition += (-gamepad2.right_stick_y) * controllersensitivity;
        if (targetposition > specimenslidemaximum) {
            targetposition = specimenslidemaximum;
        }
        if (targetposition < specimenslideminimum) {
            targetposition = specimenslideminimum;
        }

        specimenslidepower = powersensitivy * (targetposition - specimenslide.getCurrentPosition());


        if (specimenslidepower > 1) {
            specimenslidepower = 1;
        } else if (specimenslidepower < -1) {
            specimenslidepower = -1;
        }

        if (gamepad2.a) {
            claw.setPosition(0.73);
        } else if (gamepad2.b) {
            claw.setPosition(0.8);
        }

        specimenslide.setPower(specimenslidepower);
        telemetry.addData(Double.toString(specimenslidemaximum), "specimentSliemaximum");
        telemetry.addData(Double.toString(specimenslideminimum), "specimenslideminimum");
        telemetry.addData(Double.toString(specimenslide0), "specimentSlie0");
        telemetry.addData(Double.toString(specimenslidepower), "specimentSlidePower");
        telemetry.addData(Double.toString(targetposition), "targetposition");
        telemetry.addData(Double.toString(specimenslide.getCurrentPosition()), "currentposition");
        telemetry.addData(Double.toString(targetposition - specimenslide.getCurrentPosition()), "calculateddifference");
        telemetry.update();
    }

    private void addAndUpdateTelemetry() {
        telemetry.addData("Status", "Initialized version 3");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
    }

    private void configureWheelsSlideAndOdometer() {
        backright = hardwareMap.get(DcMotor.class, "frontright");
        frontright = hardwareMap.get(DcMotor.class, "backright");
        backleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontleft = hardwareMap.get(DcMotor.class, "backleft");

        specimenslide = hardwareMap.get(DcMotor.class, "specimenslide");
        claw = hardwareMap.get(Servo.class, "claw");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odocomp");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);

        specimenslide.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setOffsets(-72, -112);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        addAndUpdateTelemetry();

    }
}
