
    package org.firstinspires.ftc.teamcode;
    import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
    import com.qualcomm.hardware.rev.RevTouchSensor;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.TouchSensor;
    import com.qualcomm.robotcore.util.ElapsedTime;

    @TeleOp(name="ANTSMAINCODE", group="Linear OpMode")

    public class ANTSMAINCODE extends LinearOpMode {

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

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "fleft");
            leftBackDrive  = hardwareMap.get(DcMotor.class, "bleft");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
            rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
            rightintake = hardwareMap.get(CRServo.class,"rightintake");
            leftintake = hardwareMap.get(CRServo.class,"leftintake");
            angle = hardwareMap.get(DcMotor.class,"angle");
            extension = hardwareMap.get(DcMotor.class,"extension");
            nehal = hardwareMap.get(RevTouchSensor.class, "nehal");
            hangingAngle = hardwareMap.get(DcMotor.class,"hangingAngle");
            hangingExtension = hardwareMap.get(DcMotor.class, "hangingExtension");


            // ########################################################################################
            // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
            // ########################################################################################
            // Most robots need the motors on one side to be reversed to drive forward.
            // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
            // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
            // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
            // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
            // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
            // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            // Wait for the game to start (driver presses PLAY)

            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            int desiredPosition = angle.getCurrentPosition();
            int extensionPosition = extension.getCurrentPosition();
            int adit = hangingAngle.getCurrentPosition();
            int eric = hangingExtension.getCurrentPosition();
            boolean counter = false;
            while (opModeIsActive()) {
                telemetry.addData("Roan says ", "I wanna molest atremus velasco because hes mean to me unghhhhhhhhh");

                double max;
//                double max2;

                double axial   =  -gamepad1.left_stick_y;
                double lateral =  gamepad1.left_stick_x;
                double yaw     =  gamepad1.right_stick_x;

                double lateral2 = gamepad2.left_trigger - gamepad2.right_trigger;

                // axial is forward backward
                // lateral is side to side
                // yaw is angle
                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;

//                double leftFrontPower2 = lateral2;
//                double rightFrontPower2 = -lateral2;
//                double leftBackPower2 = -lateral2;
//                double rightBackPower2 = lateral2;

                double anglepower = -gamepad2.left_stick_y;

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

//                max2 = Math.max(Math.abs(leftFrontPower2), Math.abs(rightFrontPower2));
//                max2 = Math.max(max, Math.abs(leftBackPower2));
//                max2 = Math.max(max, Math.abs(rightBackPower2));
//                max2 = Math.max(max, Math.abs((gamepad1.right_trigger - gamepad1.left_trigger)*0.5));
//                if (max2 > 1.0) {
//                    leftFrontPower  /= max2; //leftFrontPower = leftFrontPower / max
//                    rightFrontPower /= max2;
//                    leftBackPower   /= max2;
//                    rightBackPower  /= max2;
//
//                }
                double extensionpower = gamepad2.left_stick_y;

                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);



                if(gamepad2.a){
                    rightintake.setPower(1);
                    leftintake.setPower(-1);
                } else if (gamepad2.b){
                    rightintake.setPower(-1);
                    leftintake.setPower(1);
                } else {
                    rightintake.setPower(0);
                    leftintake.setPower(0);
                }


                    telemetry.addData("angle position", angle.getCurrentPosition());
                    telemetry.addData("extension position", extension.getCurrentPosition());
                    telemetry.addData("stick position", gamepad2.left_stick_y);
                    telemetry.addData("extension power", extension.getPower());
                    telemetry.addData("angle power", angle.getPower());
                    telemetry.addData("extension variable value", extensionPosition);
                    telemetry.addData("eric value (extension)", eric);
                    telemetry.addData("hanging extension position", hangingExtension.getCurrentPosition());
                    telemetry.update();




                //if the touch sensor(white button) is pressed, arm goes back down

                if(nehal.isPressed()){
                    desiredPosition -= 30;
                } else {
                    //affects where the motor has to go when the user moves the right stick
                    if(gamepad2.right_stick_button){
                        desiredPosition += -10 * gamepad2.right_stick_y;
                    } else {
                        desiredPosition += -70 * gamepad2.right_stick_y;
                    }

                    angle.setTargetPosition(desiredPosition);
                    angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if(angle.getCurrentPosition() == desiredPosition){
                        angle.setPower(0);
                    } else {
                        angle.setPower(1);
                    }
//                    if(angle.getCurrentPosition() > desiredPosition){
//                        angle.setPower(0.5);
//                    } else {
//                        angle.setPower(gamepad2.right_stick_y);
//                    }
//                    if(gamepad1.left_stick_button){
//                        desiredPosition = angle.getCurrentPosition();
//                    }

                }


                //left trigger - right trigger
//                if (gamepad1.a){
//                    rightintake.setPower(100);
//                }

                if(gamepad1.x){
                    counter = true;
                } else if (gamepad1.y){
                    counter = false;
                }
                if(counter) {
                    extensionPosition += -30 * gamepad2.left_stick_y;
                    extension.setTargetPosition(extensionPosition);
                    extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //when to move and when to not move
                    if (extension.getCurrentPosition() == extensionPosition) {
                        extension.setPower(0);
                    } else {
                        extension.setPower(5);
                    }


                    if(gamepad2.left_bumper){
                        hangingExtension.setPower(1);
                    } else if (gamepad2.right_bumper){
                        hangingExtension.setPower(-1);
                    } else {
                        hangingExtension.setPower(0);
                    }
//                    hangingExtension.setTargetPosition(eric);
//                    hangingExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    //when to move and when to not move
//                    if (hangingExtension.getCurrentPosition() == eric) {
//                        hangingExtension.setPower(0);
//                    } else {
//                        hangingExtension.setPower(1);
//                    }


                    hangingAngle.setPower(0.7 * gamepad2.right_trigger - gamepad2.left_trigger);
                } else {
                    if(angle.getCurrentPosition){
                        if(extension.getCurrentPosition < 3500){
                            
                        }
                    }
                    extension.setPower(extensionpower);
                    extensionPosition = extension.getCurrentPosition();
                }
            }
        }
    }
