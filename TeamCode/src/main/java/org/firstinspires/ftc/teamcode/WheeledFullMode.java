package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Teleop Tank", group = "Oceanus")
//@Disabled
public class WheeledFullMode extends WheeledBotHardware {

    float driveMagnitude = 1f; //0.4f  // default drive magnitude

    @Override
    public void start() {

        // do what our parent says first
        super.start();

        //Set drive mode
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set drive mode0
        setElevatorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set gripper to open
        //closeGripper();

        telemetry.addData("joy", String.format("%.2f %.2f", 0.0, 0.0));
        telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f", positionX, positionY, Math.toDegrees(heading)));
        //telemetry.addData("rot", String.format("p:%3.0f r:%3.0f h:%3.0f", orientation.getPitch(), orientation.getRoll(), orientation.getHeading()));
    }

    @Override
    public void loop() {

        // do what our parent says first
        super.loop();

        //Get the values from the gamepads
        //Note: pushing the stick all the way up returns -1,
        // so we need to reverse the y values
        //We had to change the values here because they were inverted and backwards. X would drive forward and backwards and front was back and back was front
        float yValue = -gamepad1.left_stick_y;
        float xValue = gamepad1.left_stick_x;
        // float lValue = -gamepad1.right_stick_y;

        // scale inputs to non-linear to smooth control
        // lValue = (float) scaleInput(lValue);
        xValue = (float) scaleInput(xValue);
        yValue = (float) scaleInput(yValue);

        // arm control

        /** if (Math.abs(lValue) > 0.05) {
         float armMagnitude = lValue > 0 ? 0.4f : 0.3f;
         moveArm( armMagnitude * lValue);
         } else {
         stopArm();
         }

         **/
        if (gamepad2.left_stick_y < -0.1) {
            moveArmUp();
        }
        if (gamepad2.left_stick_y > 0.1) {
            moveArmDown();
        } else stopElevator();

        //Calculate the power needed for each motor
        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;

        // gripper control
        // This code will open and close the gripper with two buttons
        // using 'a' button to open and 'x' to close the gripper
        if (gamepad2.left_bumper) {
            openGripper();
            //driveMagnitude = 0.4f;
        }
        if (gamepad2.right_bumper ) {
            closeGripper();
            //driveMagnitude = 0.18f;
        }

        //Set the power of the motors with the gamepad values
        setDrivePower(driveMagnitude * leftPower, driveMagnitude * rightPower);

        // gripper push position
        /**  if ( gamepad1.left_bumper )
         pushLeftGripper();
         if ( gamepad1.right_bumper )
         pushRightGripper();
         **/
        telemetry.addData("joy", String.format("%.2f %.2f", xValue, yValue));
        telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f", positionX, positionY, Math.toDegrees(heading)));
        //telemetry.addData("rot", String.format("p:%3.0f r:%3.0f h:%3.0f", orientation.getPitch(), orientation.getRoll(), orientation.getHeading()));
        //telemetry.addData("touch", armTouch != null ? armTouch.isPressed() : "null");
        //telemetry.addData("arm", String.format("%s %.2f %d %s", onArmReset, armMotor.getPower(), armMotor.getCurrentPosition(), armMotor.getMode().toString()));
        //telemetry.addData("distance", String.format("%.2f", opticalDistanceSensor.getLightDetected()));
    }
}

