package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Base class for the op-modes for the Clock robot (FTC team 9785 / Cronos).
 * <p>
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any  fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 */
public class WheeledBotHardware extends OpMode {

    final double LEFT_OPEN_POSITION    = 0.7;
    final double LEFT_CLOSED_POSITION  = 1.0;
    final double LEFT_PUSH_POSITION    = 0.9;
    final double RIGHT_OPEN_POSITION   = 1.0;
    final double RIGHT_CLOSED_POSITION = 0.7;
    final double RIGHT_PUSH_POSITION   = 0.9;

    DcMotor leftRearMotor;
    DcMotor leftFrontMotor;
    DcMotor rightRearMotor;
    DcMotor rightFrontMotor;
    DcMotor elvMotor;
    Servo leftGrip;
    Servo rightGrip;
    GyroSensor gyroSensor;
    TouchSensor armTouch;
    TouchSensor beaconTouch;
    OpticalDistanceSensor opticalDistanceSensor;
    //GyroIntegrator orientation;

    private int prevLeftRearStep;
    private int prevLeftFrontStep;
    private int prevRightRearStep;
    private int prevRightFrontStep;

    /**
     * Absolute position of the robot in the x-axis.
     */
    public double positionX;
    /**
     * Absolute position of the robot in the y-axis.
     */
    public double positionY;

    /**
     * Absolute heading (z axis / up) of the robot in radians.
     */
    public double heading;

    /**
     * Indicates that location encoders is under reset.
     */
    public boolean onArmReset;


    @Override
    public void init() {

        StringBuilder sb = new StringBuilder();

        sb.append("lr_drive: ");
        try {
            leftRearMotor = hardwareMap.dcMotor.get("lr_drive");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("lf_drive: ");
        try {
            leftFrontMotor = hardwareMap.dcMotor.get("lf_drive");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("rr_drive: ");
        try {
            rightRearMotor = hardwareMap.dcMotor.get("rr_drive");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("rf_drive: ");
        try {
            rightFrontMotor = hardwareMap.dcMotor.get("rf_drive");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("arm_drive: ");
        try {
            elvMotor = hardwareMap.dcMotor.get("elevator_drive");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("arm_touch: ");
        try {
            armTouch = hardwareMap.touchSensor.get("arm_touch");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("beacon_touch: ");
        try {
            beaconTouch = hardwareMap.touchSensor.get("beacon_touch");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("left_grip: ");
        try {
            leftGrip = hardwareMap.servo.get("left_grip");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("right_grip: ");
        try {
            rightGrip = hardwareMap.servo.get("right_grip");
            sb.append("OK ");
        } catch (Exception ex) {
            sb.append("ERR ");
        }
//        sb.append("gyro ");
//        try {
//            gyroSensor = hardwareMap.gyroSensor.get("gyro");
//            sb.append("OK ");
//        } catch (Exception ex) {
//            sb.append("ERR ");
//        }
//        sb.append("optical ");
//        try {
//            opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("optical");
//            sb.append("OK ");
//        } catch (Exception ex) {
//            sb.append("ERR ");
//        }

        //Reverse the left-rear and right-front motors
        if (leftRearMotor != null)
            leftRearMotor.setDirection(DcMotor.Direction.FORWARD);

        //Both motors on the right side need to move in the same direction
        if (rightRearMotor != null)
            rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        if (rightFrontMotor != null)
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        // calibrate gyro
//        if (gyroSensor != null) {
//            gyroSensor.calibrate();
//
//            // make sure the gyro is calibrated
//            while (gyroSensor.isCalibrating()) {
//                try {
//                    Thread.sleep(50);
//                }
//                catch(InterruptedException ex) {
//                    sb.append(String.format("exception: %s", ex.toString()));
//                }
//            }
//
//            //orientation = new GyroIntegrator();
//        }

        //Set gripper to close
        //closeGripper();

        //Prepare drive
        resetDriveEncoders();

        //Prepare arm
        //resetArmEncoders();

        //Report status
        telemetry.addData("Status", sb.toString());
    }

    @Override
    public void loop() {
        /***
         // on arm reset, keep changing drive mode until ready
         DcMotor.RunMode mode = armMotor.getMode();
         if ( onArmReset ) {
         // force a reset until we detect a position==0
         if ( mode != DcMotor.RunMode.STOP_AND_RESET_ENCODER || armMotor.getCurrentPosition() != 0)
         armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         // signal reset done when ready
         if ( mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER && armMotor.getCurrentPosition() == 0) {
         onArmReset = false;
         }
         }
         else if (mode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
         // force a power mode until we detect it
         armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         }
         ***/

        // update absolution position
        //updatePosition();

//        if ( orientation != null && gyroSensor != null && !gyroSensor.isCalibrating()) {
//            orientation.update( gyroSensor.rawX(), gyroSensor.rawY(), gyroSensor.rawZ());
//        }
    }

    /**
     * Set the gripper to open position.
     */
    void openGripper() {
        if (leftGrip != null)
            leftGrip.setPosition(LEFT_OPEN_POSITION);
        if (rightGrip != null)
            rightGrip.setPosition(RIGHT_OPEN_POSITION);
    }

    /**
     * Set the gripper to close position.
     */
    void closeGripper() {
        if (leftGrip != null)
            leftGrip.setPosition(LEFT_CLOSED_POSITION);
        if (rightGrip != null)
            rightGrip.setPosition(RIGHT_CLOSED_POSITION);
    }

    /**
     * Set the right gripper to push position.
     */
    /**void pushRightGripper() {
     if (rightGrip != null)
     rightGrip.setPosition(RIGHT_PUSH_POSITION);
     }
     **/
    /**
     * Set the left gripper to push position.
     */
    /**
     void pushLeftGripper() {
     if (leftGrip != null)
     leftGrip.setPosition(LEFT_PUSH_POSITION);
     }
     **/
    /**
     * Move the arm with the specified power: positive value raises the arm.
     *
     * @param power the power level: positive (up)/negative (down)
     */
    void moveElevator(double power) {
        //Clip the power values so that it only goes from -1 to 1
        power = Range.clip(power, -1, 1);

        if (elvMotor != null && elvMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            boolean atLowerLimit = armTouch != null && armTouch.isPressed();
            boolean atUpperLimit = false; //elvMotor.getCurrentPosition() > 2000;

            if (atLowerLimit && power < 0 && !onArmReset) {
                stopElevator();
                resetElevatorEncoders();

                // we are done
                return;
            }

            // do not keep raising the arm if at upper limit
            if (atUpperLimit && power > 0)
                power = 0;

            // do not keep lowering the arm if at lower limit
            if (atLowerLimit && power < 0)
                power = 0;

            // ok send the power level
            elvMotor.setPower(power);
        }
    }
    /**  void moveArm(double power) {
     //Clip the power values so that it only goes from -1 to 1
     power = Range.clip(power, -1, 1);

     if (armMotor != null && armMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
     boolean atLowerLimit = armTouch != null && armTouch.isPressed();
     boolean atUpperLimit = armMotor.getCurrentPosition() > 2000;

     if (atLowerLimit && power < 0 && !onArmReset) {
     stopArm();
     resetArmEncoders();

     // we are done
     return;
     }


     // do not keep raising the arm if at upper limit
     if (atUpperLimit && power > 0)
     power = 0;

     // do not keep lowering the arm if at lower limit
     if (atLowerLimit && power < 0)
     power = 0;

     // ok send the power level
     armMotor.setPower(power);
     armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
     }
     }
     **/
    /**
     * Stop arm movement.
     */
    void moveArmUp() {
        elvMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elvMotor.setPower(.5);
    }

    void moveArmDown() {
        elvMotor.setPower(-.5);
    }

    void stopElevator() {
        if (elvMotor != null) {
            elvMotor.setPower(0);
        }
    }

    /**
     * Reset arm motor encoders.
     */
    void resetElevatorEncoders() {
        if (elvMotor != null) {
            // stop motor
            elvMotor.setPower(0);

            // send command
            elvMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // we are now in reset mode
            onArmReset = true;
        }
    }

    /**
     * Set the arm motor to the specified mode.
     *
     * @param mode the motor run mode
     */
    void setElevatorMode(DcMotor.RunMode mode) {
        if (elvMotor != null)
            elvMotor.setMode(mode);
    }


    /**
     * Reset drive motor encoders.
     */
    void resetDriveEncoders() {
        if (leftRearMotor != null)
            leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (leftFrontMotor != null)
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rightRearMotor != null)
            rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (rightFrontMotor != null)
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (gyroSensor != null) {
            gyroSensor.resetZAxisIntegrator();
        }

        // reset relative variables
        prevLeftRearStep = 0;
        prevLeftFrontStep = 0;
        prevRightRearStep = 0;
        prevRightFrontStep = 0;

        // reset absolute variables
        positionX = 0.0;
        positionY = 0.0;
        heading = 0.0;
    }

    /**
     * Use the motor encoders and the gyro sensor to update the absolute position of the robot.
     */
    private void updatePosition() {
        double distance = 0;
        int motors = 0;
        int leftRearStep = 0;
        int leftFrontStep = 0;
        int rightRearStep = 0;
        int rightFrontStep = 0;
        if (leftRearMotor != null) {
            leftRearStep = leftRearMotor.getCurrentPosition();
            distance += leftRearStep - prevLeftRearStep;
            motors += 1;
        }
        if (leftFrontMotor != null) {
            leftFrontStep = leftFrontMotor.getCurrentPosition();
            distance += leftFrontStep - prevLeftFrontStep;
            motors += 1;
        }
        if (rightRearMotor != null) {
            rightRearStep = rightRearMotor.getCurrentPosition();
            distance += rightRearStep - prevRightRearStep;
            motors += 1;
        }
        if (rightFrontMotor != null) {
            rightFrontStep = rightFrontMotor.getCurrentPosition();
            distance += rightFrontStep - prevRightFrontStep;
            motors += 1;
        }

        if (motors > 0 && gyroSensor != null) {
            // compute average of distance
            distance = distance / motors;

            // read angle
            int rawdeg = gyroSensor.getHeading();
            heading = Math.toRadians(rawdeg);

            //telemetry.addData("raw", String.format("%d %.0f", raw, distance));

            // compute displacement
            double dx = distance * Math.sin(heading);
            double dy = distance * Math.cos(heading);

            // update position
            positionX = positionX + dx;
            positionY = positionY + dy;
        }

        // prepare for next reading
        prevLeftRearStep = leftRearStep;
        prevLeftFrontStep = leftFrontStep;
        prevRightRearStep = rightRearStep;
        prevRightFrontStep = rightFrontStep;

        //telemetry.addData("front", String.format("%d %d", -leftFrontStep, -rightFrontStep));
        //telemetry.addData("rear", String.format("%d %d", -leftRearStep, -rightRearStep));
    }

    /**
     * Set the drive motors to the specified mode.
     *
     * @param mode the motor run mode
     */
    void setDriveMode(DcMotor.RunMode mode) {
        if (leftRearMotor != null)
            leftRearMotor.setMode(mode);
        if (leftFrontMotor != null)
            leftFrontMotor.setMode(mode);
        if (rightRearMotor != null)
            rightRearMotor.setMode(mode);
        if (rightFrontMotor != null)
            rightFrontMotor.setMode(mode);
    }

    /**
     * Set the power for both drive motors using the specified value.
     *
     * @param power the power level
     */
    void setDrivePower(double power) {
        //Clip the power values so that it only goes from -1 to 1
        power = Range.clip(power, -1, 1);

        // stop moving when distance sensor says we are close to an obstacle and we want to move forward
        boolean stop = false;//opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2 && power > 0;

        if (!stop) {
            if (leftRearMotor != null)
                leftRearMotor.setPower(power);
            if (leftFrontMotor != null)
                leftFrontMotor.setPower(power);
            if (rightRearMotor != null)
                rightRearMotor.setPower(power);
            if (rightFrontMotor != null)
                rightFrontMotor.setPower(power);
        }
    }

    /**
     * Set the power for both drive motors using the specified values.
     *
     * @param leftPower  the power level of the left drive motor
     * @param rightPower the power level of the right drive motor
     */
    void setDrivePower(double leftPower, double rightPower) {
        //Clip the power values so that it only goes from -1 to 1
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        // stop moving when distance sensor says we are close to an obstacle and we want to move forward
        boolean stop = false;//opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2 && (leftPower > 0 || rightPower > 0);

        if (!stop) {
            if (leftRearMotor != null)
                leftRearMotor.setPower(leftPower);
            if (leftFrontMotor != null)
                leftFrontMotor.setPower(leftPower);
            if (rightRearMotor != null)
                rightRearMotor.setPower(rightPower);
            if (rightFrontMotor != null)
                rightFrontMotor.setPower(rightPower);
        }
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double value) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) Math.round(value * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (value < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    /**
     * Normalizes an angle in radians between the range of [reference, reference + 2 PI).
     *
     * @param angle     the angle in radians
     * @param reference the reference in radians
     * @return the normalized angle in radians
     */
    public static double NormalizeAngle(double angle, double reference) {
        final double TWO_PI = 2.0 * Math.PI;
        double result = (angle - reference) % TWO_PI;
        return result < 0.0 ? TWO_PI + reference + result : reference + result;
    }
}