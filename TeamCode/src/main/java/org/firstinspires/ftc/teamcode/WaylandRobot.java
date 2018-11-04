package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class WaylandRobot {

    int leftmotor_offset = 0;
    int rightmotor_offset = 0;

    double servo_offset = 0;
    double motor_power = .75;

    double dropPinOut = .34;
    double dropPinIn = 0;

    int arm_offest = -35;

    double SIZE_OF_WHEEL_CM =9;
    double COUNTS_PER_CM = 288/(SIZE_OF_WHEEL_CM * 3.14) ;
    // take the initial value of the motor and add this.
    int armdistance = -4757;
    HardwarePushbot myself = new HardwarePushbot();

    public void initLeftmotor_offset() {
        leftmotor_offset = myself.leftDrive.getCurrentPosition();
    }

    public void initRightMotor_offset() {
        rightmotor_offset = myself.rightDrive.getCurrentPosition();
    }
    public void initHW(HardwareMap hardwareMap) { myself.init(hardwareMap);}

    public void initServo_offset() {
        servo_offset = myself.leftClaw.getPosition();
    }

    public void setMotor_power(double power){
        motor_power = power;
    }

    public void initArm_offset() { arm_offest = myself.leftArm.getCurrentPosition();}

    public void initRobot() {
        initLeftmotor_offset();
        initRightMotor_offset();
        initArm_offset();
        initServo_offset();
    }

    public void DriveForwardWithEncoder(int encval){
        int leftoffset = leftmotor_offset;
        int rightoffset = rightmotor_offset;
        leftoffset += encval;
        rightoffset += encval;
        myself.leftDrive.setTargetPosition(leftoffset);
        myself.rightDrive.setTargetPosition(rightoffset);
        myself.rightDrive.setPower(motor_power);
        myself.leftDrive.setPower(motor_power);

        leftmotor_offset = leftoffset;
        rightmotor_offset = rightoffset;

    }

    public void DropRobot(){
        myself.leftArm.setPower(motor_power);
        while(true) {
            //if sesnore
                myself.leftArm.setPower(0);
                myself.leftClaw.setPosition(200);
                break;
            //
        }
        myself.leftArm.setPower(-motor_power);
        while(true) {
            //if sensor2
                myself.leftArm.setPower(0);
                myself.leftClaw.setPosition(0);
                break;
            //
        }
    }

    public void OpenPin() {
        myself.leftClaw.setPosition(dropPinOut);
    }

    public void ClosePin() {
        myself.leftClaw.setPosition(dropPinIn);
    }

    public boolean IsPinOpen() {
        double position = myself.leftClaw.getPosition();
        if (position != 0) {
            return true;
        }
        return false;
    }

    public boolean IsPinClosed() {
        double position = myself.leftClaw.getPosition();
        if (position != .34){
            return true;
        }
        return false;
    }

    public void TogglePin() {
        if (IsPinClosed()) {
            OpenPin();
        } else {
            ClosePin();
        }
    }



    public void ArmUp() {
        myself.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myself.leftArm.setTargetPosition(arm_offest + armdistance);
        myself.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myself.leftArm.setPower(1);
    }

    public void ArmDown() {
        myself.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myself.leftArm.setTargetPosition(arm_offest);
        myself.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myself.leftArm.setPower(1);
    }

    public void ArmStop(){
        myself.leftArm.setPower(0);
    }

    public int ArmLocation() {
        return myself.leftArm.getCurrentPosition();
    }

    public int ArmWants() {
        return myself.leftArm.getTargetPosition();
    }

    public void leftDrive(double power) {
        myself.leftDrive.setPower(power);
    }

    public void rightDrive(double power) {
        myself.rightDrive.setPower(power);
    }


    public void wandArm(double power) { myself.rightArm.setPower(power);}

    public void stop() {
        myself.leftArm.setPower(0);
        myself.rightDrive.setPower(0);
        myself.leftDrive.setPower(0);
    }

    public boolean DetectBall() {
        return true;
    }
/*
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }*/


/*
    public void encoderDriveByCM(double speed,
                             double distance,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (true) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = myself.leftDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_CM);
            newRightTarget = myself.rightDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_CM);
            myself.leftDrive.setTargetPosition(newLeftTarget);
            myself.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            myself.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myself.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            //runtime.reset();
            myself.leftDrive.setPower(Math.abs(speed));
            myself.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (true &&
              //      (runtime.seconds() < timeoutS) &&
                    (myself.leftDrive.isBusy() && myself.rightDrive.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d",
                        myself.leftDrive.getCurrentPosition(),
                        myself.rightDrive.getCurrentPosition());
                //telemetry.update();
            }

            // Stop all motion;
            myself.leftDrive.setPower(0);
            myself.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            myself.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myself.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
        /*
        public boolean opModeIsActive(){
            return true;
        }*/
}

