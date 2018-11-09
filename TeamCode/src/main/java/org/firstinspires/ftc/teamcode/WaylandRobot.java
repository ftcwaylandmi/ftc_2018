package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.WaylandHardware;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.util.ElapsedTime;


public class WaylandRobot {

    int leftmotor_offset = 0;
    int rightmotor_offset = 0;

    double servo_offset = 0;
    double motor_power = 1;
    double drop_power = 1;

    int goldSoundID;

    double WinchMin = .1;
    double WinchMax = .55;
    double WinchPosition = 0;
    double dropPinOut = .50;
    double dropPinIn = -.44;

    /*
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    int relativeLayoutId;
    View relativeLayout;
    */

    boolean is_busy = false;

    int arm_offset = 15;



    double SIZE_OF_WHEEL_CM =9;
    double COUNTS_PER_CM = 288/(SIZE_OF_WHEEL_CM * 3.14) ;
    // take the initial value of the motor and add this.
    int armdistance = -4153;
    WaylandHardware myself = new WaylandHardware();
    private boolean goldFound;

    public void initLeftmotor_offset() {
        leftmotor_offset = myself.leftDrive.getCurrentPosition();
    }

    public void initRightMotor_offset() {
        rightmotor_offset = myself.rightDrive.getCurrentPosition();
    }
    public void initHW(HardwareMap hardwareMap) { myself.init(hardwareMap);
        goldSoundID = hardwareMap.appContext.getResources().getIdentifier("gold",   "raw", hardwareMap.appContext.getPackageName());
        if (goldSoundID != 0)
            goldFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);



    }

    public void initServo_offset() {
        servo_offset = myself.leftClaw.getPosition();
    }

    public void setMotor_power(double power){
        motor_power = power;
    }

    public void initArm_offset() { arm_offset = myself.leftArm.getCurrentPosition();}
/*
    public void initColorDistance() {
        // get a reference to the color sensor.
        sensorColor = myself.hwMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = myself.hwMap.get(DistanceSensor.class, "sensor_color_distance");

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        relativeLayoutId = myself.hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", myself.hwMap.appContext.getPackageName());
        relativeLayout = ((Activity) myself.hwMap.appContext).findViewById(relativeLayoutId);

    }
    */
    public void initRobot(boolean enableEncoders) {
        if (enableEncoders) {
            EnableEncoders();
            initLeftmotor_offset();
            initRightMotor_offset();
        }
        initArm_offset();
        initServo_offset();
        //initColorDistance();
    }
    public int GetLeftTarget() {
        return myself.leftDrive.getTargetPosition();
    }

    public int GetRightTarget() {
        return myself.rightDrive.getTargetPosition();
    }

    public int GetRightCurrent() {
        return myself.rightDrive.getCurrentPosition();
    }

    public int GetLeftCurrent() {
        return myself.leftDrive.getCurrentPosition();
    }

    public void DriveByTime(int msval){
        ElapsedTime timer =  new ElapsedTime();
        timer.reset();
        myself.rightDrive.setPower(1);
        myself.leftDrive.setPower(1);
        while (timer.milliseconds() < msval) {

        }
        myself.rightDrive.setPower(0);
        myself.leftDrive.setPower(0);
    }

    public void DriveByRightTime(int msval){
        ElapsedTime timer =  new ElapsedTime();
        timer.reset();
        myself.rightDrive.setPower(-1);
        myself.leftDrive.setPower(1);
        while (timer.milliseconds() < msval) {

        }
        myself.rightDrive.setPower(0);
        myself.leftDrive.setPower(0);
    }


    public void DriveByLeftTime(int msval){
        ElapsedTime timer =  new ElapsedTime();
        timer.reset();
        myself.rightDrive.setPower(1);
        myself.leftDrive.setPower(-1);
        while (timer.milliseconds() < msval) {

        }
        myself.rightDrive.setPower(0);
        myself.leftDrive.setPower(0);
    }


    public void EnableEncoders() {
        myself.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myself.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DisableEncoders() {
        myself.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myself.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void DriveForwardWithEncoder(int encval){
        is_busy = true;
        int leftoffset = myself.leftDrive.getCurrentPosition() + encval;
        int rightoffset = myself.rightDrive.getCurrentPosition() + encval;
        myself.leftDrive.setTargetPosition(leftoffset);
        myself.rightDrive.setTargetPosition(rightoffset);
        myself.rightDrive.setPower(motor_power);
        myself.leftDrive.setPower(motor_power);

        leftmotor_offset = leftoffset;
        rightmotor_offset = rightoffset;
        is_busy = false;
    }

    public void DriveLeftWithEncoder(int encval){
        is_busy = true;
        int leftoffset = myself.leftDrive.getCurrentPosition() - encval;
        int rightoffset = myself.rightDrive.getCurrentPosition() + encval;

        myself.leftDrive.setTargetPosition(leftoffset);
        myself.rightDrive.setTargetPosition(rightoffset);
        myself.rightDrive.setPower(motor_power);
        myself.leftDrive.setPower(motor_power);
        leftmotor_offset = leftoffset;
        rightmotor_offset = rightoffset;
        is_busy = false;

    }

    public void DriveRightWithEncoder(int encval){

        int leftoffset = myself.leftDrive.getCurrentPosition() + encval;
        int rightoffset = myself.rightDrive.getCurrentPosition() - encval;
        myself.leftDrive.setTargetPosition(leftoffset);
        myself.rightDrive.setTargetPosition(rightoffset);
        myself.rightDrive.setPower(motor_power);
        myself.leftDrive.setPower(motor_power);

        leftmotor_offset = leftoffset;
        rightmotor_offset = rightoffset;
    }

    public void DropMarker() {
        myself.dropper.setPosition(.8);
        myself.dropper.setPosition(0);
    }

    public void DropRobot(){
        SoundPlayer.getInstance().startPlaying(myself.hwMap.appContext, goldSoundID);
        is_busy = true;
        ArmDown();
        OpenPin();
        ArmUp();
        is_busy = false;
   }

   public boolean IsBusy() {
        return is_busy;
   }

   public boolean IsBusy2() {
        if (myself.rightDrive.isBusy() || myself.leftDrive.isBusy() || myself.leftArm.isBusy()) {
            return true;
        }
        return false;
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

    public void ArmMove(double outby){
        WinchPosition = WinchPosition + outby;
        if (WinchPosition > WinchMax) {
            WinchPosition = WinchMax;
        } else if(WinchPosition < WinchMin) {
            WinchPosition = WinchMin;
        }
        myself.winchservo.setPosition(WinchPosition);
    }

    public void ArmOut() {
        myself.winchservo.setPosition(WinchMax);
    }

    public void ArmIn() {
        myself.winchservo.setPosition(WinchMin);
    }


    public void ArmUp() {
        myself.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myself.leftArm.setTargetPosition(arm_offset + armdistance);
        myself.leftArm.setPower(1);
        myself.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(myself.leftArm.isBusy()) {

        }
    }

    public void ArmDown() {
        myself.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myself.leftArm.setTargetPosition(arm_offset);
        myself.leftArm.setPower(-1);
        myself.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(myself.leftArm.isBusy()) {

        }
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
/*
    public boolean DetectBall() {


        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
            if (sensorColor.red() > 200) {
                return false;
            }
            return true;
    }

    public int Red() {
        return (int) (sensorColor.red() * SCALE_FACTOR);
    }

    public int Green() {
        return (int) (sensorColor.green() * SCALE_FACTOR);
    }

    public int Blue() {
        return (int) (sensorColor.blue() * SCALE_FACTOR);
    }

    public double GetDistance() {
       return sensorDistance.getDistance(DistanceUnit.CM);
    }*/
/*
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
            // Determine new target position, and pass to motor controller
            newLeftTarget = myself.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = myself.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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



    public void encoderDriveByCM(double speed,
                             double distance,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (true) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = myself.leftDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
            newRightTarget = myself.rightDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
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
                    (myself.leftDrive.isBusy() && myself.rightDrive.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d",
                myself.leftDrive.getCurrentPosition();
                myself.rightDrive.getCurrentPosition();
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
    }
        /*
        public boolean opModeIsActive(){
            return true;
        }*/
}

