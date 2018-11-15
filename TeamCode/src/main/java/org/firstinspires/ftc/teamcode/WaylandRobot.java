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

    // START OF GLOBAL DECLARATIONS
    int leftmotor_offset = 0;
    int rightmotor_offset = 0;

    double servo_offset = 0;
    double motor_power = 1;
    double drop_power = 1;
    double cm_per_ms = 0.039;
    int goldSoundID;

    double WinchMin = .25;
    double WinchMax = .60;
    double WinchPosition = 0;
    double dropPinOut = .50;
    double dropPinIn = -.44;


    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    int relativeLayoutId;
    View relativeLayout;

    boolean is_busy = false;

    int arm_offset = 15;



    double SIZE_OF_WHEEL_CM =9;
    double COUNTS_PER_CM = 288/(SIZE_OF_WHEEL_CM * 3.14) ;
    // take the initial value of the motor and add this.
    int armdistance = -4153;
    WaylandHardware myself = new WaylandHardware();
    private boolean goldFound;


    // END OF DECLARATIONS

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

    public void initRobot(boolean enableEncoders) {
        if (enableEncoders) {
            EnableEncoders();
            initLeftmotor_offset();
            initRightMotor_offset();
        }
        initArm_offset();
        initServo_offset();
        initColorDistance();
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

    public int cm_to_ms (int cms) {

        int msec = (int)(cms/cm_per_ms);

        return msec;
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

    public void DriveByTime2(int msval){
        ElapsedTime timer =  new ElapsedTime();
        timer.reset();
        myself.rightDrive.setPower(.9);
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
        DisableEncoders();
        ElapsedTime timer =  new ElapsedTime();
        timer.reset();
        myself.rightDrive.setPower(-1);
        myself.leftDrive.setPower(1);
        while (timer.milliseconds() < msval) {

        }
        myself.rightDrive.setPower(0);
        myself.leftDrive.setPower(0);
    }

    public void DriveByLeftTime2(int msval){
        ElapsedTime timer =  new ElapsedTime();
        timer.reset();
        myself.rightDrive.setPower(-.9);
        myself.leftDrive.setPower(1);
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
        EnableEncoders();
        int leftoffset = myself.leftDrive.getCurrentPosition() + encval;
        int rightoffset = myself.rightDrive.getCurrentPosition() + encval;
        myself.leftDrive.setTargetPosition(leftoffset);
        myself.rightDrive.setTargetPosition(rightoffset);
        myself.rightDrive.setPower(motor_power);
        myself.leftDrive.setPower(motor_power);
        myself.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myself.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
        myself.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myself.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
        myself.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myself.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftmotor_offset = leftoffset;
        rightmotor_offset = rightoffset;
    }

    public void DriveForwardWithEncoder2(int encval){
        is_busy = true;
        int leftoffset = myself.leftDrive.getCurrentPosition() + encval;
        int rightoffset = myself.rightDrive.getCurrentPosition() + encval;
        DriveWithEncoder(leftoffset, rightoffset, motor_power);
        leftmotor_offset = leftoffset;
        rightmotor_offset = rightoffset;
        is_busy = false;
    }

    public void DriveLeftWithEncoder2(int encval){
        is_busy = true;
        int leftoffset = myself.leftDrive.getCurrentPosition() - encval;
        int rightoffset = myself.rightDrive.getCurrentPosition() + encval;

        DriveWithEncoder(leftoffset, rightoffset, motor_power);

        leftmotor_offset = leftoffset;
        rightmotor_offset = rightoffset;
        is_busy = false;

    }



    public void DriveRightWithEncoder2(int encval){
        is_busy = true;
        int leftoffset = myself.leftDrive.getCurrentPosition() + encval;
        int rightoffset = myself.rightDrive.getCurrentPosition() - encval;
        DriveWithEncoder(leftoffset, rightoffset, motor_power);
        leftmotor_offset = leftoffset;
        rightmotor_offset = rightoffset;
        is_busy = false;
    }

    private void DriveWithEncoder(int left, int right, double power) {
        is_busy = true;
        myself.leftDrive.setTargetPosition(left);
        myself.rightDrive.setTargetPosition(right);
        myself.rightDrive.setPower(power);
        myself.leftDrive.setPower(power);
        if(right != myself.rightDrive.getCurrentPosition()) {
            myself.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(left != myself.leftDrive.getCurrentPosition()) {
            myself.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        is_busy = false;
    }

    public void DriveByDistance(int cms, String direction, double power, boolean active) {
        int encval = (int)(COUNTS_PER_CM * cms);
        int leftencval = myself.leftDrive.getCurrentPosition();
        int rightencval = myself.rightDrive.getCurrentPosition();

        switch (direction) {
            case "forward":
                leftencval += encval;
                rightencval += encval;
            case "reverse":
                leftencval -= encval;
                rightencval -= encval;
            case "left":
                leftencval -= encval;
                rightencval += encval;
            case "right":
                leftencval += encval;
                //rightencval -= encval;
            default:
                leftencval += encval;
                rightencval += encval;

        }

        if (active) {
            DriveWithEncoder(leftencval, rightencval, power);
        }
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

    public void ArmSetVal(int distance) {
        myself.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myself.leftArm.setTargetPosition(arm_offset + distance);
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

    public void wandOut() {
        myself.winchservo.setPosition(WinchMax);
    }

    public void wandIn(){
        myself.winchservo.setPosition(WinchMin);
    }
    public void stop() {
        myself.leftArm.setPower(0);
        myself.rightDrive.setPower(0);
        myself.leftDrive.setPower(0);
    }


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
    }

}

