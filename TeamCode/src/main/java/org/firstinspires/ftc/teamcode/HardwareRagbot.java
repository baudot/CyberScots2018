/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Ragbot.
 */
public class HardwareRagbot
{

    static final double CLAW_OPEN     =  0.25;     // Maximum rotational position of the hook
    static final double CLAW_CLOSED     = 0.35;     // Minimum rotational position of the hook

    /* Public OpMode members. */
    public DigitalChannel button = null;  // Device Object
    public DcMotor frontLeftDrive   = null;
    public DcMotor frontRightDrive   = null;
    public DcMotor backLeftDrive   = null;
    public DcMotor backRightDrive   = null;  //All 4 drive motors
    public DcMotor armL   = null; //The motor that lifts the arm
    public DcMotor armR  = null; //The motor that lifts the arm
    public Servo hook = null; //The\ servo that hooks on to the lander
    public DistanceSensor sensorDistance = null;
    public ColorSensor sensorColor = null;
    public DcMotor shoulder; //The "shoulder" of the mineral arm
    public DcMotor elbow; //The "elbow" of the mineral arm
    public Servo claw = null; //The claw on the mineral arm
    public Servo holder = null; //The team marker holder
    public Servo pusher = null; //The team marker pusher
    //public Servo hook = null; //The servo that hooks on to the lander

    //public static final double MID_SERVO       =  0.5 ;


    /* local OpMode members. */
    HardwareMap hardwareMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRagbot(){

    }

    public void lockArmInPlace() {
        int arml_pos = armL.getCurrentPosition();
        int armr_pos = armR.getCurrentPosition();

        armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armL.setTargetPosition(arml_pos);
        armR.setTargetPosition(armr_pos);

        armL.setPower(0.5);
        armR.setPower(0.5);
    }

    public void unlockArm() {
        armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armL.setPower(0);
        armR.setPower(0);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        // Define and Initialize Motors
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back-left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back-right");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front-left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front-right");
        armL  = hardwareMap.get(DcMotor.class, "arml");
        armR  = hardwareMap.get(DcMotor.class, "armr");
        button  = hardwareMap.get(DigitalChannel.class, "button");     //  Use generic form of device mapping
        hook  = hardwareMap.get(Servo.class, "hook");// Set the motors from their configurations
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color");
        shoulder  = hardwareMap.get(DcMotor.class, "shoulder");
        elbow  = hardwareMap.get(DcMotor.class, "elbow");

        //button  = hardwareMap.get(DigitalChannel.class, "button");     //  Use generic form of device mapping
        //hook  = hardwareMap.get(Servo.class, "hook");// Set the motors from their configurations
        holder  = hardwareMap.get(Servo.class, "holder");// Set the motors from their configurations

        claw  = hardwareMap.get(Servo.class, "claw");// Set the motors from their configurations



        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors //Directions of motors to prevent IT SUCKS JACK HERE
        armL.setDirection(DcMotor.Direction.FORWARD);
        armR.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.REVERSE);

        //button.setMode(DigitalChannel.Mode.INPUT);

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        armL.setPower(0);
        armR.setPower(0);
        shoulder.setPower(0);
        elbow.setPower(0);

        // Set all motors to run with encoders.
        // May want to use RUN_WITHOUT_ENCODER if encoders are installed.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Initialize ALL installed servos.
        //hook.setPosition(0.5); //Set hook to center position
        claw.setPosition(0.8); //Set claw to center position
        holder.setPosition(0.8);
    }
}

