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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Ragbot.
 */
public class HardwareRagbotNoArm
{

    static final double CLAW_OPEN     =  0.25;     // Maximum rotational position of the hook
    static final double CLAW_CLOSED     = 0.35;     // Minimum rotational position of the hook

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265);

    static final double WHEEL_CIRCLE_RADIUS = 17; //The radius of the circle going around all of the wheels with the center of the robot in the middle
    static final double FULL_CIRCLE_INCHES = WHEEL_CIRCLE_RADIUS * 2 * 3.14159265;

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    /* Public OpMode members. */
    public DigitalChannel button = null;  // Device Object
    public DcMotor frontLeftDrive   = null;
    public DcMotor frontRightDrive   = null;
    public DcMotor backLeftDrive   = null;
    public DcMotor backRightDrive   = null;  //All 4 drive motors
    public DcMotor armL   = null; //The motor that lifts the arm
    public DcMotor armR  = null; //The motor that lifts the arm
    //public DcMotor shoulder; //The "shoulder" of the mineral arm
    //public DcMotor elbow; //The "elbow" of the mineral arm
    //public Servo claw = null; //The claw on the mineral arm
    public Servo holder = null; //The team marker holder
    public Servo pusher = null; //The team marker pusher
    public ColorSensor sensorColor = null;
    public Servo whipUp = null;
    public Servo whipSide = null;
    //public Servo hook = null; //The servo that hooks on to the lander

    //public static final double MID_SERVO       =  0.5 ;


    /* local OpMode members. */
    HardwareMap hardwareMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime encoderTimer = new ElapsedTime();

    /* Constructor */
    public HardwareRagbotNoArm(){

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;

        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and jack to motor controller
        newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newBackRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        frontRightDrive.setTargetPosition(newFrontRightTarget);
        backLeftDrive.setTargetPosition(newBackLeftTarget);
        backRightDrive.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        encoderTimer.reset();
        frontLeftDrive.setPower(Math.abs(speed));
        frontRightDrive.setPower(Math.abs(speed));
        backLeftDrive.setPower(Math.abs(speed));
        backRightDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((encoderTimer.seconds() < timeoutS) &&
                (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backLeftDrive.isBusy() && backRightDrive.isBusy())) {

           /* // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
            telemetry.update();*/
        }

        // Stop all motion;
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
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

    public void antiOverheatlockArm() {
        int arml_pos = armL.getCurrentPosition();
        int armr_pos = armR.getCurrentPosition();

        if (armL.getCurrentPosition() < arml_pos || armR.getCurrentPosition() > armr_pos) {
            armL.setPower(-0.5);
            armR.setPower(-0.5);
        }else {
            armL.setPower(0);
            armR.setPower(0);
        }
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
        //sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        //shoulder  = hardwareMap.get(DcMotor.class, "shoulder");
       // elbow  = hardwareMap.get(DcMotor.class, "elbow");

        //button  = hardwareMap.get(DigitalChannel.class, "button");     //  Use generic form of device mapping
        //hook  = hardwareMap.get(Servo.class, "hook");// Set the motors from their configurations
        holder  = hardwareMap.get(Servo.class, "markerholder");// Set the motors from their configurations
        whipUp  = hardwareMap.get(Servo.class, "whipUp");// Set the motors from their configurations
        whipSide  = hardwareMap.get(Servo.class, "whipSide");// Set the motors from their configurations

        //claw  = hardwareMap.get(Servo.class, "claw");// Set the motors from their configurations



        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors //Directions of motors to prevent IT SUCKS JACK HERE
        armL.setDirection(DcMotor.Direction.FORWARD);
        armR.setDirection(DcMotor.Direction.REVERSE);
        //shoulder.setDirection(DcMotor.Direction.FORWARD);
       // elbow.setDirection(DcMotor.Direction.REVERSE);

        //button.setMode(DigitalChannel.Mode.INPUT);

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        armL.setPower(0);
        armR.setPower(0);
        //shoulder.setPower(0);
        //elbow.setPower(0);

        // Set all motors to run with encoders.
        // May want to use RUN_WITHOUT_ENCODER if encoders are installed.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Initialize ALL installed servos.
        //hook.setPosition(0.5); //Set hook to center position
        holder.setPosition(0.5);
        whipUp.setPosition(0);
        whipSide.setPosition(0);
        //claw.setPosition(CLAW_OPEN); //Set claw to center position
    }
}

