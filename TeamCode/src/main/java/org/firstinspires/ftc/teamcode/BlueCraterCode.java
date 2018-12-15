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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BlueCraterCode", group="Ragbot")
public class BlueCraterCode extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRagbotNoArm         robot   = new HardwareRagbotNoArm();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     ARM_DRIVE_GEAR_REDUCTION    = 6.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415926);
    static final double     COUNTS_PER_DEGREE         = (COUNTS_PER_MOTOR_REV * ARM_DRIVE_GEAR_REDUCTION)
                                                            / 360;
    static final double     DRIVE_SPEED             = 0.2;
    static final double     FULL_SPEED              = 1.0;
    static final double     TURN_SPEED              = 0.2;

    int                  current_armL_pos        = 0;
    int                  current_armR_pos        = 0;

    VuforiaPos   vpos                    = null;

    public void lockArmInPlace() {
        robot.armL.setTargetPosition(current_armL_pos);
        robot.armR.setTargetPosition(current_armR_pos);

        robot.armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armL.setPower(0.5);
        robot.armR.setPower(0.5);
    }

    public synchronized void waitForStartWhileHanging() {
        while (!isStarted()) {
            lockArmInPlace();
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        vpos = new VuforiaPos(robot, telemetry);
        vpos.init();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.armL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("zero power l:", robot.armL.getZeroPowerBehavior());
        telemetry.addData("zero power r:", robot.armR.getZeroPowerBehavior());
        telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Path0",  "Starting at %7d :%7d",
        //        robot.frontLeftDrive.getCurrentPosition(),
        //        robot.frontRightDrive.getCurrentPosition(),
        //        robot.backLeftDrive.getCurrentPosition(),
        //        robot.backRightDrive.getCurrentPosition());
        //telemetry.update();
        robot.antiOverheatLockArm();

        // hold onto the team marker
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.dropOffLander();
        //drive to the sampling position
        vpos.driveToPoint(900,900);
        //do the sampling of the minerals
        robot.sampling();
        //drive to the depot
        vpos.driveToPoint (1500,-1500);
        dropOffMarker();
        //drive to the crater
        vpos.driveToPoint (1500,1500);
        //park in the crater
        parkInCrater();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:current_armL_pos
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            robot.frontRightDrive.setTargetPosition(newFrontRightTarget);
            robot.backLeftDrive.setTargetPosition(newBackLeftTarget);
            robot.backRightDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            //repeated four times, for four wheel drive
            runtime.reset();
            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.frontLeftDrive.getCurrentPosition(),
                                            robot.frontRightDrive.getCurrentPosition(),
                                            robot.backLeftDrive.getCurrentPosition(),
                                            robot.backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void armDrive(double speed,
                             double degrees,
                             double timeoutS) {
        int newarmLTarget;
        int newarmRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newarmLTarget = robot.armL.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
            newarmRTarget = robot.armR.getCurrentPosition() + (int)(-degrees * COUNTS_PER_DEGREE);

            robot.armL.setTargetPosition(newarmLTarget);
            robot.armR.setTargetPosition(newarmRTarget);

            // Turn On RUN_TO_POSITION
            robot.armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            //repeated four times, for four wheel drive
            runtime.reset();
            robot.armL.setPower(Math.abs(speed));
            robot.armR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newarmLTarget,  newarmRTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.armL.getCurrentPosition(),
                        robot.armR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.armL.setPower(0);
            robot.armR.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void dropOffMarker(){
        robot.holder.setPosition(0.75);// release team marker
        sleep(1000);
        robot.holder.setPosition(0.5); //Set arm back to regular  position
    }

    public void parkInCrater(){
        encoderDrive(TURN_SPEED,   -5, 5, 4.0);         // unhook
        encoderDrive(DRIVE_SPEED,  10,  10, 5.0);     //backup
        encoderDrive(TURN_SPEED,   10, -10, 4.0);       // turn
        encoderDrive(DRIVE_SPEED,  10,  10, 5.0);     //backup
        encoderDrive(TURN_SPEED,   -5, 5, 4.0);         //straighten
        encoderDrive(DRIVE_SPEED,-30,-30,5);            //back up to crator
    }
}

