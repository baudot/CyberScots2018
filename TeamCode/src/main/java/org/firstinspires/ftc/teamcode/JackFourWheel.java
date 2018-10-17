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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Controls:
 *
 * Left stick to turn and move forward/back
 *
 * Right stick to move arm
 *
 * Triggers to position hook
 *
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.

 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "RagBot 3.0 4-Wheel Drive", group = "Cyber Scots")
@Disabled
public class JackFourWheel extends LinearOpMode {
    double CLAW_POS = 0;
    // Define class members

    static final int EXPONENT = 5;
    static final double     FORWARD_SPEED = 0.5;
    static final double     TURN_SPEED    = 0.25;

    static final long CYCLE_MS = 25;

    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor arm   = null;
    public Servo hook = null;
    double motorPowerL = 0;
    double motorPowerR = 0;
    double joystickForward = 0;
    double joystickTurn = 0;

    double motorPower = 0;
    double hookPos = 0;

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    private ElapsedTime runtime = new ElapsedTime();

    //@Override
    public void runOpMode() {
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back-left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back-right");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front-left");
        backLeftDrive = hardwareMap.get(DcMotor.class, "front-right");
        arm  = hardwareMap.get(DcMotor.class, "arm");
        hook  = hardwareMap.get(Servo.class, "claw");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setPower(0);
        hook.setPosition(0.5);
        // Wait for the start button0

        telemetry.addData(">", "Press Start to Start, duh" );
        telemetry.update();
        waitForStart();


        while(opModeIsActive()){

            motorPower = gamepad1.right_stick_y;

            if (Math.abs(motorPower) < 0.05) {
                motorPower = 0;
            }

            hookPos += (gamepad1.right_trigger - gamepad1.left_trigger)/4;
            hookPos = Range.clip(hookPos, MIN_POS, MAX_POS);
            hook.setPosition(hookPos);

            // Display the current value
            telemetry.addData(">", hookPos);
            telemetry.update();

            // Set the servo to the new position and pause;

            arm.setPower(motorPower);

            joystickForward = gamepad1.left_stick_y;
            joystickTurn = gamepad1.left_stick_x;

            if (Math.abs(joystickForward) < 0.01) {
                joystickForward = 0;
            }
            if (Math.abs(joystickTurn) < 0.01) {
                joystickTurn = 0;
            }
            motorPowerL = Math.pow(joystickForward, EXPONENT);
            motorPowerR =  Math.pow(joystickForward, EXPONENT);


            motorPowerL -= Math.pow(joystickTurn, EXPONENT)*TURN_SPEED;
            motorPowerR += Math.pow(joystickTurn, EXPONENT)*TURN_SPEED;

            motorPowerL = Range.clip(motorPowerL, -1, 1);
            motorPowerR = Range.clip(motorPowerR, -1, 1);


            // Display the current value
            telemetry.addData(">", "Press Stop to Stop, double duh" );
            telemetry.update();

            // Set the servo to the new position and pause;

            frontLeftDrive.setPower(motorPowerL);
            frontRightDrive.setPower(motorPowerR);
            backLeftDrive.setPower(motorPowerL);
            backRightDrive.setPower(motorPowerR);

            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
