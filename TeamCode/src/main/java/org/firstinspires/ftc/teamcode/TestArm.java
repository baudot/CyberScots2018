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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode tests the robot arm with only one motor and one servo.
 */
@TeleOp(name = "Test the arm", group = "Cyber Scots")
//@Disabled
public class TestArm extends LinearOpMode {
    static final double INCREMENT   = 0.01;     // amount to slow servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   25;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double hookPos = 0;

    public DcMotor armL   = null;
    public DcMotor armR   = null;
    public Servo hookServo = null;
    double motorPower = 0;

    //@Override
    public void runOpMode() {

        armL  = hardwareMap.get(DcMotor.class, "arml"); //motor to lift the arm
        armR  = hardwareMap.get(DcMotor.class, "armr"); //motor to lift the arm
        hookServo  = hardwareMap.get(Servo.class, "hook");
        armL.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        armR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        hookServo.setPosition(0.5);
        // Wait for the start button
        telemetry.addData(">", "Press Start to begin test." );
        telemetry.update();
        waitForStart();


        while(opModeIsActive()){

            motorPower = gamepad1.right_stick_y;

            if (Math.abs(motorPower) < 0.05) {
                motorPower = 0;
            }

            hookPos += (gamepad1.right_trigger - gamepad1.left_trigger)/4;
            hookPos = Range.clip(hookPos, MIN_POS, MAX_POS);
            hookServo.setPosition(hookPos);

            // Display the current value
            telemetry.addData(">", hookPos );
            telemetry.update();

            // Set the servo to the new position and pause;

            armL.setPower(motorPower);
            armR.setPower(motorPower);
            sleep(CYCLE_MS);
            idle();
        }
        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
