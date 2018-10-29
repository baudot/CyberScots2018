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
//@Disabled
public class FourWheelDrive extends LinearOpMode {
    // Define class members

    HardwareRagbot         robot   = new HardwareRagbot();   // Use a Ragbot's hardware

    static final int EXPONENT = 5; //Exponent for exponential drive, higher = more fine control but harder to do medium speed
    static final double     FORWARD_SPEED = 0.3; //How fast the robot moves forward, obviously
    static final double     TURN_SPEED    = 0.3; //How fast the robot turns, obviously

    static final double MAX_POS     =  1.0;     // Maximum rotational position of the hook
    static final double MIN_POS     =  0.0;     // Minimum rotational position of the hook

    static final long CYCLE_MS = 25; //Cycle time, in milliseconds, of the opmode. The input updates every cycle.

    double  motorPowerL = 0;
    double motorPowerR = 0; //Power, from -1 to 1, to the left and right sides of the robot
    double joystickForward = 0;
    double joystickTurn = 0; //How much (from -1 to 1) the robot needs to turn or move

    double motorPower = 0; //Power to the arm motor
    //double hookPos = 0; //Position of the hook servo



    private ElapsedTime runtime = new ElapsedTime();

    //@Override
    public void runOpMode() {

        /*

          _____                    _____                    _____                    _____                   _______               _____
         /\    \                  /\    \                  /\    \                  /\    \                 /::\    \             /\    \
        /::\    \                /::\    \                /::\    \                /::\    \               /::::\    \           /::\    \
       /::::\    \              /::::\    \              /::::\    \              /::::\    \             /::::::\    \          \:::\    \
      /::::::\    \            /::::::\    \            /::::::\    \            /::::::\    \           /::::::::\    \          \:::\    \
     /:::/\:::\    \          /:::/\:::\    \          /:::/\:::\    \          /:::/\:::\    \         /:::/~~\:::\    \          \:::\    \
    /:::/__\:::\    \        /:::/__\:::\    \        /:::/  \:::\    \        /:::/__\:::\    \       /:::/    \:::\    \          \:::\    \
   /::::\   \:::\    \      /::::\   \:::\    \      /:::/    \:::\    \      /::::\   \:::\    \     /:::/    / \:::\    \         /::::\    \
  /::::::\   \:::\    \    /::::::\   \:::\    \    /:::/    / \:::\    \    /::::::\   \:::\    \   /:::/____/   \:::\____\       /::::::\    \
 /:::/\:::\   \:::\____\  /:::/\:::\   \:::\    \  /:::/    /   \:::\ ___\  /:::/\:::\   \:::\ ___\ |:::|    |     |:::|    |     /:::/\:::\    \
/:::/  \:::\   \:::|    |/:::/  \:::\   \:::\____\/:::/____/  ___\:::|    |/:::/__\:::\   \:::|    ||:::|____|     |:::|    |    /:::/  \:::\____\
\::/   |::::\  /:::|____|\::/    \:::\  /:::/    /\:::\    \ /\  /:::|____|\:::\   \:::\  /:::|____| \:::\    \   /:::/    /    /:::/    \::/    /
 \/____|:::::\/:::/    /  \/____/ \:::\/:::/    /  \:::\    /::\ \::/    /  \:::\   \:::\/:::/    /   \:::\    \ /:::/    /    /:::/    / \/____/
       |:::::::::/    /            \::::::/    /    \:::\   \:::\ \/____/    \:::\   \::::::/    /     \:::\    /:::/    /    /:::/    /
       |::|\::::/    /              \::::/    /      \:::\   \:::\____\       \:::\   \::::/    /       \:::\__/:::/    /    /:::/    /
       |::| \::/____/               /:::/    /        \:::\  /:::/    /        \:::\  /:::/    /         \::::::::/    /     \::/    /
       |::|  ~|                    /:::/    /          \:::\/:::/    /          \:::\/:::/    /           \::::::/    /       \/____/
       |::|   |                   /:::/    /            \::::::/    /            \::::::/    /             \::::/    /
       \::|   |                  /:::/    /              \::::/    /              \::::/    /               \::/____/
        \:|   |                  \::/    /                \::/____/                \::/____/                 ~~
         \|___|                   \/____/                                           ~~


        */

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        telemetry.addData(">", "Press Start to use Zorb's awesome drive for the Ragbot" );
        telemetry.addData(">", " ______    _______  _______  _______  _______  _______    _______        _______ " );
        telemetry.addData(">", "|    _ |  |   _   ||       ||  _    ||       ||       |  |       |      |  _    |" );
        telemetry.addData(">", "|   | ||  |  |_|  ||    ___|| |_|   ||   _   ||_     _|  |___    |      | | |   |" );
        telemetry.addData(">", "|   |_||_ |       ||   | __ |       ||  | |  |  |   |     ___|   |      | | |   |" );
        telemetry.addData(">", "|    __  ||       ||   ||  ||  _   | |  |_|  |  |   |    |___    | ___  | |_|   |" );
        telemetry.addData(">", "|   |  | ||   _   ||   |_| || |_|   ||       |  |   |     ___|   ||   | |       |" );
        telemetry.addData(">", "|___|  |_||__| |__||_______||_______||_______|  |___|    |_______||___| |_______|" );
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("arm left", robot.armL.getCurrentPosition());

            telemetry.addData("arm right", robot.armR.getCurrentPosition());

            motorPower = gamepad1.right_stick_y; //Arm is controlled by right stick y

            if (Math.abs(motorPower) < 0.05) { //Dead zone so the arm doesn't just move a little bit always
                motorPower = 0;
            }

            //hookPos += (gamepad1.right_trigger - gamepad1.left_trigger)/4; //Move the hook based on the triggers
            //hookPos = Range.clip(hookPos, MIN_POS, MAX_POS); //Make sure the hook isn't moving too far
            //robot.hook.setPosition(hookPos); //Actually move the hook

            // Display the current value
            //telemetry.addData(">", hookPos); //Display the hook position
            //telemetry.update();

            // Set the servo to the new position and pause;

            //if (robot.button.getState() && motorPower < 0) {
                //motorPower = 0;
            //}

            robot.armL.setPower(motorPower); //Move the arm based on the joystick
            robot.armR.setPower(motorPower); //Move the arm based on the joystick

            joystickForward = gamepad1.left_stick_y;
            joystickTurn = gamepad1.left_stick_x; //Set the turn and forward from the joystick

            if (Math.abs(joystickForward) < 0.01) {
                joystickForward = 0;
            }
            if (Math.abs(joystickTurn) < 0.01) { //Dead zone
                joystickTurn = 0;
            }
            motorPowerL = Math.pow(joystickForward, EXPONENT)*FORWARD_SPEED;
            motorPowerR =  Math.pow(joystickForward, EXPONENT)*FORWARD_SPEED;


            motorPowerL += Math.pow(joystickTurn, EXPONENT)*TURN_SPEED;
            motorPowerR -= Math.pow(joystickTurn, EXPONENT)*TURN_SPEED; //Use speed variables and exponents

            motorPowerL = Range.clip(motorPowerL, -1, 1);
            motorPowerR = Range.clip(motorPowerR, -1, 1); //Make sure the motors aren't going faster than they can


            // Display the current value
            telemetry.addData(">", "Press Stop to end Zorb's epic drive." );
            telemetry.update();

            robot.frontLeftDrive.setPower(motorPowerL);
            robot.frontRightDrive.setPower(motorPowerR);
            robot.backLeftDrive.setPower(motorPowerL);
            robot.backRightDrive.setPower(motorPowerR); //Set the drive motor power

            sleep(CYCLE_MS); //Delay until the next cycle
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
