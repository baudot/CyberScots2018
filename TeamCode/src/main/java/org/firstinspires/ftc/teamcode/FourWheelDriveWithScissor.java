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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Controls:
 *
 * Left stick to turn and move forward/back
 *
 * Right stick to move arm
 *
 * Triggers to position shoulder
 *
 * Bumpers to position elbow
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
@TeleOp(name = "RagBot 3.0 ScissorTest", group = "Cyber Scots")
//@Disabled
public class FourWheelDriveWithScissor extends LinearOpMode {
    // Define class members

    HardwareRagbotNoArm robot = new HardwareRagbotNoArm();   // Use a Ragbot's hardware

    static final int EXPONENT = 3; //Exponent for exponential drive, higher = more fine control but harder to do medium speed
    static final double FORWARD_SPEED = 0.3; //How fast the robot moves forward, obviously
    static final double TURN_SPEED = 0.5; //How fast the robot turns, obviously

    static final double LIFTING_FORWARD_SPEED = 0.1; //How fast the robot moves forward, obviously
    static final double LIFTING_TURN_SPEED = 0.1; //How fast the robot turns, obviously


    static final double MINERAL_ARM_SPEED = .3;
    static final double WINDER_SPEED = .5;
    static final int MINERAL_ENCODER_SPEED = 10;

    static final long CYCLE_MS = 25; //Cycle time, in milliseconds, of the opmode. The input updates every cycle.

    double motorPowerL = 0;
    double motorPowerR = 0; //Power, from -1 to 1, to the left and right sides of the robot
    double joystickForward = 0;
    double joystickTurn = 0; //How much (from -1 to 1) the robot needs to turn or move

    int shoulderPos = 0;
    //int elbowPos = 0;  "elbow" has no encoder"

    double mineralServoPos = 0;
    double MINERAL_SERVO_SPEED = .1;

    double motorPower = 0; //Power to the arm motor

    //boolean clawButtonWasPressed = false;
    //boolean clawOpen = true;

    boolean liftingModeButtonWasPressed = false;
    boolean liftingModeIsActive = false;

    boolean lockArmWasPressed = false;
    boolean armLocked = false;


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
        DigitalChannel shoulder_endstop;
        shoulder_endstop = hardwareMap.get(DigitalChannel.class, "shoulder_endstop");
        shoulder_endstop.setMode(DigitalChannel.Mode.INPUT);
        shoulderPos = robot.shoulder.getCurrentPosition();
        robot.shoulder.setTargetPosition(shoulderPos);

        /*telemetry.addData(">", "Press Start to use Zorb's awesome drive for the Ragbot" );
        telemetry.addData(">", "             _       _   " );
        telemetry.addData(">", " ___ ___ ___| |_ ___| |_ " );
        telemetry.addData(">", "|  _| .'| . | . | . |  _|" );
        telemetry.addData(">", "|_| |__,|_  |___|___|_|  " );
        telemetry.addData(">", "        |___|            " );
        telemetry.update();*/
        waitForStart();

        while (opModeIsActive()) {
            //if (!(shoulderPos < robot.shoulder.getTargetPosition() && !shoulder_endstop.getState())) { // The motor ISN'T going into the button
                robot.shoulder.setTargetPosition(shoulderPos);
           // }

            robot.mineralCollector.setPosition(mineralServoPos);

            mineralServoPos = (gamepad1.dpad_up ? 1 : 0) + (gamepad1.dpad_down ? -1 : 0);//+= (gamepad1.dpad_up ? 0 : MINERAL_SERVO_SPEED) - (gamepad1.dpad_down ? 0 : MINERAL_SERVO_SPEED);

            telemetry.addData("Shoulder target Pos: ", shoulderPos);
            telemetry.addData("Shoulder actual Pos: ", robot.shoulder.getCurrentPosition());

            if (gamepad1.a && !liftingModeButtonWasPressed) {
                liftingModeButtonWasPressed = true;
                liftingModeIsActive = !liftingModeIsActive;

            } else if (!gamepad1.a) {
                liftingModeButtonWasPressed = false;
            }

            telemetry.addLine(liftingModeIsActive ? "Driving Mode" : "Lifting Mode");

            if (gamepad1.y && !lockArmWasPressed) {
                lockArmWasPressed = true;
                armLocked = !armLocked;
            } else if (!gamepad1.y) {
                lockArmWasPressed = false;
            }

            if (armLocked) {
                robot.antiOverheatLockArm();
                telemetry.addLine("Arm Locked!");
            }else {
                robot.armL.setPower(motorPower); //Move the arm based on the joystick
                robot.armR.setPower(motorPower); //Move the arm based on the joystick
            }

            //robot.shoulder.setPower(gamepad1.right_trigger * MINERAL_ARM_SPEED - gamepad1.left_trigger * MINERAL_ARM_SPEED);
            robot.shoulder.setPower(1);

            if (!liftingModeIsActive) {
                shoulderPos += gamepad1.right_trigger * MINERAL_ENCODER_SPEED - gamepad1.left_trigger * MINERAL_ENCODER_SPEED;
                robot.elbow.setPower((gamepad1.left_bumper ? 0 : WINDER_SPEED) - (gamepad1.right_bumper ? 0 : WINDER_SPEED));
                //} else {
                //    if (gamepad1.right_trigger > 0.02 || gamepad1.left_trigger > 0.02 || gamepad1.right_bumper || gamepad1.left_bumper) {
                //       telemetry.addLine("Arm cannot move in lifting mode.");
                //        telemetry.addLine("Press 'a' to change mode.");
                //    }
                // }

                motorPower = Math.pow(gamepad1.right_stick_y, EXPONENT); //Arm is controlled by right stick y

                if (Math.abs(motorPower) < 0.05) { //Dead zone so the arm doesn't just move a little bit always
                    motorPower = 0;
                }

                joystickForward = gamepad1.left_stick_y;
                joystickTurn = gamepad1.left_stick_x; //Set the turn and forward from the joystick

                if (!liftingModeIsActive) {
                    joystickForward = -joystickForward;
                }

                if (Math.abs(joystickForward) < 0.01) {
                    joystickForward = 0;
                }
                if (Math.abs(joystickTurn) < 0.01) { //Dead zone
                    joystickTurn = 0;
                }

            if (!liftingModeIsActive) {
                telemetry.addLine("Driving mode");
                motorPowerL = joystickForward;//Math.pow(joystickForward, EXPONENT)*FORWARD_SPEED;
                motorPowerR =  joystickForward;//Math.pow(joystickForward, EXPONENT)*FORWARD_SPEED;
                motorPowerL += joystickTurn;//Math.pow(joystickTurn, EXPONENT)*TURN_SPEED;
                motorPowerR -= joystickTurn;//Math.pow(joystickTurn, EXPONENT)*TURN_SPEED; //Use speed variables and exponents
            }else {
                telemetry.addLine("Lifting Mode");
                motorPowerL = joystickForward;//Math.pow(joystickForward, EXPONENT)*LIFTING_FORWARD_SPEED;
                motorPowerR = joystickForward; //Math.pow(joystickForward, EXPONENT)*LIFTING_FORWARD_SPEED;
                motorPowerL += joystickTurn;//Math.pow(joystickTurn, EXPONENT)*LIFTING_TURN_SPEED;
                motorPowerR -= joystickTurn;//Math.pow(joystickTurn, EXPONENT)*LIFTING_TURN_SPEED; //Use lifting mode speed variables and exponents
            }

                double maxPower = Math.max(Math.abs(motorPowerL), Math.abs(motorPowerR));
                if (maxPower > 1) {
                    motorPowerL /= maxPower;
                    motorPowerR /= maxPower;
                }

                telemetry.addData("Left Speed: ", motorPowerL);
                telemetry.addData("Right Speed: ", motorPowerR);
                if (shoulder_endstop.getState()) {
                    telemetry.addData("Shoulder Endstop", "Is Not Pressed");
                } else {
                    telemetry.addData("Shoulder Endstop", "Is Pressed");
                }
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
}
