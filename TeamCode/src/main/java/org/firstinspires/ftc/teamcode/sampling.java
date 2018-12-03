package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

//Jack when someone changes this code: https://www.youtube.com/watch?v=cE1FrqheQNI

@Autonomous(name = "Sampling", group = "Cyber Scots")
@Disabled
public class sampling extends LinearOpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    public Servo whipUp = null;
    public Servo whipSide = null;
    static final double WHIP_UP_POSITION = 0;
    static final double WHIP_DOWN_POSITION = 0.5;
    double LEFT_NO_WHACK= 0;
    static final double LEFT_WHACK_POSITION = 0.2;
    double MID_NO_WHACK = 0;
    static final double MID_WHACK_POSITION = 0.2;
    double RIGHT_NO_WHACK = 0;
    static final double RIGHT_WHACK_POSITION = 0.2;

    public void sampling() {
        // Find the cube
        ColorSensor colorSensor;
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        colorSensor.enableLed(true);

        // Unfold the jewel whip
        whipUp.setPosition(WHIP_DOWN_POSITION);

        if ((colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue()) {
            // Knock off the cube once it has been found
            whipSide.setPosition(LEFT_WHACK_POSITION);
            whipSide.setPosition(LEFT_NO_WHACK);
            telemetry.addData("Cube", "Found Cube");
        } else {
            whipUp.setPosition(WHIP_UP_POSITION);
            if ((colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue()) {
                // Knock off the cube once it has been found
                whipUp.setPosition(WHIP_DOWN_POSITION);
                whipSide.setPosition(MID_WHACK_POSITION);
                whipSide.setPosition(MID_NO_WHACK);
                whipUp.setPosition(WHIP_UP_POSITION);
            } else {
                whipUp.setPosition(WHIP_UP_POSITION);
                if ((colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue()) {
                    // Knock off the cube once it has been found
                    whipUp.setPosition(WHIP_DOWN_POSITION);
                    whipSide.setPosition(RIGHT_WHACK_POSITION);
                    whipSide.setPosition(RIGHT_NO_WHACK);
                    whipUp.setPosition(WHIP_UP_POSITION);
                }
            }
        }
    }

    public void runOpMode() {
        sampling();
    }
}
