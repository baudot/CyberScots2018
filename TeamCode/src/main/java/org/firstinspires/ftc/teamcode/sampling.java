package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Sampling Test", group = "Cyber Scots")
@Disabled
public class sampling extends LinearOpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    public Servo whipUp = null;
    public Servo whipSide = null;
    static final double WHIP_UP_POSITION = 0;
    static final double WHIP_DOWN_POSITION = 0.5;
    double whipWhackPosition = 0.2;
    double noWhackPosition = 0;

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
            whipSide.setPosition(whipWhackPosition);
            whipSide.setPosition(noWhackPosition);
        } else {
            whipUp.setPosition(WHIP_UP_POSITION);
            whipWhackPosition = whipWhackPosition + 0.2;
            noWhackPosition = noWhackPosition + 0.2;
            if ((colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue()) {
                // Knock off the cube once it has been found
                whipUp.setPosition(WHIP_DOWN_POSITION);
                whipSide.setPosition(whipWhackPosition);
                whipSide.setPosition(noWhackPosition);
                whipUp.setPosition(WHIP_UP_POSITION);
            } else {
                whipUp.setPosition(WHIP_UP_POSITION);
                whipWhackPosition = whipWhackPosition + 0.2;
                noWhackPosition = noWhackPosition + 0.2;
                if ((colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue()) {
                    // Knock off the cube once it has been found
                    whipUp.setPosition(WHIP_DOWN_POSITION);
                    whipSide.setPosition(whipWhackPosition);
                    whipSide.setPosition(noWhackPosition);
                    whipUp.setPosition(WHIP_UP_POSITION);
                }
            }
        }
    }

    public void runOpMode() {
        sampling();
    }
}
