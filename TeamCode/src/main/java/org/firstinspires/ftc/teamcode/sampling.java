package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

//Jack when someone changes this code: https://www.youtube.com/watch?v=cE1FrqheQNI

@Autonomous(name = "Sampling Test", group = "Cyber Scots")
@Disabled
public class sampling extends LinearOpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    public Servo whipUp = null;
    public Servo whipSide = null;
    static final double WHIP_UP_POSITION = 0;
    static final double WHIP_DOWN_POSITION = 0.5;
    static final double WHIP_WHACK_POSITION = 0.4;
    static final double NO_WHACK_POSITION = 0;

    public void sampling() {// Unfold the jewel whip
        whipUp.setPosition(WHIP_DOWN_POSITION);

        // Find the cube
        ColorSensor colorSensor;
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        colorSensor.enableLed(true);
        if ((colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue()) {
            // Knock off the cube once it has been found
            whipSide.setPosition(WHIP_WHACK_POSITION);
            whipSide.setPosition(NO_WHACK_POSITION);
        } else {
            whipUp.setPosition(WHIP_UP_POSITION);
            whipSide.setPosition(0.2);
            if ((colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue()) {
                // Knock off the cube once it has been found
                whipSide.setPosition(WHIP_WHACK_POSITION + 0.2);
                whipSide.setPosition(NO_WHACK_POSITION + 0.2);
            } else {
                whipUp.setPosition(WHIP_UP_POSITION);
                whipSide.setPosition(0.4);
                if ((colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue()) {
                    // Knock off the cube once it has been found
                    whipSide.setPosition(WHIP_WHACK_POSITION + 0.4);
                    whipSide.setPosition(NO_WHACK_POSITION + 0.4);
                } else {
                    whipUp.setPosition(WHIP_UP_POSITION);
                    whipSide.setPosition(0.6);
                    if ((colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue()) {
                        // Knock off the cube once it has been found
                        whipSide.setPosition(WHIP_WHACK_POSITION + 0.6);
                        whipSide.setPosition(NO_WHACK_POSITION + 0.6);
                    } else {
                        telemetry.addData("Error", "Whoops. Can't find the cube dudes. ERROR.");
                    }
                }
            }
        }
        whipUp.setPosition(WHIP_UP_POSITION);
    }
    // Refold the jewel whip (Once the cube has been knocked off)
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        sampling();
    }
}
