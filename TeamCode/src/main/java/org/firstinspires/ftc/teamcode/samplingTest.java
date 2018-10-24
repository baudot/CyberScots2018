package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Jack when someone changes this code: https://www.youtube.com/watch?v=cE1FrqheQNI

@Autonomous(name = "Sampling Test", group = "Cyber Scots")
@Disabled
public class samplingTest extends LinearOpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();


    public void sampling() {
        // Find the cube
        ColorSensor colorSensor;
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        colorSensor.enableLed(true);


        if ((colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue()) {
            // Knock off the cube once it has been found
            telemetry.addData("Cube", "Found Cube");
        }
    }

    public void runOpMode() {
        sampling();
    }
}
