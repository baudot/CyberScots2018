package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Sampling", group = "Cyber Scots")
//@Disabled
public class sampling extends LinearOpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    public Servo whipUp = null;
    public Servo whipSide = null;

    static final double WAIT_TIME = 300;

    static final double WHIP_UP_POSITION = 0;
    static final double WHIP_DOWN_POSITION = 0.5;
    static final double FAR_LEFT_POSITION= 0;
    //static final double FAR_LEFT_WHACK_POSITION = 0.2;
    static final double LEFT_POSITION = 0.2;
   // private static final double LEFT_WHACK_POSITION = 0.4;
    static final double RIGHT_POSITION = 0.4;

    static final double WHACK_DISTANCE = 0.1;
   // static final double RIGHT_WHACK_POSITION = 0.6;



    public boolean cubeFound(ColorSensor colorSensor) {
        return (colorSensor.red() + colorSensor.green()) / 2 > colorSensor.blue();
    }

    public void moveWhipTo(double pos) {
        runtime.reset();
        while (runtime.milliseconds() < WAIT_TIME) {
            whipUp.setPosition(WHIP_UP_POSITION);
        }
        runtime.reset();
        while (runtime.milliseconds() < WAIT_TIME) {
            whipSide.setPosition(pos);
        }
        runtime.reset();
        while (runtime.milliseconds() < WAIT_TIME) {
            whipUp.setPosition(WHIP_DOWN_POSITION);
        }
    }

    public void whack(double pos) {
        runtime.reset();
        while (runtime.milliseconds() < WAIT_TIME) {
            whipSide.setPosition(pos + WHACK_DISTANCE);
        }
        runtime.reset();
        while (runtime.milliseconds() < WAIT_TIME) {
            whipUp.setPosition(WHIP_UP_POSITION);
        }
    }

    public void sampling() {
        // Find the cube
        ColorSensor colorSensor;
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        colorSensor.enableLed(true);

        // Unfold the jewel whip
        runtime.reset();
        while (runtime.milliseconds() < WAIT_TIME) {
            whipSide.setPosition(RIGHT_POSITION);
        }
        runtime.reset();
        while (runtime.milliseconds() < WAIT_TIME) {
            whipUp.setPosition(WHIP_UP_POSITION);
        }
        runtime.reset();
        while (runtime.milliseconds() < WAIT_TIME) {
            whipSide.setPosition(FAR_LEFT_POSITION);
        }
        runtime.reset();
        while (runtime.milliseconds() < WAIT_TIME) {
            whipUp.setPosition(WHIP_DOWN_POSITION);
        }

        if (cubeFound(colorSensor)) {
            whack(FAR_LEFT_POSITION);
        } else {
           moveWhipTo(LEFT_POSITION);
            if (cubeFound(colorSensor)) {
                whack(LEFT_POSITION);
            } else {
                moveWhipTo(RIGHT_POSITION);
                if (cubeFound(colorSensor)) {
                    whack(RIGHT_POSITION);
                }
            }
        }
    }

    public void runOpMode() {
        sampling();
    }
}
