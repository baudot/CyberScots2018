package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Sampling Test", group = "Cyber Scots")
//@Disabled
public class samplingTest extends LinearOpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    HardwareRagbotNoArm robot = new HardwareRagbotNoArm();
    public void sampling() {
        robot.init(hardwareMap);

        // Find the cube
        telemetry.addData("Red", robot.sensorColor.red());
        telemetry.addData("Green", robot.sensorColor.green());
        telemetry.addData("Blue ", robot.sensorColor.blue());
        robot.sensorColor.enableLed(true);
        if ((robot.sensorColor.red() + robot.sensorColor.green()) / 2 > robot.sensorColor.blue() + 15) {
            telemetry.addData("Cube", "Found Cube");
        }
        telemetry.update();
    }

    public void runOpMode() {
        waitForStart();
        while(opModeIsActive()) {
            sampling();
        }

    }
}
