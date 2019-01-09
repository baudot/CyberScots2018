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
    static final double left_whip_up_pos = 0.2;
    static final double left_whip_side_pos = 0.4;
    static final double center_whip_up_pos = 0.4;
    static final double center_whip_side_pos = 0.6;
    static final double right_whip_up_pos = 0.6;
    static final double right_whip_side_pos = 0.8;

    public void sampling() {
        //Find the cube
        telemetry.addData("Red", robot.sensorColor.red());
        telemetry.addData("Green", robot.sensorColor.green());
        telemetry.addData("Blue ", robot.sensorColor.blue());
        robot.sensorColor.enableLed(true);
        if ((robot.sensorColor.red() + robot.sensorColor.green() + robot.sensorColor.blue() >75)){
            telemetry.addData("Cube", "Found Cube");
        }
        telemetry.update();
    }

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        robot.whipUp.setPosition(left_whip_up_pos);
        robot.whipSide.setPosition(left_whip_side_pos);
        sleep(7500);
        robot.whipUp.setPosition(center_whip_up_pos);
        robot.whipSide.setPosition(center_whip_side_pos);
        sleep(7500);
        robot.whipUp.setPosition(right_whip_up_pos);
        robot.whipSide.setPosition(right_whip_side_pos);
        sleep(7500);

        while(opModeIsActive()) {
            sampling();
        }
    }
}
