package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.002965833744837573;
        ThreeWheelConstants.strafeTicksToInches = .0029358643985168227;
        ThreeWheelConstants.turnTicksToInches = .0029;
        ThreeWheelConstants.leftY = 4.375;
        ThreeWheelConstants.rightY = -4.125;
        ThreeWheelConstants.strafeX = 4.5;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "bl";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "br";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "fl";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




