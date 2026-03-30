package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class VisionConstants {

    public static final String limelight = "limelight-front";

    public static final double minTA = 0.10;
    public static final double maxTA = 0.50;

    public static final double dontTrust = 9999999;

    public static final InterpolatingDoubleTreeMap oneTagXY = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap oneTagRot = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap multiTagXY = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap multiTagRot = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap megaTagTwo = new InterpolatingDoubleTreeMap();

    public static void SetUpTrustConstants() {
        multiTagXY.put(0.20, 100.0);
        multiTagXY.put(0.25, 4.0);
        multiTagXY.put(0.30, 2.0);
        multiTagXY.put(0.50, 0.25);

        multiTagRot.put(0.20, 400.0);
        multiTagRot.put(0.25, 10.0);
        multiTagRot.put(0.30, 5.0);
        multiTagRot.put(0.60, 0.5);

        megaTagTwo.put(0.05, 100.0);
        megaTagTwo.put(0.10, 10.0);
        megaTagTwo.put(0.20, 5.0);
        megaTagTwo.put(0.25, 2.0);
        megaTagTwo.put(0.30, 1.0);
        megaTagTwo.put(0.60, 0.25);
    }
}