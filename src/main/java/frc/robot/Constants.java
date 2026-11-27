package frc.robot;

/**
 * 全機器人共用的常數：CAN ID、搖桿埠號、硬體參數等。
 * 依子系統分成內部類別，用法例如 {@code Constants.LEDConstants.kCandleIds}。
 */
public final class Constants {
    private Constants() {}

    /** 搖桿 / 手把 */
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    /** LED（CANdle） */
    public static final class LEDConstants {
        /** CANdle 的 CAN ID（要跟 Tuner X 裡的 Device ID 一致）。 */
        public static final int[] kCandleIds = {1, 2, 3, 4, 5, 6};

        /** CANdle 本體有 8 顆 LED (index 0~7)，外接燈條從 index 8 開始。 */
        public static final int kOnboardLeds = 8;
        /** 每顆 CANdle 外接燈條的 LED 數量；沒接燈條就設 0。 */
        public static final int kStripLength = 307;

        public static final int kStartIndex = 0;
        public static final int kEndIndex = kOnboardLeds + kStripLength - 1;

        /** 亮度 0.0 ~ 1.0，燈條很亮或很耗電時調低。 */
        public static final double kBrightness = 0.5;
    }
}
