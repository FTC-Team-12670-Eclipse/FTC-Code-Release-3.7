package org.firstinspires.ftc.teamcode.Archive;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.widget.LinearLayout;

import org.firstinspires.ftc.teamcode.R;

public class JewelColorDetector extends Activity {
    enum AllianceColor {
        Red, Blue, NotSure
    }

    public class Reading {
        int confidence;
        public AllianceColor left;
        AllianceColor right;

        Reading(AllianceColor l, int c) {
            left = l;
            if (l == AllianceColor.Red) {
                right = AllianceColor.Blue;
            } else if (l == AllianceColor.Blue) {
                right = AllianceColor.Red;
            } else {
                right = AllianceColor.NotSure;
            }
            confidence = c;
        }
    }

    private int getPixel(Bitmap b, int x, int y) {
        return b.getPixel(x, y);
    }

    private Reading analyze(Bitmap b, int x1, int x2, int y, int radius) {
        int redLeft = 0;
        int blueLeft = 0;
        int redRight = 0;
        int blueRight = 0;
        int upLeft, downLeft, leftLeft, rightLeft;
        int upRight, downRight, leftRight, rightRight;

        for (int i = 0; i < radius; i++) {
            upLeft = getPixel(b, x1, y + i);
            downLeft = getPixel(b, x1, y - i);
            leftLeft = getPixel(b, x1 - i, y);
            rightLeft = getPixel(b, x1 + i, y);

            redLeft += Color.red(upLeft);
            redLeft += Color.red(downLeft);
            redLeft += Color.red(leftLeft);
            redLeft += Color.red(rightLeft);

            blueLeft += Color.blue(upLeft);
            blueLeft += Color.blue(downLeft);
            blueLeft += Color.blue(leftLeft);
            blueLeft += Color.blue(rightLeft);

            upRight = getPixel(b, x2, y + i);
            downRight = getPixel(b, x2, y - i);
            leftRight = getPixel(b, x2 - i, y);
            rightRight = getPixel(b, x2 + i, y);

            redRight += Color.red(upRight);
            redRight += Color.red(downRight);
            redRight += Color.red(leftRight);
            redRight += Color.red(rightRight);

            blueRight += Color.blue(upRight);
            blueRight += Color.blue(downRight);
            blueRight += Color.blue(leftRight);
            blueRight += Color.blue(rightRight);
        }

        int redLeftTotal = redLeft / (radius * 4);
        int blueLeftTotal = blueLeft / (radius * 4);
        int redRightTotal = redRight / (radius * 4);
        int blueRightTotal = blueRight / (radius * 4);

        AllianceColor leftLikely = redLeftTotal > redRightTotal ? AllianceColor.Red : blueLeftTotal > blueRightTotal ? AllianceColor.Blue : AllianceColor.NotSure;
        AllianceColor rightLikely = redLeftTotal < redRightTotal ? AllianceColor.Red : blueLeftTotal < blueRightTotal ? AllianceColor.Blue : AllianceColor.NotSure;

        if (leftLikely == AllianceColor.NotSure || rightLikely == AllianceColor.NotSure) {
            if (leftLikely != AllianceColor.NotSure) {
                return new Reading(leftLikely, 30);
            } else if (rightLikely != AllianceColor.NotSure) {
                return new Reading(rightLikely, 30);
            }
            return new Reading(AllianceColor.NotSure, 100);
        }

        if (leftLikely != rightLikely) {
            return new Reading(leftLikely, 95);
        }

        if (leftLikely == AllianceColor.Red) {
            if (redLeftTotal > redRightTotal) {
                return new Reading(AllianceColor.Red, 100 * (redLeftTotal - redRightTotal) / (redLeftTotal));
            } else {
                return new Reading(AllianceColor.Blue, 100 * (redRightTotal - redLeftTotal) / (redRightTotal));
            }
        } else {
            if (blueLeftTotal > blueRightTotal) {
                return new Reading(AllianceColor.Blue, 100 * (blueLeftTotal - blueRightTotal) / (blueLeftTotal));
            } else {
                return new Reading(AllianceColor.Red, 100 * (blueRightTotal - blueLeftTotal) / (blueRightTotal));
            }

        }

    }

    public Reading getColor() {
        LinearLayout cameraMonitorViewId = (LinearLayout) findViewById(R.id.cameraMonitorViewId);
        if (!cameraMonitorViewId.isShown()) {
            return new Reading(AllianceColor.NotSure, 100);
        }
        int height = 50;
        int leftPos = 100;
        int rightPos = leftPos + 400;
        Bitmap bitmap = cameraMonitorViewId.getDrawingCache();
        return analyze(bitmap, leftPos, rightPos, height, 15);
    }

}
