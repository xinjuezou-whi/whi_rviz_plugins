/******************************************************************
custom widget for drawing twist

Features:
- drawing linear and angular
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/widget_twist.h"

#include <QPainter>
#include <math.h>

namespace whi_rviz_plugins
{
    TwistWidget::TwistWidget(QWidget* Parent/* = nullptr*/)
        : QWidget(Parent) {}

    QSize TwistWidget::minimumSizeHint() const
    {
        return QSize(200, 100);
    }

    QSize TwistWidget::sizeHint() const
    {
        return QSize(200, 100);
    }

    void TwistWidget::setLinear(float Linear)
    {
        linear_velocity_ = Linear;
        update();
    }

    void TwistWidget::setLinearMin(float Min)
    {
        linear_min_ = Min;
    }

    void TwistWidget::setLinearMax(float Max)
    {
        linear_scale_ = Max;
        update();
    }

	void TwistWidget::setLinearStep(float Step)
    {
        linear_step_ = Step;
    }

    void TwistWidget::setAngular(float Angular)
    {
        angular_velocity_ = Angular;
        update();
    }

	void TwistWidget::setAngularMin(float Min)
    {
        angular_min_ = Min;
    }

	void TwistWidget::setAngularMax(float Max)
    {
        angular_scale_ = Max;
    }

	void TwistWidget::setAngularStep(float Step)
    {
        angular_step_ = Step;
    }

    float TwistWidget::getLinear()
    {
        return linear_velocity_;
    }

    float TwistWidget::getLinearMin()
    {
        return linear_min_;
    }

    float TwistWidget::getLinearMax()
    {
        return linear_scale_;
    }

    float TwistWidget::getLinearStep()
    {
        return linear_step_;
    }

	float TwistWidget::getAngular()
    {
        return angular_velocity_;
    }

    float TwistWidget::getAngularMin()
    {
        return angular_min_;
    }

    float TwistWidget::getAngularMax()
    {
        return angular_scale_;
    }

    float TwistWidget::getAngularStep()
    {
        return angular_step_;
    }

    void TwistWidget::paintEvent(QPaintEvent* Event)
    {
        int areaWidth = width() - 5;
        int areaHeight = height() - 5;

        /// twist visual area
        QPainter painter(this);
        QColor background(Qt::white);
        QColor crosshair(Qt::black);
        painter.setBrush(background);
        painter.setPen(crosshair);
        // draw the background square
        painter.drawRect(QRect(0, 0, areaWidth, areaHeight));
        // draw a cross-hair inside the square
        int halfWidth = areaWidth >> 1;
        int halfHeight = areaHeight >> 1;
        painter.drawLine(0, halfHeight, areaWidth, halfHeight);
        painter.drawLine(halfWidth, 0, halfWidth, areaHeight);
        // draw frame
        int length = 0.6 * std::min(halfWidth, halfHeight);
        QPen frameX;
        frameX.setWidth(4);
        frameX.setColor(Qt::red);
        painter.setPen(frameX);
        painter.drawLine(areaWidth - 10, areaHeight - 10 - length, areaWidth - 10, areaHeight - 10);
        QPen frameY;
        frameY.setWidth(4);
        frameY.setColor(Qt::green);
        painter.setPen(frameY);
        painter.drawLine(areaWidth - 10, areaHeight - 10, areaWidth - 10 - length, areaHeight - 10);
        QPen frameZ;
        frameZ.setWidth(4);
        frameZ.setColor(Qt::blue);
        painter.setPen(frameZ);
        painter.drawPoint(areaWidth - 10, areaHeight - 10);

        /// twist visulaization
        if (fabs(angular_velocity_) > 1e-3 || fabs(linear_velocity_) > 1e-3)
        {
            // sampling along a central arc defined by the linear and angular velocites
            // at each step, it computes where the left and right wheels would be
            // and collects the resulting points in the lefTrack and right_track arrays
            QPen arrow;
            arrow.setWidth(8);
            arrow.setColor(Qt::green);
            arrow.setCapStyle(Qt::RoundCap);
            arrow.setJoinStyle(Qt::RoundJoin);
            painter.setPen(arrow);

            const int STEP_COUNT = 100;
            QPointF lefTrack[STEP_COUNT];
            QPointF rightTrack[STEP_COUNT];

            float halfTrackWidth = areaHeight / 4.0;

            float cx = 0.5 * areaWidth;
            float cy = 0.5 * areaHeight;
            lefTrack[0].setX(cx - halfTrackWidth);
            lefTrack[0].setY(cy);
            rightTrack[0].setX(cx + halfTrackWidth);
            rightTrack[0].setY(cy);
            float angle = 0.5 * M_PI;
            float deltaAngle = angular_velocity_ / STEP_COUNT;
            float stepDist = 0.5 * areaHeight * linear_velocity_ / linear_scale_ / STEP_COUNT;
            for (int step = 1; step < STEP_COUNT; ++step)
            {
                angle += 0.5 * deltaAngle;
                float nextCx = cx + stepDist * cosf(angle);
                float nextCy = cy - stepDist * sinf(angle);
                angle += 0.5 * deltaAngle;

                lefTrack[step].setX(nextCx + halfTrackWidth * cosf(angle + 0.5 * M_PI));
                lefTrack[step].setY(nextCy - halfTrackWidth * sinf(angle + 0.5 * M_PI));
                rightTrack[step].setX(nextCx + halfTrackWidth * cosf(angle - 0.5 * M_PI));
                rightTrack[step].setY(nextCy - halfTrackWidth * sinf(angle - 0.5 * M_PI));

                cx = nextCx;
                cy = nextCy;
            }
            painter.drawPolyline(lefTrack, STEP_COUNT);
            painter.drawPolyline(rightTrack, STEP_COUNT);

            // to decide which direction each arrowhead will point (forward or backward)
            // this works by comparing the arc length travelled by the center in one step (stepDist)
            // with the arc length travelled by the wheel (halfTrackWidth * deltaAngle)
            int leftArrowDir = (-stepDist + halfTrackWidth * deltaAngle > 0);
            int rightArrowDir = (-stepDist - halfTrackWidth * deltaAngle > 0);

            // use MiterJoin for the arrowheads so we get a nice sharp point
            arrow.setJoinStyle(Qt::MiterJoin);
            painter.setPen(arrow);

            // compute and draw polylines for each arrowhead
            const float HEAD_LEN = 6.0;
            QPointF arrowHead[3];
            if (fabsf(-stepDist + halfTrackWidth * deltaAngle) > 0.01)
            {
                float x = lefTrack[STEP_COUNT - 1].x();
                float y = lefTrack[STEP_COUNT - 1].y();
                arrowHead[0].setX(x + HEAD_LEN * cosf(angle + 3.0 * M_PI / 4.0 + leftArrowDir * M_PI));
                arrowHead[0].setY(y - HEAD_LEN * sinf(angle + 3.0 * M_PI / 4.0 + leftArrowDir * M_PI));
                arrowHead[1].setX(x);
                arrowHead[1].setY(y);
                arrowHead[2].setX(x + HEAD_LEN * cosf(angle - 3.0 * M_PI / 4.0 + leftArrowDir * M_PI));
                arrowHead[2].setY(y - HEAD_LEN * sinf(angle - 3.0 * M_PI / 4.0 + leftArrowDir * M_PI));
                painter.drawPolyline(arrowHead, 3);
            }
            if (fabsf(-stepDist - halfTrackWidth * deltaAngle) > 0.01)
            {
                float x = rightTrack[STEP_COUNT - 1].x();
                float y = rightTrack[STEP_COUNT - 1].y();
                arrowHead[0].setX(x + HEAD_LEN * cosf(angle + 3.0 * M_PI / 4.0 + rightArrowDir * M_PI));
                arrowHead[0].setY(y - HEAD_LEN * sinf(angle + 3.0 * M_PI / 4.0 + rightArrowDir * M_PI));
                arrowHead[1].setX(x);
                arrowHead[1].setY(y);
                arrowHead[2].setX(x + HEAD_LEN * cosf(angle - 3.0 * M_PI / 4.0 + rightArrowDir * M_PI));
                arrowHead[2].setY(y - HEAD_LEN * sinf(angle - 3.0 * M_PI / 4.0 + rightArrowDir * M_PI));
                painter.drawPolyline(arrowHead, 3);
            }
        }
    }
} // end namespace whi_rviz_plugins
