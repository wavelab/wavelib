#include "wave/wave_test.hpp"
#include "wave/kinematics/two_wheel.hpp"

namespace wave {

TEST(TwoWheelRobot2DModel, update) {
    // setup
    TwoWheelRobot2DModel robot;

    // Motor inputs for circle trajectory:
    //
    // Assuming we want to travel a 10 m radius circle at 1 ms^-1
    //
    // distance = 2 * pi * radius = 20 * pi
    // time_taken = distance / velocity = 20 * pi / 1.0 = 20 * pi
    // angular_velocity = (2 * pi) / time_taken = (2 * pi) / (20 * pi) = 0.1
    //
    // therefore motor inputs is [1.0, 0.1]
    double v = 1.0;
    double t = 0.0;
    double t_end = 20 * M_PI;
    double dt = 0.001;
    double w = (2 * M_PI) / t_end;
    Vec2 motor_inputs = Vec2{v, w};

    // simulate and assert
    while (t <= t_end) {
        robot.update(motor_inputs, dt);
        t += dt;
    }

    // expect robot to travel a full circle back near where it started from
    EXPECT_NEAR(robot.pose(0), 0.0, 0.001);
    EXPECT_NEAR(robot.pose(1), 0.0, 0.001);
    EXPECT_NEAR(robot.pose(2), 2 * M_PI, 0.001);
}

}  // end of wave namespace
