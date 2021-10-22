#include <ros/ros.h>

#include <functional>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "enshu3/detection_camera.hpp"
#include "enshu3/myrobot.h"

int main(int argc, char** argv)
{
    // ROS node
    ros::init(argc, argv, "enshu3_driving");
    ros::NodeHandle n;
    ros::Rate rate(MAIN_RATE);
    DetectionCamera camera(n, rate);
    MyRobot robot(n, rate);

    int stops = 0;
    // Main loop
    while (ros::ok())
    {
    //////////// <write your code from here> /////////////

    int ls = robot.get_ls();
    int rs = robot.get_rs();

    bool shoul_stop = false;

    int robot_x_start = robot.get_x();
    int robot_y_start = robot.get_y();

    if (robot_y_start > 1.0) {
        robot.stop();
    }

    ROS_INFO("Left Sonar %i cm", ls);
    ROS_INFO("Right Sonar %i cm", rs);

    printf("start x: %d, y: %d", robot_x_start, robot_y_start);

    cv::Mat img = camera.get_img();
    // 画像がある場合
    if (!img.empty())
    {
        // Command for robot
        double v = 0;
        double omega = 0;

        // Set robot velocity according to the detected object
        // (Only the following types of objects can be detected)
        // - stop sign
        // - person
        // - cat
        // - traffic light
        std::vector<BBox> detection = camera.get_detection();
        for (int i = 0; i < detection.size(); i++)
        {
            BBox bbox = detection[i];
            int center_x = (bbox.ul.x + bbox.br.x) / 2;
            int center_y = (bbox.ul.y + bbox.br.y) / 2;
            int width = (bbox.br.x - bbox.ul.x);
            int height = (bbox.br.y - bbox.ul.y);
            ROS_INFO("[%s] center:(%d, %d), width:%d, height:%d", bbox.label.c_str(), center_y, center_x, width, height);

            // 物体が止まれの場合
            if (bbox.label == "stop sign" && height > 10 && width > 10) {
                stops++;
                shoul_stop = true;
                if (stops > 5) {
                    printf("Too many stops");
                    robot.end();
                }
                if (center_x < (640 / 2) - 50) {
                    v = 0.02;
                    omega = 0.2;
                    robot.move(0.02, 0.2);
                    robot.wait(0.1);
                }
                else if (center_x > (640 / 2) + 50) {
                    v = 0.02;
                    omega = -0.2;
                    robot.move(0.02, -0.2);
                    robot.wait(0.1);
                }
                else if (height > 150 && width > 200) {
                    printf("Too close to the stop")
                    robot.end();
                }
                else{
                    v = 0.04;
                    omega = 0.0;
                }
            }
            // 信号のとき
            if (bbox.label == "trafic light" && height > 100) {
                // 色の判定をする。
                bool is_red = false;
                // 赤かどうか？
                for (int y = bbox.ul.y; y < bbox.br.y; y++) {
                    for (int x = bbox.ul.x; x < bbox.br.x; x++) {
                        if (img.at<cv::Vec3b>(y, x)[2] > 126) {
                            printf("Red: %d", img.at<cv::Vec3b>(y, x)[2]);
                            is_red = true;
                            break;
                        }
                    }
                }

                if (is_red) {
                    v = 0.0;
                    omega = 0.0;
                    robot.move(0.0, 0.0);
                    robot.wait(2.0);
                } else {
                    v = 0.1;
                    omega = 0.0;
                }
            }
            // 猫のとき
            if (bbox.label == "cat") {
                robot.move(-1.0, 0.0);
                robot.wait(0.3);
                if (center_x < (640 / 2) - 50) {
                    v = 0.01;
                    omega = -0.2;
                    robot.move(0.0, -0.1);
                    robot.wait(0.3);
                }
                else if (center_x > (640 / 2) + 50) {
                    v = 0.02;
                    omega = 0.2;
                    robot.move(0.0, 0.1);
                    robot.wait(0.3);
                }
            }
            if (bbox.label == "person") {
                continue;
            }
        }

        // Send command to robot
        robot.move(v, omega);
        // robot.wait(0.1);

        /// Display robot command
        camera.add_command(v, omega);
        /// Display detections
        camera.add_detection();
        /// Show image
        camera.show_img();
    } 

    if (shoul_stop) {
        continue;
    }
    if (ls != 0 && ls < 10) {
        robot.stop();
        robot.move(0.0, -1.0);
        robot.wait(0.3);
    } else if (rs != 0 && rs < 10) {
        robot.stop();
        robot.move(0.0, 1.0);
        robot.wait(0.3);
    } else {
        robot.move(0.1, 0.0);
        robot.wait(0.5);
    }


    ////////////////////////////////////////////////////////
    ros::spinOnce();
    rate.sleep();
    }

    return 0;
}
