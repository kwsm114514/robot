
#include <ros/ros.h>

#include <functional>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "enshu3/detection_camera.hpp"
#include "enshu3/myrobot.h"

// 自分で追加
#include <string> 

// 課題の改良版。vとomegaを削除して可読性を上げた。あと書式を自分好みにした。猫のところを壁すれすれの運動を三回繰り返して攻略する。
// 信号機の部分は大きさの制限を解除した。とまれの記号の部分は初めに記号が見えた方向と同じ壁の向きにすれすれで五回動いて止まるようにした。
// 具体的な数値は授業で改良する。
int main(int argc, char** argv) {
  // ROS node
    ros::init(argc, argv, "enshu3_driving");
    ros::NodeHandle n;
    ros::Rate rate(MAIN_RATE);
    DetectionCamera camera(n, rate);
    MyRobot robot(n, rate);

    // 自分で追加した。
    int stops = 0;

    // Main loop
    while (ros::ok() && !robot.is_finished() && !camera.is_finished()) {
        int ls = robot.get_ls();
        int rs = robot.get_rs();

        bool should_stop = false;

        ROS_INFO("Left Sonar %i cm", ls);
        ROS_INFO("Right Sonar %i cm", rs);

        printf("y: %lf, x: %lf", robot.get_y(), robot.get_x());
        printf("tie; %lf", robot.get_time());

        // 目的地までの時間を短縮できるので制限時間をつけても問題ないはず。
        if (robot.get_time() > 100) {
            printf("time end");
            robot.end();
        }

        cv::Mat img = camera.get_img();
        if (!img.empty()) {
            // Set robot velocity according to the detected object
            // (Only the following types of objects can be detected)
            // - stop sign
            // - person
            // - cat
            // - traffic light
            std::vector<BBox> detection = camera.get_detection();
            for (int i = 0; i < detection.size(); i++) {
                BBox bbox = detection[i];
                int center_x = (bbox.ul.x + bbox.br.x) / 2;
                int center_y = (bbox.ul.y + bbox.br.y) / 2;
                int width = (bbox.br.x - bbox.ul.x);
                int height = (bbox.br.y - bbox.ul.y);
                ROS_INFO("[%s] center:(%d, %d), width:%d, height:%d", bbox.label.c_str(), center_y, center_x, width, height);

                // 物体が止まれの場合
                if (bbox.label == "stop sign" && height > 10 && width > 10) {
                    should_stop = true;
                    stops++;
                    if (stops > 7) {
                        printf("too many stops");
                        robot.end();
                    } else if (center_x < (640 / 2) - 50) {
                        robot.move(0.0, 1.0);
                        robot.wait(0.1);
                        // 左すれすれに動く。
                        nearly(robot, "left", 5);
                        printf("left end");
                        robot.end();
                    } else if (center_x > (640 / 2) + 50) {
                        // 右すれすれに動く。
                        nearly(robot, "right", 5);
                        printf("right end");
                        robot.end();
                    } else if (height > 150 && width > 90) {
                        printf("too close to stop");
                        robot.end();
                    } else{
                        robot.move(0.1, 0.0);
                        robot.wait(0.2);
                    }
                }
                // 信号のとき 大きさの制限を排除した。
                if (bbox.label == "trafic light") {
                    // 色の判定をする。
                    bool is_red = false;
                    // 赤かどうか？
                    for (int y = bbox.ul.y; y < bbox.br.y; y++) {
                        for (int x = bbox.ul.x; x < bbox.br.x; x++) {
                            // 赤の強さを判定する。
                            if (img.at<cv::Vec3b>(y, x)[2] > 126) {
                                printf("Red: %d", img.at<cv::Vec3b>(y, x)[2]);
                                is_red = true;
                                break;
                            }
                        }
                    }

                    if (is_red) {
                        robot.move(0.0, 0.0);
                        robot.wait(2.0);
                    } else {
                        continue;
                    }
                }
                // 猫のとき
                if (bbox.label == "cat") {
                    // 後ろに下がる。
                    robot.move(-1.0, 0.0);
                    robot.wait(0.2);
                    // 左に猫がいるとき
                    if (center_x < (640 / 2) - 50) {
                        robot.move(0.0, -0.3);
                        robot.wait(0.3);
                        nearly(robot, "right", 3);
                        // 左を向く
                        robot.move(0.0, 0.2);
                        robot.wait(0.3);
                    } else if (center_x > (640 / 2) + 50) {
                        robot.move(0.0, 0.3);
                        robot.wait(0.3);
                        nearly(robot, "left", 3);
                        // 右を向く
                        robot.move(0.0, -0.2);
                        robot.wait(0.3);
                    } else {
                        robot.move(0.2, 0.0);
                        robot.wait(0.3);
                    }
                }
                if (bbox.label == "person") {
                    continue;
                }
            }

            /// Display detections
            camera.add_detection();
            /// Show image
            camera.show_img();
        } 

        ls = robot.get_ls();
        rs = robot.get_rs();
        if (should_stop) {
            continue;
        } else if (ls < 20 && rs < 20) {
            robot.stop();
            robot.move(-0.1, 0.0);
            robot.wait(0.3);
        } else if (ls != 0 && ls < 10) {
            robot.stop();
            robot.move(0.0, -1.0);
            robot.wait(0.3);
        } else if (rs != 0 && rs < 10) {
            robot.stop();
            robot.move(0.0, 1.0);
            robot.wait(0.3);
        } else {
            robot.move(0.2, 0.0);
            robot.wait(0.5);
        }

        ////////////////////////////////////////////////////////
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

// どちらかの方向の壁にすれすれに動く。
void nearly(MyRobot &robot, string direction, int repeat) {
    // dir == 1.0 が左、dir == -1.0　が右
    double dir;
    if (direction == "left") {
        dir = 1.0;
    } else {
        dir = -1.0;
    }
    // near がすれすれの方向 far が反対の方向
    int near;
    int far;

    // 引数のrepeat分繰り返す。
    for (int i = 0; i < repeat; i++) {
        if (direction == "left") {
            near = robot.get_ls();
            far = robot.get_rs();
        } else {
            near = robot.get_rs();
            far = robot.get_ls();
        }
        if (far != 0 && far < 10) {
            robot.stop();
            robot.move(0.0, dir);
            robot.wait(0.1);
            robot.move(0.1, 0.0);
            robot.wait(0.1);
        } else if (near != 0 && near < 10) {
            robot.stop();
            robot.move(0.0, -dir);
            robot.wait(0.2);
            robot.move(0.1, 0.0);
            robot.wait(0.1);
            robot.move(0.0, dir);
            robot.wait(0.1);
        } else {
            robot.move(0.2, 0.0);
            robot.wait(0.5);
        }
    }
}
