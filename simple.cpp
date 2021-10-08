int main() {
    // debug

    int cnt = 0;

    int ls = robot.get_ls();
    int rs = robot.get_rs();

    int robot_x_start = robot.get_x();
    int robot_y_start = robot.get_y();
    printf("start x: %d, y: %d", robot_x_start, robot_y_start);

    ROS_INFO("Left Sonar %i cm", ls);
    ROS_INFO("Right Sonar %i cm", rs);

    // ゴール
    if (ls < 5 && rs < 5) {
        if (ls < rs) {
            // 右に回転
            robot.move(0.0, -1.5);
            robot.move(0.05, 0.0);
            robot.wait(0.5);
        } else {
            // 左に回転
            robot.move(0.0, 1.5);
            robot.move(0.05, 0.0);
            robot.wait(0.5);
        }
        cnt += 1;
        if (cnt == 2) {
            robot.end();
        }
    } else if (ls != 0 && ls < 10) {
        robot.stop();
        robot.move(0.0, -1.0);
        robot.wait(0.5);
    } else if (rs != 0 && rs < 10) {
        robot.stop();
        robot.move(0.0, 1.0);
        robot.wait(0.5);
    } else {
        robot.move(0.1, 0.0);
        robot.wait(0.5);
    }


    // debug
    int robot_x_end = robot.get_x();
    int robot_y_end = robot.get_y();
    printf("start x: %d, y: %d", robot_x_end, robot_y_end);
}

// cd catkin_wsb catkin_make
