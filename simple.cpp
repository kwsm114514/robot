int main() {
    // debug
    int ls = robot.get_ls();
    int rs = robot.get_rs();

    int robot_x_start = robot.get_x();
    int robot_y_start = robot.get_y();
    printf("start x: %d, y: %d", robot_x_start, robot_y_start);

    ROS_INFO("Left Sonar %i cm", ls);
    ROS_INFO("Right Sonar %i cm", rs);

    // ゴール
    if (ls != 0 && ls < 3 && rs != 0 && rs < 3) {
        robot.end();
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
