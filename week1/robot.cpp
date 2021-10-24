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
    if (ls != 0 && ls < 10 && rs != 0 && rs < 10) {
        robot.stop();
        // 左を向く。
        robot.move(0.0, 3.1415 / 2.0);
        // 左の情報を集める。
        int left_ls = robot.get_ls();
        int left_rs = robot.get_rs();
        int left_distance = (left_ls + left_rs) / 2;
        // 右を向く。
        robot.move(0.0, -3.1415);
        int right_ls = robot.get_ls();
        int right_rs = robot.get_rs();
        int right_distance = (right_ls + right_rs) / 2;

        // 動きを終了させる。
        if (left_distance < 5 && right_distance < 5) {
            robot.end();
        }

        // 左と右で遠い方を選ぶ。
        int longer_distance;
        bool choose_left = false;
        // 大きすぎてもだめ
        if (left_distance < right_distance && right_distance < 10) {
            longer_distance = right_distance;
        } else {
            longer_distance = left_distance;
            choose_left = true;
        }

        // 左を選ぶか？
        if (choose_left) {
            robot.move(0.0, 3.1415);
        }

        // 動かせる
        robot.move(0.05, 0.0);
        robot.wait(0.5);
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
