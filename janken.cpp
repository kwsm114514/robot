int main() {
    // -1:未検出　0:グー 1: チョキ　2: パー
    int left_result = -1;
    int right_result = -1;

    if (camera.has_left_hand()) {
        std::vector<csv::Point2i> lms = camera.get_lefthand_lm();
        // 手首
        cv::Point2i lm0 = lms[0];
        // 親指
        cv::Point2i lm1 = lms[1];
        // 人差し指
        cv::Point2i lm2 = lms[2];
        // 中指
        cv::Point2i lm3 = lms[3];
        // 薬指
        cv::Point2i lm4 = lms[4];
        // 親指
        cv::Point2i lm5 = lms[5];

        // ジェスチャーの認識

        // 座標へのアクセス
        int dist_x = lm1.x - lm0.x;
        int dist_y = lm4.y - lm2.y;
        // 二点間のユークリッド距離
        double dist = cv::norm(lm1 - lm3);
        double dist_from1to5 = cv::norm(lm1 - lm5);
        double dist_form1to2 = cv::norm(lm1 - lm2);
        double dist_form2to3 = cv::norm(lm2 - lm3);
        double dist_form3to4 = cv::norm(lm3 - lm4);
        double dist_form4to5 = cv::norm(lm4 - lm5);
        if (dist < 100) {
            left_result = 0;
        } else if (dist_form1to2 / dist_from1to5 > 2.0) {
            left_result = 1;
        } else if (0.5 < dist_form2to3 / dist_form3to4 && dist_form2to3 / dist_form3to4 < 2.0
        && 0.5 < dist_form3to4 / dist_form4to5 && dist_form3to4 / dist_form4to5 < 2.0)
        {
            left_result = 2;
        }
    }

    if (camera.has_right_hand()) {
        std::vector<csv::Point2i> lms = camera.get_righthand_lm();
        // 手首
        cv::Point2i lm0 = lms[0];
        // 親指
        cv::Point2i lm1 = lms[1];
        // 人差し指
        cv::Point2i lm2 = lms[2];
        // 中指
        cv::Point2i lm3 = lms[3];
        // 薬指
        cv::Point2i lm4 = lms[4];
        // 親指
        cv::Point2i lm5 = lms[5];

        // ジェスチャーの認識

        // 座標へのアクセス
        int dist_x = lm1.x - lm0.x;
        int dist_y = lm4.y - lm2.y;
        // 二点間のユークリッド距離
        double dist = cv::norm(lm1 - lm3);
        double dist_from1to5 = cv::norm(lm1 - lm5);
        double dist_form1to2 = cv::norm(lm1 - lm2);
        double dist_form2to3 = cv::norm(lm2 - lm3);
        double dist_form3to4 = cv::norm(lm3 - lm4);
        double dist_form4to5 = cv::norm(lm4 - lm5);
        if (dist < 100) {
            right_result = 0;
        } else if (dist_form1to2 / dist_from1to5 > 2.0) {
            right_result = 1;
        } else if (0.5 < dist_form2to3 / dist_form3to4 && dist_form2to3 / dist_form3to4 < 2.0
        && 0.5 < dist_form3to4 / dist_form4to5 && dist_form3to4 / dist_form4to5 < 2.0)
        {
            right_result = 2;
        }
    }
}
