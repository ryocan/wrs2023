double getOrientation(const std::vector<cv::Point> &pts, cv::Mat &img) {
    // 主成分分析のために、対象となる輪郭のみを別のバッファにコピーします。
    cv::Mat contours_copy = cv::Mat(contours[i].size(), 2, CV_64F); // [contours[i].size() x 2] 行列
    for (int i = 0; i < contours_copy.rows; i++) {
        contours_copy.at<double>(i, 0) = contours[i][i].x;
        contours_copy.at<double>(i, 1) = contours[i][i].y;
    }

    // 主成分分析の実行
    cv::PCA pca_analysis(contours_copy, cv::Mat(), cv::PCA::DATA_AS_ROW);

    // [分析結果] 中心の座標
    cv::Point center = cv::Point(pca_analysis.mean.at<double>(0, 0),
                               pca_analysis.mean.at<double>(0, 1));
    std::cout << pca_analysis.mean << std::endl;

    // [分析結果] 固有値と固有ベクトル
    std::vector<cv::Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; i++) {
        eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                    pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
    }
    std::cout << pca_analysis.eigenvectors << std::endl;
    std::cout << pca_analysis.eigenvalues << std::endl; // 固有値が小さい側について、次元の削除を検討します。

    // 分析結果を可視化のために描画します。
    cv::circle(img, center, 3, cv::Scalar(255, 0, 255), 2);
    cv::Point p1 = center + 0.02 * cv::Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]);
    cv::Point p2 = center - 0.02 * cv::Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]);
    drawAxis(img, center, p1, cv::Scalar(0, 255, 0), 1);
    drawAxis(img, center, p2, cv::Scalar(255, 255, 0), 5);

    // 物体の方向をラジアンで返します。
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x);
    return angle;
}