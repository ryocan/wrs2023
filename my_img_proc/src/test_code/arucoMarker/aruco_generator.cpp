//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// about image processing
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/aruco.hpp>

class arMarkerGenerator{
public:
	int pixel = 150;
	int offset = 10;
	static const int CNT = 9;

	void generateArMarker()
	{
		cv::Mat img = cv::Mat::zeros(pixel + offset, pixel + offset, CV_8UC1);
		cv::Mat hconcat_img, vconcat_img;
		img = cv::Scalar(255, 255, 255);
		int x_offset, y_offset;
		x_offset = y_offset = (int)(offset / 2);

		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

		for(int i = 0; i < CNT; i++){
			cv::Mat ar_img;
			cv::aruco::drawMarker(dictionary, i, pixel, ar_img, 1);
			// ファイル名の設定
			std::string file_nm = "ar" + std::to_string(i) + ".png";
			// アフィン行列
			cv::Mat mat = (cv::Mat_<double>(2,3)<<1.0, 0.0, x_offset, 0.0, 1.0, y_offset);
			
			// アフィン変換
			cv::warpAffine(ar_img, img, mat, img.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
			// グレースケールからRGBへ変換
			cv::Mat rgb_img;
			cv::cvtColor(img, rgb_img, cv::COLOR_GRAY2RGB);

			if(i % 3 == 0){
				hconcat_img = rgb_img;
			} else if(i % 3 <=2){
				cv::hconcat(hconcat_img, rgb_img, hconcat_img);				
				if(i % 3 == 2 && i / 3 == 0){
					vconcat_img = hconcat_img;
				} else if(i % 3 == 2 && i / 3 > 0){
					cv::vconcat(vconcat_img, hconcat_img, vconcat_img);
				}
			}
			// Output RGB image.
			cv::imwrite(file_nm, rgb_img);
		}
        // まとめた画像
		// cv::imwrite("ar" + std::to_string(CNT) + ".png", vconcat_img);
	}
};

int main(int argc, char* argv[]){
	arMarkerGenerator ar;
	ar.generateArMarker();

	return 0;
}