#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <librealsense2/rs.hpp>

int main(int argc, const char* argv[])
{
  // OpenCVのバージョンをログ
  std::cout << "OpenCV v" << cv::getVersionMajor() << "."
    << cv::getVersionMinor() << "."
    << cv::getVersionRevision() << std::endl;

  // RealSense SDKのバージョンをログ
  std::cout << "RealSense SDK v" << RS2_API_MAJOR_VERSION << "."
    << RS2_API_MINOR_VERSION << "."
    << RS2_API_PATCH_VERSION << std::endl;

  // ウィンドウを作成
  cv::namedWindow("viewer", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("viewer2", cv::WINDOW_AUTOSIZE);

  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  // RealSenseの動作開始
  // （やっつけ複数台カメラ対応）
  rs2::context ctx;
  std::vector<rs2::pipeline> pipelines;

  for (auto&& dev : ctx.query_devices()) {
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    pipelines.emplace_back(pipe);
  }

  while (true) {
    for (int camera_i = 0; camera_i < 2; camera_i++) {
      // RealSenseからカラー映像を取得
      auto& pipe = pipelines[camera_i];
      auto frames = pipe.wait_for_frames();
      auto color = frames.get_color_frame();

      auto intr = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
      std::cout << pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;

      // OpenCV形式に変換（コピーしない）
      cv::Mat cv_frame(cv::Size(1280, 720), CV_8UC3, const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);

      // マーカー画像検出
      std::vector<int> marker_ids;
      std::vector<std::vector<cv::Point2f>> marker_corners;
      auto parameters = cv::aruco::DetectorParameters::create();
      cv::aruco::detectMarkers(cv_frame, dictionary, marker_corners, marker_ids, parameters);

      // 検出したマーカー位置を描画
      cv::aruco::drawDetectedMarkers(cv_frame, marker_corners, marker_ids);

      // カメラの内部パラメータはRealSenseSDKのAPIから取得する（補正は専用ツールで行う）
      cv::Mat K = (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.width, 0, intr.fy, intr.height, 0, 0, 1);
      cv::Mat distortion = (cv::Mat_<double>(5, 1) << intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]);

      // マーカーの3D空間内の位置を推定
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.2f, K, distortion, rvecs, tvecs);

      for (int i = 0; i < marker_ids.size(); i++) {
        // 推定した位置を描画
        cv::aruco::drawAxis(cv_frame, K, distortion, rvecs[i], tvecs[i], 0.1);
      }

      const char* win_names[] = {
        "viewer",
        "viewer2"
      };
      cv::imshow(win_names[camera_i], cv_frame);
    }

    auto c = static_cast<char>(cv::waitKey(30));
    if (c == 27) {
      // Escで抜ける
      break;
    }
  }
  cv::destroyWindow("viewer");
  cv::destroyWindow("viewer2");
}