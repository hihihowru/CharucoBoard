#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <librealsense2/rs_advanced_mode.hpp>
#include <streambuf>
using namespace std::literals::chrono_literals;

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <librealsense2/rs.hpp>

int main(int argc, const char* argv[])
{
  //get opencv version
  rs2::colorizer color_map;
  std::cout << "OpenCV v" << cv::getVersionMajor() << "."
    << cv::getVersionMinor() << "."
    << cv::getVersionRevision() << std::endl;

  // get realsense sdk version
  std::cout << "RealSense SDK v" << RS2_API_MAJOR_VERSION << "."
    << RS2_API_MINOR_VERSION << "."
    << RS2_API_PATCH_VERSION << std::endl;

  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  auto board = cv::aruco::CharucoBoard::create(5, 7, 0.04, 0.02, dictionary);
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
        auto& pipe = pipelines[camera_i];
        auto frames = pipe.wait_for_frames();
        auto color = frames.get_color_frame();
        rs2::frame depth = frames.get_depth_frame().apply_filter(color_map);
        auto intr = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
        std::cout << pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        cv::Mat cv_frame(cv::Size(w, h), CV_8UC3, const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_markers;
        auto parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(cv_frame, dictionary, marker_corners, marker_ids, parameters, rejected_markers);

        int interpolated_corners = 0;
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.width, 0, intr.fy, intr.height, 0, 0, 1);
        cv::Mat distortion = (cv::Mat_<double>(5, 1) << intr.coeffs[0], intr.coeffs[1], intr .coeffs[2], intr.coeffs[3], intr.coeffs[4]);
        if(marker_ids.size() > 0) {
            std::vector<cv::Point2f> charuco_corners;
            std::vector<int> charuco_ids;
            interpolated_corners = interpolated_corners + 1;
            cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, cv_frame, board, charuco_corners, charuco_ids);
            if(charuco_ids.size()>0){
                // cv::Scalar color;
                // color = Scalar(255, 0, 0);
                cv::aruco::drawDetectedCornersCharuco(cv_frame, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));
                cv::Vec3d rvec, tvec;
                
                bool valid = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, board, camera_matrix, distortion, rvec, tvec);
                if (valid){
                    cv::aruco::drawAxis(cv_frame, camera_matrix, distortion, rvec, tvec, 0.1);
                }
            }
        }
        



      //cv::aruco::drawDetectedMarkers(cv_frame, marker_corners, marker_ids);

    //   std::vector<cv::Vec3d> rvecs, tvecs;
    //   //cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.2f, K, distortion, rvecs, tvecs);
    //   for (int i = 0; i < marker_ids.size(); i++) {
    //     cv::aruco::drawAxis(cv_frame, K, distortion, rvecs[i], tvecs[i], 0.1);
    //   }

      const char* win_names[] = {
        "viewer",
        "viewer2"
      };
      cv::imshow(win_names[camera_i], cv_frame);
    }

    auto c = static_cast<char>(cv::waitKey(30));
    if (c == 27) {

      break;
    }
  }
  cv::destroyWindow("viewer");
  cv::destroyWindow("viewer2");
}