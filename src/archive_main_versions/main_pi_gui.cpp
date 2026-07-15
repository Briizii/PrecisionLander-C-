/**
 * @file main.cpp
 * @project PrecisionLander
 * @brief ArUco marker detection and pose estimation for precision landing.
 * @author Alex Smith
 * @date 2026-01-09
 * @license GPLv3
 * 
 * Requires camera, outputs Pose struct to serial, run with a pipe to publisher.py for operation.
 * The license used requires that the fractal detector algorithm remains separate to any proprietary code.
 * For example, anything across the mavlink bridge is protected.
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/fractaldetector.h>
#include <aruco/marker.h>
#include <filesystem>
#include <thread>
#include <iomanip>
#include <fstream>
#include <vector>

namespace {
    constexpr float MARKER_SIZE = 0.01f;
    constexpr int DEFAULT_WAIT_TIME = 10;
    constexpr int LOG_EVERY_N_FRAMES = 5;
    const std::string CAMERA_PARAMS_FILE = "camera_result.xml";
    const std::string LOG_FILE = "pose_log.csv";

    struct Pose {
        double tx, ty, tz;
        double distance;
        double angle_x;
        double angle_y;
    };

    void printWelcomeMessage() {
        std::cerr << '\n' << std::string(80, '-') << '\n' << std::endl;
        std::cerr << "Starting ArUco Detection!" << std::endl;
        std::cerr << '\n' << std::string(80, '-') << '\n' << std::endl;
    }

    bool loadCameraParameters(const std::string& filename, aruco::CameraParameters& camParam) {
        if (!std::filesystem::exists(filename)) {
            std::cerr << "\033[31mCamera calibration file does not exist: " << filename << "\033[0m" << std::endl;
            return false;
        }

        std::cerr << "Camera calibration file exists!" << std::endl;
        camParam.readFromXMLFile(filename);

        if (camParam.CameraMatrix.empty() || camParam.Distorsion.empty()) {
            std::cerr << "\033[31mError: Camera matrices are empty. Check " << filename << " file content.\n"
                      << "Use the camera calibration script in the included Aruco library to generate.\033[0m" << std::endl;
            return false;
        }

        std::cerr << "Camera matrix is: \n" << camParam.CameraMatrix << "\n"
                  << "Camera distortion is: \n" << camParam.Distorsion << std::endl;
        return true;
    }

    std::vector<cv::Point3f> getObjectPoints(float markerSize) {
        const float s = markerSize * 0.5f;
        return {
            {-s, +s, 0.0f},
            {+s, +s, 0.0f},
            {+s, -s, 0.0f},
            {-s, -s, 0.0f},
        };
    }

    Pose estimatePose(const std::vector<cv::Point3f>& objPts,
                      const std::vector<cv::Point2f>& imgPts,
                      const cv::Mat& camMatrix,
                      const cv::Mat& distCoeffs) {
        cv::Mat rvec, tvec;
        cv::solvePnP(objPts, imgPts, camMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

        const double tx = tvec.at<double>(0, 0);
        const double ty = tvec.at<double>(1, 0);
        const double tz = tvec.at<double>(2, 0);
        const double distance = std::sqrt(tx * tx + ty * ty + tz * tz);
        const double angle_x = std::atan2(tx, tz);
        const double angle_y = std::atan2(ty, tz);

        return {tx, ty, tz, distance, angle_x, angle_y};
    }

    void logPose(std::ofstream& log, int counter, const Pose& pose) {
        log << std::fixed << std::setprecision(6)
            << "position = " << counter << "    |    "
            << pose.tx << "," << pose.ty << "," << pose.tz << "\n";
        log.flush();
        std::cout << "Logged position " << counter << ".\n";
    }
} //end function declarations

int main() {
    printWelcomeMessage();

    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(6);

    aruco::CameraParameters camParam;
    if (!loadCameraParameters(CAMERA_PARAMS_FILE, camParam)) {
        return -1;
    }

    // Using Gstreamer for OV9281 Camera
    std::string pipeline =
    "libcamerasrc ! "
    "video/x-raw,width=1280,height=800,framerate=30/1 ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=true sync=false";

    cv::VideoCapture camera(pipeline, cv::CAP_GSTREAMER);

    if (!camera.isOpened()) {
        std::cerr << "\033[31mCould not open camera input\033[0m" << std::endl;
        return -1;
    }

    std::cerr << "\nCamera Open!\nPress 'ESC' to cancel\n" << std::endl;

    cv::Mat frame;
    camera >> frame;
    if (frame.empty()) {
        std::cerr << "\033[31mFailed to capture first frame\033[0m" << std::endl;
        return -1;
    }

    camParam.resize(frame.size());
    const auto objPts = getObjectPoints(MARKER_SIZE);

    aruco::FractalDetector detector;
    detector.setConfiguration("FRACTAL_3L_6");
    detector.setParams(camParam, MARKER_SIZE);

    std::ofstream log(LOG_FILE, std::ios::app);
    if (!log.is_open()) {
        std::cerr << "\033[33mWarning: Could not open log file " << LOG_FILE << "\033[0m" << std::endl;
    } else {
        log << "position        |       est_x,est_y,est_z\n";
    }

    int waitTime = DEFAULT_WAIT_TIME;
    int counter = 0;
    int frameCounter = 0;
    Pose lastPose = {0, 0, 0, 0, 0, 0};

    std::cerr << '\n' << std::string(80, '-') << '\n' << std::endl;

    while (true) { // main loop start
        if (!camera.read(frame) || frame.empty()) { // check valid camera frame before detection
            std::cerr << "\033[31mInvalid frame received\n\033[0m" << std::endl;
            break;
        }

        frameCounter++;

        // detection algorithm
        if (detector.detect(frame)) { // will fail without valid frame
            auto markers = detector.getMarkers();
            if (markers.empty()) { // markers are stored in a vector
                if (frameCounter % LOG_EVERY_N_FRAMES == 0) { // throttle the log messaging to preserve compute
                    std::cerr << "\r\033[31mMarkers are empty\033[0m" << std::flush;
                }
            } else {
                lastPose = estimatePose(objPts, markers[0], camParam.CameraMatrix, camParam.Distorsion);

                if (frameCounter % LOG_EVERY_N_FRAMES == 0) { // throttle the log messaging to preserve compute
                    char buf[256];
                    std::snprintf(buf, sizeof(buf),
                                  "\r Sent: x = %8.3fm, y= %8.3fm, z = %8.3fm, Distance = %8.3fm, angle_x = %8.3f°, angle_y = %8.3f°   ",
                                  lastPose.tx, lastPose.ty, lastPose.tz, lastPose.distance,
                                  (lastPose.angle_x * 180.0 / M_PI), (lastPose.angle_y * 180.0 / M_PI));
                    std::cerr << buf << std::flush;
                }

                // Output to stdout. Not throttled in the c++ program.
                std::cout << lastPose.tx << " " << lastPose.ty << " " << lastPose.tz << " "
                          << lastPose.distance << " " << lastPose.angle_x << " " << lastPose.angle_y << "\n" << std::flush;
            }
        } else {
            if (frameCounter % LOG_EVERY_N_FRAMES == 0) {
                std::cerr << "\r\033[33mNo ArUco in FOV\033[0m" << std::flush;
            }
        }

        if (waitTime != 0 || frameCounter % 30 == 0) {
            detector.drawMarkers(frame);
            cv::imshow("in", frame);
        }
        
        const int key = cv::waitKey(waitTime == 0 ? 1 : waitTime);

        if (key == 27) { // ESC
            break;
        } else if (key == 's') { // essential to allow opencv to render the GUI
            waitTime = (waitTime == 0) ? DEFAULT_WAIT_TIME : 0;
        } else if (key == 'l') { // prints the most recent position to the log file. Useful for debugging.
            if (log.is_open()) {
                logPose(log, counter++, lastPose);
            }
        }
    }

    return 0;
}
