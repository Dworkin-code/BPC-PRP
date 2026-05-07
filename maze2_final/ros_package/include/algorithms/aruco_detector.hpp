#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>

namespace algorithms {

    // ---------------------------------------------------------------------------
    // ArucoDetector
    //
    // Detects ArUco markers (DICT_4X4_50) in a BGR cv::Mat frame.
    // Returns a list of Aruco structs, each containing the marker ID and its
    // four corner points in image coordinates.
    //
    // Usage:
    //   algorithms::ArucoDetector detector;
    //   auto results = detector.detect(frame);
    //   for (auto& a : results)
    //       std::cout << "ID: " << a.id << "\n";
    // ---------------------------------------------------------------------------

    class ArucoDetector {
    public:

        // Represents one detected marker
        struct Aruco {
            int id;
            std::vector<cv::Point2f> corners;  // TL, TR, BR, BL (OpenCV convention)
        };

        ArucoDetector() {
            // 4x4 markers with up to 50 unique IDs — matches the lab marker set
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        }

        ~ArucoDetector() = default;

        // Detect markers in the input BGR image.
        // Returns an empty vector if no markers are found.
        std::vector<Aruco> detect(const cv::Mat& frame) {
            std::vector<Aruco> arucos;

            if (frame.empty())
                return arucos;

            std::vector<int>                       marker_ids;
            std::vector<std::vector<cv::Point2f>>  marker_corners;

            // Run OpenCV ArUco detection
            cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids);

            if (!marker_ids.empty()) {
                std::cout << "Arucos found: ";
                for (size_t i = 0; i < marker_ids.size(); i++) {
                    std::cout << marker_ids[i] << " ";
                    arucos.emplace_back(Aruco{ marker_ids[i], marker_corners[i] });
                }
                std::cout << std::endl;
            }

            return arucos;
        }

    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
    };

} // namespace algorithms
