#ifndef CV_CORE_HELPERS_H
#define CV_CORE_HELPERS_H

#include <opencv2/opencv.hpp>

namespace map_merge_2d
{
    namespace cv_core_helper
    {
        inline static void writeDebugMatchingInfo(
            const std::vector<cv::Mat>& images,
            const std::vector<cv::detail::ImageFeatures>& image_features,
            const std::vector<cv::detail::MatchesInfo>& pairwise_matches)
        {
            for (auto& match_info : pairwise_matches) {
                if (match_info.H.empty() ||
                    match_info.src_img_idx >= match_info.dst_img_idx) {
                continue;
                }
                // std::cout << match_info.src_img_idx << " " << match_info.dst_img_idx
                //         << std::endl
                //         << "features: "
                //         << image_features[size_t(match_info.src_img_idx)].keypoints.size()
                //         << " "
                //         << image_features[size_t(match_info.dst_img_idx)].keypoints.size()
                //         << std::endl
                //         << "matches: " << match_info.matches.size() << std::endl
                //         << "inliers: " << match_info.num_inliers << std::endl
                //         << "inliers/matches ratio: "
                //         << match_info.num_inliers / double(match_info.matches.size())
                //         << std::endl
                //         << "confidence: " << match_info.confidence << std::endl
                //         << match_info.H << std::endl;
                cv::Mat img;
                // draw all matches
                cv::drawMatches(images[size_t(match_info.src_img_idx)],
                                image_features[size_t(match_info.src_img_idx)].keypoints,
                                images[size_t(match_info.dst_img_idx)],
                                image_features[size_t(match_info.dst_img_idx)].keypoints,
                                match_info.matches, img);
                // cv::imwrite(std::to_string(match_info.src_img_idx) + "_" +
                //                 std::to_string(match_info.dst_img_idx) + "_matches.png",
                //             img);
                // draw inliers only
                // cv::imshow(std::to_string(match_info.src_img_idx) + "_" +
                //                 std::to_string(match_info.dst_img_idx) + "_matches", img);
                // cv::waitKey(0);
                cv::drawMatches(
                    images[size_t(match_info.src_img_idx)],
                    image_features[size_t(match_info.src_img_idx)].keypoints,
                    images[size_t(match_info.dst_img_idx)],
                    image_features[size_t(match_info.dst_img_idx)].keypoints,
                    match_info.matches, img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    *reinterpret_cast<const std::vector<char>*>(&match_info.inliers_mask));
                // cv::imwrite(std::to_string(match_info.src_img_idx) + "_" +
                //                 std::to_string(match_info.dst_img_idx) +
                //                 "_matches_inliers.png",
                //             img);

                cv::Mat scaled_image;
                cv::Size scale(img.cols / 2, img.rows / 2);
                cv::resize(img, scaled_image, scale, cv::INTER_LINEAR);
                cv::imshow(std::to_string(match_info.src_img_idx) + "_" +
                                std::to_string(match_info.dst_img_idx) + "_matches_inliers", scaled_image);
                cv::waitKey(500);
            }
        }
    } // cv_core_helper
} // map_merge_2d

#endif // CV_CORE_HELPERS_H