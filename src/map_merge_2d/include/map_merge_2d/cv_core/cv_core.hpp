/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez.
 *  Copyright (c) 2022, Achala Athukorala
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef CV_CORE_H
#define CV_CORE_H

#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <map_merge_2d/cv_core/cv_helpers.hpp>

#define NDEBUG

namespace map_merge_2d
{
    namespace cv_core
    {
        /**
         * @brief Struct containing an image and global coordinate offset
         * 
         * @var image : cv::Mat object containing image data
         * @var origin: cv::Point2d object containing 'real' [x,y] coordinates of the (0,0) pixel in image
         * 
         * @note cv::Mat images only contain positive pixel coordinates. Hence, an image cannot be
         *              directly represented in negative pixel locations (e.g. Cannot show an
         *              image which was translated by -100, -100 pixels in x and y axes)
         *              To represent images in full cartesian pixel plane, the 'orgin' variable is used. 
         *              The 'image' matrix will infact contain image data in positive pixel coordinate system,
         *              but the 'origin' variable will track the absolute location of the (0,0) pixel in 'image' matrix.
         */
        struct CVImage
        {
            CVImage(cv::Mat image_=cv::Mat() , cv::Point2d origin_ = cv::Point2d())
            {
                image = image_;
                origin = origin_;
            }

            CVImage(cv::Mat image_, double x, double y)
            {
                image = image_;
                origin = cv::Point2d(x, y);
            }

            cv::Mat image;      /** Image */
            cv::Point2d origin; /** Offset of Image(0,0) pixel in global coordinate frame */
        };

        enum class FeatureType 
        { 
            AKAZE, 
            ORB, 
            SURF 
        };

        cv::Ptr<cv::Feature2D> chooseFeatureFinder(FeatureType type);

        void image_transform(const CVImage src, CVImage &dest, cv::Mat transform);
        std::map<int, cv::Mat> estimateTransforms(std::vector<cv::Mat> images, FeatureType feature_type, double confidence,
                                                    std::map<int, double> &estimation_confidences);

        
    } // namespace cv_core

}  // namespace map_merge_2d

#endif  // CV_CORE_H