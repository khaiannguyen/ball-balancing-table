#pragma once

#include <opencv2/opencv.hpp>

#include <vector>
#include <cstdio>

/*
 * Detects the white ball on the dark circular table and converts its image
 * position into robot-frame coordinates.
 *
 * Detection pipeline:
 *
 *   BGR frame
 *      -> grayscale
 *      -> optional circular ROI
 *      -> optional CLAHE
 *      -> brightness threshold
 *      -> morphological filtering
 *      -> contour extraction
 *      -> shape filtering
 *      -> background filtering
 *      -> centroid estimation
 *      -> pixel-to-world conversion
 *
 * Pixel coordinates are converted to millimeters by:
 *
 *   undistortion
 *       -> camera ray
 *       -> world-frame ray using the calibrated rotation
 *       -> intersection with the table plane Z = 0
 *       -> robot-frame (X, Y) position
 */
struct BallDetectorConfig
{
    /*
     * Grayscale threshold applied after CLAHE when enabled.
     *
     * Valid range: 0..255.
     */
    double white_threshold = 150.0;

    /*
     * Enables Contrast Limited Adaptive Histogram Equalization before
     * thresholding.
     *
     * CLAHE improves local contrast when illumination across the table is
     * non-uniform.
     */
    bool use_clahe = true;

    /*
     * CLAHE contrast limit.
     */
    double clahe_clip_limit = 2.0;

    /*
     * CLAHE tile size in pixels.
     */
    int clahe_tile_size = 8;

    /*
     * Accepted contour area range in pixels.
     *
     * The area is calculated from the contour convex hull.
     */
    double min_area_px = 500.0;
    double max_area_px = 80000.0;

    /*
     * Minimum circularity required for a candidate.
     *
     * Circularity is estimated as:
     *
     *   hull_area / ideal_circle_area
     *
     * A higher value rejects elongated or irregular bright regions.
     */
    double min_circularity = 0.75;

    /*
     * Accepted enclosing-circle radius range in pixels.
     *
     * This is the primary size filter used to reject small bright objects
     * such as LEDs and excessively large regions.
     */
    double min_radius_px = 38.0;
    double max_radius_px = 150.0;

    /*
     * Enables diagnostic output for candidate regions and intermediate
     * grayscale values.
     */
    bool debug_print = false;

    /*
     * Circular region of interest used to restrict the search area.
     *
     * A radius of zero disables the ROI mask and processes the complete
     * image.
     */
    cv::Point2f roi_center_px{ 640.0f, 360.0f };
    float roi_radius_px = 0.0f;

    /*
     * Enables adaptive background subtraction.
     *
     * Background subtraction helps reject stationary bright objects that
     * satisfy the geometric ball filters but do not move like the ball.
     */
    bool use_background_subtraction = true;

    /*
     * EMA learning rate used to update the background model.
     *
     * Smaller values produce slower but more stable background adaptation.
     */
    double bg_learning_rate = 0.01;

    /*
     * Minimum mean grayscale difference between a candidate region and the
     * learned background.
     *
     * A candidate below this threshold is considered part of the background.
     */
    double bg_min_diff = 12.0;

    /*
     * Number of initial frames used to warm up the background model.
     *
     * Background filtering is enabled only after this period.
     */
    int bg_warmup_frames = 60;
};

class BallDetector
{
public:
    /*
     * Creates a detector from the camera calibration and camera-to-world
     * extrinsic parameters.
     *
     * camera_matrix and dist_coeffs describe the camera model.
     * rvec and tvec describe the calibrated camera pose relative to the
     * robot/table coordinate frame.
     */
    BallDetector(
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        const cv::Mat& rvec,
        const cv::Mat& tvec,
        BallDetectorConfig cfg = {})
        : camera_matrix_(camera_matrix),
        dist_coeffs_(dist_coeffs),
        rvec_(rvec),
        tvec_(tvec),
        cfg_(cfg)
    {
        cv::Rodrigues(
            rvec_,
            R_
        );

        Rinv_ =
            R_.t();

        cam_pos_world_ =
            -Rinv_ * tvec_;

        cam_z_ =
            cam_pos_world_.at<double>(2, 0);
    }

    /*
     * Detects the ball in a BGR frame.
     *
     * Returns true when a valid candidate is found and its position can be
     * projected onto the table plane.
     *
     * x_mm and y_mm are expressed in the robot/table coordinate frame.
     *
     * out_pixel and out_radius_px are optional outputs for visualization
     * and diagnostics.
     */
    bool detect(
        const cv::Mat& frame_bgr,
        double& x_mm,
        double& y_mm,
        cv::Point2f* out_pixel = nullptr,
        float* out_radius_px = nullptr) const
    {
        cv::Mat gray;

        cv::cvtColor(
            frame_bgr,
            gray,
            cv::COLOR_BGR2GRAY
        );

        /*
         * Restrict processing to the configured circular table region when
         * an ROI radius is provided.
         */
        if (cfg_.roi_radius_px > 0.0f)
        {
            cv::Mat mask =
                cv::Mat::zeros(
                    gray.size(),
                    CV_8UC1
                );

            cv::circle(
                mask,
                cfg_.roi_center_px,
                static_cast<int>(
                    cfg_.roi_radius_px
                    ),
                cv::Scalar(255),
                -1
            );

            cv::Mat masked;

            gray.copyTo(
                masked,
                mask
            );

            gray =
                masked;
        }

        /*
         * Improve local contrast before thresholding when enabled.
         *
         * This is useful when the ball is only slightly brighter than the
         * surrounding table surface.
         */
        if (cfg_.use_clahe)
        {
            cv::Ptr<cv::CLAHE> clahe =
                cv::createCLAHE(
                    cfg_.clahe_clip_limit,
                    cv::Size(
                        cfg_.clahe_tile_size,
                        cfg_.clahe_tile_size
                    )
                );

            clahe->apply(
                gray,
                gray
            );
        }

        if (cfg_.debug_print)
        {
            std::fprintf(
                stderr,
                "Gray level at ball sample (630,645) = %d (threshold=%.0f)\n",
                static_cast<int>(
                    gray.at<uchar>(645, 630)
                    ),
                cfg_.white_threshold
            );

            /*
             * Sample several image rows to inspect local grayscale values
             * around the expected ball region during detector tuning.
             */
            for (int y = 600; y <= 690; y += 5)
            {
                std::fprintf(
                    stderr,
                    "y=%d:",
                    y
                );

                for (int x = 650; x <= 900; x += 15)
                {
                    std::fprintf(
                        stderr,
                        " %d",
                        static_cast<int>(
                            gray.at<uchar>(y, x)
                            )
                    );
                }

                std::fprintf(
                    stderr,
                    "\n"
                );
            }
        }

        /*
         * Convert bright regions into a binary candidate mask.
         */
        cv::Mat binary;

        cv::threshold(
            gray,
            binary,
            cfg_.white_threshold,
            255,
            cv::THRESH_BINARY
        );

        /*
         * Remove small isolated regions and fill small gaps inside the
         * detected ball region.
         *
         * A 3x3 kernel is intentionally used to preserve small geometric
         * details in low-light conditions.
         */
        cv::Mat kernel =
            cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                { 3, 3 }
        );

        cv::morphologyEx(
            binary,
            binary,
            cv::MORPH_OPEN,
            kernel
        );

        cv::morphologyEx(
            binary,
            binary,
            cv::MORPH_CLOSE,
            kernel
        );

        if (cfg_.debug_print)
        {
            cv::imwrite(
                "/tmp/debug_binary.png",
                binary
            );
        }

        std::vector<
            std::vector<cv::Point>
        > contours;

        cv::findContours(
            binary,
            contours,
            cv::RETR_EXTERNAL,
            cv::CHAIN_APPROX_SIMPLE
        );

        /*
         * First-stage candidate filtering.
         *
         * Candidates are evaluated only by geometric properties here.
         * Background filtering is applied separately so that valid ball
         * regions are not accidentally incorporated into the initial
         * background model.
         */
        double best_area = -1.0;
        int best_idx = -1;

        std::vector<int>
            shape_plausible_idx;

        for (size_t i = 0; i < contours.size(); ++i)
        {
            std::vector<cv::Point> hull;

            cv::convexHull(
                contours[i],
                hull
            );

            double area =
                cv::contourArea(
                    hull
                );

            cv::Point2f c;
            float r;

            cv::minEnclosingCircle(
                contours[i],
                c,
                r
            );

            double ideal_area =
                CV_PI *
                r *
                r;

            if (ideal_area <= 0.0)
            {
                continue;
            }

            double circularity =
                area /
                ideal_area;

            if (
                cfg_.debug_print &&
                circularity > 0.5
                )
            {
                std::fprintf(
                    stderr,
                    "[ball_detector] Candidate at (%.0f,%.0f): "
                    "hull_area=%.0f circularity=%.2f radius=%.1f px\n",
                    c.x,
                    c.y,
                    area,
                    circularity,
                    r
                );
            }

            if (
                area < cfg_.min_area_px ||
                area > cfg_.max_area_px
                )
            {
                continue;
            }

            if (
                r < cfg_.min_radius_px ||
                r > cfg_.max_radius_px
                )
            {
                continue;
            }

            if (
                circularity <
                cfg_.min_circularity
                )
            {
                continue;
            }

            shape_plausible_idx.push_back(
                static_cast<int>(i)
            );
        }

        /*
         * Initialize or update the adaptive background model.
         *
         * During initial model creation, plausible bright regions are
         * inpainted from their surroundings instead of being copied into the
         * background. This prevents a stationary ball present in the first
         * frame from becoming permanently embedded in the learned background.
         */
        cv::Mat diff;
        bool diff_ready = false;

        if (cfg_.use_background_subtraction)
        {
            if (
                background_.empty() ||
                background_.size() != gray.size()
                )
            {
                if (!shape_plausible_idx.empty())
                {
                    cv::Mat inpaint_mask =
                        cv::Mat::zeros(
                            gray.size(),
                            CV_8UC1
                        );

                    for (int idx : shape_plausible_idx)
                    {
                        std::vector<cv::Point> hull;

                        cv::convexHull(
                            contours[idx],
                            hull
                        );

                        std::vector<
                            std::vector<cv::Point>
                        > hull_wrap{
                            hull
                        };

                        /*
                         * Expand the mask slightly so that the inpainted
                         * region does not retain a bright contour edge.
                         */
                        cv::fillPoly(
                            inpaint_mask,
                            hull_wrap,
                            cv::Scalar(255)
                        );
                    }

                    cv::dilate(
                        inpaint_mask,
                        inpaint_mask,
                        cv::getStructuringElement(
                            cv::MORPH_ELLIPSE,
                            { 9, 9 }
                        )
                    );

                    cv::Mat inpainted;

                    cv::inpaint(
                        gray,
                        inpaint_mask,
                        inpainted,
                        7,
                        cv::INPAINT_TELEA
                    );

                    inpainted.convertTo(
                        background_,
                        CV_32F
                    );
                }
                else
                {
                    gray.convertTo(
                        background_,
                        CV_32F
                    );
                }

                bg_frame_count_ =
                    0;
            }

            /*
             * Enable background comparison only after the configured warmup
             * period has completed.
             */
            if (
                bg_frame_count_ >=
                cfg_.bg_warmup_frames
                )
            {
                cv::Mat bg_8u;

                background_.convertTo(
                    bg_8u,
                    CV_8U
                );

                cv::absdiff(
                    gray,
                    bg_8u,
                    diff
                );

                diff_ready =
                    true;
            }
        }

        /*
         * Select the largest candidate that passes both geometric filtering
         * and the optional background-difference test.
         *
         * Convex hull area is used here to reduce the effect of small
         * brightness gaps or reflections along the ball boundary.
         */
        for (int idx : shape_plausible_idx)
        {
            std::vector<cv::Point> hull;

            cv::convexHull(
                contours[idx],
                hull
            );

            double area =
                cv::contourArea(
                    hull
                );

            if (diff_ready)
            {
                cv::Mat hull_mask =
                    cv::Mat::zeros(
                        gray.size(),
                        CV_8UC1
                    );

                std::vector<
                    std::vector<cv::Point>
                > hull_wrap{
                    hull
                };

                cv::fillPoly(
                    hull_mask,
                    hull_wrap,
                    cv::Scalar(255)
                );

                double mean_diff =
                    cv::mean(
                        diff,
                        hull_mask
                    )[0];

                if (cfg_.debug_print)
                {
                    std::fprintf(
                        stderr,
                        "[ball_detector] Candidate idx=%d: "
                        "mean_diff_vs_bg=%.1f (required >= %.1f)\n",
                        idx,
                        mean_diff,
                        cfg_.bg_min_diff
                    );
                }

                if (
                    mean_diff <
                    cfg_.bg_min_diff
                    )
                {
                    continue;
                }
            }

            if (area > best_area)
            {
                best_area =
                    area;

                best_idx =
                    idx;
            }
        }

        /*
         * Update the background using only regions that were not classified
         * as plausible foreground objects.
         *
         * The update is performed every frame, including frames where no
         * valid ball is returned.
         */
        if (cfg_.use_background_subtraction)
        {
            cv::Mat update_mask(
                gray.size(),
                CV_8UC1,
                cv::Scalar(255)
            );

            for (int idx : shape_plausible_idx)
            {
                std::vector<cv::Point> hull;

                cv::convexHull(
                    contours[idx],
                    hull
                );

                std::vector<
                    std::vector<cv::Point>
                > hull_wrap{
                    hull
                };

                cv::fillPoly(
                    update_mask,
                    hull_wrap,
                    cv::Scalar(0)
                );
            }

            cv::Mat gray_f;

            gray.convertTo(
                gray_f,
                CV_32F
            );

            cv::accumulateWeighted(
                gray_f,
                background_,
                cfg_.bg_learning_rate,
                update_mask
            );

            if (
                bg_frame_count_ <
                cfg_.bg_warmup_frames
                )
            {
                ++bg_frame_count_;
            }
        }

        if (best_idx < 0)
        {
            return false;
        }

        cv::Moments m =
            cv::moments(
                contours[best_idx]
            );

        if (m.m00 <= 0.0)
        {
            return false;
        }

        cv::Point2f centroid(
            static_cast<float>(
                m.m10 / m.m00
                ),
            static_cast<float>(
                m.m01 / m.m00
                )
        );

        if (
            !pixel_to_world_mm(
                centroid,
                x_mm,
                y_mm
            )
            )
        {
            return false;
        }

        if (out_pixel)
        {
            *out_pixel =
                centroid;
        }

        if (out_radius_px)
        {
            float r =
                std::sqrt(
                    static_cast<float>(
                        best_area /
                        CV_PI
                        )
                );

            *out_radius_px =
                r;
        }

        return true;
    }

private:
    /*
     * Converts an image pixel into its intersection with the table plane.
     *
     * The calibrated camera model is used to:
     *
     *   1. Remove lens distortion.
     *   2. Construct the corresponding camera-frame ray.
     *   3. Transform the ray into the world/robot frame.
     *   4. Intersect the ray with Z = 0.
     *
     * Returns false when the ray is parallel to the table plane.
     */
    bool pixel_to_world_mm(
        const cv::Point2f& px,
        double& x_mm,
        double& y_mm) const
    {
        std::vector<cv::Point2d> pixel_pts{
            {
                static_cast<double>(px.x),
                static_cast<double>(px.y)
            }
        };

        std::vector<cv::Point2d> norm_pts;

        cv::undistortPoints(
            pixel_pts,
            norm_pts,
            camera_matrix_,
            dist_coeffs_
        );

        cv::Mat ray_cam =
            (cv::Mat_<double>(3, 1)
                << norm_pts[0].x,
                norm_pts[0].y,
                1.0);

        cv::Mat ray_world =
            Rinv_ *
            ray_cam;

        double ray_z =
            ray_world.at<double>(
                2,
                0
                );

        if (
            std::abs(ray_z) <
            1e-9
            )
        {
            return false;
        }

        double s =
            -cam_z_ /
            ray_z;

        x_mm =
            cam_pos_world_.at<double>(
                0,
                0
                ) +
            s *
            ray_world.at<double>(
                0,
                0
                );

        y_mm =
            cam_pos_world_.at<double>(
                1,
                0
                ) +
            s *
            ray_world.at<double>(
                1,
                0
                );

        return true;
    }

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat rvec_;
    cv::Mat tvec_;

    cv::Mat R_;
    cv::Mat Rinv_;
    cv::Mat cam_pos_world_;

    double cam_z_ = 0.0;

    BallDetectorConfig cfg_;

    /*
     * Learned grayscale background used by the adaptive foreground filter.
     *
     * The detector keeps this state mutable so detect() can preserve its
     * const-facing API while updating the background model internally.
     *
     * background_ uses CV_32F and has the same dimensions as the processed
     * grayscale image.
     */
    mutable cv::Mat background_;

    mutable int bg_frame_count_ = 0;
};