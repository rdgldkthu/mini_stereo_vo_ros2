// mini_stereo_vo_ros2 — stereo_vo_node.cpp
#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "svo/camera.h"
#include "svo/estimator.h"
#include "svo/frame.h"
#include "svo/frontend.h"
#include "svo/map.h"
#include "svo/map_point.h"
#include "svo/stereo_initializer.h"
#include "svo/tracker.h"

namespace {

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

std::vector<cv::Point2f>
makeInitialActivePoints(const svo::StereoInitResult &r) {
  std::vector<cv::Point2f> pts;
  const size_t n = std::min(r.features.size(), r.landmarks.size());
  pts.reserve(n);
  for (size_t i = 0; i < n; ++i)
    pts.push_back(r.features[i].kp_left.pt);
  return pts;
}

std::vector<svo::MapPoint>
makeInitialActiveLandmarks(const svo::StereoInitResult &r) {
  std::vector<svo::MapPoint> lms;
  const size_t n = std::min(r.features.size(), r.landmarks.size());
  lms.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    svo::MapPoint lm = r.landmarks[i];
    lm.observed_times = 1;
    lm.tracked_times = 1;
    lm.missed_times = 0;
    lm.is_outlier = false;
    lm.is_active = true;
    lms.push_back(lm);
  }
  return lms;
}

Eigen::Matrix4d makePoseWcFromPnP(const Eigen::Matrix3d &R_cw,
                                  const Eigen::Vector3d &t_cw) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R_cw.transpose();
  T.block<3, 1>(0, 3) = -R_cw.transpose() * t_cw;
  return T;
}

void makePoseCwFromPoseWc(const Eigen::Matrix4d &T_wc,
                          Eigen::Matrix3d &R_cw, Eigen::Vector3d &t_cw) {
  const Eigen::Matrix3d R_wc = T_wc.block<3, 3>(0, 0);
  const Eigen::Vector3d t_wc = T_wc.block<3, 1>(0, 3);
  R_cw = R_wc.transpose();
  t_cw = -R_cw * t_wc;
}

void gatherPnPInliers(const std::vector<Eigen::Vector3d> &obj_pts,
                      const std::vector<cv::Point2f> &img_pts,
                      const std::vector<int> &inlier_idx,
                      std::vector<Eigen::Vector3d> &out_obj,
                      std::vector<cv::Point2f> &out_img) {
  out_obj.clear();
  out_img.clear();
  for (const int i : inlier_idx) {
    if (i < 0 || i >= static_cast<int>(obj_pts.size()))
      continue;
    out_obj.push_back(obj_pts[i]);
    out_img.push_back(img_pts[i]);
  }
}

std::vector<svo::MapPoint>
transformLandmarksToWorld(const std::vector<svo::MapPoint> &local,
                          const Eigen::Matrix4d &T_wc) {
  auto world = local;
  const Eigen::Matrix3d R = T_wc.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T_wc.block<3, 1>(0, 3);
  for (auto &lm : world)
    lm.p_w = R * lm.p_w + t;
  return world;
}

} // namespace

class StereoVONode : public rclcpp::Node {
public:
  StereoVONode()
      : Node("stereo_vo_node"),
        initializer_(makeStereoInitOptions()),
        tracker_(makeTrackerOptions()),
        estimator_(makeEstimatorOptions()),
        frontend_(makeFrontendOptions()),
        map_(makeMapOptions()),
        tf_broadcaster_(this) {
    declare_parameter("image_transport", std::string("raw"));
    declare_parameter("odom_frame", std::string("odom"));
    declare_parameter("base_frame", std::string("base_footprint"));
    declare_parameter("camera_frame", std::string("camera_link"));
    declare_parameter("publish_path", true);
    declare_parameter("reinit_on_failure", true);
    // Override baseline (metres) when the right camera_info P[3] is zero,
    // e.g. two independent monocular cameras in Gazebo simulation.
    declare_parameter("baseline_m", 0.0);

    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_path_ = get_parameter("publish_path").as_bool();
    reinit_on_failure_ = get_parameter("reinit_on_failure").as_bool();
    baseline_override_m_ = get_parameter("baseline_m").as_double();

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/vo/odometry", 10);
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vo/pose", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/vo/path", 10);

    left_sub_.subscribe(this, "/left/image_raw");
    right_sub_.subscribe(this, "/right/image_raw");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(5), left_sub_, right_sub_);
    sync_->registerCallback(
        std::bind(&StereoVONode::imageCb, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Use sensor-data QoS (best-effort, volatile) to match ros_gz_bridge output.
    // Transient-local would work with a true latched publisher but the bridge
    // publishes volatile, causing a QoS incompatibility warning.
    auto cam_info_qos = rclcpp::SensorDataQoS();
    left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/left/camera_info", cam_info_qos,
        [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          if (!left_info_) left_info_ = msg;
          tryInitCamera();
        });
    right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/right/camera_info", cam_info_qos,
        [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          if (!right_info_) right_info_ = msg;
          tryInitCamera();
        });

    path_msg_.header.frame_id = odom_frame_;
    RCLCPP_INFO(get_logger(), "Waiting for camera info on /left/camera_info and /right/camera_info ...");
  }

private:
  // ---------------------------------------------------------------------------
  // Camera setup
  // ---------------------------------------------------------------------------
  void tryInitCamera() {
    if (camera_ready_ || !left_info_ || !right_info_)
      return;

    camera_.fx = left_info_->k[0];
    camera_.fy = left_info_->k[4];
    camera_.cx = left_info_->k[2];
    camera_.cy = left_info_->k[5];
    // right CameraInfo P[3] = -fx * baseline  (Tx in pixels)
    if (baseline_override_m_ > 0.0) {
      camera_.baseline = baseline_override_m_;
    } else {
      // P[3] = -fx * baseline for the right camera in a standard stereo pair
      camera_.baseline = -right_info_->p[3] / camera_.fx;
    }

    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c) {
        camera_.P_left(r, c)  = left_info_->p[r * 4 + c];
        camera_.P_right(r, c) = right_info_->p[r * 4 + c];
      }

    camera_.print();
    camera_ready_ = true;
    RCLCPP_INFO(get_logger(),
                "Camera initialized: fx=%.2f fy=%.2f cx=%.2f cy=%.2f baseline=%.4f m",
                camera_.fx, camera_.fy, camera_.cx, camera_.cy, camera_.baseline);
  }

  // ---------------------------------------------------------------------------
  // Image sync callback
  // ---------------------------------------------------------------------------
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
               const sensor_msgs::msg::Image::ConstSharedPtr &right_msg) {
    if (!camera_ready_)
      return;

    cv_bridge::CvImageConstPtr left_cv, right_cv;
    try {
      left_cv  = cv_bridge::toCvShare(left_msg,  "mono8");
      right_cv = cv_bridge::toCvShare(right_msg, "mono8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    const double ts = left_msg->header.stamp.sec +
                      left_msg->header.stamp.nanosec * 1e-9;

    svo::Frame frame;
    frame.id        = frame_id_++;
    frame.timestamp = ts;
    frame.left_img  = left_cv->image.clone();
    frame.right_img = right_cv->image.clone();

    stamp_ = left_msg->header.stamp;
    processFrame(frame);
  }

  // ---------------------------------------------------------------------------
  // VO pipeline (mirrors run_kitti.cpp)
  // ---------------------------------------------------------------------------
  void processFrame(svo::Frame &curr_frame) {
    // First frame: stereo bootstrap
    if (!initialized_) {
      const svo::StereoInitResult init = initializer_.run(curr_frame, camera_, false);
      if (init.num_triangulated < 20) {
        RCLCPP_WARN(get_logger(), "Stereo init failed (%d landmarks). Retrying...",
                    init.num_triangulated);
        return;
      }

      auto active_lms = makeInitialActiveLandmarks(init);
      map_.assignNewLandmarkIds(active_lms);
      frontend_.initialize(curr_frame, makeInitialActivePoints(init), active_lms);

      curr_frame.pose_wc = Eigen::Matrix4d::Identity();
      curr_frame.is_keyframe = true;
      curr_frame.tracked_points = frontend_.activePoints();
      for (const auto &lm : active_lms)
        curr_frame.tracked_landmark_ids.push_back(lm.id);

      map_.addKeyframe(curr_frame);
      map_.setActiveLandmarks(active_lms);
      frontend_.setPreviousFrame(curr_frame);

      initialized_ = true;
      RCLCPP_INFO(get_logger(), "VO initialized with %d landmarks.", init.num_triangulated);
      publishPose(curr_frame.pose_wc);
      return;
    }

    // Track frame-to-frame
    const svo::TrackResult track =
        tracker_.trackFrameToFrame(frontend_.previousFrame(), curr_frame,
                                   frontend_.activePoints(),
                                   frontend_.activeLandmarks(), false);

    svo::FrontendFrameStats stats;

    // PnP + optional refinement
    if (track.num_valid_correspondences >= estimator_opts_.min_pnp_points) {
      Eigen::Matrix3d init_R_cw;
      Eigen::Vector3d init_t_cw;
      makePoseCwFromPoseWc(frontend_.currentPose(), init_R_cw, init_t_cw);

      const svo::PoseEstimateResult raw =
          estimator_.estimatePosePnPRansac(track.object_points, track.image_points,
                                           camera_, init_R_cw, init_t_cw, true);

      if (raw.success) {
        svo::PoseEstimateResult final_pose = raw;

        if (raw.num_inliers >= estimator_opts_.min_refine_inliers) {
          std::vector<Eigen::Vector3d> inlier_obj;
          std::vector<cv::Point2f> inlier_img;
          gatherPnPInliers(track.object_points, track.image_points,
                           raw.inlier_indices, inlier_obj, inlier_img);
          const svo::PoseEstimateResult refined =
              estimator_.refinePosePoseOnly(inlier_obj, inlier_img, camera_,
                                            raw.rotation, raw.translation);
          if (refined.success)
            final_pose = refined;
        }

        const Eigen::Matrix4d candidate =
            makePoseWcFromPnP(final_pose.rotation, final_pose.translation);
        frontend_.acceptPose(curr_frame.id, raw.num_inliers,
                             track.num_valid_correspondences, candidate, stats);
      } else {
        frontend_.rejectPose(curr_frame.id, track.num_valid_correspondences, stats);
      }
    } else {
      frontend_.rejectPose(curr_frame.id, track.num_valid_correspondences, stats);
    }

    // Reinitialization
    if (frontend_.shouldReinitialize(curr_frame.id, stats.pose_accepted,
                                     static_cast<int>(track.curr_points.size()))) {
      const svo::StereoInitResult reinit = initializer_.run(curr_frame, camera_, false);
      if (reinit.num_triangulated >= 20) {
        auto new_lms = makeInitialActiveLandmarks(reinit);
        map_.assignNewLandmarkIds(new_lms);
        new_lms = transformLandmarksToWorld(new_lms, frontend_.currentPose());
        frontend_.setActiveTracks(makeInitialActivePoints(reinit), new_lms);
        map_.setActiveLandmarks(new_lms);
        frontend_.noteReinitialized(curr_frame.id);
        stats.reinitialized = true;
        RCLCPP_WARN(get_logger(), "VO reinitialized at frame %d.", curr_frame.id);
      } else {
        frontend_.setActiveTracks(track.curr_points, track.tracked_landmarks);
      }
    } else {
      frontend_.setActiveTracks(track.curr_points, track.tracked_landmarks);
    }

    if (!stats.reinitialized) {
      map_.markTrackedLandmarks(track.tracked_landmarks);
      map_.markMissedLandmarks(track.landmark_ids);
      map_.pruneLandmarks();
    }

    // Keyframe insertion
    if (stats.pose_accepted) {
      curr_frame.pose_wc = frontend_.currentPose();
      curr_frame.tracked_points = frontend_.activePoints();
      for (const auto &lm : frontend_.activeLandmarks())
        curr_frame.tracked_landmark_ids.push_back(lm.id);

      if (frontend_.needNewKeyframe(curr_frame.pose_wc,
                                    static_cast<int>(frontend_.activePoints().size()),
                                    curr_frame.id)) {
        curr_frame.is_keyframe = true;
        const svo::StereoInitResult kf_init =
            initializer_.run(curr_frame, camera_, false);

        if (kf_init.num_triangulated >= 20) {
          auto new_lms = makeInitialActiveLandmarks(kf_init);
          map_.assignNewLandmarkIds(new_lms);
          new_lms = transformLandmarksToWorld(new_lms, curr_frame.pose_wc);

          const auto new_pts = makeInitialActivePoints(kf_init);
          curr_frame.tracked_points.insert(curr_frame.tracked_points.end(),
                                           new_pts.begin(), new_pts.end());
          for (const auto &lm : new_lms)
            curr_frame.tracked_landmark_ids.push_back(lm.id);

          map_.addKeyframe(curr_frame);
          map_.addLandmarks(new_lms);
          frontend_.setActiveTracks(new_pts, new_lms);
        } else {
          map_.addKeyframe(curr_frame);
        }

        frontend_.noteKeyframeInserted(curr_frame.id, curr_frame.pose_wc);
        stats.inserted_keyframe = true;

        // Local bundle adjustment
        if (map_.numActiveKeyframes() >= 3 &&
            map_.numActiveLandmarks() >= 20 &&
            frontend_.shouldRunLocalBA()) {
          auto kf_backup  = map_.activeKeyframes();
          auto lm_backup  = map_.activeLandmarks();

          svo::LocalBAResult ba = estimator_.runLocalBundleAdjustment(
              map_.mutableActiveKeyframes(), map_.mutableActiveLandmarks(), camera_);

          if (ba.success && ba.rmse_after > 0.0 && ba.rmse_after <= ba.rmse_before) {
            frontend_.refreshActiveLandmarksFromMap(map_.activeLandmarks());
            frontend_.noteLocalBaAccepted();
          } else {
            map_.mutableActiveKeyframes() = kf_backup;
            map_.mutableActiveLandmarks() = lm_backup;
          }
        }
      }
    }

    frontend_.setPreviousFrame(curr_frame);
    publishPose(frontend_.currentPose());
  }

  // ---------------------------------------------------------------------------
  // Publishing
  // ---------------------------------------------------------------------------
  void publishPose(const Eigen::Matrix4d &T_wc) {
    // The VO world frame is the camera optical frame at initialisation:
    //   cam-optical: Z forward, X right, Y down
    // ROS odom / base_link convention: X forward, Y left, Z up
    // This fixed rotation converts camera-optical coords to ROS body coords.
    // It matches the base_link → oakd_left_camera_optical_frame TF (transposed):
    //   rpy = (-90°, 0°, -90°) → R_opt_from_bl = [[0,0,1],[-1,0,0],[0,-1,0]]
    //   ⟹  R_odom_from_cam = R_opt_from_bl^T  = [[0,-1,0],[0,0,-1],[1,0,0]]
    // Equivalently (verified from TF at runtime):
    //   cam Z → odom X  (forward → forward)
    //   cam X → odom -Y (right   → -left  )
    //   cam Y → odom -Z (down    → -up    )
    static const Eigen::Matrix3d R_odom_from_cam =
        (Eigen::Matrix3d() <<  0,  0,  1,
                              -1,  0,  0,
                               0, -1,  0).finished();

    const Eigen::Vector3d t    = R_odom_from_cam * T_wc.block<3,1>(0,3);
    const Eigen::Quaterniond q(R_odom_from_cam * T_wc.block<3,3>(0,0) *
                                R_odom_from_cam.transpose());

    // Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp_;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;
    odom.pose.pose.position.x    = t.x();
    odom.pose.pose.position.y    = t.y();
    odom.pose.pose.position.z    = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    pub_odom_->publish(odom);

    // PoseStamped
    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.pose   = odom.pose.pose;
    pub_pose_->publish(ps);

    // Path
    if (publish_path_) {
      path_msg_.header.stamp = stamp_;
      path_msg_.poses.push_back(ps);
      pub_path_->publish(path_msg_);
    }

    // TF: odom → base_footprint
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = stamp_;
    tf.header.frame_id         = odom_frame_;
    tf.child_frame_id          = base_frame_;
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation.x    = q.x();
    tf.transform.rotation.y    = q.y();
    tf.transform.rotation.z    = q.z();
    tf.transform.rotation.w    = q.w();
    tf_broadcaster_.sendTransform(tf);
  }

  // ---------------------------------------------------------------------------
  // Module options (matching run_kitti.cpp defaults)
  // ---------------------------------------------------------------------------
  static svo::StereoInitializer::Options makeStereoInitOptions() {
    svo::StereoInitializer::Options o;
    o.max_features          = 1500;
    o.hamming_threshold     = 40;
    o.row_tolerance_px      = 2.0;
    o.min_disparity_px      = 3.0;
    o.max_disparity_px      = 120.0;
    o.max_depth_m           = 80.0;
    o.image_border_px       = 10;
    o.max_visualized_matches = 100;
    return o;
  }

  static svo::Tracker::Options makeTrackerOptions() {
    svo::Tracker::Options o;
    o.win_size                  = cv::Size(21, 21);
    o.max_level                 = 3;
    o.max_bidirectional_error_px = 1.5;
    o.image_border_px           = 10;
    o.max_visualized_tracks     = 150;
    return o;
  }

  static svo::Estimator::Options makeEstimatorOptions() {
    svo::Estimator::Options o;
    o.use_extrinsic_guess             = false;
    o.iterations_count                = 100;
    o.reprojection_error_px           = 4.0f;
    o.confidence                      = 0.99;
    o.min_pnp_points                  = 6;
    o.pose_refine_iterations          = 10;
    o.pose_refine_epsilon             = 1e-6;
    o.pose_refine_huber_delta         = 5.0;
    o.min_refine_inliers              = 10;
    o.local_ba_iterations             = 3;
    o.local_ba_epsilon                = 1e-6;
    o.local_ba_huber_delta            = 5.0;
    o.local_ba_damping                = 1e-3;
    o.max_ba_keyframes                = 3;
    o.max_ba_landmarks                = 100;
    o.min_ba_observations             = 20;
    o.min_ba_landmark_observations    = 2;
    return o;
  }

  static svo::Frontend::Options makeFrontendOptions() {
    svo::Frontend::Options o;
    o.keyframe_translation_threshold_m          = 1.5;
    o.keyframe_rotation_threshold_deg           = 12.0;
    o.keyframe_min_tracked_points               = 20;
    o.keyframe_min_frame_gap                    = 5;
    o.keyframe_low_track_translation_threshold_m = 0.5;
    o.min_pose_inliers                          = 8;
    o.min_pose_inlier_ratio                     = 0.10;
    o.max_frame_translation_m                   = 2.0;
    o.min_reinit_frame_gap                      = 10;
    o.weak_track_threshold                      = 30;
    o.emergency_rejected_poses_count            = 3;
    o.local_ba_keyframe_interval                = 2;
    return o;
  }

  static svo::Map::Options makeMapOptions() {
    svo::Map::Options o;
    o.max_active_keyframes  = 5;
    o.max_active_landmarks  = 2000;
    o.min_observed_times    = 2;
    o.max_missed_times      = 8;
    return o;
  }

  // ---------------------------------------------------------------------------
  // Members
  // ---------------------------------------------------------------------------
  svo::Camera             camera_;
  svo::StereoInitializer  initializer_;
  svo::Tracker            tracker_;
  svo::Estimator          estimator_;
  svo::Estimator::Options estimator_opts_ = makeEstimatorOptions();
  svo::Frontend           frontend_;
  svo::Map                map_;

  bool camera_ready_  = false;
  bool initialized_   = false;
  int  frame_id_      = 0;

  std::string odom_frame_;
  std::string base_frame_;
  bool        publish_path_;
  bool        reinit_on_failure_;
  double      baseline_override_m_  = 0.0;

  sensor_msgs::msg::CameraInfo::SharedPtr left_info_;
  sensor_msgs::msg::CameraInfo::SharedPtr right_info_;

  builtin_interfaces::msg::Time stamp_;
  nav_msgs::msg::Path           path_msg_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr         pub_path_;

  message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoVONode>());
  rclcpp::shutdown();
  return 0;
}
