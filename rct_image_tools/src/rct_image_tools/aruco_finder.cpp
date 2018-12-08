#include "rct_image_tools/aruco_finder.h".h"

rct_image_tools::ArucoGridBoardObservationFinder::ArucoGridBoardObservationFinder(const cv::aruco::GridBoard& board)
    : board_(board)
{

}

boost::optional<std::map<int, std::vector<Eigen::Vector2d>>>
rct_image_tools::ArucoGridBoardObservationFinder::findObservations(const cv::Mat &image) const
{
  std::map<int, std::vector<Eigen::Vector2d>> map_ids_to_obs_corners;

  std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
  std::vector<int> marker_ids;
  cv::Ptr<cv::aruco::DetectorParameters> parameters(new cv::aruco::DetectorParameters);

  cv::aruco::detectMarkers(image, board_.dictionary, marker_corners, marker_ids, parameters, rejected_candidates);

  for(int i = 0; i < marker_ids.size(); i++)
  {
    std::vector<cv::Point2f> corner_pts = marker_corners[i];

    std::vector<Eigen::Vector2d> obs_pts(4);
    for (int j = 0; j < corner_pts.size(); j++)
    {
      obs_pts[j] << corner_pts[j].x, corner_pts[j].y;
    }
    map_ids_to_obs_corners.insert(std::make_pair(marker_ids[i], obs_pts));
  }
  return map_ids_to_obs_corners;
}

//cv::Mat
//rct_image_tools::ArucoGridBoardObservationFinder::drawObservations(const cv::Mat& image,
//                                                                   const std::map<int, std::vector<Eigen::Vector2d>>& observations) const
//{
//  ObservationPoints cv_obs(observations.size());
//  std::transform(observations.begin(), observations.end(), cv_obs.begin(),
//                 [](const Eigen::Vector2d& o) { return cv::Point2d(o.x(), o.y()); });
//  return renderObservations(image, cv_obs, target_);
//}
