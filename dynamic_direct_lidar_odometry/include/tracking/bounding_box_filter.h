#include <odometry/ddlo.h>

#include <tracking/object.h>
#include <tracking/kalman.h>

typedef Eigen::Matrix<double, 7, 1> Vector7d;

class BoundingBoxFilter
{
public:
  BoundingBoxFilter(Object object, int id, double min_travel_dist, double res_height_ratio, size_t min_dynamic_hits,
                    size_t max_undefined_hits);
  BoundingBoxFilter();
  ~BoundingBoxFilter() = default;

  /*!
   * \brief predict Get the new state prediction from the kalman filter
   * \param dt time delta
   */
  void predict(double dt);

  /*!
   * \brief update Updates the estimation using the new detection
   * \param object New matched detection
   */
  void update(const Object& object, const std_msgs::Header& header);
  Object getObject();
  Eigen::VectorXd getFullState();
  double time();
  int getSSLU() const;
  int getFilterID() const;
  int getObjectID() const;
  int getHits() const;
  double getDistToOrigin() const;
  double getAvgResiduum();
  int getNumPoints() const;
  std::vector<int> getIndices() const;
  ObjectStatus getObjectStatus() const;
  std::string getObjectStatusString() const;
  bool getStaticBBoxes(jsk_recognition_msgs::BoundingBoxArray& bboxArray);

private:
  int n = 10;  // Const velocity w/o angular velocity [x,y,z,theta,l,w,h,vx,vy,vz]
  int m = 7;   // Size of detection: [x,y,z,theta,l,w,h]

  // A unique ID
  int filter_id_;

  // How many times this object was detected amd matched
  size_t hits_ = 1;
  size_t min_dynamic_hits_;
  size_t max_undefined_hits_;
  size_t steps_since_last_update_ = 0;

  // The distance an objects needs to dislocate from its first detected pose to
  // be declared dynamic
  double min_travel_dist_;

  double residuum_height_ratio_;

  // The filter that is used for predictions
  KalmanFilter kfilter_;

  // The object that this filter is tracking
  Object tracked_object_;

  // First and current pose
  geometry_msgs::PoseStamped first_pose_;
  geometry_msgs::PoseStamped current_pose_;

  // History of bounding boxes for static objects only
  jsk_recognition_msgs::BoundingBoxArray static_bboxes_;
  bool has_turned_dynamic_;

  void updateDynamicStatus();
  void updateBBoxHistory();
};
