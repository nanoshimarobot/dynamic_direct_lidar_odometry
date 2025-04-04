#include <tracking/bounding_box_filter.h>

BoundingBoxFilter::BoundingBoxFilter(Object object, int id, double min_travel_dist, double res_height_ratio,
                                     size_t min_dynamic_hits, size_t max_undefined_hits)
  : filter_id_(id)
  , tracked_object_(object)
  , min_travel_dist_(min_travel_dist)
  , residuum_height_ratio_(res_height_ratio)
  , min_dynamic_hits_(min_dynamic_hits)
  , max_undefined_hits_(max_undefined_hits)
  , has_turned_dynamic_(false)
{
  double dt = 0.0;  // Time step

  Eigen::MatrixXd A(n, n);  // System dynamics matrix
  Eigen::MatrixXd C(m, n);  // Output matrix
  Eigen::MatrixXd Q(n, n);  // Process noise covariance
  Eigen::MatrixXd R(m, m);  // Measurement noise covariance
  Eigen::MatrixXd P(n, n);  // Estimate error covariance

  A << 1, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

  C << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  P << 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 10000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10000;

  R << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1;

  Q << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, .01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, .01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, .01;

  Eigen::VectorXd x0(n);
  x0 << tracked_object_.state.head<7>(), 0, 0, 0;
  kfilter_ = KalmanFilter(dt, A, C, Q, R, P);
  kfilter_.init(0.0, x0);

  first_pose_.pose.position.x = x0[0];
  first_pose_.pose.position.y = x0[1];
  first_pose_.pose.position.z = x0[2];
  first_pose_.pose.orientation.x = x0[3];
  first_pose_.pose.orientation.y = x0[4];
  first_pose_.pose.orientation.z = x0[5];
  first_pose_.pose.orientation.w = x0[6];
}

void BoundingBoxFilter::predict(double dt)
{
  Eigen::MatrixXd new_A(n, n);
  new_A << 1, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

  kfilter_.predict(dt, new_A);
  steps_since_last_update_++;
}

void BoundingBoxFilter::update(const Object& object, const std_msgs::Header& header)
{
  hits_++;
  steps_since_last_update_ = 0;
  ObjectStatus old_status = tracked_object_.status;
  tracked_object_ = object;
  tracked_object_.status = old_status;

  current_pose_.header = header;
  current_pose_.pose.position.x = tracked_object_.state[0];
  current_pose_.pose.position.y = tracked_object_.state[1];
  current_pose_.pose.position.z = tracked_object_.state[2] - tracked_object_.state[6] / 2;  // bottom of bbox
  current_pose_.pose.orientation.x = 0;
  current_pose_.pose.orientation.y = 0;
  current_pose_.pose.orientation.z = 0;
  current_pose_.pose.orientation.w = 1;

  updateDynamicStatus();
  updateBBoxHistory();

  kfilter_.update(tracked_object_.state);
}

Object BoundingBoxFilter::getObject()
{
  tracked_object_.state = kfilter_.state();
  return tracked_object_;
}

Eigen::VectorXd BoundingBoxFilter::getFullState()
{
  return kfilter_.state();
}

double BoundingBoxFilter::time()
{
  return kfilter_.time();
}

int BoundingBoxFilter::getSSLU() const
{
  return steps_since_last_update_;
}

int BoundingBoxFilter::getFilterID() const
{
  return filter_id_;
}

int BoundingBoxFilter::getObjectID() const
{
  return tracked_object_.id;
}

int BoundingBoxFilter::getHits() const
{
  return hits_;
}

double BoundingBoxFilter::getAvgResiduum()
{
  return tracked_object_.avg_residuum;
}

int BoundingBoxFilter::getNumPoints() const
{
  return tracked_object_.num_points;
}

std::vector<int> BoundingBoxFilter::getIndices() const
{
  return tracked_object_.indices;
}

ObjectStatus BoundingBoxFilter::getObjectStatus() const
{
  return tracked_object_.status;
}

std::string BoundingBoxFilter::getObjectStatusString() const
{
  switch (tracked_object_.status)
  {
    case UNDEFINED:
      return "UNDEFINED";
    case STATIC:
      return "STATIC";
    case DYNAMIC:
      return "DYNAMIC";
  }
  return "UNDEFINED";
}

bool BoundingBoxFilter::getStaticBBoxes(jsk_recognition_msgs::BoundingBoxArray& bboxArray)
{
  if (has_turned_dynamic_)
  {
    bboxArray = static_bboxes_;
    has_turned_dynamic_ = false;
    static_bboxes_.boxes.clear();
    return true;
  }
  return false;
}

void BoundingBoxFilter::updateDynamicStatus()
{
  // std::cout << "hits_: " << hits_ << std::endl;
  switch (tracked_object_.status)
  {
    case DYNAMIC:
      // Once an object is dynamic, it cannot lose this status
      has_turned_dynamic_ = false;
      break;
    case UNDEFINED:
      std::cout << "hits_: " << hits_ << " , maxundfined_hits : " << max_undefined_hits_ << std::endl;
      if (hits_ > max_undefined_hits_)
      {
        tracked_object_.status = STATIC;
        break;
      }
      if (hits_ < min_dynamic_hits_)  // Not enough hits to be declared dynamic
        break;
      // continue with STATIC check otherwise (direct transition to DYNAMIC possible)
    case STATIC:  // transition to DYNAMIC possible
      // Check avg residuum
      std::cout << "avg_residuum: " << tracked_object_.avg_residuum << std::endl;
      float min_required_residuum = tracked_object_.state[6] * residuum_height_ratio_;
      if (tracked_object_.avg_residuum < min_required_residuum)
        return;

      // Check traj length (offset from origin, only x-y)
      auto x0 = first_pose_.pose.position.x;
      auto y0 = first_pose_.pose.position.y;
      //  auto z0 = trajectory_[0].pose.position.z;
      auto x = current_pose_.pose.position.x;
      auto y = current_pose_.pose.position.y;
      //  auto z = trajectory_.back().pose.position.z;
      auto d_squared = (x - x0) * (x - x0) + (y - y0) * (y - y0);  // + (z - z0) * (z - z0);

      // avoid computing the actual square root

      // ROS_INFO("d_squared: %f", d_squared);
      std::cout << "d_squared: " << d_squared << std::endl;
      if (d_squared >= min_travel_dist_ * min_travel_dist_)
      {
        tracked_object_.status = DYNAMIC;
        has_turned_dynamic_ =
            !static_bboxes_.boxes.empty();  // From static to dynamic. Only true if we have at least one static position
      }
      break;
  }
  return;
}

void BoundingBoxFilter::updateBBoxHistory()
{
  if (tracked_object_.status == STATIC)
  {
    jsk_recognition_msgs::BoundingBox bbox;
    bbox.pose.position.x = tracked_object_.state[0];
    bbox.pose.position.y = tracked_object_.state[1];
    bbox.pose.position.z = tracked_object_.state[2];
    bbox.pose.orientation.x = 0;
    bbox.pose.orientation.y = 0;
    bbox.pose.orientation.z = tracked_object_.state[3];
    bbox.pose.orientation.w = std::cos(std::asin(tracked_object_.state[3]));  // = sqrt(1-x**2)
    bbox.dimensions.x = tracked_object_.state[4];
    bbox.dimensions.y = tracked_object_.state[5];
    bbox.dimensions.z = tracked_object_.state[6];

    static_bboxes_.boxes.push_back(bbox);

    // Keep a rolling window of size five
    if (static_bboxes_.boxes.size() > 5)
    {
      static_bboxes_.boxes.erase(static_bboxes_.boxes.begin());
    }
  }
}

double BoundingBoxFilter::getDistToOrigin() const
{
  if (hits_ < 2)
    return 0;
  auto p0 = first_pose_.pose.position;
  auto p = current_pose_.pose.position;
  return std::sqrt((p.x - p0.x) * (p.x - p0.x) + (p.y - p0.y) * (p.y - p0.y) + (p.z - p0.z) * (p.z - p0.z));
}
