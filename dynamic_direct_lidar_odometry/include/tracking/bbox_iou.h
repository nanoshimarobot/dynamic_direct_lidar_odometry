#pragma once

// TODO rewrite this file, check Eigen dependencies

// Implementation adapted from
// https://stackoverflow.com/questions/44797713/calculate-the-area-of-
// intersection-of-two-rotated-rectangles-in-python

class Vector
{
public:
  float x, y;
  Vector(float, float);
  Vector operator+(const Vector& other)
  {
    return Vector(x + other.x, y + other.y);
  }
  Vector operator-(const Vector& other)
  {
    return Vector(x - other.x, y - other.y);
  }
  float cross(const Vector& other)
  {
    return x * other.y - y * other.x;
  }
};

Vector::Vector(float x_, float y_)
{
  x = x_;
  y = y_;
}

class Line
{
public:
  float a, b, c;
  Line(Vector&, Vector&);
  float call(const Vector& p)
  {
    return a * p.x + b * p.y + c;
  }
  Vector intersection(const Line& other)
  {
    float w = a * other.b - b * other.a;
    return Vector((b * other.c - c * other.b) / w, (c * other.a - a * other.c) / w);
  }
};

Line::Line(Vector& v1, Vector& v2)
{
  a = v2.y - v1.y;
  b = v1.x - v2.x;
  c = v2.cross(v1);
}

inline std::vector<Vector> rectangle_vertices(float cx, float cy, float w, float h, float r)
{
  // float angle = M_PI*r/180.0;
  float angle = r;
  float dx = w / 2.0;
  float dy = h / 2.0;
  float dxcos = dx * std::cos(angle);
  float dxsin = dx * std::sin(angle);
  float dycos = dy * std::cos(angle);
  float dysin = dy * std::sin(angle);
  std::vector<Vector> result_vec;
  result_vec.push_back(Vector(cx, cy) + Vector(-dxcos - -dysin, -dxsin + -dycos));
  result_vec.push_back(Vector(cx, cy) + Vector(dxcos - -dysin, dxsin + -dycos));
  result_vec.push_back(Vector(cx, cy) + Vector(dxcos - dysin, dxsin + dycos));
  result_vec.push_back(Vector(cx, cy) + Vector(-dxcos - dysin, -dxsin + dycos));
  return result_vec;
}

inline double intersection_area(float cx1, float cy1, float w1, float h1, float r1, float cx2, float cy2, float w2,
                                float h2, float r2)
{
  std::vector<Vector> rect1 = rectangle_vertices(cx1, cy1, w1, h1, r1);
  std::vector<Vector> rect2 = rectangle_vertices(cx2, cy2, w2, h2, r2);
  std::vector<Vector> intersection = rect1;

  for (int i = 0; i < rect2.size(); i++)
  {
    if (intersection.size() <= 2)
      break;
    Vector p = rect2[i];
    Vector q = rect2[(i + 1) % rect2.size()];
    Line line = Line(p, q);
    std::vector<Vector> new_intersection;
    std::vector<float> line_values;
    for (int j = 0; j < intersection.size(); j++)
    {
      line_values.push_back(line.call(intersection[j]));
    }
    for (int k = 0; k < intersection.size(); k++)
    {
      float s_value = line_values[k];
      float t_value = line_values[(k + 1) % intersection.size()];
      Vector s = intersection[k];
      Vector t = intersection[(k + 1) % intersection.size()];
      if (s_value <= 0.0)
      {
        new_intersection.push_back(s);
      }
      if (s_value * t_value < 0)
      {
        Vector intersection_point = line.intersection(Line(s, t));
        new_intersection.push_back(intersection_point);
      }
    }
    intersection = new_intersection;
  }

  if (intersection.size() <= 2)
  {
    return 0.0;
  }

  float area_sum = 0.0;
  for (int i = 0; i < intersection.size(); i++)
  {
    Vector p = intersection[i];
    Vector q = intersection[(i + 1) % intersection.size()];

    area_sum += p.x * q.y - p.y * q.x;
  }

  return 0.5 * area_sum;
}

inline double OBBIoU(Eigen::Matrix<double, 7, 1> bbox_1, Eigen::Matrix<double, 7, 1> bbox_2)
{
  Eigen::Matrix<float, 7, 1> bbox_1_f = bbox_1.cast<float>();
  Eigen::Matrix<float, 7, 1> bbox_2_f = bbox_2.cast<float>();

  double intersection = intersection_area(bbox_1_f(0), bbox_1_f(1), bbox_1_f(4), bbox_1_f(5), bbox_1_f(3), bbox_2_f(0),
                                          bbox_2_f(1), bbox_2_f(4), bbox_2_f(5), bbox_2_f(3));

  float min_1 = bbox_1_f(2) - bbox_1_f(6) / 2.0;
  float min_2 = bbox_2_f(2) - bbox_2_f(6) / 2.0;
  float max_1 = bbox_1_f(2) + bbox_1_f(6) / 2.0;
  float max_2 = bbox_2_f(2) + bbox_2_f(6) / 2.0;

  float max_min = std::max(min_1, min_2);
  float min_max = std::min(max_1, max_2);
  float height_overlap = std::max(min_max - max_min, 0.0f);

  float intersection_volume = height_overlap * intersection;

  float total_volume =
      bbox_1_f(4) * bbox_1_f(5) * bbox_1_f(6) + bbox_2_f(4) * bbox_2_f(5) * bbox_2_f(6) - intersection_volume;

  float iou = std::max(intersection_volume / total_volume, 0.0f);
  iou = std::min(iou, 1.0f);

  return iou;
}
