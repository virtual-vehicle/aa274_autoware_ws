#include <amathutils_lib/amathutils.hpp>

namespace amathutils
{
geometry_msgs::Point getNearPtOnLine(const geometry_msgs::Point &_p, const geometry_msgs::Point &_a,
                                     const geometry_msgs::Point &_b)
{
  double len = find_distance(_a, _b);

  geometry_msgs::Point vnab;
  geometry_msgs::Point vap;
  geometry_msgs::Point ret;

  vnab.x = (_b.x - _a.x) / len;
  vnab.y = (_b.y - _a.y) / len;
  vnab.z = (_b.z - _a.z) / len;

  vap.x = _p.x - _a.x;
  vap.y = _p.y - _a.y;
  vap.z = _p.z - _a.z;

  double dist_ax = vnab.x * vap.x + vnab.y * vap.y + vnab.z * vap.z;

  ret.x = _a.x + (vnab.x * dist_ax);
  ret.y = _a.y + (vnab.y * dist_ax);
  ret.z = _a.z + (vnab.z * dist_ax);

  return ret;
}
double find_distance(const geometry_msgs::Point &_from, const geometry_msgs::Point &_to)
{
  return std::hypot(std::hypot(_from.x - _to.x, _from.y - _to.y), _from.z - _to.z);
}
double find_distance(const geometry_msgs::Pose &_from, const geometry_msgs::Pose &_to)
{
  return find_distance(_from.position, _to.position);
}

double find_angle(const geometry_msgs::Point &_from, const geometry_msgs::Point &_to)
{
  double _angle = std::atan2(_to.y - _from.y, _to.x - _from.x);
  if (_angle < 0.0)
    _angle = _angle + 2 * M_PI;

  return _angle * 360 / (2 * M_PI);
}

bool isIntersectLine(const geometry_msgs::Point &_l1_p1, const geometry_msgs::Point &_l1_p2,
                     const geometry_msgs::Point &_l2_p1, const geometry_msgs::Point &_l2_p2)
{
  const double ta = (_l2_p1.x - _l2_p2.x) * (_l1_p1.y - _l2_p1.y) + (_l2_p1.y - _l2_p2.y) * (_l2_p1.x - _l1_p1.x);
  const double tb = (_l2_p1.x - _l2_p2.x) * (_l1_p2.y - _l2_p1.y) + (_l2_p1.y - _l2_p2.y) * (_l2_p1.x - _l1_p2.x);
  const double tc = (_l1_p1.x - _l1_p2.x) * (_l2_p1.y - _l1_p1.y) + (_l1_p1.y - _l1_p2.y) * (_l1_p1.x - _l2_p1.x);
  const double td = (_l1_p1.x - _l1_p2.x) * (_l2_p2.y - _l1_p1.y) + (_l1_p1.y - _l1_p2.y) * (_l1_p1.x - _l2_p2.x);

  if (tc * td < 0 && ta * tb < 0)
    return true;
  else
    return false;
}

/*
 *        |
 *     .　|       =>LEFT = 1
 *     　 |   .   =>RIGHT = -1
 *      　|　
 */

int isPointLeftFromLine(const geometry_msgs::Point &_target, const geometry_msgs::Point &_line_p1,
                        const geometry_msgs::Point &_line_p2)
{
  const int LEFT = 1;
  const int RIGHT = -1;
  const int ONLINE = 0;
  const double n = _target.x * (_line_p1.y - _line_p2.y) + _line_p1.x * (_line_p2.y - _target.y) +
                   _line_p2.x * (_target.y - _line_p1.y);

  return n > 0 ? LEFT : n < 0 ? RIGHT : ONLINE;
}


//Following implementation comes from the website below:
//Author: Dan Sunday
//site: http://geomalgorithms.com/a02-_lines.html
//viewed: 2019/5/20
double distanceFromSegment( const geometry_msgs::Point &_l1, const geometry_msgs::Point &_l2, const geometry_msgs::Point &_p)
{
  geometry_msgs::Point v, w;
  v.x = _l2.x - _l1.x;
  v.y = _l2.y - _l1.y;
  v.z = _l2.z - _l1.z;
  
  w.x = _p.x - _l1.x;
  w.y = _p.y - _l1.y;
  w.z = _p.z - _l1.z;
  
  double dot_vw = v.x * w.x + v.y * w.y + v.z * w.z;
  double dot_vv = v.x * v.x + v.y * v.y + v.z * v.z;
  
  if ( dot_vw <= 0 )
  {
      return find_distance(_p, _l1);
  }
  if ( dot_vv <= dot_vw)
  {
      return find_distance(_p, _l2);
  }
  
  double b = dot_vw / dot_vv;
  geometry_msgs::Point pb;
  pb.x = _l1.x + b * v.x; 
  pb.y = _l1.y + b * v.y; 
  pb.z = _l1.z + b * v.z; 
  
  return find_distance(_p, pb);
}


double getPoseYawAngle(const geometry_msgs::Pose &_pose)
{
  double r, p, y;

  // get Quaternion
  tf::Quaternion quat(_pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w);
  // converted to RPY[-pi : pi]
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

geometry_msgs::Quaternion getQuaternionFromYaw(const double &_yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, _yaw);
  return tf2::toMsg(q);
}

double calcPosesAngleDiffRaw(const geometry_msgs::Pose &p_from, const geometry_msgs::Pose &_p_to)
{
  return getPoseYawAngle(p_from) - getPoseYawAngle(_p_to);
}

double normalizeRadian(const double _angle)
{
  double n_angle = std::fmod(_angle, 2 * M_PI);
  n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;

  // another way
  // Math.atan2(Math.sin(_angle), Math.cos(_angle));
  return n_angle;
}

double calcPosesAngleDiffDeg(const geometry_msgs::Pose &_p_from, const geometry_msgs::Pose &_p_to)
{
  // convert to [-pi : pi]
  return rad2deg(normalizeRadian(calcPosesAngleDiffRaw(_p_from, _p_to)));
}

double calcPosesAngleDiffRad(const geometry_msgs::Pose &_p_from, const geometry_msgs::Pose &_p_to)
{
  // convert to [-pi : pi]
  return normalizeRadian(calcPosesAngleDiffRaw(_p_from, _p_to));
}
}
