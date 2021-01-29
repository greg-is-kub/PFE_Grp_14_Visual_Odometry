#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/surface_matching/icp.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
// #include "extra.h" // used in opencv2
using namespace std;
using namespace cv;

void find_feature_matches(
  const Mat &img_1, const Mat &img_2,
  std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,
  std::vector<DMatch> &matches);

void pose_estimation_2d2d(
  const std::vector<KeyPoint> &keypoints_1,
  const std::vector<KeyPoint> &keypoints_2,
  const std::vector<DMatch> &matches,
  Mat &R, Mat &t);

void triangulation(
  const vector<KeyPoint> &keypoint_1,
  const vector<KeyPoint> &keypoint_2,
  const std::vector<DMatch> &matches,
  const Mat &R, const Mat &t,
  vector<Point3d> &points
);

/// 作图用
inline cv::Scalar get_color(float depth) {
  float up_th = 50, low_th = 10, th_range = up_th - low_th;
  if (depth > up_th) depth = up_th;
  if (depth < low_th) depth = low_th;
  return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

// 像素坐标转相机归一化坐标
Point2f pixel2cam(const Point2d &p, const Mat &K);

int main(int argc, char **argv) {
  if (argc != 5) {
    cout << "usage: triangulation img1 img2" << endl;
    return 1;
  }
  //-- 读取图像
  Mat img_1 = imread(argv[1], 1);
  Mat img_2 = imread(argv[2], 1);
  Mat img_3 = imread(argv[3], 1);
  Mat img_4 = imread(argv[4], 1);
  vector<KeyPoint> keypoints_1, keypoints_2,keypoints_3, keypoints_4;
  vector<DMatch> matches1, matches2; 
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches1);
  find_feature_matches(img_3, img_4, keypoints_3, keypoints_4, matches2);
  cout << "一共找到了" << matches1.size() << "组匹配点" << endl;
  cout << "一共找到了" << matches2.size() << "组匹配点" << endl;


  //-- 估计两张图像间运动
  
  
  Mat R = (Mat_<double>(3, 3) << 1.00, 0.0133051171055671,
                                -0.00534470796279029, -0.0133694262332647, 0.999836410542635,
                                -0.0121823887399877,0.00518174551610382,
                                0.0122525920533588, 0.999911507835259);
  Mat t = (Mat_<double>(1, 3) << -50.28, 0.077, 0.45);
  //pose_estimation_2d2d(keypoints_1, keypoints_2, matches1, R, t);
  //pose_estimation_2d2d(keypoints_3, keypoints_4, matches2, R, t);
  //-- 三角化
  vector<Point3d> points1,points2;
  triangulation(keypoints_1, keypoints_2, matches1, R, t, points1);
  triangulation(keypoints_3, keypoints_4, matches2, R, t, points2);
  cv::ppf_match_3d::ICP iterativeClosestPoint;
  double error;
  cv::Matx44d transformation;
  
  Mat points11 = Mat(points1, CV_32F).reshape(1);
  Mat points22 = Mat(points2, CV_32F).reshape(1);
  
  Mat p1, p2;
  Vec3f f = Vec3f(0,0,0);
  cv::ppf_match_3d::computeNormalsPC3d(points11, p1, 12, 0, f);
  cv::ppf_match_3d::computeNormalsPC3d(points22, p2, 12, 0, f);
  
  cout << "一共找到了" << p1.cols << "组匹配点" << endl;
  
  int mini = min(p1.cols, p2.cols);
  	
  points11=points11(Range(0,mini), Range::all());
  points22=points22(Range(0,mini), Range::all());
  
  
  
  


  iterativeClosestPoint.registerModelToScene(p1,p2, error, transformation);
  cout << transformation << endl;
  //-- 验证三角化点与特征点的重投影关系
  
  

  return 0;
}

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches) {
  //-- 初始化
  Mat descriptors_1, descriptors_2;
  // used in OpenCV3
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  // use this if you are in OpenCV2
  // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
  // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  //-- 第一步:检测 Oriented FAST 角点位置
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);

  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  vector<DMatch> match;
  // BFMatcher matcher ( NORM_HAMMING );
  matcher->match(descriptors_1, descriptors_2, match);

  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;

  //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors_1.rows; i++) {
    double dist = match[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  for (int i = 0; i < descriptors_1.rows; i++) {
    if (match[i].distance <= max(2 * min_dist, 30.0)) {
      matches.push_back(match[i]);
    }
  }
}



void triangulation(
  const vector<KeyPoint> &keypoint_1,
  const vector<KeyPoint> &keypoint_2,
  const std::vector<DMatch> &matches,
  const Mat &R, const Mat &t,
  vector<Point3d> &points) {
  Mat T1 = (Mat_<float>(3, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0);
  Mat T2 = (Mat_<float>(3, 4) <<
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
  );

  Mat K_d = (Mat_<double>(3, 3) << 1535.50, 0, 739.86, 0, 1535.00, 606.36, 0, 0, 1);
  Mat K_g = (Mat_<double>(3, 3) << 1534.72, 0, 676.97, 0, 1535.00, 698.28, 0, 0, 1);
  vector<Point2f> pts_1, pts_2;
  for (DMatch m:matches) {
    // 将像素坐标转换至相机坐标
    pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K_g));
    pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K_d));
  }

  Mat pts_4d;
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

  // 转换成非齐次坐标
  for (int i = 0; i < pts_4d.cols; i++) {
    Mat x = pts_4d.col(i);
    x /= x.at<float>(3, 0); // 归一化
    Point3d p(
      x.at<float>(0, 0),
      x.at<float>(1, 0),
      x.at<float>(2, 0)
    );
    points.push_back(p);
  }
}

Point2f pixel2cam(const Point2d &p, const Mat &K) {
  return Point2f
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

