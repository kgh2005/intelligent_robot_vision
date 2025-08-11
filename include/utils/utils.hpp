#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> 

#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

// 점들을 모은 뒤 임의의 두 점으로 가설 직선을 만들고, 
// 그 직선에 가까운 점(임계값 이하)을 인라이어로 세며 
// 이 과정을 여러 번 반복해 인라이어가 가장 많은 가설을 고름

const int MAX_ITER = 100;          // RANSAC 반복 횟수
const double DIST_THRESHOLD = 2.0; // 점이 직선에 얼마나 가까우면 인라이어로 볼지
const int MIN_INLIERS = 1;         // 최소 인라이어 수

// 최소제곱 직선 피팅
cv::Vec4f fitLine(const std::vector<cv::Point2f> &points)
{
  cv::Vec4f line;
  cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
  return line;
}

// 점-직선 거리
double distanceToLine(cv::Point2f point, cv::Point2f lineStart, cv::Point2f lineEnd)
{
  double num = std::abs((lineEnd.y - lineStart.y) * point.x - (lineEnd.x - lineStart.x) * point.y + lineEnd.x * lineStart.y - lineEnd.y * lineStart.x);
  double den = std::sqrt(std::pow(lineEnd.y - lineStart.y, 2) + std::pow(lineEnd.x - lineStart.x, 2));
  return num / den;
}

// RANSAC 직선 추정
cv::Vec4f ransacLine(const std::vector<cv::Point2f> &points)
{
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<int> uni(0, points.size() - 1);

  int bestInliers = 0;
  cv::Vec4f bestLine;

  for (int i = 0; i < MAX_ITER; ++i)
  {
    // 임의 두 점으로 가설 직선
    cv::Point2f p1 = points[uni(rng)];
    cv::Point2f p2 = points[uni(rng)];

    // 인라이어 수집
    std::vector<cv::Point2f> inliers;
    for (const auto &point : points)
    {
      if (distanceToLine(point, p1, p2) < DIST_THRESHOLD)
      {
        inliers.push_back(point);
      }
    }

    // 인라이어 최대 갱신 시 최소제곱 재피팅
    if (inliers.size() > bestInliers && inliers.size() >= MIN_INLIERS)
    {
      bestInliers = inliers.size();
      bestLine = fitLine(inliers);
    }
  }

  return bestLine;
}