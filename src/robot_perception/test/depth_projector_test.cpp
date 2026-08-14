#include "robot_perception/depth_projector.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace {

sensor_msgs::msg::Image DepthImage(
    const std::uint32_t width, const std::uint32_t height,
    const std::vector<float>& values) {
  sensor_msgs::msg::Image image;
  image.width = width;
  image.height = height;
  image.encoding = "32FC1";
  image.is_bigendian = 0U;
  image.step = width * sizeof(float);
  image.data.resize(values.size() * sizeof(float));
  std::memcpy(image.data.data(), values.data(), image.data.size());
  return image;
}

sensor_msgs::msg::CameraInfo CameraInfo(
    const std::uint32_t width = 5U, const std::uint32_t height = 5U,
    const double fx = 100.0, const double fy = 100.0,
    const double cx = 2.0, const double cy = 2.0) {
  sensor_msgs::msg::CameraInfo info;
  info.width = width;
  info.height = height;
  info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  return info;
}

robot_perception::BoundingBox2D FullBox(const double center_u = 2.0,
                                        const double center_v = 2.0) {
  return {center_u, center_v, 4.0, 4.0};
}

robot_perception::DepthSamplingConfig Config(
    const double roi_ratio = 1.0, const std::size_t minimum_samples = 1U) {
  return {0.2, 8.0, roi_ratio, minimum_samples};
}

TEST(DepthProjectorTest, ValidDepthMedian) {
  robot_perception::DepthProjector projector(Config());
  const auto image = DepthImage(3U, 1U, {1.0F, 3.0F, 2.0F});
  const auto info = CameraInfo(3U, 1U, 100.0, 100.0, 1.0, 0.0);
  const robot_perception::BoundingBox2D bbox{1.0, 0.0, 2.0, 1.0};

  const auto result = projector.Project(bbox, image, info);

  ASSERT_TRUE(result.valid());
  EXPECT_DOUBLE_EQ(result.depth, 2.0);
  EXPECT_EQ(result.valid_sample_count, 3U);
}

TEST(DepthProjectorTest, RejectsZeroDepth) {
  robot_perception::DepthProjector projector(Config(1.0, 2U));
  const auto result = projector.Project(
      FullBox(), DepthImage(5U, 5U, std::vector<float>(25U, 0.0F)), CameraInfo());

  EXPECT_EQ(result.status, robot_perception::ProjectionStatus::kInsufficientValidDepth);
  EXPECT_EQ(result.valid_sample_count, 0U);
}

TEST(DepthProjectorTest, RejectsNanDepth) {
  robot_perception::DepthProjector projector(Config(1.0, 2U));
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const auto result = projector.Project(
      FullBox(), DepthImage(5U, 5U, std::vector<float>(25U, nan)), CameraInfo());

  EXPECT_EQ(result.status, robot_perception::ProjectionStatus::kInsufficientValidDepth);
  EXPECT_EQ(result.valid_sample_count, 0U);
}

TEST(DepthProjectorTest, RejectsInfiniteDepth) {
  robot_perception::DepthProjector projector(Config(1.0, 2U));
  const auto infinity = std::numeric_limits<float>::infinity();
  const auto result = projector.Project(
      FullBox(), DepthImage(5U, 5U, std::vector<float>(25U, infinity)), CameraInfo());

  EXPECT_EQ(result.status, robot_perception::ProjectionStatus::kInsufficientValidDepth);
  EXPECT_EQ(result.valid_sample_count, 0U);
}

TEST(DepthProjectorTest, RejectsDepthOutsideConfiguredRange) {
  robot_perception::DepthProjector projector(Config(1.0, 2U));
  std::vector<float> values(25U, 0.1F);
  values[12] = 8.1F;
  const auto result = projector.Project(FullBox(), DepthImage(5U, 5U, values), CameraInfo());

  EXPECT_EQ(result.status, robot_perception::ProjectionStatus::kInsufficientValidDepth);
  EXPECT_EQ(result.valid_sample_count, 0U);
}

TEST(DepthProjectorTest, SamplesOnlyCentralRoi) {
  robot_perception::DepthProjector projector(Config(0.3, 1U));
  std::vector<float> values(49U, 7.0F);
  values[3U * 7U + 3U] = 2.0F;
  const auto image = DepthImage(7U, 7U, values);
  const auto info = CameraInfo(7U, 7U, 100.0, 100.0, 3.0, 3.0);
  const robot_perception::BoundingBox2D bbox{3.0, 3.0, 6.0, 6.0};

  const auto result = projector.Project(bbox, image, info);

  ASSERT_TRUE(result.valid());
  EXPECT_DOUBLE_EQ(result.depth, 2.0);
  EXPECT_EQ(result.valid_sample_count, 1U);
}

TEST(DepthProjectorTest, ProjectsValidPixelIntoCameraFrame) {
  robot_perception::DepthProjector projector(Config(0.3, 1U));
  const auto image = DepthImage(5U, 5U, std::vector<float>(25U, 2.0F));
  const auto result = projector.Project(FullBox(3.0, 1.0), image, CameraInfo());

  ASSERT_TRUE(result.valid());
  EXPECT_NEAR(result.point.x, 0.02, 1e-9);
  EXPECT_NEAR(result.point.y, -0.02, 1e-9);
  EXPECT_NEAR(result.point.z, 2.0, 1e-9);
}

TEST(DepthProjectorTest, RejectsInvalidIntrinsics) {
  robot_perception::DepthProjector projector(Config(0.3, 1U));
  const auto image = DepthImage(5U, 5U, std::vector<float>(25U, 2.0F));

  const auto result = projector.Project(FullBox(), image, CameraInfo(5U, 5U, 0.0));

  EXPECT_EQ(result.status, robot_perception::ProjectionStatus::kInvalidIntrinsics);
}

TEST(DepthProjectorTest, ProjectsKnownNumericalResult) {
  robot_perception::DepthProjector projector(Config(0.3, 1U));
  const auto image = DepthImage(5U, 5U, std::vector<float>(25U, 4.0F));
  const auto info = CameraInfo(5U, 5U, 200.0, 100.0, 1.0, 1.0);

  const auto result = projector.Project(FullBox(3.0, 2.0), image, info);

  ASSERT_TRUE(result.valid());
  EXPECT_NEAR(result.point.x, 0.04, 1e-9);
  EXPECT_NEAR(result.point.y, 0.04, 1e-9);
  EXPECT_NEAR(result.point.z, 4.0, 1e-9);
}

TEST(DepthProjectorTest, RejectsInsufficientValidSamples) {
  robot_perception::DepthProjector projector(Config(1.0, 3U));
  std::vector<float> values(25U, 0.0F);
  values[12] = 2.0F;
  values[13] = 2.1F;

  const auto result = projector.Project(FullBox(), DepthImage(5U, 5U, values), CameraInfo());

  EXPECT_EQ(result.status, robot_perception::ProjectionStatus::kInsufficientValidDepth);
  EXPECT_EQ(result.valid_sample_count, 2U);
}

TEST(DepthProjectorTest, RejectsUnsupportedEncoding) {
  robot_perception::DepthProjector projector(Config());
  auto image = DepthImage(5U, 5U, std::vector<float>(25U, 2.0F));
  image.encoding = "16UC1";

  const auto result = projector.Project(FullBox(), image, CameraInfo());

  EXPECT_EQ(result.status, robot_perception::ProjectionStatus::kUnsupportedDepthEncoding);
}

}  // namespace
