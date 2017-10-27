#pragma once

#include <opencv2/core.hpp>

#include <vector>

void
drawImageShapes(cv::Mat &image, std::vector<std::vector<cv::Point2i>> const &imageShapes);

void
drawImageGrid(cv::Mat &image, std::vector<std::vector<int>> const &imageGrid, int imageCountTarget);

void
drawMessage(cv::Mat &image, std::string const &message, cv::Scalar color);
