#pragma once

#include <opencv2/core.hpp>

#include <vector>

/**
 * @brief Draws the Shapes of multiple polygons.
 * @param image Destination image
 * @param imageShapes vector of polygon points
 */
void
drawImageShapes(cv::Mat &image, std::vector<std::vector<cv::Point2i>> const &imageShapes);

/**
 * @brief Draws the image grid
 * @param image Destination image
 * @param imageGrid 2D grid with image counts
 * @param imageCountTarget Amount of images that should be in each grid cell
 */
void
drawImageGrid(cv::Mat &image, std::vector<std::vector<int>> const &imageGrid, int imageCountTarget);

/**
 * @brief Draws a message.
 * @param image Destination image
 * @param message Message
 * @param color Text color
 */
void
drawMessage(cv::Mat &image, std::string const &message, cv::Scalar color);
