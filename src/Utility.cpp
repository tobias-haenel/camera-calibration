#include "Utility.h"

#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

void
drawImageShapes(Mat &image, const vector<vector<Point2i>> &imageShapes) {
    Mat imageOverlay = image.clone();
    for (auto const &shape : imageShapes) {
        fillConvexPoly(imageOverlay, shape, Scalar(255, 255, 255));
    }
    addWeighted(image, 0.8, imageOverlay, 0.2, 0.0, image);
}

void
drawImageGrid(Mat &image, const vector<vector<int>> &imageGrid, int imageCountTarget) {
    Mat imageOverlay = image.clone();
    for (size_t i = 0; i < imageGrid.size(); ++i) {
        for (size_t j = 0; j < imageGrid[i].size(); ++j) {
            int x1 = static_cast<int>(i * (image.cols * 1.0 / imageGrid.size())) + 1;
            int x2 = static_cast<int>((i + 1) * (image.cols * 1.0 / imageGrid.size())) - 1;
            int y1 = static_cast<int>(j * (image.rows * 1.0 / imageGrid[i].size())) + 1;
            int y2 = static_cast<int>((j + 1) * (image.rows * 1.0 / imageGrid[i].size())) - 1;
            Point topLeft{x1, y1};
            Point bottomRight{x2, y2};
            int imageCount = imageGrid[i][j];
            double targetFrac = static_cast<double>(imageCount) / imageCountTarget;
            Scalar color = Scalar(0, 255, 0) * targetFrac + Scalar(0, 0, 255) * (1 - targetFrac);
            rectangle(imageOverlay, topLeft, bottomRight, color, 2);
        }
    }
    addWeighted(image, 0.5, imageOverlay, 0.5, 0.0, image);
}

void
drawMessage(Mat &image, const string &message, Scalar color) {
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.2;
    int thickness = 2;
    int baseLine = 0;
    Size textSize = getTextSize(message, fontFace, fontScale, thickness, &baseLine);
    baseLine += thickness;
    Point textOrigin((image.cols - textSize.width) / 2, static_cast<int>(image.rows * 0.95));
    putText(image, message, textOrigin, fontFace, fontScale, color, thickness, LINE_AA);
}
