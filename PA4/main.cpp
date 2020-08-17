#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// tries to achieve anti-aliasing on bezier curve
void set_pixel(cv::Point2f &point, cv::Mat &window){
    int row = point.y;
    int col = point.x;
    float d0 = sqrt((point.y - row - 0.5) * (point.y - row - 0.5)
                    + (point.x - col - 0.5) * (point.x - col - 0.5));
    typedef std::pair<int, int> pii;
    pii drow, dcol;
    drow = (point.y - row < 0.5) ? pii(-1, 0) : pii(0, 1);
    dcol = (point.x - col < 0.5) ? pii(-1, 0) : pii(0, 1);
    for(int i = drow.first; i <= drow.second; ++ i){
        for(int j = dcol.first; j <= dcol.second; ++ j){
            int nrow = row + i;
            int ncol = col + j;
            if(nrow < 0 || ncol < 0 || nrow >= window.rows || ncol >= window.cols){
                continue;
            }
            float dist = sqrt((point.y - nrow - 0.5) * (point.y - nrow - 0.5)
                              + (point.x - ncol - 0.5) * (point.x - ncol - 0.5));
            float color = 0;
            if(dist < 1e-7){
                color = 1;
            }
            else {
                color = d0 / dist;
            }
            assert(color <= 1 && color >= 0);
            auto &pixel = window.at<cv::Vec3b>(nrow, ncol);
            pixel[1] = MAX(pixel[1], color*255);
        }
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1) {
        return control_points[0];
    }

    std::vector<cv::Point2f> next_points;
    for (size_t i = 1; i < control_points.size(); ++i) {
        const cv::Point2f& prv = control_points[i - 1];
        const cv::Point2f& next = control_points[i];
        next_points.push_back(prv * t + (1 - t) * next);
    }

    return recursive_bezier(next_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001)  {
        cv::Point2f point = recursive_bezier(control_points, t);
        //set_pixel(point, window);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
