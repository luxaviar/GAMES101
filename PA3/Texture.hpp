//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColor(const Eigen::Vector2i p) {
        auto color = image_data.at<cv::Vec3b>(p.y(), p.x());
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        //auto u_img = u * width;
        //auto v_img = (1 - v) * height;
        //cv::Mat patch;
        //cv::getRectSubPix(image_data, cv::Size(1, 1), cv::Point2f(u_img, v_img), patch);
        //auto color = patch.at<cv::Vec3b>(0, 0);
        //return Eigen::Vector3f(color[0], color[1], color[2]);
        auto u_img = u * width;
        auto v_img = (1.0 - v) * height;
        Eigen::Vector2i u00{std::floor(u_img), std::ceil(v_img)};
        Eigen::Vector2i u10{std::ceil(u_img), std::ceil(v_img)};

        Eigen::Vector2i u01{std::floor(u_img), std::floor(v_img)};
        Eigen::Vector2i u11{std::ceil(u_img), std::floor(v_img)};

        float s = u_img - std::floor(u_img);
        float t = std::ceil(v_img) - v_img;

        Eigen::Vector3f bc = getColor(u00) + (getColor(u10) - getColor(u00)) * s;
        Eigen::Vector3f tc = getColor(u01) + (getColor(u11) - getColor(u01)) * s;

        Eigen::Vector3f color = bc + (tc - bc) * t;
        return color;
    }
};
#endif //RASTERIZER_TEXTURE_H
