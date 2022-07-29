// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

inline double DEG2RAD(double deg) {return deg * MY_PI/180;}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    Eigen::Matrix4f projection;
    float top = -tan(DEG2RAD(eye_fov/2.0f) * abs(zNear));
    float right = top * aspect_ratio;

    projection << zNear/right,0,0,0,
                  0,zNear/top,0,0,
                  0,0,(zNear+zFar)/(zNear-zFar),(2*zNear*zFar)/(zFar-zNear),
                  0,0,1,0;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)//esc
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on

//openCV中convertTo的用法
//convertTo函数一般用于CV_32s转为CV_8U：
//原函数解析：
//src.convertTo(dst, type, scale, shift)
//缩放并转换到另外一种数据类型：
//dst：目的矩阵；
//type：需要的输出矩阵类型，或者更明确的，是输出矩阵的深度，如果是负值（常用 - 1）则输出矩阵和输入矩阵类型相同；
//scale : 比例因子；
//shift：将输入数组元素按比例缩放后添加的值；
//
//dst(i) = src(i)xscale + (shift, shift, …)
//
//如果scale = 1，shift = 0，则不进行比例缩放。
//
//场景应用一：
//在应用分水岭算法分割图像时，标记图像为32位有符号整型CV_32S变量构成的矩阵markers（其中像素值有 - 1，1, 2，3……），由于imshow()函数无法显示，想要将标记图像显示出来必须转换其数据类型。
//1、CV_32s转为CV_8U
//
//watershed(src, label_img);
////显示图像
//double maxVal = 0;
//double minVal = 0;
//minMaxLoc(label_img, &minVal, &maxVal);
//Mat dst = Mat::zeros(src.size(), CV_8U);
//label_img.convertTo(dst, CV_8U, 255.0 / (maxVal - minVal), 0);
//imshow("marks", dst);
//————————————————
//版权声明：本文为CSDN博主「梦游城市」的原创文章，遵循CC 4.0 BY - SA版权协议，转载请附上原文出处链接及本声明。
//原文链接：https ://blog.csdn.net/qq_43702097/article/details/105567290
