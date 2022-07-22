#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
inline double DEG2RAD(double deg) {return deg * MY_PI/180;}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    double rad = DEG2RAD(rotation_angle);
    model << cos(rad),-sin(rad),0,0,
             sin(rad),cos(rad),0,0,
             0,0,1,0,
             0,0,0,1;

    //model << //绕x轴
   //    1, 0, 0, 0,
   //    0, std::cos(angle), -std::sin(angle), 0,
   //    0, std::sin(angle), std::cos(angle), 0,
   //    0, 0, 0, 1;
   //model << //绕y轴
   //    std::cos(angle), 0, std::sin(angle), 0,
   //    0, 1, 0, 0,
   //    -std::sin(angle), 0, std::cos(angle), 0,
   //    0, 0, 0, 1;

    return model;
}

//投影可以看这个
//https://zhuanlan.zhihu.com/p/45757899
//https://zhuanlan.zhihu.com/p/142943803
//https://zhuanlan.zhihu.com/p/448904350
// https://zhuanlan.zhihu.com/p/361156478
//eyefov表示视野角度，aspect_ratio表示xy的比例
//前文提到，透视变换即就是先透视投影变换，再做正交投影变换。所以可以先写出透视投影矩阵（其中参数只有n, f），然后通过计算出b, t, l, r的值，写出正交投影的矩阵，二者相乘可得到结果
//透视投影，根据名字判断，4个参数分别是：上下可视角度，长宽比width / height，z近距离，z远距离（分别为视频5,5,4,4提到的内容）
//https://zhuanlan.zhihu.com/p/386137659
//截面与远截面之间构成的四棱台称为棱台观察体，而透视投影矩阵的任务就是把位于观察体内的物体的顶点 x, y, z x, y, zx, y, z 坐标映射到[− 1, 1][-1, 1][−1, 1] 范围。
//这相当于把这个四棱台扭曲变形成一个立方体。这个立方体叫做规范化观察体（Normalized View Volume)
//https://zhuanlan.zhihu.com/p/463027517
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    /*float top = -tan(DEG2RAD(eye_fov/2.0f) * abs(zNear));
    float right = top * aspect_ratio;

    projection << zNear/right,0,0,0,
                  0,zNear/top,0,0,
                  0,0,(zNear+zFar)/(zNear-zFar),(2*zNear*zFar)/(zFar-zNear),
                  0,0,1,0;*/


    
    Eigen::Matrix4f m;
    m << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;////透视投影矩阵,即透视转正交

    float halve = eye_fov / 2 * MY_PI / 180;
    float top = tan(halve) * zNear;
    float bottom = -top;
    float right = top * aspect_ratio;
    float left = -right;

    Eigen::Matrix4f n, p;
    //这个是将立方体进行规范化（-1，1）
    n << 2 / (right - left), 0, 0, 0,
        0, 2 / (top - bottom), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    //这里是将三角形位移到原点
    p << 1, 0, 0, -(right + left) / 2,
        0, 1, 0, -(top + bottom) / 2,
        0, 0, 1, -(zFar + zNear) / 2,
        0, 0, 0, 1;

    projection = n * p * m;//这里是左乘所以是先进行透视转正交，然后位移，然后规范化

//https://zhuanlan.zhihu.com/p/463027517  显示不对....
//    projection << 2 * zNear / (right - left), 0, 0, 0,
//        0, 2 * zNear / (top - bottom), 0, 0,
//        (left + right) / (left - right), (bottom + top) / (bottom - top), (zFar + zNear) / (zFar - zNear), 1,
//        0, 0, 2 * zNear * zFar / (zNear - zFar), 0;
                    

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5}; //眼睛所在位置

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) { //esc
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos)); //[0][1][2] 0,0,5
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
