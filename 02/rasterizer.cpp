// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f P(x+0.5f,y+0.5f,1.0f);

    const Vector3f& A = _v[0];
    const Vector3f& B = _v[1];
    const Vector3f& C = _v[2];

    Vector3f AB =  B - A;
    Vector3f BC =  C - B;
    Vector3f CA =  A - C;

    Vector3f AP = P - A;
    Vector3f BP = P - B;
    Vector3f CP = P - C;

    float z1 = AB.cross(AP).z();
    float z2 = BC.cross(BP).z();
    float z3 = CA.cross(CP).z();

    return (z1 > 0 && z2 >0 && z3 > 0) ||  (z1 < 0 && z2 <0 && z3 < 0);
}

//?????
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;//????
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)//所有的三角形???
    {
        Triangle t;
        Eigen::Vector4f v[] = {//这里是3个，是三角形的三个顶点
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division 齐次坐标归一化
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation 屏幕坐标?????
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);//???????
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)//???????
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    float aabb_minx = 0;
    float aabb_miny = 0;
    float aabb_maxx = 0;
    float aabb_maxy = 0;
    for (size_t i = 0; i < 3; i++)
    {
        const Vector3f& p = t.v[i];
        if(i == 0)
        {
            aabb_minx = aabb_maxx = p.x();
            aabb_miny = aabb_maxy = p.y();
            continue;
        }

        aabb_minx = p.x() < aabb_minx ? p.x() : aabb_minx;
        aabb_miny = p.y() < aabb_miny ? p.y() : aabb_miny;

        aabb_maxx = p.x() > aabb_maxx ? p.x() : aabb_maxx;
        aabb_maxy = p.y() > aabb_maxy ? p.y() : aabb_maxy;
    }
    
    // iterate through the pixel and find if the current pixel is inside the triangle

    for(int x = (int)aabb_minx;x < aabb_maxx;x++)
    {
        for (int y = (int)aabb_miny; y < aabb_maxy; y++)
        {
            if(!insideTriangle(x,y,t.v)) continue;
            // If so, use the following code to get the interpolated z value.
            auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());//reciprocal 倒数
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;

            int buf_index = get_index(x,y);
            
            if(z_interpolated >= depth_buf[buf_index]) continue;

            depth_buf[buf_index] = z_interpolated;//??????

            set_pixel(Vector3f(x,y,1),t.getColor());
        }
    }



    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

//Screen space rasterization
//第一步 确定bounding box
//给定一个三角形，我们先要扫描出这个三角占据了哪些格子(像素单元)，即定义bounding box，取得这个三角形的x和y轴上的边界值，使得这个bounding box尽可能的小，但又必须包裹住这个三角形。
//第二步 扫描bounding box
//逐一扫描这个bounding box内的所有单元，以每个单元的中心位置代替此单元，将坐标传入insideTriangle函数中，如果不在，则扫描下一个，如果在，则以三角形三点做插值得到这个中心点的z值, 
//进一步判断，这个z值是否比深度缓存中对应位置的值要小，不小则不处理，小则替换深度缓存中的值为当前值。随后设置这个单元的颜色值。
//这里很容易就能想到，如果这个单元尺寸越小，就能更加精确的确定边界。也就是MSAA，多重采样抗锯齿，这个并不是从物理上将单元切割成更小的，而是将一个单元看成多个单元组合，然后对这个更小单元进行采样。
//每个单元的尺寸越小，成像的边界锯齿效果就越虚，即成像效果越好，但这样会使效率降低，将一个单元切割成2x2的四个小单元，会使原来一个单位的工作量升至四个单位的工作量。
void rst::rasterizer::rasterize_triangle1(const Triangle& t) {
    auto v = t.toVector4();//这里是把三角形结构体换成三个顶点如（x1,y1,z1,1） 
               //还记得嘛？最后的1表示它是一个点,用于齐次坐标

    //bounding box
    float min_x = std::min(v[0][0], std::min(v[1][0], v[2][0]));
    float max_x = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    float min_y = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    float max_y = std::max(v[0][1], std::max(v[1][1], v[2][1]));

    min_x = static_cast<int>(std::floor(min_x));
    min_y = static_cast<int>(std::floor(min_y));
    max_x = static_cast<int>(std::ceil(max_x));
    max_y = static_cast<int>(std::ceil(max_y));

    //左边界小数部分全部直接舍，右边界小数部分直接入，确保单元边界坐标都是整数，三角形一定在bounding box内。

    bool MSAA = true;//MSAA是否启用

    if (MSAA)
    {
        std::vector<Eigen::Vector2f> pos
        {                               //对一个像素分割四份 当然你还可以分成4x4 8x8等等甚至你还可以为了某种特殊情况设计成不规则的图形来分割单元
            {0.25,0.25},                //左下
            {0.75,0.25},                //右下
            {0.25,0.75},                //左上
            {0.75,0.75}                 //右上
        };
        for (int i = min_x; i <= max_x; ++i)
        {
            for (int j = min_y; j <= max_y; ++j)
            {
                int count = 0;
                float minDepth = FLT_MAX;
                for (int MSAA_4 = 0; MSAA_4 < 4; ++MSAA_4)
                {
                    if (insideTriangle(static_cast<float>(i + pos[MSAA_4][0]), static_cast<float>(j + pos[MSAA_4][1]), t.v))
                    {
                        auto [alpha, beta, gamma] = computeBarycentric2D(static_cast<float>(i + pos[MSAA_4][0]), static_cast<float>(j + pos[MSAA_4][1]), t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        minDepth = std::min(minDepth, z_interpolated);
                        ++count;
                    }
                }
                if (count)
                {
                    if (depth_buf[get_index(i, j)] > minDepth)
                    {
                        depth_buf[get_index(i, j)] = minDepth;//更新深度

                        Eigen::Vector3f color = t.getColor() * (count / 4.0);//对颜色进行平均，使得边界更平滑，也是一种模糊的手段
                        Eigen::Vector3f point;
                        point << static_cast<float>(i), static_cast<float>(j), minDepth;
                        set_pixel(point, color);//设置颜色
                    }
                }
            }
        }
    }
    else
    {
        for (int i = min_x; i <= max_x; ++i)
        {
            for (int j = min_y; j <= max_y; ++j)
            {
                if (insideTriangle(static_cast<float>(i + 0.5), static_cast<float>(j + 0.5), t.v))
                {
                    auto [alpha, beta, gamma] = computeBarycentric2D(static_cast<float>(i + 0.5), static_cast<float>(j + 0.5), t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    if (depth_buf[get_index(i, j)] > z_interpolated)
                    {
                        depth_buf[get_index(i, j)] = z_interpolated;//更新深度
                        Eigen::Vector3f color = t.getColor();
                        Eigen::Vector3f point;
                        point << static_cast<float>(i), static_cast<float>(j), z_interpolated;
                        set_pixel(point, color);//设置颜色
                    }
                }
            }
        }
    }
}



void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on