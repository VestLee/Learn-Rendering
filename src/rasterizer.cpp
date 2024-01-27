#include "../include/rasterizer.hpp"

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

#include "utils/Worker.h"

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

// static float insideTriangle(float x, float y, const Vector3f *_v)
//{
//     // 使用重心坐标同侧法 叉乘之后同号
//     // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
//     // 重心坐标同侧法
//     // 三角形的三个边
//     Vector3f e0 = _v[1] - _v[0];
//     Vector3f e1 = _v[2] - _v[1];
//     Vector3f e2 = _v[0] - _v[2];
//     // 三角形的三个顶点到点(x,y)的向量
//     Vector3f v0p = Vector3f(x, y, 0) - _v[0];
//     Vector3f v1p = Vector3f(x, y, 0) - _v[1];
//     Vector3f v2p = Vector3f(x, y, 0) - _v[2];
//     // 三角形的三个边与三个顶点到点(x,y)的向量的叉乘
//     Vector3f c0 = e0.cross(v0p);
//     Vector3f c1 = e1.cross(v1p);
//     Vector3f c2 = e2.cross(v2p);
//     // 如果三个叉乘的结果都是正数或者都是负数，说明点(x,y)在三角形内部
//     if ((c0.z() > 0 && c1.z() > 0 && c2.z() > 0) || (c0.z() < 0 && c1.z() < 0 && c2.z() < 0))
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = (int)(x2 - x1);
    dy = (int)(y2 - y1);
    dx1 = (int)fabs(dx);
    dy1 = (int)fabs(dy);
    px = 2 * dy1 - dx1;
    py = 2 * dx1 - dy1;

    if (dy1 <= dx1)
    {
        if (dx >= 0)
        {
            x = (int)x1;
            y = (int)y1;
            xe = (int)x2;
        }
        else
        {
            x = (int)x2;
            y = (int)y2;
            xe = (int)x1;
        }
        Eigen::Vector3f point = Eigen::Vector3f((float)x, (float)y, 1.0f);
        set_pixel(point, line_color);
        for (i = 0; x < xe; i++)
        {
            x = x + 1;
            if (px < 0)
            {
                px = px + 2 * dy1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    y = y + 1;
                }
                else
                {
                    y = y - 1;
                }
                px = px + 2 * (dy1 - dx1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f((float)x, (float)y, 1.0f);
            set_pixel(point, line_color);
        }
    }
    else
    {
        if (dy >= 0)
        {
            x = (int)x1;
            y = (int)y1;
            ye = (int)y2;
        }
        else
        {
            x = (int)x2;
            y = (int)y2;
            ye = (int)y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f((float)x, (float)y, 1.0f);
        set_pixel(point, line_color);
        for (i = 0; y < ye; i++)
        {
            y = y + 1;
            if (py <= 0)
            {
                py = py + 2 * dx1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    x = x + 1;
                }
                else
                {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f((float)x, (float)y, 1.0f);
            set_pixel(point, line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    // TODO ： 这里是在调节什么？没有理解
    // far = 50, near = 0.1
    float f1 = (float)((50 - 0.1) / 2.0);
    float f2 = (float)((50 + 0.1) / 2.0);

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto &i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)};
        // Homogeneous division 归一化
        for (auto &vec : v)
        {
            vec /= vec.w();
        }
        // Viewport transformation
        for (auto &vert : v)
        {
            vert.x() = 0.5f * width * (vert.x() + 1.0f);
            vert.y() = 0.5f * height * (vert.y() + 1.0f);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
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

void rst::rasterizer::rasterize_wireframe(const Triangle &t)
{
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
}

// auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
// define a function to compute the barycentric coordinates of a point p(x,y) in a triangle (x1,y1),(x2,y2),(x3,y3)
inline static auto computeBarycentric2D(float x, float y, const Eigen::Vector3f *pts)
{
    Eigen::Vector3f v0 = pts[2] - pts[0];
    Eigen::Vector3f v1 = pts[1] - pts[0];
    Eigen::Vector3f v2 = Eigen::Vector3f(x, y, 1.f) - pts[0];

    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float denom_inv = 1.f / (d00 * d11 - d01 * d01);

    float v = (d11 * d20 - d01 * d21) * denom_inv;
    float w = (d00 * d21 - d01 * d20) * denom_inv;
    float u = 1.f - v - w;

    return std::array<float, 3>{u, v, w};
}

// gain performance a lot on debug build, but little on release build
// TODO:不太理解为什么可以这样简化？
inline static auto computeBarycentric2D_2(float x, float y, const Eigen::Vector3f *pts)
{
    float ax = pts[2].x() - pts[0].x();
    float ay = pts[2].y() - pts[0].y();
    float bx = pts[1].x() - pts[0].x();
    float by = pts[1].y() - pts[0].y();
    float cx = pts[0].x() - x;
    float cy = pts[0].y() - y;

    float d = ax * by - bx * ay;
    float wa = (bx * cy - cx * by) / d;
    float wb = (cx * ay - ax * cy) / d;
    float wc = 1.0f - wa - wb;

    return std::array<float, 3>{wa, wb, wc};
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t)
{
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // 规范一个矩形包围盒 ， 遍历举行包围盒中的像素
    float x_min = std::clamp(std::min(std::min(v[0][0], v[1][0]), v[2][0]), 0.f, (float)width - 1.f);
    float x_max = std::clamp(std::max(std::max(v[0][0], v[1][0]), v[2][0]), 0.f, (float)width - 1.f);
    float y_min = std::clamp(std::min(std::min(v[0][1], v[1][1]), v[2][1]), 0.f, (float)height - 1.f);
    float y_max = std::clamp(std::max(std::max(v[0][1], v[1][1]), v[2][1]), 0.f, (float)height - 1.f);

    // multithreading
    // 当fps超过一定值时，多线程反而会降低性能，因为每帧的时间都很短，线程同步带来的开销相对会变大
    constexpr unsigned int parts = 4;
    // 这里使用线程池，相对于裸std::thread会节省创建线程的开销
    static std::vector<LRR::utils::Worker> workers(parts);
    static std::vector<std::pair<float, float>> y_segs(parts);
    auto y_step = (y_max - y_min) / parts;
    for (int i = 0; i < parts; i++)
    {
        y_segs[i].first = y_min + y_step * i;
        y_segs[i].second = y_min + y_step * (i + 1);
    }

    if (sample_method == SampleMethod::None)
    {
        // without anti-aliasing
        for (int i = 0; i < parts; i++)
        {
            workers[i].Dispatch(
                [&, i]()
                {
                    for (int x = (int)x_min; x <= (int)x_max; ++x)
                    {
                        for (int y = (int)y_segs[i].first; y < (int)y_segs[i].second; ++y)
                        {
                            auto [alpha, beta, gamma] = computeBarycentric2D_2((float)x + 0.5f, (float)y + 0.5f, t.v);
                            // we need to decide whether this point is actually inside the triangle
                            if (alpha < 0.f || beta < 0.f || gamma < 0.f)
                                continue;
                            // get z value--depth
                            // If so, use the following code to get the interpolated z value.
                            // float w_reciprocal = float(1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w()));
                            // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            // z_interpolated *= w_reciprocal;

                            // the same operation but less division
                            float f1 = alpha * v[1].w() * v[2].w();
                            float f2 = beta * v[0].w() * v[2].w();
                            float f3 = gamma * v[0].w() * v[1].w();
                            float z_interpolated = (f1 * v[0].z() + f2 * v[1].z() + f3 * v[2].z()) / (f1 + f2 + f3);

                            // compare the current depth with the value in depth buffer
                            if (depth_buf[get_index(x, y)] > z_interpolated) // note: we use get_index to get the index of current point in depth buffer
                            {
                                // we have to update this pixel
                                depth_buf[get_index(x, y)] = z_interpolated; // update depth buffer
                                // assign color to this pixel
                                set_pixel(Vector3f((float)x, (float)y, z_interpolated), t.getColor());
                            }
                        }
                    }
                });
        }
    }
    else
    {
        for (int i = 0; i < parts; i++)
        {
            workers[i].Dispatch(
                [&, i]()
                {
                    // 保存需要着色的像素的索引
                    auto shade_index = std::make_unique<int[]>(static_cast<int>(sample_method));
                    // 简化倒数运算
                    auto inv_sample_method = 1.f / (float)sample_method;

                    for (int x = (int)x_min; x <= (int)x_max; x++)
                    {
                        for (int y = (int)y_segs[i].first; y < (int)y_segs[i].second; y++)
                        {
                            auto frame_index = get_index(x, y);
                            auto extend_index = frame_index * (int)sample_method;

                            int count = 0;

                            for (int i = 0; i < (int)sample_method; i++)
                            {
                                auto [alpha, beta, gamma] = computeBarycentric2D_2(float(x) + sample_points[i][0], float(y) + sample_points[i][1], t.v);
                                if (alpha < 0.f || beta < 0.f || gamma < 0.f)
                                    continue;
                                float f1 = alpha * v[1].w() * v[2].w();
                                float f2 = beta * v[0].w() * v[2].w();
                                float f3 = gamma * v[0].w() * v[1].w();
                                float z_interpolated = (f1 * v[0].z() + f2 * v[1].z() + f3 * v[2].z()) / (f1 + f2 + f3);
                                auto index = extend_index + i;
                                if (depth_buf[index] > z_interpolated)
                                {
                                    depth_buf[index] = z_interpolated;
                                    shade_index[count++] = index;
                                }
                            }

                            if (count > 0)
                            {
                                // auto [alpha, beta, gamma] = computeBarycentric2D_2(float(x) + 0.5f, float(y) + 0.5f, t.v);
                                // bool middle = alpha > 0.f && beta > 0.f && gamma > 0.f;
                                // 当三角形在像素中心，就用中心着色，否则看情况选择某个采样点作为着色点
                                // shade at middle or shade_index[0]
                                // but now the shading is simple
                                auto color = t.getColor();

                                for (int i = 0; i < count; i++)
                                {
                                    Eigen::Vector3f old_color = color_buf[shade_index[i]];
                                    color_buf[shade_index[i]] = color;
                                    // 可以将frame_buf的更新放在所有三角形都遍历完之后，用color_buf进行一次统一的resolve
                                    // 也可以像现在这样直接更新
                                    // f = f - old / 4 + new / 4
                                    frame_buf[frame_index] += (color - old_color) * inv_sample_method;
                                }
                            }
                        }
                    }
                });
        }
    }

    for (int i = 0; i < parts; i++)
    {
        while (workers[i].Busy())
        {
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(color_buf.begin(), color_buf.end(), Eigen::Vector3f{0, 0, 0});
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

inline int rst::rasterizer::get_index(int x, int y)
{
    return (height - y - 1) * width + x;
}

inline void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color)
{
    // old index: auto ind = point.y() + point.x() * width;
    // if (point.x() < 0 || point.x() >= width ||
    //    point.y() < 0 || point.y() >= height)
    //    return;
    auto ind = (size_t)((height - point.y() - 1) * width + point.x());
    frame_buf[ind] = color;
}

void rst::rasterizer::set_sample_method(SampleMethod method)
{
    if (method == sample_method)
        return;

    sample_method = method;
    auto w = width;
    auto h = height;
    switch (method)
    {
    case SampleMethod::None:
        // 不开启抗锯齿时color_buf用frame_buf代替
        color_buf.resize(0);
        depth_buf.resize(w * h);
        break;
    case SampleMethod::MSAA_2X:
        color_buf.resize(w * h * 2);
        depth_buf.resize(w * h * 2);
        sample_points =
            {{0.25f, 0.25f}, {0.75f, 0.75f}};
        break;
    case SampleMethod::MSAA_4X:
    {
        color_buf.resize(w * h * 4);
        depth_buf.resize(w * h * 4);
        sample_points =
            {{0.125f, 0.125f}, {0.375f, 0.375f}, {0.625f, 0.625f}, {0.875f, 0.875f}};
        break;
    }
    case SampleMethod::MSAA_8X:
        color_buf.resize(w * h * 8);
        depth_buf.resize(w * h * 8);
        sample_points =
            {{0.0625f, 0.0625f}, {0.1875f, 0.1875f}, {0.3125f, 0.3125f}, {0.4375f, 0.4375f}, {0.5625f, 0.5625f}, {0.6875f, 0.6875f}, {0.8125f, 0.8125f}, {0.9375f, 0.9375f}};
        break;
    case SampleMethod::MSAA_16X:
        color_buf.resize(w * h * 16);
        depth_buf.resize(w * h * 16);
        sample_points =
            {{0.03125f, 0.03125f}, {0.09375f, 0.09375f}, {0.15625f, 0.15625f}, {0.21875f, 0.21875f}, {0.28125f, 0.28125f}, {0.34375f, 0.34375f}, {0.40625f, 0.40625f}, {0.46875f, 0.46875f}, {0.53125f, 0.53125f}, {0.59375f, 0.59375f}, {0.65625f, 0.65625f}, {0.71875f, 0.71875f}, {0.78125f, 0.78125f}, {0.84375f, 0.84375f}, {0.90625f, 0.90625f}, {0.96875f, 0.96875f}};
        break;
    }

    // memory compact
    color_buf.shrink_to_fit();
    depth_buf.shrink_to_fit();
}
