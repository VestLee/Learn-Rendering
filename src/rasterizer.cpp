#include <algorithm>
#include "../include/rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

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

static float insideTriangle(float x, float y, const Vector3f *_v)
{
    // 使用重心坐标同侧法 叉乘之后同号
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    std::vector<Vector3f> ver, ver2;
    ver.push_back({_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0});
    ver2.push_back({x - _v[0].x(), y - _v[0].y(), 0});
    ver.push_back({_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0});
    ver2.push_back({x - _v[1].x(), y - _v[1].y(), 0});
    ver.push_back({_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0});
    ver2.push_back({x - _v[2].x(), y - _v[2].y(), 0});

    for (int i = 0; i < 3; i++)
    {
        if (ver[i].cross(ver2[i]).z() < 0)
            return false;
    }
    return true;
}

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
auto computeBarycentric2D(float x, float y, const Eigen::Vector3f *pts)
{
    Eigen::Vector3f v0 = pts[2] - pts[0];
    Eigen::Vector3f v1 = pts[1] - pts[0];
    Eigen::Vector3f v2 = Eigen::Vector3f(x, y, 1.0f) - pts[0];

    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float denom = d00 * d11 - d01 * d01;

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    return std::make_tuple(u, v, w);
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t)
{
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // 规范一个矩形包围盒 ， 遍历举行包围盒中的像素
    float x_min = std::min(std::min(v[0][0], v[1][0]), v[2][0]);
    float x_max = std::max(std::max(v[0][0], v[1][0]), v[2][0]);
    float y_min = std::min(std::min(v[0][1], v[1][1]), v[2][1]);
    float y_max = std::max(std::max(v[0][1], v[1][1]), v[2][1]);

    // anti-alising
    bool MSAA4X = false;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    if (!MSAA4X)
    {
        // without anti-alising
        for (int x = (int)x_min; x <= (int)x_max; x++)
        {
            for (int y = (int)y_min; y <= (int)y_max; y++)
            {
                // we need to decide whether this point is actually inside the triangle
                if (!insideTriangle((float)x, (float)y, t.v))
                    continue;
                // get z value--depth
                // If so, use the following code to get the interpolated z value.
                auto [alpha, beta, gamma] = computeBarycentric2D((float)x, (float)y, t.v);
                float w_reciprocal = float(1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w()));
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

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
    }
    else
    {
        for (int x = (int)x_min; x <= (int)x_max; x++)
        {
            for (int y = (int)y_min; y <= (int)y_max; y++)
            {
                // you have to record the min-depth of the 4 sampled points(in one pixel)
                float min_depth = FLT_MAX;
                // the number of the 4 sampled points that are inside triangle
                int count = 0;
                std::vector<std::vector<float>> sampled_points{{0.25, 0.25}, {0.25, 0.75}, {0.75, 0.25}, {0.75, 0.75}};
                for (int i = 0; i < 4; i++)
                {
                    if (insideTriangle(float(x) + sampled_points[i][0], float(y) + sampled_points[i][1], t.v))
                    {
                        auto [alpha, beta, gamma] = computeBarycentric2D((float)x, (float)y, t.v);
                        float w_reciprocal = float(1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w()));
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        min_depth = std::min(min_depth, z_interpolated);
                        count += 1;
                    }
                }
                if (count > 0 && depth_buf[get_index(x, y)] > min_depth)
                {
                    // update
                    depth_buf[get_index(x, y)] = min_depth;
                    // note: the color should be changed too
                    set_pixel(Vector3f((float)x, (float)y, min_depth), t.getColor() * count / 4.0 + frame_buf[get_index(x, y)] * (4 - count) / 4.0); // frame_buf contains the current color
                }
            }
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
    return (height - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color)
{
    // old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height)
        return;
    auto ind = (size_t)((height - point.y()) * width + point.x());
    frame_buf[ind] = color;
}
