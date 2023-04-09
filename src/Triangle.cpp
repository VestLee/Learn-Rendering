//
// Created by LEI XU on 4/11/19.
//

#include "../include//Triangle.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>

Triangle::Triangle()
{
    v[0] << 0, 0, 0;
    v[1] << 0, 0, 0;
    v[2] << 0, 0, 0;

    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

void Triangle::setVertex(int ind, Eigen::Vector3f ver) { v[ind] = ver; }

void Triangle::setNormal(int ind, Vector3f n) { normal[ind] = n; }

void Triangle::setColor(int ind, float r, float g, float b)
{
    if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) ||
        (b > 255.))
    {
        throw std::runtime_error("Invalid color values");
    }

    color[ind] = Vector3f((float)r / 255.f, (float)g / 255.f, (float)b / 255.f);
    return;
}
void Triangle::setTexCoord(int ind, float s, float t)
{
    tex_coords[ind] = Vector2f(s, t);
}

std::array<Vector4f, 3> Triangle::toVector4() const
{
    return {Vector4f(v[0][0], v[0][1], v[0][2], 1.f),
            Vector4f(v[1][0], v[1][1], v[1][2], 1.f),
            Vector4f(v[2][0], v[2][1], v[2][2], 1.f)};
}
