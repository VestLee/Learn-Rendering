#include "../include/Transform.hpp"
namespace LRR
{
    // MVP 矩阵的学习可以参考： Games 101 Transformation 章节
    // Model Matrix 绕 Z 轴旋转
    // the model matrix for rotating the triangle around the Z axis.
    Matrix4f Transform::get_Z_rotation_matrix(float rotation_angle)
    {
        Matrix4f model = Matrix4f::Identity();

        auto degree = (float)Degree(rotation_angle);
        model << cosf(degree), -sinf(degree), 0, 0,
            sinf(degree), cosf(degree), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        return model;
    }

    // Rodrigues 旋转公式
    // vrot=cosθv+(1−cosθ)(k⋅v)k+sinθk×v
    // Matrix:R=[I+(1−cos(θ))∗N^2+sin(θ)∗N]
    // Matrix:R=cos(θ)∗I+(1−cos(θ))∗n∗n^T+sin(θ)∗n
    // 参考：https://blog.csdn.net/renhaofan/article/details/103706544
    Matrix4f Transform::get_Rodrigues_rotation(Vector3f axis, float rotation_angle)
    {
        Matrix4f model = Matrix4f::Identity();
        double degree = Degree(rotation_angle);
        Matrix4f I, N;
        Vector4f axi;
        RowVector4f taxi;
        axis.normalize();
        axi << axis.x(), axis.y(), axis.z(), 0;
        taxi << axis.x(), axis.y(), axis.z(), 0;

        I << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        N << 0, -axis.z(), axis.y(), 0,
            axis.z(), 0, -axis.x(), 0,
            -axis.y(), axis.x(), 0, 0,
            0, 0, 0, 1;

        model = cos(degree) * I + (1 - cos(degree)) * axi * taxi + sin(degree) * N;
        model(3, 3) = 1; // 前面计算可以用3x3的矩阵
        return model;
    }

    // 使用四元数 TODO
    Matrix4f Transform::get_Quaternions_rotation(Vector3f point, Vector3f axis, float rotation_angle)
    {
        Matrix4f model = Matrix4f::Identity();
        // TODO
        return model;
    }

    // View Matrix--Translate Matrix
    Matrix4f Transform::get_view_matrix(Vector3f eye_pos)
    {
        Matrix4f view = Matrix4f::Identity();

        Matrix4f translate;
        translate << 1, 0, 0, -eye_pos[0],
            0, 1, 0, -eye_pos[1],
            0, 0, 1, -eye_pos[2],
            0, 0, 0, 1;

        view = translate * view;
        return view;
    }

    // Projection Matrix
    Matrix4f Transform::get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
    {
        Matrix4f projection = Matrix4f::Identity();

        float n = zNear;
        float f = zFar;
        float t = (float)(-abs(zNear) * tan(Degree(eye_fov) / 2.0)); // you do not need to add "-" here, it is just for visualizatin
        float r = t * aspect_ratio;

        projection << n / r, 0, 0, 0,
            0, n / t, 0, 0,
            0, 0, (n + f) / (n - f), -2 * n * f / (n - f),
            0, 0, 1, 0;

        return projection;
    }
}