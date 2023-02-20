#include "../include/rasterizer.hpp"
#include "../include/Triangle.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
inline double Degree(double angle) { return angle * MY_PI / 180.0; }

// MVP 矩阵的学习可以参考： Games 101 Transformation 章节
// Model Matrix 绕 Z 轴旋转
Eigen::Matrix4f get_Z_rotation_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    double degree = Degree(rotation_angle);
    model << cos(degree), -sin(degree), 0, 0,
        sin(degree), cos(degree), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return model;
}

// Model Matrix 绕过原点的任意轴旋转
// 参考
// 矩阵推导 旋转公式 TODO
Eigen::Matrix4f get_Rodrigues_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // TODO
    return model;
}
// Rodrigues 旋转公式 TODO
Eigen::Matrix4f get_Rodrigues_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // TODO
    return model;
}
// 使用四元数 TODO
Eigen::Matrix4f get_Quaternions_rotation(Vector3f point, Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // TODO
    return model;
}

// View Matrix
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// Projection Matrix 
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float n=zNear;
    float f=zFar;
    float t=-abs(zNear)*tan(Degree(eye_fov)/2.0); // you do not need to add "-" here, it is just for visualizatin
    float r=t*aspect_ratio;

    projection << n/r, 0, 0, 0,
                0, n/t, 0, 0,
                0, 0, (n+f)/(n-f), -2*n*f/(n-f),
                0, 0, 1, 0;

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

    // 设置光栅化 FrameBuffer & Depth Buffer 大小
    rst::rasterizer r(700, 700);
    // 设置观察点位置 
    Eigen::Vector3f eye_pos = {0, 0, 5};
    // 设置顶点位置 & 顶点索引
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

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        cv::imwrite("resutl.png",image);
        key = cv::waitKey(10);

        // std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
