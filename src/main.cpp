#include "../include/rasterizer.hpp"
#include "../include/Triangle.hpp"
#include "../include/Transform.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace LRR;

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    // 设置光栅化 FrameBuffer & Depth Buffer 大小
    rst::rasterizer r(700, 700);
    // 设置观察点位置
    Eigen::Vector3f eye_pos = {0, 0, 5};
    // 设置顶点位置 & 顶点索引 & 顶点颜色
    std::vector<Eigen::Vector3f> pos{
        {2, 0, -2},
        {0, 2, -2},
        {-2, 0, -2},
        {3.5, -1, -5},
        {2.5, 1.5, -5},
        {-1, 0.5, -5}};

    std::vector<Eigen::Vector3i> ind{
        {0, 1, 2},
        {3, 4, 5}};

    std::vector<Eigen::Vector3f> cols{
        {217.0, 238.0, 185.0},
        {217.0, 238.0, 185.0},
        {217.0, 238.0, 185.0},
        {185.0, 217.0, 238.0},
        {185.0, 217.0, 238.0},
        {185.0, 217.0, 238.0}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    r.set_sample_method(rst::SampleMethod::TAA);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(LRR::Transform::get_Rodrigues_rotation(Vector3f(0, 0, 1), angle));
        r.set_view(LRR::Transform::get_view_matrix(eye_pos));
        r.set_projection(LRR::Transform::get_projection_matrix(45, 1, 0.1f, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    auto time_start = std::chrono::steady_clock::now();
    long long counter_nsec = 0;

    while (key != 27)
    {
        auto frame_start = std::chrono::steady_clock::now();

        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(LRR::Transform::get_Rodrigues_rotation(Vector3f(0, 0, 2), angle));
        r.set_view(LRR::Transform::get_view_matrix(eye_pos));
        r.set_projection(LRR::Transform::get_projection_matrix(45, 1, 0.1f, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        // cv::imwrite("resutl.png",image);
        key = cv::pollKey();

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }

        ++frame_count;

        auto now = std::chrono::steady_clock::now();
        auto frame_time = std::chrono::duration(now - frame_start);
        auto time_elapsed = std::chrono::duration(now - time_start);
        counter_nsec += frame_time.count();
        if (counter_nsec >= 1000 * 1000 * 1000)
        {
            // NOTE: set to 0 will move the timeline forward according to the remainder of the last frame
            // counter_nsec = 0;
            // NOTE: set to the remainder of global time can mostly fix the timeline on precise seconds
            // however if frame time takes more than 1 second, the timeline will still move forward
            counter_nsec = time_elapsed.count() % (1000 * 1000 * 1000);
            std::cout << "time: " << time_elapsed.count() * 0.001 * 0.001 * 0.001 << "s, fps: " << frame_count << std::endl;
            frame_count = 0;
        }
    }

    return 0;
}
