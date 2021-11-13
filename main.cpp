#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

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
    model <<
        cos(rotation_angle / 180), -sin(rotation_angle / 180), 0, 0,
        sin(rotation_angle / 180), cos(rotation_angle / 180), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f orthogonal_1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f orthogonal_2 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f perspective = Eigen::Matrix4f::Identity();


    //[l，r][b, t] [f, n]
    float n, f, t, b, r, l;

    n = zNear; // -z方向近平面
    f = zFar; // -z方向远平面

    static int i = 1;

    // L方向

    t = n * tan(eye_fov / 2);
    b = -t;

    l = -t * aspect_ratio;
    r = -l;

    orthogonal_1 <<
        2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;

    orthogonal_2 <<
        1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;

    perspective <<
        n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    projection = orthogonal_1 * orthogonal_2 * perspective;

    projection <<
        1 / aspect_ratio / tan(eye_fov / 2), 0, 0, 0,
        0, 1 / tan(eye_fov / 2), 0, 0,
        0, 0, (f + n) / (n - f), -2 * f * n / (n - f),
        0, 0, -1, 0;

    return projection;
}

/*
*   函数的作用是得到绕任意
    过原点的轴的旋转变换矩阵
    罗德里格斯旋转公式，认为这个轴是过圆心的，且方向是轴的方向
    这里考的是罗德里根斯旋转公式
*/
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix3f matrix = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

    I << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;

    Eigen::Matrix3f n = Eigen::Matrix3f::Identity();

    n << 0, -axis[2], axis[1],
        axis[2], 0, -axis[0],
        -axis[1], axis[0], 0;


    matrix = cos(angle / 180) * I + sin(angle / 180) * n + (1 - cos(angle / 180)) * axis * axis.transpose();


    Eigen::Matrix4f matrix4 = Eigen::Matrix4f::Identity();

    matrix4 << matrix(0, 0), matrix(0, 1), matrix(0, 2), 0,
        matrix(1, 0), matrix(1, 1), matrix(1, 2), 0,
        matrix(2, 0), matrix(2, 1), matrix(2, 2), 0,
        0, 0, 0, 1;

    std::cout << matrix4 << std::endl;

    return matrix4;
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
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

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
