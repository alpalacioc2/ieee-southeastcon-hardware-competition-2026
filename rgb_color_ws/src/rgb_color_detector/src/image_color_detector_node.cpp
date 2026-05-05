#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <algorithm>
#include <cstdint>

class ImageColorDetectorNode : public rclcpp::Node
{
public:
    ImageColorDetectorNode() : Node("image_color_detector_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                processImage(msg);
            });

        publisher_ = this->create_publisher<std_msgs::msg::String>("/image_colors", 10);

        RCLCPP_INFO(this->get_logger(), "Image color detector node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /image_raw and publishing detected color to /image_colors");
    }

private:
    bool isBlackish(int r, int g, int b)
    {
        return (r < 25 && g < 25 && b < 25);
    }

    bool isGreen(int r, int g, int b)
    {
        // Based on your samples:
        // (86,243,117), (91,248,138), etc.
        return (
            g >= 170 &&
            r >= 50 && r <= 150 &&
            b >= 70 && b <= 190 &&
            g > r + 70 &&
            g > b + 60
        );
    }

    bool isBlue(int r, int g, int b)
    {
        // Based on your samples:
        // (10,140,237), (62,180,255), (9,86,188)
        return (
            b >= 170 &&
            r <= 90 &&
            g >= 70 && g <= 220 &&
            b > g + 25 &&
            b > r + 80
        );
    }

    bool isRed(int r, int g, int b)
    {
        // Based on your samples:
        // (160,50,50), (111,25,38), (234,29,87), (212,11,59), etc.
        return (
            r >= 100 &&
            g <= 85 &&
            b <= 110 &&
            r > g + 45 &&
            r > b + 25
        );
    }

    bool isPurple(int r, int g, int b)
    {
        // Based on your samples:
        // (90,51,123), (72,66,116), (113,101,175), (206,111,205), (164,96,206)
        return (
            r >= 60 &&
            b >= 110 &&
            g <= 140 &&
            b > g + 25 &&
            r > g + 10
        );
    }

    void processImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        count_++;

        // Process every 5th frame to reduce spam / CPU load.
        // Change 5 to 1 if you want every frame.
        if (count_ % 5 != 0)
        {
            return;
        }

        if (msg->encoding != "rgb8" && msg->encoding != "bgr8")
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "Unsupported image encoding: %s. Expected rgb8 or bgr8.",
                msg->encoding.c_str());
            return;
        }

        int red_count = 0;
        int green_count = 0;
        int blue_count = 0;
        int purple_count = 0;

        const int width = static_cast<int>(msg->width);
        const int height = static_cast<int>(msg->height);
        const int step = static_cast<int>(msg->step);

        // Optional speedup:
        // pixel_stride = 1 checks every pixel.
        // pixel_stride = 2 checks every other pixel in x and y.
        const int pixel_stride = 1;

        for (int row = 0; row < height; row += pixel_stride)
        {
            for (int col = 0; col < width; col += pixel_stride)
            {
                int index = row * step + col * 3;

                int r, g, b;

                if (msg->encoding == "rgb8")
                {
                    r = msg->data[index];
                    g = msg->data[index + 1];
                    b = msg->data[index + 2];
                }
                else // bgr8
                {
                    b = msg->data[index];
                    g = msg->data[index + 1];
                    r = msg->data[index + 2];
                }

                // Ignore black/dark background
                if (isBlackish(r, g, b))
                {
                    continue;
                }

                if (isGreen(r, g, b))
                {
                    green_count++;
                }
                else if (isBlue(r, g, b))
                {
                    blue_count++;
                }
                else if (isRed(r, g, b))
                {
                    red_count++;
                }
                else if (isPurple(r, g, b))
                {
                    purple_count++;
                }
            }
        }

        int max_count = std::max({red_count, green_count, blue_count, purple_count});
        std::string detected_color = "NONE";

        if (max_count == red_count && red_count > 0)
        {
            detected_color = "RED";
        }
        else if (max_count == green_count && green_count > 0)
        {
            detected_color = "GREEN";
        }
        else if (max_count == blue_count && blue_count > 0)
        {
            detected_color = "BLUE";
        }
        else if (max_count == purple_count && purple_count > 0)
        {
            detected_color = "PURPLE";
        }

        std_msgs::msg::String out_msg;
        out_msg.data = detected_color;
        publisher_->publish(out_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Detected: %s | counts -> R:%d G:%d B:%d P:%d | image: %dx%d encoding=%s",
            detected_color.c_str(),
            red_count,
            green_count,
            blue_count,
            purple_count,
            width,
            height,
            msg->encoding.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    unsigned long int count_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageColorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
