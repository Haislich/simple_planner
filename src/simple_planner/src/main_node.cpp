#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
int main()
{
    // File path of the image
    std::string imagePath = "/home/lattinone/catkin_ws/simple_planner_ws/maps/map1.pgm";

    // Read the image from file path
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);

    // Check if the image was loaded successfully
    if (image.empty())
    {
        std::cerr << "Failed to load image from path: " << imagePath << std::endl;
        return -1;
    }
    int finalWidth = 400;
    int finalHeight = 400;
    cv::Mat resizedImage;
    cv::resize(image, resizedImage, cv::Size(finalWidth, finalHeight), 0, 0, cv::INTER_NEAREST);
     
    // Create a copy of the original image for manipulation
    image = resizedImage.clone();
    // Determine the size of the image
    cv::Size imageSize = image.size();
    int width = imageSize.width;
    int height = imageSize.height;

    // Define the height of the text area
    int textAreaHeight = 100;

    // Create a larger Mat (canvas) that includes space for the image and text area
    cv::Mat canvas = cv::Mat::zeros(height + textAreaHeight, width, image.type());

    // Copy the image onto the canvas
    image.copyTo(canvas(cv::Rect(0, 0, width, height)));

    // Define the position for the text, just below the image
    //cv::Point textPosition(10, height + 20); // Adjust the (x, y) coordinates as needed

    // Draw text options on the canvas just below the image
    cv::putText(canvas, "Press 'q' to quit", cv::Point(10, height + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(canvas, "Press 'n' to move to the next planned point", cv::Point(10, height + 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(canvas, "Press 'r' to restart", cv::Point(20, height + 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);


    // Display the canvas (image and text area) in a window
    cv::imshow("Interactive Image", canvas);

    // Interactive loop to handle user input
    while (true) {
        // Wait for a key press
        int key = cv::waitKey(0);

        // Check the key and perform the corresponding action
        if (key == 'q' || key == 'Q') {
            // Quit the program if 'q' or 'Q' is pressed
            break;
        } else if (key == 'r' || key == 'R') {
            // Rotate the image clockwise if 'r' or 'R' is pressed
            cv::Mat rotatedImage;
            cv::rotate(image, rotatedImage, cv::ROTATE_90_CLOCKWISE);

            // Update the image with the rotated version
            image = rotatedImage.clone();

            // Create a new canvas with the rotated image and text options
            canvas = cv::Mat::zeros(height + textAreaHeight, width, image.type());
            image.copyTo(canvas(cv::Rect(0, 0, width, height)));

            // Redraw the text options on the new canvas
            cv::putText(canvas, "Press 'q' to quit", cv::Point(10, height + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            cv::putText(canvas, "Press 'r' to rotate clockwise", cv::Point(10, height + 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

            // Redisplay the updated canvas (image and text area)
            cv::imshow("Interactive Image", canvas);
        }
    }

    // Clean up and close the window
    cv::destroyAllWindows();

    return 0;
}

