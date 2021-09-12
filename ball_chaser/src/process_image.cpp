#include <stdint.h>
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
bool bRobotStopped = true;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    //Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service and pass the linear x and angular z values
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

	int white_pixel = 255;
	bool bBallFound = false;

	// Loop through each pixel in the image and check if there's a bright white one
	uint32_t oneFourth = img.width / 4;
	uint32_t threeFourth = oneFourth * 3;

	enum class moveDirection { Left, Middle, Right };
	moveDirection dir = moveDirection::Middle;
	uint32_t bytesPerCol = img.step / img.width;

	uint32_t totalNoOfPixels = img.height * img.step;
	uint32_t noOfWhitePixels = 0;
	for (uint32_t i = 0; i < img.height; i++)//rows
	{
		for (uint32_t j = 0; j < img.width; j++)//cols
		{
			for (uint32_t k = 0; k < bytesPerCol; k++)//bytes per column
			{
				uint32_t n = i * img.step + j * bytesPerCol + k;
				if (n < img.height * img.step && img.data[n] == white_pixel)
				{
					if (!bBallFound)
					  bBallFound = true;

					noOfWhitePixels++;

					//Identify if this pixel falls in the left, middle, or right side of the image
					if (j < oneFourth)
					{
						dir = moveDirection::Left;
					}
					else if (j > oneFourth && j < threeFourth)
					{
						dir = moveDirection::Middle;
					}
					else if (j > threeFourth)
					{
						dir = moveDirection::Right;
					}
				}
			}
		}
	}

	// Depending on the white ball position, call the drive_bot function and pass velocities to it
	float lin_x, ang_z;
	bool bStopTheRobot = false;
	if (bBallFound)
	{
		//check ratio of white pixels to the total number of pixels
		float nPercentageWhite = ((float)noOfWhitePixels / (float)totalNoOfPixels);
		//ROS_INFO("percentage : %1.2f", (float)nPercentageWhite);
		if (nPercentageWhite > 0.15f)//Too close. Stop the robot.
		{
			bStopTheRobot = true;
			ROS_INFO_STREAM("Close to ball....");
		}
		else
		{
			switch (dir)
			{
			case moveDirection::Left:
			{
				lin_x = 0.0;
				ang_z = 0.5;
				ROS_INFO_STREAM("Move Left");
			}
			break;
			case moveDirection::Right:
			{
				lin_x = 0.0;
				ang_z = -0.5;
				ROS_INFO_STREAM("Move Right");
			}
			break;
			case moveDirection::Middle:
			{
				lin_x = 2.0;
				ang_z = 0.0;
				ROS_INFO_STREAM("Move Forward");
			}
			break;
			default:
			{
				lin_x = 0.0;
				ang_z = 0.0;
				ROS_INFO_STREAM("In DEFAULT");
			}
			break;
			}

			drive_robot(lin_x, ang_z);
			if (bRobotStopped)
				bRobotStopped = false;
		}
	}
	else 
	{
		// Request a stop when there's no white ball seen by the camera
		if (!bRobotStopped)//if robot was moving, issue a stop move
		  bStopTheRobot = true;
	}

	if (bStopTheRobot)
	{
		drive_robot(0.0, 0.0);
		bRobotStopped = true;
		ROS_INFO_STREAM("Stop the robot");
	}
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
