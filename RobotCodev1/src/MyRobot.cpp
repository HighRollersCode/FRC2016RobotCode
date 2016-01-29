#include "MyRobot.h"


RobotDemo * RobotDemo::TheRobot = NULL;
float spd = 0;
int minR = 0;
int maxR = 30;
int minG = 80; //160 for ip cam, 80 to support MS webcam
int maxG = 255;
int minB = 0;
int maxB = 30;

//GLOBAL CONSTANTS
const double PI = 3.141592653589793;

//Target Ratio Ranges
double MinHRatio = 1.5;
double MaxHRatio = 6.6;

double MinVRatio = 1.5;
double MaxVRatio = 8.5;

int MAX_SIZE = 255;
int test = 0;
//Some common colors to draw with

/*
const cv::Scalar RED = cv::Scalar(0, 0, 255),
			BLUE = cv::Scalar(255, 0, 0),
			GREEN = cv::Scalar(0, 255, 0),
			ORANGE = cv::Scalar(0, 128, 255),
			YELLOW = cv::Scalar(0, 255, 255),
			PINK = cv::Scalar(255, 0,255),
			WHITE = cv::Scalar(255, 255, 255);



//GLOBAL MUTEX LOCK VARIABLES
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t matchStartMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;


//Thread Variables
pthread_t MJPEG;
pthread_t AutoCounter;


//Store targets in global variable
Target targets;
cv::Mat frame;

//Global Timestamps for auto
struct timespec autoStart, autoEnd;


//Control process thread exectution
bool progRun;
*/
RobotDemo::RobotDemo(void)
{

	TheRobot = this;
	//grip = NetworkTable::GetTable("grip");
//	if (execle(JAVA, "-jar", GRIP_JAR, GRIP_PROJECT, (char*)0) == -1)
//		   {
			  //wpi_setErrnoErrorWithContext("Failed to run GRIP");
//			}

	commandForward = 0;
	commandArmShooter = 0;
	commandTurn = 0;
	commandLift = 0;
	commandTurret = 0;

	leftStick = new Joystick(0);
	rightStick = new Joystick(1);			// create the joysticks
	turretStick = new Joystick(2);

	DriveTrain = new Drivetrain();

/*
	ProgParams params;

	params.ROBOT_IP = "10.9.87.2";
	params.ROBOT_PORT = "0";
	params.CAMERA_IP = "";
	params.IMAGE_FILE ="";

	params.From_Camera = true;
	params.From_File = false;
	params.Visualize = true;
	params.Timer = true;
	params.Debug = true;
	params.Process = true;
	params.USB_Cam = true;
	//start mjpeg stream thread

	void parseCommandInputs(int argc, const char* argv[], ProgParams &params);
	void printCommandLineUsage();
	void initializeParams(ProgParams& params);
	double diffClock(timespec start, timespec end);
	cv::Mat ThresholdImage(cv::Mat img);
	void findTarget(cv::Mat original, cv::Mat thresholded, Target& targets, const ProgParams& params);
	void NullTargets(Target& target);
	void CalculateDist(Target& targets);
	void error(const char *msg);
	void *VideoCap(void *args);
	pthread_create(&MJPEG, NULL, VideoCap, &params);

*/

}
RobotDemo::~RobotDemo(void)
{
	TheRobot = NULL;
}

void RobotDemo::Autonomous(void)
{}
void RobotDemo::UpdateInputs()
{
		//float deadzone = .1;

		commandForward = leftStick->GetY();
		commandTurn = rightStick->GetY();
		commandArmShooter = turretStick->GetZ();
		commandLift = turretStick->GetY();
		commandTurret = turretStick->GetX();
		//commandTurn = -rightStick->GetRawButton(2);


		/*if(rightStick->GetY() == deadzone || rightStick->GetY() > 0 )
		{
			commandTurn = rightStick->GetY()-rightStick->GetY();
		}
		if(rightStick->GetY() == -deadzone || rightStick->GetY() < 0 )
		{
					commandTurn = rightStick->GetY()+rightStick->GetY();
		}*/
		if(turretStick->GetRawButton(6))
		{
			commandArmShooter = (turretStick->GetZ()-1)*.5f;
		}
		else if(turretStick->GetRawButton(7))
		{
			commandArmShooter = (-turretStick->GetZ()+1)*.5f;
		}
		else
		{
			commandArmShooter = 0;
		}


}
/*double diffClock(timespec start, timespec end)
{
 return	(end.tv_sec - start.tv_sec) + (double) (end.tv_nsec - start.tv_nsec)/ 1000000000.0f;
}
 * This function uses FFMPEG codec apart of openCV to open a
 * MJPEG stream and buffer it. This function should be ran
 * in its own thread so it can run as fast as possibe and store frames.
 *
 * A mutable lock should be used in another thread to copy the latest frame
 *
 * Note: Opening the stream blocks execution. Also
 * Based on my own tests it appears the beaglebone can capture
 * frames at 30fps with 320 x 240 resolution, however
 * the framerate needs to be reduced to allow for processing time.
 *
 * Only run the camera as 10FPS, with a 10kbs limit per frame
 */
/*
void *VideoCap(void *args)
{
	//copy passed in variable to programStruct
	ProgParams *struct_ptr = (ProgParams *) args;

	if (struct_ptr->From_File)
	{
		cout<<"Loading Image from file"<<endl;

		//read img and store it in global variable
		pthread_mutex_lock(&frameMutex);
		frame = cv::imread(struct_ptr->IMAGE_FILE);
		pthread_mutex_unlock(&frameMutex);

		if (!frame.empty())
		{
			cout<<"File Loaded: Starting Processing Thread"<<endl;
			progRun = true;
		}
		else
		{
			cout<<"Error Loading File"<<endl;
			exit(0);
		}


	}

	else if(struct_ptr->From_Camera)
	{
		//create timer variables
		struct timespec start, end, bufferStart, bufferEnd;

		//seconds to wait for buffer to clear before we start main process thread
		int waitForBufferToClear = 12;

		//start timer to time how long it takes to open stream
		clock_gettime(CLOCK_REALTIME, &start);

		cv::VideoCapture vcap;


		// For IP cam this works on a AXIS M1013
		// For USB cam this works on Microsoft HD 3000


		std::string videoStreamAddress;
		if (struct_ptr->USB_Cam)
		{

			int videoStreamAddress = 0; //represents /dev/video0

			std::cout<<"Trying to connect to Camera stream... at: "<<videoStreamAddress<<std::endl;

			int count =1;

			//open the video stream and make sure it's opened
			//We specify desired frame size and fps in constructor
			//Camera must be able to support specified framesize and frames per second
			//or this will set camera to defaults
			while (!vcap.open(videoStreamAddress, 320,240,7.5))
			{
				std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
				count++;
				usleep(1000000);
			}

			//After Opening Camera we need to configure the returned image setting
			//all opencv v4l2 camera controls scale from 0.0 - 1.0

			//vcap.set(CV_CAP_PROP_EXPOSURE_AUTO, 1);
			vcap.set(CV_CAP_PROP_EXPOSURE_ABSOLUTE, 0.1);
			vcap.set(CV_CAP_PROP_BRIGHTNESS, 1);
			vcap.set(CV_CAP_PROP_CONTRAST, 0);

			cout<<vcap.get(CV_CAP_PROP_FRAME_WIDTH)<<endl;
			cout<<vcap.get(CV_CAP_PROP_FRAME_HEIGHT)<<endl;

		}
		else //connect to IP Cam
		{
			std::string videoStreamAddress = "http://" + struct_ptr->CAMERA_IP +"/mjpg/video.mjpg";

			std::cout<<"Trying to connect to Camera stream... at: "<<videoStreamAddress<<std::endl;

			int count = 1;

			//open the video stream and make sure it's opened
			//image settings, resolution and fps are set via axis camera webpage
			while (!vcap.open(videoStreamAddress))
			{

				std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
				count++;
				usleep(1000000);
			}

		}



		//Stream started
		cout << "Successfully connected to Camera Stream" << std::endl;

		//set true boolean
		pthread_mutex_lock(&targetMutex);
		targets.cameraConnected = true;
		pthread_mutex_unlock(&targetMutex);

		//end clock to determine time to setup stream
		clock_gettime(CLOCK_REALTIME, &end);

		cout << "It took " << diffClock(start,end) << " seconds to set up stream " << endl;

		clock_gettime(CLOCK_REALTIME, &bufferStart);


		cout<<"Waiting for stream buffer to clear..."<<endl;


		//run in continuous loop
		while (true)
		{
			//start timer to get time per frame
			clock_gettime(CLOCK_REALTIME, &start);

			//read frame and store it in global variable
			pthread_mutex_lock(&frameMutex);
			vcap.read(frame);
			pthread_mutex_unlock(&frameMutex);

			//end timer to get time per frame
			clock_gettime(CLOCK_REALTIME, &end);


			if(struct_ptr->Timer)
				cout << "It took FFMPEG " << diffClock(start,end) << " seconds to grab stream \n";


			//end timer to get time since stream started
			clock_gettime(CLOCK_REALTIME, &bufferEnd);
			double bufferDifference = diffClock(bufferStart, bufferEnd);

			//The stream takes a while to start up, and because of it, images from the camera
			//buffer. We don't have a way to jump to the end of the stream to get the latest image, so we
			//run this loop as fast as we can and throw away all the old images. This wait, waits some number of seconds
			//before we are at the end of the stream, and can allow processing to begin.
			if ((bufferDifference >= waitForBufferToClear) && !progRun)
			{
				cout<<"Buffer Cleared: Starting Processing Thread"<<endl;
				progRun = true;

			}
			usleep(1000); //sleep for 5ms
		}

	}

	return NULL;
}
*/
void RobotDemo::OperatorControl(void)
{

	while (IsOperatorControl())
	{

		UpdateInputs();
		DriveTrain->StandardArcade(-commandForward, -commandTurn, -commandArmShooter, -commandLift, -commandTurret);
		DriveTrain->Shifter_Update(rightStick->GetTrigger());
		DriveTrain->Arm_Update(turretStick->GetTrigger());
		Wait(0.002);
	}

/*
	auto areas = grip->GetNumberArray("myContoursReport/area", llvm::ArrayRef<double>()),
	         xs    = grip->GetNumberArray("myContoursReport/x",    llvm::ArrayRef<double>()),
	         ys    = grip->GetNumberArray("myContoursReport/y",    llvm::ArrayRef<double>());

	    for (int i = 0; i < areas.size(); i++) {
	        double area = areas[i], x = xs[i], y = ys[i];
	        std::cout << "Got contour: area=" << area << ", x=" << x << ", y=" << y << std::endl;
	    }
	    */
}

START_ROBOT_CLASS(RobotDemo);
