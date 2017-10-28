#include "app.h"
#include "util.h"
#include "OSCSendMessage.h"
#include <thread>
#include <chrono>

#include <ppl.h>

#include <math.h>
/*
Simple example of sending an OSC message using oscpack.
*/


using namespace std;
using namespace cv;
// Constructor
Kinect::Kinect()
{
	
		   // Initialize
    initialize();
}

// Destructor
Kinect::~Kinect()
{
    // Finalize
    finalize();
}

/// 
// Variables
double t; // timer

// store data, needed to calculate rates of change
std::array<Joint, 25> All_joints;
std::array<int, 25> X, Y;
std::array<double, 25> Pitch, Yaw, Roll;

// rates of change
std::array<int, 25> V_X, V_Y;
std::array<double, 25> Omega_Pitch, Omega_Yaw, Omega_Roll;

std::map<int, std::string> joint_name = { {0, "SpineBase"}, {1, "SpineMid"}, { 2, "Neck" }, {3, "Head"},  
{4, "ShoulderLeft"}, {5, "ElbowLeft"}, {6, "WristLeft"}, {7, "HandLeft"}, {8, "ShoulderRight"}, {9, "ElbowRight"},  
{10, "WristRight"}, {11, "HandRight"}, {12, "HipLeft"}, {13, "KneeLeft"}, {14, "AnkleLeft"}, {15, "FootLeft"},  
{16, "HipRight"}, {17, "KneeRight"}, {18, "AnkleRight"}, {19, "FootRight"}, {20, "SpineShoulder"}, 
{21, "HandTipLeft"}, {22, "ThumbLeft"}, {23, "HandTipRight"}, {24, "HandTipRight"}};

// Processing
void Kinect::run()
{
	
    // Main Loop
    while( true ){
		double e1 = cv::getTickCount();	
				
        // Update Data - joints, orientations, rate of changes
        update();
		
        // Draw Data
        draw();

        // Show Data 
		show();
			
		


        // Key Check
        const int key = cv::waitKey( 10 );
        if( key == VK_ESCAPE ){
            break;
        }

		double e2 = cv::getTickCount();
		t = (e2 - e1) / cv::getTickFrequency();
		//cout << "time ellapsed: "<< t << endl;
	}
}

// Initialize
void Kinect::initialize()
{
    cv::setUseOptimized( true );

    // Initialize Sensor
    initializeSensor();

    // Initialize Color
    initializeColor();

    // Initialize Body
    initializeBody();

    // Wait a Few Seconds until begins to Retrieve Data from Sensor ( about 2000-[ms] )
    std::this_thread::sleep_for( std::chrono::seconds( 2 ) );


	
}

// Initialize Sensor
inline void Kinect::initializeSensor()
{
    // Open Sensor
    ERROR_CHECK( GetDefaultKinectSensor( &kinect ) );

    ERROR_CHECK( kinect->Open() );

    // Check Open
    BOOLEAN isOpen = FALSE;
    ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
    if( !isOpen ){
        throw std::runtime_error( "failed IKinectSensor::get_IsOpen( &isOpen )" );
    }

    // Retrieve Coordinate Mapper
    ERROR_CHECK( kinect->get_CoordinateMapper( &coordinateMapper ) );
}

// Initialize Color
inline void Kinect::initializeColor()
{
	

    // Open Color Reader
    ComPtr<IColorFrameSource> colorFrameSource;
    ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
    ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

    // Retrieve Color Description
    ComPtr<IFrameDescription> colorFrameDescription;
    ERROR_CHECK( colorFrameSource->CreateFrameDescription( ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
    ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) ); // 1920
    ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) ); // 1080
    ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) ); // 4

    // Allocation Color Buffer
    colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );
}

// Initialize Body
inline void Kinect::initializeBody()
{
	
    // Open Body Reader
    ComPtr<IBodyFrameSource> bodyFrameSource;
    ERROR_CHECK( kinect->get_BodyFrameSource( &bodyFrameSource ) );
    ERROR_CHECK( bodyFrameSource->OpenReader( &bodyFrameReader ) );

    // Initialize Body Buffer
    Concurrency::parallel_for_each( bodies.begin(), bodies.end(), []( IBody*& body ){
        SafeRelease( body );
    } );

    // Color Table for Visualization
    colors[0] = cv::Vec3b( 255,   0,   0 ); // Blue
    colors[1] = cv::Vec3b(   0, 255,   0 ); // Green
    colors[2] = cv::Vec3b(   0,   0, 255 ); // Red
    colors[3] = cv::Vec3b( 255, 255,   0 ); // Cyan
    colors[4] = cv::Vec3b( 255,   0, 255 ); // Magenta
    colors[5] = cv::Vec3b(   0, 255, 255 ); // Yellow
}


// Finalize
void Kinect::finalize()
{
    cv::destroyAllWindows();

    // Release Body Buffer
    Concurrency::parallel_for_each( bodies.begin(), bodies.end(), []( IBody*& body ){
        SafeRelease( body );
    } );

    // Close Sensor
    if( kinect != nullptr ){
        kinect->Close();
    }
}

// Update Data
void Kinect::update()
{
    // Update Color
    updateColor();

    // Update Body
    updateBody();
}

// Update Color
inline void Kinect::updateColor()
{
    // Retrieve Color Frame
    ComPtr<IColorFrame> colorFrame;
    const HRESULT ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Convert Format ( YUY2 -> BGRA )
    ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray( static_cast<UINT>( colorBuffer.size() ), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );
}

// Update Body
inline void Kinect::updateBody()
{
    // Retrieve Body Frame
    ComPtr<IBodyFrame> bodyFrame;
    const HRESULT ret = bodyFrameReader->AcquireLatestFrame( &bodyFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Release Previous Bodies
    Concurrency::parallel_for_each( bodies.begin(), bodies.end(), []( IBody*& body ){
        SafeRelease( body );
    } );

    // Retrieve Body Data
    ERROR_CHECK( bodyFrame->GetAndRefreshBodyData( static_cast<UINT>( bodies.size() ), &bodies[0] ) );
}

// Draw Data
void Kinect::draw()
{
    // Draw Color
    drawColor();

    // Draw Body
    drawBody();
}

// Draw Color
inline void Kinect::drawColor()
{
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
}



// Draw Body
inline void Kinect::drawBody()
{
    // Draw Body Data to Color Data
    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        const ComPtr<IBody> body = bodies[count];
        if( body == nullptr ){
            return;
        }

        // Check Body Tracked
        BOOLEAN tracked = FALSE;
        ERROR_CHECK( body->get_IsTracked( &tracked ) );
        if( !tracked ){
            return;
        }

        // Retrieve Joints
        std::array<Joint, JointType::JointType_Count> joints;
        ERROR_CHECK( body->GetJoints( static_cast<UINT>( joints.size() ), &joints[0] ) );


        Concurrency::parallel_for_each( joints.begin(), joints.end(), [&]( const Joint& joint ){
            // Check Joint Tracked
            if( joint.TrackingState == TrackingState::TrackingState_NotTracked ){
                return;
            }

            // Draw Joint Position
			if ((joint.JointType == JointType::JointType_HandLeft)
				|| (joint.JointType == JointType::JointType_Head)
				|| (joint.JointType == JointType::JointType_Neck)
				|| (joint.JointType == JointType::JointType_SpineShoulder)
				|| (joint.JointType == JointType::JointType_SpineMid)
				|| (joint.JointType == JointType::JointType_SpineBase)
				|| (joint.JointType == JointType::JointType_ShoulderLeft)
				|| (joint.JointType == JointType::JointType_ShoulderRight)
				|| (joint.JointType == JointType::JointType_HandLeft)
				|| (joint.JointType == JointType::JointType_HandRight)) 
			{
				drawEllipse(colorMat, joint, 5, colors[count]);
			}
			
			// Draw Left Hand State
            if( joint.JointType == JointType::JointType_HandLeft ){
                HandState handState;
                TrackingConfidence handConfidence;
                ERROR_CHECK( body->get_HandLeftState( &handState ) );
                ERROR_CHECK( body->get_HandLeftConfidence( &handConfidence ) );

                drawHandState( colorMat, joint, handState, handConfidence );
            }

            // Draw Right Hand State
            if( joint.JointType == JointType::JointType_HandRight ){
                HandState handState;
                TrackingConfidence handConfidence;
                ERROR_CHECK( body->get_HandRightState( &handState ) );
                ERROR_CHECK( body->get_HandRightConfidence( &handConfidence ) );

                drawHandState( colorMat, joint, handState, handConfidence );
            }

			/// 
			// save current joints in an array
			All_joints[joint.JointType] = joint;
        } );		

 		/// 
        //Retrieve Joint Orientations
        std::array<JointOrientation, JointType::JointType_Count> orientations;
        ERROR_CHECK( body->GetJointOrientations( JointType::JointType_Count, &orientations[0] ) );
		float SpineBaseX=0;
		float SpineBaseY=0;
		float SpineBaseZ=0;
		float SpineBaseYaw = 0;
		float SpineBasePitch = 0;
		float SpineBaseRoll = 0;


		//SpineMid
		float SpineMidX = 0;
		float SpineMidY = 0;
		float SpineMidZ = 0;
		float SpineMidYaw = 0;
		float SpineMidPitch = 0;
		float SpineMidRoll = 0;
		//neck
		float NeckX = 0;
		float NeckY = 0;
		float NeckZ = 0;
		float NeckYaw = 0;
		float NeckPitch = 0;
		float NeckRoll = 0;
		//SpineBase
		float HeadX = 0;
		float HeadY = 0;
		float HeadZ = 0;
		float HeadYaw = 0;
		float HeadPitch = 0;
		float HeadRoll = 0;

		float ShoulderLeftX = 0;
		float ShoulderLeftY = 0;
		float ShoulderLeftZ = 0;
		float ShoulderLeftYaw = 0;
		float ShoulderLeftPitch = 0;
		float ShoulderLeftRoll = 0;
		//SpineBase
		float HandLeftX = 0;
		float  HandLeftY = 0;
		float  HandLeftZ = 0;
		float  HandLeftYaw = 0;
		float  HandLeftPitch = 0;
		float  HandLeftRoll = 0;
		//SpineMid
		float ShoulderRightX = 0;
		float ShoulderRightY = 0;
		float ShoulderRightZ = 0;
		float ShoulderRightYaw = 0;
		float ShoulderRightPitch = 0;
		float ShoulderRightRoll = 0;
		//SpineBase
		float HandRightX = 0;
		float  HandRightY = 0;
		float  HandRightZ = 0;
		float HandRightYaw = 0;
		float HandRightPitch = 0;
		float HandRightRoll = 0;
		//SpineBase
		float SpineShoulderX = 0;
		float SpineShoulderY = 0;
		float SpineShoulderZ = 0;
		float SpineShoulderYaw = 0;
		float SpineShoulderPitch = 0;
		float SpineShoulderRoll = 0;
		// ************ get and print screen coordinates of joints, and their euler angles relative to the parent joints 
		for (int i = 0; i < 25; i++) {
			ColorSpacePoint colorSpacePoint;
			ERROR_CHECK(coordinateMapper->MapCameraPointToColorSpace(All_joints[i].Position, &colorSpacePoint));
			const int x = static_cast<int>(colorSpacePoint.X + 0.5f);
			const int y = static_cast<int>(colorSpacePoint.Y + 0.5f);
			int type = All_joints[i].JointType;
			
			if ((0 <= x) && (x < 1920) && (0 <= y) && (y < 1080)) {
				// print screen coordinates
				
				//cout << "type: " << joint_name[type] << endl;
				//cout << "x: " << x << ", y: " << y;

				//quaternion2Euler
				Vector4 q = orientations[i].Orientation;
				
				double pitch, yaw, roll;
				quaternion2Euler(q, pitch, yaw, roll);

				// calculate rate of change
				V_X[i] = (x - X[i]) / t;
				V_Y[i] = (y - Y[i]) / t;
				Omega_Pitch[i] = (pitch - Pitch[i]) / t;
				Omega_Yaw[i] = (yaw - Yaw[i]) / t;
				Omega_Roll[i] = (roll - Roll[i]) / t;
				//cout << "yaw = " << Yaw[i] << endl;
				

				//<< "Pitch, Yaw, Roll: " << (int)pitch << ", " << (int)yaw << ", " << (int)roll << endl;
				//<< "X, Y, Z: " << (float)q.x << ", " << (float)q.y << ", " << (float)q.z << endl;
				/*                       
				 3,"Head"             2,"Neck"
				20,"SpineShoulder"    1,"SpineMid"             0, "SpineBase" 
				4,"ShoulderLeft" 	  5,"ElbowLeft"           6,"WristLeft"      
				8,"ShoulderRight"     9,"ElbowRight"         10,"WristRight"    
				7,"HandLeft"         21,"HandTipLeft"        22,"ThumbLeft" 
			   11,"HandRight"        23,"HandTipRight"       24,"HandTipRight"   
				
				12,"HipLeft"
				16,"HipRight"
				*/
				

				// store values to calculate rate of change for next time
				X[i] = x; Y[i] = y; Pitch[i] = pitch; Yaw[i] = yaw; Roll[i] = roll;
				
				if(i==3 || i == 2 || i == 20 || i == 0 || i == 4 || i == 7 || 
				   i == 8 || i == 11|| i==1){
					if (i == 0) {
						//SpineBase
						SpineBaseX = All_joints[i].Position.X;
						SpineBaseY = All_joints[i].Position.Y;
						SpineBaseZ = All_joints[i].Position.Z;
						SpineBaseYaw = Yaw[i];
						SpineBasePitch = Pitch[i];
						SpineBaseRoll = Roll[i];
					}
					else if (i == 1) {
						//SpineMid
						SpineMidX = All_joints[i].Position.X;
						SpineMidY = All_joints[i].Position.Y;
						SpineMidZ = All_joints[i].Position.Z;
						SpineMidYaw = Yaw[i];
						SpineMidPitch = Pitch[i];
						SpineMidRoll = Roll[i];
					}
					else if (i == 2) {
						//Neck
						NeckX = All_joints[i].Position.X;
						NeckY = All_joints[i].Position.Y;
						NeckZ = All_joints[i].Position.Z;
						SpineMidYaw = Yaw[i];
						SpineMidPitch = Pitch[i];
						SpineMidRoll = Roll[i];
					}
					else if (i == 3) {
						// Head
						HeadX = All_joints[i].Position.X;
						HeadY = All_joints[i].Position.Y;
						HeadZ = All_joints[i].Position.Z;
						SpineMidYaw = Yaw[i];
						SpineMidPitch = Pitch[i];
						SpineMidRoll = Roll[i];
					}
					else if (i == 4) {
						//ShoulderLeft
						ShoulderLeftX = All_joints[i].Position.X;
						ShoulderLeftY = All_joints[i].Position.Y;
						ShoulderLeftZ = All_joints[i].Position.Z;
						SpineMidYaw = Yaw[i];
						SpineMidPitch = Pitch[i];
						SpineMidRoll = Roll[i];
					}
					else if (type == 11) {
						// HandLeft
						HandLeftX = All_joints[i].Position.X;
						HandLeftY = All_joints[i].Position.Y;
						HandLeftZ = All_joints[i].Position.Z;
						SpineMidYaw = Yaw[i];
						SpineMidPitch = Pitch[i];
						SpineMidRoll = Roll[i];
					}
					else if (type == 8) {
						//ShoulderRight
						ShoulderRightX = All_joints[i].Position.X;
						ShoulderRightY = All_joints[i].Position.Y;
						ShoulderRightZ = All_joints[i].Position.Z;
						SpineMidYaw = Yaw[i];
						SpineMidPitch = Pitch[i];
						SpineMidRoll = Roll[i];
					}
					else if (type == 7) {
						//HandRight
						HandRightX = All_joints[i].Position.X;
						HandRightY = All_joints[i].Position.Y;
						HandRightZ = All_joints[i].Position.Z;
						SpineMidYaw = Yaw[i];
						SpineMidPitch = Pitch[i];
						SpineMidRoll = Roll[i];
					}
					else if (type == 20) {
						//SpineShoulder
						SpineShoulderX = All_joints[i].Position.X;
						SpineShoulderY = All_joints[i].Position.Y;
						SpineShoulderZ = All_joints[i].Position.Z;
						SpineShoulderYaw = Yaw[i];
						SpineShoulderPitch = Pitch[i];
						SpineShoulderRoll = Roll[i];
					}
					
					

										
					
				}
			}			
		}
		OSCSendMessage message;
		string oscMessages = "/wek/inputs";
		message.setMesseageToSendData(oscMessages, HandLeftX, HandLeftY, HandLeftZ, HandRightX, HandRightY, HandRightZ, NeckX, NeckY, NeckZ,
			HandLeftYaw, HandLeftPitch, HandLeftRoll, HandRightYaw, HandRightPitch, HandRightRoll, NeckYaw, NeckPitch, NeckRoll);
		


        /* 
		// Retrieve Amount of Body Lean
        PointF amount;
        ERROR_CHECK( body->get_Lean( &amount ) );
		*/
       
        
    } );
}

// Draw Ellipse
inline void Kinect::drawEllipse( cv::Mat& image, const Joint& joint, const int radius, const cv::Vec3b& color, const int thickness )
{
    if( image.empty() ){
        return;
    }

    // Convert Coordinate System and Draw Joint
    ColorSpacePoint colorSpacePoint;
    ERROR_CHECK( coordinateMapper->MapCameraPointToColorSpace( joint.Position, &colorSpacePoint ) );
    const int x = static_cast<int>( colorSpacePoint.X + 0.5f );
    const int y = static_cast<int>( colorSpacePoint.Y + 0.5f );
    if( ( 0 <= x ) && ( x < image.cols ) && ( 0 <= y ) && ( y < image.rows ) ){
        cv::circle( image, cv::Point( x, y ), radius, static_cast<cv::Scalar>( color ), thickness, cv::LINE_AA );
	}
	
}


// save all joints
/*inline void Kinect::saveJoints(const Joint& joint, Joint* All_joints) {
	
	All_joints[joint.JointType] = joint;
}*/

const double PI = std::atan(1.0) * 4;
inline void Kinect::quaternion2Euler(Vector4& q, double& pitch, double& yaw, double& roll) {
	double x = q.x;
	double y = q.y;
	double z = q.z;
	double w = q.w;

	// convert face rotation quaternion to Euler angles in degrees
	double yawD, pitchD, rollD;
	pitchD = atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / PI * 180.0;
	yawD = asin(2 * ((w * y) - (x * z))) / PI * 180.0;
	rollD = atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / PI * 180.0;

	// clamp the values to a multiple of the specified increment to control the refresh rate
	double increment = 1;
	pitch = (int)(floor((pitchD + ((increment / 2.0) * (pitchD > 0 ? 1.0 : -1.0))) / increment) * increment);
	yaw = (int)(floor((yawD + ((increment / 2.0) * (yawD > 0 ? 1.0 : -1.0))) / increment) * increment);
	roll = (int)(floor((rollD + ((increment / 2.0) * (rollD > 0 ? 1.0 : -1.0))) / increment) * increment);
}

// Draw Hand State
inline void Kinect::drawHandState( cv::Mat& image, const Joint& joint, HandState handState, TrackingConfidence handConfidence )
{
    if( image.empty() ){
        return;
    }

    // Check Tracking Confidence
    if( handConfidence != TrackingConfidence::TrackingConfidence_High ){
        return;
    }

    // Draw Hand State 
    const int radius = 75;
    const cv::Vec3b blue = cv::Vec3b( 128, 0, 0 ), green = cv::Vec3b( 0, 128, 0 ), red = cv::Vec3b( 0, 0, 128 );
    switch( handState ){
        // Open
        case HandState::HandState_Open:
            drawEllipse( image, joint, radius, green, 5 );
            break;
        // Close
        case HandState::HandState_Closed:
            drawEllipse( image, joint, radius, red, 5 );
            break;
        // Lasso
        case HandState::HandState_Lasso:
            drawEllipse( image, joint, radius, blue, 5 );
            break;
        default:
            break;
    }
}

// Show Data
void Kinect::show()
{
    // Show Body
    showBody();
}

// Show Body
inline void Kinect::showBody()
{
    if( colorMat.empty() ){
        return;
    }

    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( colorMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "Body", resizeMat );


}
