#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

using namespace std;
using namespace cv;


ros::Publisher topicinfo;
ros::Publisher vel_sp_pub;
float longitud, latitud, altitud;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


geometry_msgs::TwistStamped changeTwist(float x, float y, float z, float turn){
    geometry_msgs::TwistStamped msg_vel;
    msg_vel.twist.angular.x=0;
    msg_vel.twist.angular.y=0;
    msg_vel.twist.angular.z=turn;
    msg_vel.twist.linear.x=x;
    msg_vel.twist.linear.y=y;
    msg_vel.twist.linear.z=z;
    return(msg_vel);
}


//forward
void forwardx (void){
    geometry_msgs::TwistStamped msg_vel;
    msg_vel=changeTwist(0.5,0,0,0);
    // topic publishing
    vel_sp_pub.publish(msg_vel);
}
//left
void forwardy (void){
    geometry_msgs::TwistStamped msg_vel;
    msg_vel=changeTwist(0,0.5,0,0);
    // topic publishing
    vel_sp_pub.publish(msg_vel);
}
//backward
void backx (void){
    geometry_msgs::TwistStamped msg_vel;
    msg_vel=changeTwist(-0.5,0,0,0);
    // topic publishing
    vel_sp_pub.publish(msg_vel);
}
//right
void backy (void){
    geometry_msgs::TwistStamped msg_vel;
    msg_vel=changeTwist(0,-0.5,0,0);
    // topic publishing
    vel_sp_pub.publish(msg_vel);
}
//up
void up (void){
    geometry_msgs::TwistStamped msg_vel;
    msg_vel=changeTwist(0,0,0.5,0);
    // topic publishing
    vel_sp_pub.publish(msg_vel);
}
//down
void down (void){
    geometry_msgs::TwistStamped msg_vel;
    msg_vel=changeTwist(0,0,-0.5,0);
    // topic publishing
    vel_sp_pub.publish(msg_vel);
}
void stop (void){
    geometry_msgs::TwistStamped msg_vel;
    msg_vel=changeTwist(0,0,0,0);
    // topic publishing
    vel_sp_pub.publish(msg_vel);
}
//====================================================================================

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_sub_depth;
    image_transport::Subscriber image_sub_threshold;
    image_transport::Publisher image_pub_;
    image_transport::Publisher image_pub_depth;
    image_transport::Publisher image_pub_threshold;
    cv::HOGDescriptor hog;
   // static float _hitThreshold;
   // static float _scaleFactor;
   // static float _groupThreshold;

public:
    ImageConverter()
        : it_(nh_), hog()
    {
	//subscribe to camera image
        image_sub_=it_.subscribe("/cv_camera/image_raw",1,&ImageConverter::imageCb,this); 
        //cv::namedWindow(OPENCV_WINDOW);
        image_pub_=it_.advertise("/image_converter/output_video",1);

        //set hog detector
        hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        
    }
    
    virtual ~ImageConverter()
    {
        //cv::destroyWindow(OPENCV_WINDOW);
    }

   // image_raw topic에 대한 call back 함수
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv::Mat frame;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
	// 이미지를 opencv가 처리할 수 있는 형태인 Mat 타입으로 변환
	frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        //frame = cv::imdecode(cv::Mat(msg->data),1);
        
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

    /*
    cv::Size size = frame.size();
    CvMat * A = cvCreateMat(size.height,size.width,CV_8UC1);
    cv::Mat mask = cv::cvarrToMat(A);

    //morphological closing (fill small holes in the foreground
    // 필터 내부의 가장 높은(밝은)값으로 변환 - 팽창연산, 한 객체를 추출했을 때 두개 이상의 작은 부분으로 나올 경우 큰 객체로 합칠 때 사용
    dilate(mask,mask,getStructuringElement(MORPH_RECT,Size(21,21)));
    // 필터 내부의 가장 낮은(어두운)값으로 변환 - 침식연산, 작은 노이즈 제거
    erode(mask, mask, getStructuringElement(MORPH_RECT,Size(5,5)));

    //opening
    erode(mask,mask,getStructuringElement(MORPH_RECT,Size(11,11)));
    dilate(mask,mask,getStructuringElement(MORPH_RECT,Size(5,5)));

    GaussianBlur(mask, mask, Size(15,15), 2, 2);
    std::vector<Vec3f> circles;
    imshow("filter",mask);
*/
    // HOG people detect 시작
    vector<Rect> found, found_filtered;
    //hog.detectMultiScale(mask, found, _hitThreshold, Size(8, 8), Size(32, 32), _scaleFactor, _groupThreshold);
    hog.detectMultiScale(frame, found, 0, Size(8, 8), Size(32, 32), 1.059, 0.0);

    int i, j;
    // overlaaping 된 rectangle 제거
    for(i = 0; i < found.size(); i++){
        Rect r = found[i];
        for(j = 0; j < found.size(); j++){
            if(j!=i){
                Rect iRect = r;
                Rect jRect = found[j];
                Rect intersectRect = (iRect & jRect);
                if (intersectRect.area() >= iRect.area()*0.9) break;
            }
        }
        if (j == found.size()) found_filtered.push_back(r);
    }

    // HOG detector는 실제 객체보다 조금 더 큰 bounding box를 return하게 된다. 따라서 bounding box의 크기를 조금 줄여주어야 한다.
    float x, y, rect_size;
    for (i = 0; i < found_filtered.size(); i++){
        Rect r = found_filtered[i];
        r.x += cvRound(r.width*0.5);
        r.x += -40;
        r.width = cvRound(r.width*0.3);
        r.y += cvRound(r.height*0.07);
        r.height = cvRound(r.height*0.8);
        rectangle(frame, r.tl(), r.br(), cv::Scalar(0, 255, 0), 3);
        x = (r.x*2 + r.width)/2; // 중심좌표의 x 값 = (x1 + x2) /2 (x2 = x1 + width)
        y = (r.y*2 + r.height)/2; // 중심좌표의 y 값 = (y1 + y2) / 2 (y2 = y1 + heigth)
        rect_size = r.width/2;
        ROS_INFO("people detected! x = %f, y = %f", x, y);
    }



//in my case, the resolution of camera is 640x480, the range of radius of cicle detected and the region of left, right, up and down sides should be adjusted according to your camera properties

    cv::imshow("people detect", frame);
            bool temp1,temp2; //help variables
            if(found_filtered.size()>0)
                {
                    //printf("detect \n");

            if(rect_size<40 && found_filtered.size()>0)
                {
                    forwardx();
                    //printf("forward \n");
                    temp1=true;
                }
            if(rect_size>60)
                {
                    backx();
                    printf("back \n");
                    temp1=true;
                }
            if(rect_size<60 && rect_size>40)
                {
                    if(temp1==true)
                    {
                        stop();
                        //printf("stop \n");
                        temp1=false;
                    }
                    if(y<200 && found_filtered.size()>0) //go up
                    {
                        up();
                        //printf("up \n");
                        temp2=true;
                    }
                    if(y>280 && found_filtered.size()>0) //go down
                    {
                        down();
                        //printf("down \n");
                        temp2=true;
                    }
                    if(y<280 && y>200)
                    {
                        if(temp2==true)
                        {
                            stop();
                            //printf("stop \n");
                            temp2=false;
                        }
                        if(x>360 && found_filtered.size()>0)
                        {
                            backy();
                            //printf("right \n");
                        }
                        if(x<280 && found_filtered.size()>0)
                        {
                            forwardy();
                            //printf("left \n");
                        }
                        if(x<360 && x>280)
                        {
                            stop();
                            //printf("stop \n");
                        }
                    }
                 
		}
                }
                       

            else if(found_filtered.size()==0)
                {
                    stop();
                }

    }
};




//===========================================================================================

int main(int argc, char **argv){
    // mavros 노드에 움직임 명령 보내는 노드 init
    ros::init(argc, argv, "drone_prj_offboard");
    ros::NodeHandle nh;
    // Subscriber, Publisher init
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    vel_sp_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    // people detect를 위한 노드 init
    ros::init(argc, argv, "image_converter");


    // mavros msg를 publish 하기 위해서 반드시 2Hz보다 빨라야 한다.
    ros::Rate rate(20.0);
    // FCU connection 기다림
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    ImageConverter converter;

    while(ros::ok()){
    	ros::spinOnce();
    }

    return 0;

}


      
