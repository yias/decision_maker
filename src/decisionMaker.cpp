




#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <decision_maker/vtmsg.h>
#include "sensor_msgs/JointState.h"


#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <unistd.h>
#include <termios.h>


#include "decision_maker/MatlabLib.h"
#include "decision_maker/additional_functions.h"


struct stat st = {0};

enum hand{
    left= 0,
    right=1,
};

int sRate=300;                                                   // set the sample rate (Hz)
int trialCounter=0;                                              // counter for the trials
double startTime;



/*-- Variables related to the mocap system --*/

int mocapCounter=0;                                              // counter for messages from the mocap system
double lookBack=0.1;                                             // the timewindow to look back for the average velocity
double velThreshold=0.018;                                       // velocity threshold for destinguish the motion or no=motion of the hand
int mocapRate=250;                                               // the sample rate of the motion capture system

std::vector<double> mocapPosition(3,0);                          // vector for the position of the hand
std::vector<double> mocapVelocity(3,0);                          // vector for the velocity of the hand

std::vector< std::vector<double> > mocapHistoryPosition(3);      // vector for the position of the marker on hand
std::vector< std::vector<double> > mocapHistoryVelocity(3);      // vector for velocity of the marker on hand
std::vector<double> velocityNormHistory;                         // vector for the norm of the velocity of the hand

std::vector<double> mocapTime;                                   // timestamp for the mocap system
std::vector<double> checkVelocityHistory;                           // history of the velocity checking


/*-- Variables related to the windows system --*/

int daqCounter=0;                                                // counter for messages from the windows machine

int nbClasses=5;                                                 // number of different classes

int grasp_type=0;                                                // the outecome of the majority vote

double grasp_threshold=0.2;                                      // confidence threshold to change the grasp

std::vector<int> mVotes;                                         // a vector to store the classification outcome for each time window

std::vector<int> graspTypeHistory;                               // a vector with all the grasptypes

std::vector<double> graspTime;                                   // the time stamp of the listener of the windows machine



/*-- Variables related to the allegro hand --*/

int allegroCounter=0;                                            // counter for messages from the mocap system

std::vector< std::vector<double> > rightHandHistory(17);         // a matrix to store the joint angles of the right Allego hand (the extra row corresponds to the time stamp)
std::vector< std::vector<double> > leftHandHistory(17);          // a matrix to store the joint angles of the left Allego hand (the extra row corresponds to the time stamp)

double initialTime;                                              // the time that the first message with the joint angles of the Allegro hand was reveived


/*-- functions for the system --*/

void saveRecordings();                                           // a function to save the data after each trial
int getch_();                                                    // a function to catch a key press asynchronously




/*-- Callback functions --*/

void daqListener(const decision_maker::vtmsg daqmsg){



    /*-- Callback function for subscriber of the windows machine --*/

    graspTime.push_back((ros::Time::now().toSec())-startTime);



    if(daqCounter>0){
    graspTime.push_back((ros::Time::now().toSec())-startTime);
    mVotes.push_back(daqmsg.vote);

    //checkVelocityHistory.push_back(check_velocity(velocityNormHistory.back(),velThreshold));

    if(check_velocity(velocityNormHistory.back(),velThreshold)) {

        grasp_type=majority_vote(mVotes,nbClasses, grasp_threshold,grasp_type);

        std::cout<<"grasp type: "<<grasp_type<<"\n";
        graspTypeHistory.push_back(grasp_type);

    }
    }

    daqCounter++;

    //ROS_INFO("I heard: [%d] messages from daq\n", daqCounter);
}

void mocapListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/

    mocapPosition[0]=mocapmsg.pose.position.x;
    mocapPosition[1]=mocapmsg.pose.position.y;
    mocapPosition[2]=mocapmsg.pose.position.z;


    mocapTime.push_back((ros::Time::now().toSec())-startTime);

    for(int i=0;i<3;i++){

        mocapHistoryPosition[i].push_back(mocapPosition[i]);

    }

    if(mocapCounter>0){
       std::vector<double> previousSample(3,0);
       for(int i=0;i<3;i++){
           previousSample[i]=mocapHistoryPosition[i][mocapCounter-1];
       }

       //std::cout<<"x: "<<previousSample[0]<<", y: "<<previousSample[1]<<",z: "<<previousSample[2]<<"\n";
       mocapVelocity=calcDtVelocity(mocapPosition,previousSample,(double)1/std::min((double)sRate,(double)mocapRate));

        for(int i=0;i<3;i++){
            mocapHistoryVelocity[i].push_back(mocapVelocity[i]);
        }
        velocityNormHistory.push_back(velocityNorm(mocapHistoryVelocity,(int)(lookBack*sRate)));
        //std::cout<<"\nvel: "<<velocityNormHistory.back()<<"\n";
        //checkVelocityHistory.push_back(check_velocity(velocityNormHistory.back(),velThreshold));
        //checkVelocityHistory.push_back(1);
       // std::cout<<"velocity: " << check_velocity(velocityNormHistory.back(),velThreshold) << "\n";// << velocityNormHistory.back() << " "



    }

    //std::cout<<"mocap listener okokkokoko\n";


    mocapCounter++;
    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}


void rightHandListener(const sensor_msgs::JointState& jointStatemsg){

    /*-- Callback function for subscriber of the right Allegro hand --*/

    for(int i=0;i<16;i++){

         rightHandHistory[i].push_back(jointStatemsg.position[i]);
    }


    if(allegroCounter<1) {

        rightHandHistory[16].push_back((ros::Time::now().toSec())-startTime);

        //initialTime=tm2double(jointStatemsg.header.stamp.sec,jointStatemsg.header.stamp.nsec);

    }else{
        //double tmp=tm2double(jointStatemsg.header.stamp.sec,jointStatemsg.header.stamp.nsec)-initialTime;

        //rightHandHistory[16].push_back(tmp);
        rightHandHistory[16].push_back((ros::Time::now().toSec())-startTime);


    }

    allegroCounter++;

}


void leftHandListener(const sensor_msgs::JointState& jointStatemsg){

    /*-- Callback function for subscriber of the left Allegro hand --*/

    for(int i=0;i<16;i++){

         leftHandHistory[i].push_back(jointStatemsg.position[i]);
    }


    if(allegroCounter<1) {

        leftHandHistory[16].push_back((ros::Time::now().toSec())-startTime);

        //initialTime=tm2double(jointStatemsg.header.stamp.sec,jointStatemsg.header.stamp.nsec);

    }else{
        //double tmp=tm2double(jointStatemsg.header.stamp.sec,jointStatemsg.header.stamp.nsec)-initialTime;

        //leftHandHistory[16].push_back(tmp);
        leftHandHistory[16].push_back((ros::Time::now().toSec())-startTime);

    }

    allegroCounter++;

}



int main(int argc, char **argv)
{


    // set the configurations of the hands
    std::vector< std::vector<double> > rightHandConfiguration=handConfiguations(right, nbClasses);

    std::vector< std::vector<double> > leftHandConfiguration=handConfiguations(left, nbClasses);

    // set the velocity of the joints to zero (maximum)
    std::vector<double> joint_velocity(16,0);


    // set the messages that will include the desired joint angles to the allegro node. This message will contain the velocity and the position of the joints
    sensor_msgs::JointState righHand_msg;

    sensor_msgs::JointState leftHand_msg;

    // set the message for publishing the graps type
    decision_maker::vtmsg graspmsg;


    // resize the vectors of position and velocity according to the numbers of DOF of the allegro hand
    righHand_msg.velocity.resize(16);
    righHand_msg.position.resize(16);
    leftHand_msg.velocity.resize(16);
    leftHand_msg.position.resize(16);


    // setting the velocity of the joints to maximum
    righHand_msg.velocity=joint_velocity;
    leftHand_msg.velocity=joint_velocity;


    // initialize the node
    ros::init(argc, argv, "decisionMaker");

    ros::NodeHandle n;


    // set the publishers for the allegro hand

    ros::Publisher allegorRight_pub = n.advertise<sensor_msgs::JointState>("allegroHand_1/joint_cmd", 100);

    ros::Publisher allegorLeft_pub = n.advertise<sensor_msgs::JointState>("allegroHand_0/joint_cmd", 100);



    // set a publisher for publishing the grasp type

    ros::Publisher graspType_pub=n.advertise<decision_maker::vtmsg>("decisionMaker/grasp_type", 100);



    // set the subscribers to listen the classification outcome from the windows machine and the position of the hand

    ros::Subscriber daqSub = n.subscribe("win_pub", 2, daqListener);

    ros::Subscriber mocapSub=n.subscribe("HAND/pose", 10, mocapListener);



    // set subscribers to listen the joint states of the Allegro hands

    ros::Subscriber allegorRight_sub = n.subscribe("allegroHand_1/joint_states", 1000, rightHandListener);

    ros::Subscriber allegorLeft_sub=n.subscribe("allegroHand_0/joint_states", 1000, leftHandListener);



    startTime=ros::Time::now().toSec();

    // set the loop rate
    ros::Rate loop_rate(sRate);




    int count = 0;
    while (ros::ok())
    {
        // set the joint position of the allegro hand

        righHand_msg.position=rightHandConfiguration[grasp_type];
        leftHand_msg.position=leftHandConfiguration[grasp_type];

        // set the grasp type

        graspmsg.vote=grasp_type;

        // publish the messages

        allegorRight_pub.publish(righHand_msg);
        allegorLeft_pub.publish(leftHand_msg);
        graspType_pub.publish(graspmsg);

        // if the key 't' is pressed, save the data and clear the data for the next trial

        if(getch_()=='t'){
            saveRecordings();

        }



        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}


void saveRecordings(){

    /*
     * This function saves the data of each trial to mat files and clears the vector
     *
     *
     */



    // create directory inside the folder data for saving the data of the trial

    std::string dirName="src/decision_maker/data/trial"+std::to_string(trialCounter+1);

    if (stat(dirName.c_str(), &st) == -1) {
        mkdir(dirName.c_str(), 0700);
    }


    // save the data to the folder

    if(!mocapHistoryPosition[0].empty()) saveData("src/decision_maker/data/trial"+std::to_string(trialCounter+1)+"/mocapHistoryPosition","mocapHistoryPosition",mocapHistoryPosition);
    if(!mocapHistoryVelocity[0].empty()) saveData("src/decision_maker/data/trial"+std::to_string(trialCounter+1)+"/mocapHistoryVelocity","mocapHistoryVelocity",mocapHistoryVelocity);

    if(!mocapTime.empty()) writeMatFile("src/decision_maker/data/trial"+std::to_string(trialCounter+1)+"/mocapTime","mocapTime",mocapTime);
    //if(!checkVelocityHistory.empty()) writeMatFile("src/decision_maker/data/trial"+std::to_string(trialCounter+1)+"/checkVelocityHistory","checkVelocityHistory",checkVelocityHistory);
    if(!graspTypeHistory.empty()) writeMatFile("src/decision_maker/data/trial"+std::to_string(trialCounter+1)+"/graspTypeHistory","graspTypeHistory",graspTypeHistory);
    if(!velocityNormHistory.empty()) writeMatFile("src/decision_maker/data/trial"+std::to_string(trialCounter+1)+"/velocityNormHistory","velocityNormHistory",velocityNormHistory);
    if(!graspTime.empty()) writeMatFile("src/decision_maker/data/trial"+std::to_string(trialCounter+1)+"/graspTime","graspTime",graspTime);

    if(!rightHandHistory[0].empty()) saveData("src/decision_maker/data/trial"+std::to_string(trialCounter+1)+"/rightHandHistory","rightHandHistory",rightHandHistory);
    if(!leftHandHistory[0].empty()) saveData("src/decision_maker/data/trial"+std::to_string(trialCounter+1)+"/leftHandHistory","leftHandHistory",leftHandHistory);


    std::cout<<"\nTrial " << trialCounter+1 <<" has been saved\n";

    // clear the data for the next trial

    for(int i=0;i<(int)mocapHistoryPosition.size();i++){
        mocapHistoryPosition[i].clear();
    }

    for(int i=0;i<(int)mocapHistoryVelocity.size();i++){
        mocapHistoryVelocity[i].clear();
    }

    mocapTime.clear();
    checkVelocityHistory.clear();
    graspTypeHistory.clear();
    mVotes.clear();
    velocityNormHistory.clear();

    if(!rightHandHistory[0].empty()){
        for(int i=0;i<(int)rightHandHistory.size();i++){
            rightHandHistory[i].clear();
        }
    }

    if(!leftHandHistory[0].empty()){
        for(int i=0;i<(int)leftHandHistory.size();i++){
            leftHandHistory[i].clear();
        }
    }

    std::cout<<"Data cleared\n";
    std::cout<<"Ready for the next trial\n";
    trialCounter++;
    grasp_type=0;
    mocapCounter=0;
    daqCounter=0;
    allegroCounter=0;
    startTime=ros::Time::now().toSec();

}

int getch_(){
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);                  // save old settings

  newt = oldt;
  newt.c_lflag &= ~(ICANON);                        // disable buffering
  newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);         // apply new settings

  int c = getchar();                                // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);         // restore old settings
  return c;
}


