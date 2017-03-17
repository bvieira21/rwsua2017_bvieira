/**
 *      @file  player_bvieira_node.cpp
 *      @brief  
 *
 * 		Contains the player creation and behaviour
 *
 *      @author   Bruno Vieira - bruno.v@ua.pt
 *
 *   	@internal
 *     	Created  10-Mar-2017
 *     	Company  University of Aveiro
 *   	Copyright  Copyright (c) 2017, Live session user
 *
 * =====================================================================================
 */
 #



#include <iostream>  
#include <vector>    
//ROS INCLUDES
#include <ros/ros.h>  
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <rwsua2017_libs/player.h>   
#include <rwsua2017_msgs/MakeAPlay.h>                                                                            

using namespace std;

namespace rwsua2017 {

    
    class MyPlayer : public Player //inherits the class Player
    {
    public:

        ros::Subscriber sub;

        tf::TransformListener listener;

        tf::TransformBroadcaster br;
        tf::Transform t1;

		 /**
		 * @brief  Class constructor, initializes the player 
		 * @param[in] argin_name - player name
		 * @param[in] argin_team_name - player team name (red, green or blue)
		 * @return  
	     * @author B.Vieira
		 */
        MyPlayer(string argin_name, string argin_team_name) : Player(argin_name, argin_team_name) {
          
            sub = n.subscribe("/make_a_play/cat", 1000, &MyPlayer::makeAPlayCallback, this);
            cout << "Initialized MyPlayer" << endl;

         

            t1.setOrigin(tf::Vector3(1, 1, 0));
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            t1.setRotation(q);
            br.sendTransform(tf::StampedTransform(t1, ros::Time::now(), "map", name));

        }
        
		 /**
		 * @brief  Generation of a ramdom number
		 * @return Random number generated  
	     * @author B.Vieira
		 */
        double randNumber() {
            struct timeval t1;
            gettimeofday(&t1, NULL);
            srand(t1.tv_usec);
            double x = ((((double) rand() / (double) RAND_MAX)*2 - 1)*5);

            return x;
        }
		
		/**
		 * @brief  Calculates the angle to the input player 
		 * @param[in] player_name - player name
		 * @return Angle to point for selected player  
	     * @author B.Vieira
		 */
        float getAngleTo(string player_name) {


            tf::StampedTransform trans;
            try {
                listener.lookupTransform(name, player_name, ros::Time(0), trans);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            float x = trans.getOrigin().x();
            float y = trans.getOrigin().y();

            cout << "x=" << x << " y= " << y << endl;

            

            return atan2(y, x);
        }

		 /**
		 * @brief  Callback executed each time this event is triggered (by the referee, check!)
		 * @param[in] 
		 *  @author B.Vieira
		 */
        void makeAPlayCallback(const rwsua2017_msgs::MakeAPlay::ConstPtr & msg) {

            cout << "msg: max displacement -> " << msg->max_displacement << endl;

            //definição dos angulos de rotação e valores de translação
            //deveria ser calculado pela AI do sistema

            float turn_angle = getAngleTo("vsilva");
            //float turn_angle = M_PI/30;
            float displacement = msg->max_displacement;

            double max_t = (M_PI / 30);
            if (turn_angle > max_t) turn_angle = max_t;
            else if (turn_angle < -max_t) turn_angle = -max_t;

            //Compute the new reference frame
            tf::Transform t_mov;
            tf::Quaternion q;
            q.setRPY(0, 0, turn_angle);
            t_mov.setRotation(q);
            t_mov.setOrigin(tf::Vector3(displacement, 0.0, 0.0));

            tf::Transform t = t1 * t_mov;
            //Send the new transform to ROS
            br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));
            t1 = t;
        }


        vector<string> teammates;
    };
}



int main(int argc, char **argv) {
    //because we used <using namespace std>  we can replace the other line
    //std::cout << "Hello world" << std::endl;
    cout << "Hello world" << endl;

    ros::init(argc, argv, "player_bvieira");
    
    //Creating an instance of class Player
    rwsua2017::MyPlayer myplayer("bvieira", "red");



    //cout << "Created an instance of class player with public name " << player.name << endl;
    cout << "name = " << myplayer.name << endl;
    cout << "team name = " << myplayer.get_team_name() << endl;

    myplayer.teammates.push_back("rodolfo");
    myplayer.teammates.push_back("arnaldo");



    // size_t = unsigned long int (sortchut)
    for (size_t i = 0; i < myplayer.teammates.size(); ++i) {
        cout << myplayer.teammates[i] << endl;

    }

    ros::spin();

    return 1;
}




