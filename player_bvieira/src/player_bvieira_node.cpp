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

#define time_to_wait 0.1

#include <iostream>  
#include <vector>    
//ROS INCLUDES
#include <ros/ros.h>  
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <rwsua2017_libs/player.h>   
#include <rwsua2017_msgs/MakeAPlay.h>

#include <visualization_msgs/Marker.h>

using namespace std;

namespace rwsua2017 {

    class MyPlayer : public Player //inherits the class Player
    {
    public:

        ros::Subscriber sub;
        tf::TransformListener listener;
        tf::TransformBroadcaster br;
        ros::Publisher vis_pub;

        /**
         * @brief  Class constructor, initializes the player 
         * @param[in] argin_name - player name
         * @param[in] argin_team_name - player team name (red, green or blue)
         * @return  
         * @author B.Vieira
         */
        MyPlayer(string argin_name, string argin_team_name) : Player(argin_name, argin_team_name) {

            sub = n.subscribe("/make_a_play/cat", 1000, &MyPlayer::makeAPlayCallback, this);

            vis_pub = n.advertise<visualization_msgs::Marker>("/bocas", 0);

            tf::Transform t1;
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
            ros::Time now = ros::Time(0); //gets the latest transform received
            try {
                listener.waitForTransform(name, player_name, now, ros::Duration(time_to_wait));
                listener.lookupTransform(name, player_name, now, trans);
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
         * @brief  Gets my player position form referee
         * @return My position  
         * @author B.Vieira
         */
        tf::StampedTransform getPose(void) {

            tf::StampedTransform trans;
            ros::Time now = ros::Time(0); //gets the latest transform received
            try {
                listener.waitForTransform("/map", name, now, ros::Duration(time_to_wait));
                listener.lookupTransform("/map", name, now, trans);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            return trans;
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

            //string neaby_players = getNearbyPlayers(); //in 1 is the prey, 2 the hunter
            std::vector<std::string> enemies = getNearbyPlayers();
            float turn_angle;
            
            if (enemies[2]=="run")
                turn_angle=getAngleTo(enemies[1])+5*M_PI/6;
            else
                turn_angle=getAngleTo(enemies[0]);

            
            float displacement = msg->max_displacement;



            //move my player
            move(displacement, turn_angle, msg->max_displacement, M_PI / 30);

            //enviar boca
            visualization_msgs::Marker marker;
            marker.header.frame_id = name;
            marker.header.stamp = ros::Time();
            marker.ns = name;
            marker.id = 0;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0.4;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.z = 0.4;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.3;
            marker.color.g = 0.3;
            marker.color.b = 0.3;
            marker.frame_locked = 1;
            marker.lifetime = ros::Duration(1);
            marker.text = "atencao: isto e uma boca!";
            vis_pub.publish(marker);
        }

        /**
         * @brief  Generation of a ramdom number
         * @return Random number generated  
         * @author B.Vieira
         */
        void move(float displacement, float turn_angle, float max_displacement, float max_turn_angle) {

            //saturation of turn_angle
            if (turn_angle > max_turn_angle) turn_angle = max_turn_angle;
            else if (turn_angle < -max_turn_angle) turn_angle = -max_turn_angle;

            //saturation of displacement
            if (displacement > max_displacement) displacement = max_displacement;
            else if (displacement < -max_displacement) displacement = -max_displacement;


            //Compute the new reference frame
            tf::Transform t_mov;
            tf::Quaternion q;
            q.setRPY(0, 0, turn_angle);
            t_mov.setRotation(q);
            t_mov.setOrigin(tf::Vector3(displacement, 0.0, 0.0));

            tf::Transform t = getPose() * t_mov;
            //Send the new transform to ROS
            br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));
        }

        std::vector<std::string> getNearbyPlayers() {

            tf::StampedTransform trans1, trans2, trans3, trans4, trans5, trans6;
            ros::Time now = ros::Time(0); //gets the latest transform received
            try {
                listener.waitForTransform(name, "jferreira", now, ros::Duration(time_to_wait));
                listener.lookupTransform(name, "jferreira", now, trans1);

                listener.waitForTransform(name, "fsilva", now, ros::Duration(time_to_wait));
                listener.lookupTransform(name, "fsilva", now, trans2);

                listener.waitForTransform(name, "rmartins", now, ros::Duration(time_to_wait));
                listener.lookupTransform(name, "rmartins", now, trans3);

                listener.waitForTransform(name, "vsilva", now, ros::Duration(time_to_wait));
                listener.lookupTransform(name, "vsilva", now, trans4);

                listener.waitForTransform(name, "dcorreia", now, ros::Duration(time_to_wait));
                listener.lookupTransform(name, "dcorreia", now, trans5);

                listener.waitForTransform(name, "jsousa", now, ros::Duration(time_to_wait));
                listener.lookupTransform(name, "jsousa", now, trans6);

            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }


            float x1, x2, x3, x4, x5, x6;
            float y1, y2, y3, y4, y5, y6;
            float norm1, norm2, norm3, norm4, norm5, norm6;

            string prey_name, hunter_name;
            float dists[2];

            // ===================== PREYS =============================
            x1 = trans1.getOrigin().x();
            y1 = trans1.getOrigin().y();

            x2 = trans2.getOrigin().x();
            y2 = trans2.getOrigin().y();

            x3 = trans3.getOrigin().x();
            y3 = trans3.getOrigin().y();

            norm1 = sqrt(x1 * x1 + y1 * y1);
            norm2 = sqrt(x2 * x2 + y2 * y2);
            norm3 = sqrt(x3 * x3 + y3 * y3);

            if (norm1 < norm2 && norm1 < norm3) {
                prey_name = "jferreira";
                dists[0] = norm1;
            } else if (norm2 < norm1 && norm2 < norm3) {
                prey_name = "fsilva";
                dists[0] = norm2;
            } else {
                prey_name = "rmartins";
                dists[0] = norm3;
            }

            // ===================== HUNTERS ========================
            x4 = trans4.getOrigin().x();
            y4 = trans4.getOrigin().y();

            x5 = trans5.getOrigin().x();
            y5 = trans5.getOrigin().y();

            x6 = trans6.getOrigin().x();
            y6 = trans6.getOrigin().y();

            norm4 = sqrt(x4 * x4 + y4 * y4);
            norm5 = sqrt(x5 * x5 + y5 * y5);
            norm6 = sqrt(x6 * x6 + y6 * y6);


            if (norm4 < norm5 && norm4 < norm6) {
                hunter_name = "vsilva";
                dists[1] = norm4;
            } else if (norm5 < norm4 && norm5 < norm6) {
                hunter_name = "dcorreia";
                dists[1] = norm5;
            } else {
                hunter_name = "jsousa";
                dists[1] = norm6;
            }

            std::vector<std::string> v;
            v.push_back(prey_name);
            v.push_back(hunter_name);

            if (dists[0]>dists[1])
                v.push_back("run");
            else
                v.push_back("hunt");

            return v;
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




