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

    /*
        class Player
        {
                public:

                Player(string argin_name, string argin_team_name="blue")
                {
                        cout << "player name " << argin_name << endl;
                        this->name = argin_name;
                        this->team_name = argin_team_name;
                        set_team_name(argin_team_name);
                }

                string name;
		
                //accessor SET
                void set_team_name(string argin_team_name)
                {
                        if (argin_team_name == "red" || argin_team_name == "green" || argin_team_name == "blue")
                                this ->team_name=argin_team_name;

                        else
                                cout<< "Error: incorrect team name " << endl;
                }
		
                //Overloaded accessor
                void set_team_name(void)
                {
                        set_team_name("red"); // default value
                }
		
                //accessor SET
                string get_team_name(void)
                {
                        return this->team_name;
                }
		
		
                private:
                string team_name;
        };
	
     */

    class MyPlayer : public Player //inherits the class Player
    {
    public:

        ros::Subscriber sub;

        tf::TransformListener listener;

        tf::TransformBroadcaster br;
        tf::Transform t1;

        MyPlayer(string argin_name, string argin_team_name) : Player(argin_name, argin_team_name) {
            /*
            n.getParam("red", red_team);
            n.getParam("green", green_team);
            n.getParam("blue", blue_team);
            
            cout << "red_team:" << endl;
            for (size_t i = 0; i < red_team->size(); i++)
                cout << red_team[i] << endl;

            cout << "green_team:" << endl;
            for (size_t i = 0; i < green_team->size(); i++)
                cout << green_team[i] << endl;

            cout << "blue_team:" << endl;
            for (size_t i = 0; i < blue_team->size(); i++)
                cout << blue_team[i] << endl;
             */
            sub = n.subscribe("/make_a_play/cat", 1000, &MyPlayer::makeAPlayCallback, this);
            cout << "Initialized MyPlayer" << endl;

            /*
            struct timeval time1;
            gettimeofday(&time1,NULL);
            srand(time1.tv_usec);
            double x=*/

            t1.setOrigin(tf::Vector3(1, 1, 0));
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            t1.setRotation(q);
            br.sendTransform(tf::StampedTransform(t1, ros::Time::now(), "map", name));

        }

        double randNumber() {
            struct timeval t1;
            gettimeofday(&t1, NULL);
            srand(t1.tv_usec);
            double x = ((((double) rand() / (double) RAND_MAX)*2 - 1)*5);

            return x;
        }

        float getAngleTo(string player_name) {


            tf::StampedTransform trans;
            try {
                listener.lookupTransform(name, player_name,ros::Time(0), trans);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            
            float x = trans.getOrigin().x();
            float y = trans.getOrigin().y();
            
            cout << "x="<<x<<" y= "<< y<<endl;
            
            /*
            turtlesim::Velocity vel_msg;
            vel_msg.angular = 4.0 * atan2(trans.getOrigin().y(),trans.getOrigin().x());
            vel_msg.linear = 0.5 * sqrt(pow(trans.getOrigin().x(), 2) + pow(trans.getOrigin().y(), 2));
            turtle_vel.publish(vel_msg);*/
            
            return atan2(y,x);
        }

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




