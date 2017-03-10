#include <iostream>  
#include <vector>    
//ROS INCLUDES
#include <ros/ros.h>  
#include <tf/transform_broadcaster.h>

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
            sub = n.subscribe("/make_a_play", 1000, &MyPlayer::makeAPlayCallback, this);
            cout << "Initialized MyPlayer" << endl;

            t1.setOrigin(tf::Vector3(1, 1, 0));
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            t1.setRotation(q);
            br.sendTransform(tf::StampedTransform(t1, ros::Time::now(), "map", name));
        
        }

        void makeAPlayCallback(const rwsua2017_msgs::MakeAPlay::ConstPtr& msg) {

            cout << "msg: max displacement -> " << msg->max_displacement << endl;

            //definição dos angulos de rotação e valores de translação
            //deveria ser calculado pela AI do sistema
            float turn_angle = M_PI / 10;
            float displacement = 0.5;
            
            tf::Transform t_mov;
            tf::Quaternion q;
            q.setRPY(0, 0, turn_angle);
            t_mov.setRotation(q);
            t_mov.setOrigin( tf::Vector3(displacement , 0.0, 0.0) );
            tf::Transform t = t1  * t_mov;

            br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "map", name));
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




