#include <iostream>  
#include <vector>    
//ROS INCLUDES
#include <ros/ros.h>  

#include <rwsua2017_libs/player.h>   
#include <rwsua2017_msgs/MakeAPlay.h>                                                                            

using namespace std;

namespace rwsua2017
{

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
	
	class MyPlayer: public Player //inherits the class Player
	{
		public:
		
		ros::NodeHandle n;
		
		MyPlayer(string argin_name, string argin_team_name): Player (argin_name, argin_team_name)
		{
			ros::Subscriber sub = n.subscribe("/make_a_play", 1000, &MyPlayer::makeAPlayCallback,this);
			cout<< "Initialized MyPlayer"<<endl;
		}
		
		void makeAPlayCallback(const rwsua2017_msgs::MakeAPlay::ConstPtr& msg)
		{
			ROS_INFO("func"	);
		}
		
		
		vector<string> teammates;
	};
}

int main(int argc, char **argv)
{
	//because we used <using namespace std>  we can replace the other line
	//std::cout << "Hello world" << std::endl;
	cout << "Hello world" << endl;

	ros::init(argc, argv, "player_bvieira");
   
    //Creating an instance of class Player
    rwsua2017::MyPlayer myplayer("bvieira","red");

    //cout << "Created an instance of class player with public name " << player.name << endl;
    cout << "name = " << myplayer.name << endl;
    cout << "team name = " << myplayer.get_team_name()<<endl;
    
    myplayer.teammates.push_back("rodolfo");
    myplayer.teammates.push_back("arnaldo");
    
    
    
    // size_t = unsigned long int (sortchut)
    for(size_t i=0; i<myplayer.teammates.size(); ++i)
    {
		cout<<myplayer.teammates[i]<<endl;
	}
    
    return 1;
}



//std::vector <boost::shared_pts<Player>> teammates;

