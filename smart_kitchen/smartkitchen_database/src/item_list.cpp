#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>

#include <database_interface/db_class.h>

#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>

#include <stdio.h>
#include "boost/date_time/posix_time/posix_time.hpp"


class ItemList : public database_interface::DBClass
{
	public:
		database_interface::DBField<int> item_list_id_;
		database_interface::DBField<int> item_id_;
		database_interface::DBField<double> item_quantity_;
		database_interface::DBField<std::string> expiration_date_;

		ItemList() : 
			item_list_id_(database_interface::DBFieldBase::TEXT, 
					this, "item_list_id", "item_list", true),
			item_id_(database_interface::DBFieldBase::TEXT, 
					this, "item_id", "item_list", true),
			item_quantity_(database_interface::DBFieldBase::TEXT, 
					this, "item_quantity", "item_list", true),
			expiration_date_(database_interface::DBFieldBase::TEXT, 
					this, "expiration_date", "item_list", true)
	{
		primary_key_field_ = &item_list_id_;

		fields_.push_back(&item_list_id_);
		fields_.push_back(&item_id_);
		fields_.push_back(&item_quantity_);
		fields_.push_back(&expiration_date_);

		setAllFieldsReadFromDatabase(true);
		setAllFieldsWriteToDatabase(true);
	}
};


class ItemProperties : public database_interface::DBClass
{
	public:
		database_interface::DBField<int> item_id_;
		database_interface::DBField<std::string> item_name_;
		database_interface::DBField<int> expiration_time_;
		database_interface::DBField<std::string> last_time_used_;
		database_interface::DBField<std::string> last_time_buy_;
		database_interface::DBField<double> mean_week_usage_;
		database_interface::DBField<int> number_in_packet_;

		ItemProperties() :
			item_id_(database_interface::DBFieldBase::TEXT,
					this, "item_id", "item_properties", true),
			item_name_(database_interface::DBFieldBase::TEXT,
					this, "item_name", "item_properties", true),
			expiration_time_(database_interface::DBFieldBase::TEXT,
					this, "expiration_time", "item_properties", true),
			last_time_used_(database_interface::DBFieldBase::TEXT,
					this, "last_time_used", "item_properties", true),
			last_time_buy_(database_interface::DBFieldBase::TEXT,
					this, "last_time_buy", "item_properties", true),
			mean_week_usage_(database_interface::DBFieldBase::TEXT,
					this, "mean_week_usage", "item_properties", true),
			number_in_packet_(database_interface::DBFieldBase::TEXT,
					this, "number_in_packet", "item_properties", true)
	{
		primary_key_field_ = &item_id_;

		fields_.push_back(&item_id_);
		fields_.push_back(&item_name_);
		fields_.push_back(&expiration_time_);
		fields_.push_back(&last_time_used_);
		fields_.push_back(&last_time_buy_);
		fields_.push_back(&mean_week_usage_);
		fields_.push_back(&number_in_packet_);

		setAllFieldsReadFromDatabase(true);
		setAllFieldsWriteToDatabase(true);
	}
};


class UniqueItem 
{
	public:
		std::string item_name;
		std::string item_real_name;
		double item_quantity;
		time_t expiration_date;

		UniqueItem()
		{
			item_name = "";
			item_real_name = "";
			item_quantity = 0.0;
		}
};

// Used to sort UniqueItem list by expiration date
bool sortItemByDate(const UniqueItem & item1, const UniqueItem & item2)
{
	double diff = difftime(item1.expiration_date, item2.expiration_date);
	return (diff < 0);
}


class ItemListROS {
	public:
		ItemListROS();
		~ItemListROS();

		ros::Subscriber read_sub_;
		ros::Publisher test_pub;
	private:
		void readCallback(const std_msgs::Empty::ConstPtr & empty);
		void fillOpenHabFiles(void);
		std::string findItemName(std::vector< boost::shared_ptr<ItemProperties> > & item_list, int item_id);
		ros::NodeHandle nh;

		database_interface::PostgresqlDatabase *database;
		std::string address;
		std::string port;
		std::string user;
		std::string password;
		std::string db_name;

		std::list<UniqueItem> list_for_openhab;
};


ItemListROS::ItemListROS()
{

	ros::NodeHandle nhp("~");

	nhp.getParam("address", address);
	nhp.getParam("port", port);
	nhp.getParam("user", user);
	nhp.getParam("password", password);
	nhp.getParam("db_name", db_name);

	read_sub_ = nh.subscribe < std_msgs::Empty > ("/DB/read", 2, &ItemListROS::readCallback, this);
	test_pub = nh.advertise < std_msgs::String > ("/DB/test", 2);


	database = new database_interface::PostgresqlDatabase(address, port,
			user, password, db_name);
	if (!database->isConnected())
	{
		std::cerr << "Database failed to connect \n";
	}
	else
	{
		std::cerr << "Database connected successfully \n";
	}
	/*
	   std::string strTime = "2007-04-11 06:18:29";
	   std::tm tmTime = boost::posix_time::to_tm(boost::posix_time::time_from_string(strTime));
	   time_t time = mktime(&tmTime);
	   std::cout << asctime(&tmTime);
	 */
}


ItemListROS::~ItemListROS()
{
	ROS_INFO("Destroy object");
}

void ItemListROS::readCallback(const std_msgs::Empty::ConstPtr & empty)
{

	if (!database->isConnected())
	{
		std::cerr << "Database failed to connect \n";
	}
	else
	{
		std::cerr << "Database connected successfully \n";

		std::vector< boost::shared_ptr<ItemList> > item_list;
		if (!database->getList(item_list))
		{
			std::cerr << "Failed to get list of items\n";
			//return -1;
		}
		std::cerr << "Retrieved " << item_list.size() << " item(s) \n";

		std::vector< boost::shared_ptr<ItemProperties> > item_properties;
		if (!database->getList(item_properties))
		{
			std::cerr << "Failed to get list of properties\n";
			//return -1;
		}
		std::cerr << "Retrieved " << item_properties.size() << " property(ies) \n";

		std::vector<UniqueItem> items_for_openhab;

		for (size_t i=0; i<item_list.size(); i++)
		{
			std::string strTime = item_list[i]->expiration_date_.data() + " 23:59:59";
			std::tm tmTime = boost::posix_time::to_tm(boost::posix_time::time_from_string(strTime));
			time_t time = mktime(&tmTime);
			time_t nowTime = std::time(NULL);
			//std::cout << asctime(&tmTime);
			//std::cout << ctime(&nowTime);
			//double seconds = difftime(time,nowTime);
			//std::cerr << "Temps avant expiration : " << seconds/60/60/24 << " jours." << std::endl;

			//std::string current_name = item_properties[item_list[i]->item_id_.data()]->item_name_.data();
			std::string current_name = findItemName(item_properties, item_list[i]->item_id_.data());

			//std::cerr << item_list[i]->item_id_.data() << " " << item_properties[item_list[i]->item_id_.data()]->item_name_.data();

			for (int i=0; i < current_name.length(); i++){
				if ( (current_name[i] == '/') || 
						(current_name[i] == ' ') || 
						(current_name[i] == '\'') || 
						(current_name[i] == '\"') )
					current_name[i] = '_';
			}

			UniqueItem tmp_item;
			tmp_item.item_name = current_name;
			tmp_item.item_real_name = findItemName(item_properties, item_list[i]->item_id_.data());//item_properties[item_list[i]->item_id_.data()]->item_name_.data();
			tmp_item.item_quantity = item_list[i]->item_quantity_.data();
			tmp_item.expiration_date = time;

			std::cerr << tmp_item.item_name << " " << ctime(&(tmp_item.expiration_date));

			std::vector<int> remove_index;

			for (unsigned i=0; i<items_for_openhab.size(); ++i)
			{
				if( items_for_openhab[i].item_name.compare(tmp_item.item_name) == 0) // same
				{
					tmp_item.item_quantity = tmp_item.item_quantity + items_for_openhab[i].item_quantity;
					double diff = difftime(tmp_item.expiration_date, items_for_openhab[i].expiration_date); // 
					if(diff > 0)
					{
						tmp_item.expiration_date = items_for_openhab[i].expiration_date;
					}
					//std::cerr << "SAME " << tmp_item.item_quantity << std::endl;
					remove_index.push_back(i);
				} 
			}

			for (unsigned i=0; i<remove_index.size(); ++i)
			{
				items_for_openhab.erase(items_for_openhab.begin()+remove_index[i]-i);
			}


			items_for_openhab.push_back(tmp_item);

			//std::cerr << "Nombre de valeurs : " << items_for_openhab.size() << std::endl;

		}

		list_for_openhab.clear();

		// Fill a list to be able to sort easily
		for (unsigned i=0; i<items_for_openhab.size(); ++i)
		{
			list_for_openhab.push_back(items_for_openhab[i]); 
			//std::cerr << items_for_openhab[i].item_name << " " << ctime(&(items_for_openhab[i].expiration_date));
		}

		list_for_openhab.sort(sortItemByDate);
/*
		std::cerr << "mylist contains:";
  		for (std::list<UniqueItem>::iterator it=list_for_openhab.begin(); it!=list_for_openhab.end(); ++it)
    			std::cerr << ' ' << it->item_name;
  		std::cerr << '\n';
*/
		fillOpenHabFiles();

		
	}


}

std::string ItemListROS::findItemName(std::vector< boost::shared_ptr<ItemProperties> > & item_list, int item_id)
{
	std::string str;
	for (size_t i=0; i<item_list.size(); i++)
        {
		if(item_list[i]->item_id_.data() == item_id)
		{
			str = item_list[i]->item_name_.data();
			break;
		}
	}
	return str;
}

void ItemListROS::fillOpenHabFiles(void)
{
	std::ofstream items_file;
	items_file.open ("smartkitchen.items");
	std::ofstream sitemap_file;
	sitemap_file.open ("smartkitchen.sitemap");
	std::ofstream rules_file;
	rules_file.open ("smartkitchen.rules");

	items_file << "Group ROS (all)\n\nString ROS_Status \"ROS [\%s]\"\n\n";

	sitemap_file << "sitemap default label=\"Smart Kitchen\" \n{ \nFrame { \n";


	// Write the var number in the beginning of rule file
	for (std::list<UniqueItem>::iterator it=list_for_openhab.begin(); it!=list_for_openhab.end(); ++it)
        {
		rules_file << "var Number " << it->item_name << "_value = " << it->item_quantity << "\n";
	}

	rules_file << "\nrule \"Initialize all items\"\nwhen\n    System started\nthen\n";

	time_t nowTime = std::time(NULL);
	double seconds = 0.0;


	// Write sitemap, item and rules(only init) file
	for (std::list<UniqueItem>::iterator it=list_for_openhab.begin(); it!=list_for_openhab.end(); ++it)
        {                
		seconds = difftime(it->expiration_date,nowTime);

                sitemap_file << "Switch item="
                        << it->item_name         
                        << " mappings=[DECREASE='REMOVE ONE'] valuecolor=[>4=\"green\",>1=\"orange\",<=1=\"red\"]\n";

                items_file << "Dimmer "
                        << it->item_name 
                        << " \""        
                        << it->item_real_name
                        //<< "[\%s]" // items_for_openhab[i].item_quantity
                        << " .Expire in : " << floor(seconds/60/60/24) << " days. [\%s] "
                        << "\" (ROS)\n";

                rules_file << "    postUpdate( "
                        << it->item_name
                        << ", " << it->item_quantity
                        << ")\n";
        }


	sitemap_file << "} \n}\n";

	rules_file << "end\n\n";

	// Write all rules
	for (std::list<UniqueItem>::iterator it=list_for_openhab.begin(); it!=list_for_openhab.end(); ++it)
        {
		rules_file << "rule \"" << it->item_name << " rule\"\n    when\n        Item "
                        << it->item_name
                        << " received command\n    then\n"
                        << "    if (receivedCommand==DECREASE) {\n"
                        << "        " << it->item_name << "_value = "
                        << it->item_name << "_value - 1\n"
                        << "        if(" << it->item_name << "_value<0)   " << it->item_name << "_value = 0\n"
                        << "        postUpdate(" << it->item_name << ", " << it->item_name << "_value)\n"
                        << "    }\nend\n";
        }

	items_file.close();
	sitemap_file.close();
	rules_file.close();
}


int main(int argc, char **argv)
{  
	ros::init(argc, argv, "smartkitchen_test");

	ItemListROS item_list_ROS;

	ros::spin();
	ros::shutdown();


}

