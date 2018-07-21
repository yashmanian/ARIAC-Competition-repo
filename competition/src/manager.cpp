/**
* @author Vaibhav Bhilare
* @copyright 2018, Vaibhav Bhilare
*/

#include "../include/manager.h"

int Wait_Time=3.5;
double Z_Buffer=0.1;
double Belt_Y_Buffer=0.1;
double Belt_Dead_Timeout=20;
int Pick_Failures=0;


manager::manager(ros::NodeHandle & node)
: current_score_(0),has_been_zeroed_(false),currentAgv(1) {
	// Instance of custom class from above.
	current_score_subscriber = node.subscribe(
		"/ariac/current_score", 10,
		&manager::current_score_callback, this);

	// Subscribe to the '/ariac/competition_state' topic.
    competition_state_subscriber = node.subscribe(
    	"/ariac/competition_state", 10,
    	&manager::competition_state_callback, this);

    // Subscribe to the '/ariac/orders' topic.
    orders_subscriber = node.subscribe(
    	"/ariac/orders", 10,
    	&manager::order_callback, this);
    
    // Subscribe to the '/ariac/logical_camera_1' topic.
    logical_camera_1_subscriber = node.subscribe(
    	"/ariac/logical_camera_1", 10,
    	&manager::logical_camera_1_callback, this);

	// Subscribe to the '/ariac/break_beam_1_change' topic.
	break_beam_subscriber = node.subscribe(
		"/ariac/break_beam_1_change", 10,
		&manager::break_beam_1_callback, this);

    // // Subscribe to the '/ariac/logical_camera_1' topic.
    // logical_camera_2_subscriber = node.subscribe(
    // 	"/ariac/logical_camera_2", 10,
    // 	&manager::logical_camera_2_callback, this);

    // // Subscribe to the '/ariac/logical_camera_1' topic.
    // logical_camera_3_subscriber = node.subscribe(
    // 	"/ariac/logical_camera_3", 10,
    // 	&manager::logical_camera_3_callback, this);

    // // Subscribe to the '/ariac/logical_camera_1' topic.
    // logical_camera_4_subscriber = node.subscribe(
    // 	"/ariac/logical_camera_4", 10,
    // 	&manager::logical_camera_4_callback, this);

    // // Subscribe to the '/ariac/logical_camera_1' topic.
    // logical_camera_5_subscriber = node.subscribe(
    // 	"/ariac/logical_camera_5", 10,
    // 	&manager::logical_camera_5_callback, this);

    // Subscribe to the '/ariac/logical_camera_1' topic.
    armController = node.subscribe(
    	"/armControllerFlag", 10,
    	&manager::armController_Callback, this);


    tf2_ros::TransformListener tfListener(tfBuffer);

    material_client = node.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    agv1_client = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
    agv2_client = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
    
    ReadyState = node.advertise<competition::manager>("ReadyState", 10);

    Velocity=-0.2;
    armFeedback=2;

    ros::Rate rate(100.0);
    competition_manager(node);

}

// Score Callback
void manager::current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
	if (msg->data != current_score_) {
		ROS_INFO_STREAM("Score: " << msg->data);
	}
	current_score_ = msg->data;
}

void manager::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
	if (msg->data == "done" && competition_state_ != "done") {
		ROS_INFO("Competition ended.");
	}
	competition_state_ = msg->data;
}

// Order Callback
void manager::order_callback(const osrf_gear::Order::ConstPtr & currentOrder) {
	received_order.push_back(*currentOrder);
	manager::order_manager(received_order);
}

/// Called when a new LogicalCameraImage message is received.
void manager::logical_camera_1_callback(
	const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
	osrf_gear::LogicalCameraImage scan=*image_msg;
	LogicalImage_1=scan;
	if(!LogicalImage_1.models.empty() && Same_Object==false) {
		Same_Object=true;
		// Add object to inventory
		Y_Previous=LogicalImage_1.models[0].pose.position.y;
		Time_Previous =ros::Time::now().toSec();
		Conveyor_Objects.push_back(LogicalImage_1.models[0]);
		Conveyor_Times.push_back(Time_Previous);
		Conveyor_Y.push_back(Y_Previous);
	}
	else if(!LogicalImage_1.models.empty() && Same_Object==true) {
		Y_Now=LogicalImage_1.models[0].pose.position.y;
		Time_Now =ros::Time::now().toSec();
	}
	else if(LogicalImage_1.models.empty() && Same_Object==true){
		Velocity=(Y_Now-Y_Previous)/(Time_Now-Time_Previous);
//		ROS_INFO_STREAM(Y_Now);
		//ROS_INFO_STREAM(Y_Previous);
//		ROS_INFO_STREAM(Time_Now);
//		ROS_INFO_STREAM(Time_Previous);
		ROS_INFO_STREAM(Velocity);
		Same_Object=false;
		Dead_Time =ros::Time::now().toSec();
	}
}

/// Called when a new Proximity message is received.
void manager::break_beam_1_callback(const osrf_gear::Proximity::ConstPtr & msg) {
	if (msg->object_detected) {  // If there is an object in proximity.
		ROS_INFO_STREAM("Break beam triggered.");
//		Conveyor_Objects.erase(Conveyor_Objects.begin());
//		Conveyor_Times.erase(Conveyor_Times.begin());
//		Conveyor_Y.erase(Conveyor_Y.begin());
		ROS_INFO_STREAM(Conveyor_Y.size());
	}
}

// void manager::logical_camera_2_callback(
// const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
// osrf_gear::LogicalCameraImage scan=*image_msg;
// LogicalImage_2=scan;
// }

// void manager::logical_camera_3_callback(
// const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
// osrf_gear::LogicalCameraImage scan=*image_msg;
// LogicalImage_3=scan;
// }

// void manager::logical_camera_4_callback(
// const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
// osrf_gear::LogicalCameraImage scan=*image_msg;
// LogicalImage_4=scan;
// }

// void manager::logical_camera_5_callback(
// const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
// osrf_gear::LogicalCameraImage scan=*image_msg;
// LogicalImage_5=scan;
// }

// Score Callback
void manager::armController_Callback(const std_msgs::Bool::ConstPtr& CompletionFlag) {
	if (CompletionFlag->data == true) {
		ROS_INFO_STREAM("Part Deposited");
		armFeedback=1;
	}
	if (CompletionFlag->data ==false) {
		ROS_INFO_STREAM("Part Failed");
		armFeedback=0;	
	}
}

/// Start the competition by waiting for and then calling the start ROS Service.
void manager::start_competition(ros::NodeHandle & node) {

    // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
    ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  	// If it's not already ready, wait for it to be ready.
  	// Calling the Service using the client before the server is ready would fail.
  	if (!start_client.exists()) {
  		ROS_INFO("Waiting for the competition to be ready...");
    	start_client.waitForExistence();
    	ROS_INFO("Competition is now ready.");
    }
  	ROS_INFO("Requesting competition start...");
  	std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  	start_client.call(srv);  // Call the start Service.
  	if (!srv.response.success) {  // If not successful, print out why.
    	ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  	} else {
  		ROS_INFO("Competition started!");
  	}
}

void manager::competition_manager(ros::NodeHandle & node) {

	ROS_INFO("Setup complete.");
	start_competition(node);
	ros::Duration(0.5).sleep();
	////    ros::Duration(25).sleep();
	while(ros::ok()){
	    ros::spinOnce();  // This executes callbacks on new data.
	    while(!kitList.empty() || !backlogKit.empty()) {
	    	if(!kitList.empty()){
	    		currentObjectList = kit_manager(kitList.back());
	    		kitList.erase(kitList.end());
	        agv_switch();
	    	}
	    	else{
	    		currentObjectList = backlogKit;
	    		currentKitId=backlogKitId;
	    		agv_switch();
	    		backlogKit.clear();
	    	}
	    	while(!currentObjectList.empty()) {
	    		bool found = false;
	    		int orderCase = 0;
	    		osrf_gear::Model Target_Object;
	  			std::string Target_Bin;
	  			ros::spinOnce();
	  			bool beltSearch = false;
	        	competition::manager ReadyStateMsg;
	        	armFeedback=2;
	  			// Search for part
	  			while(found==false || beltSearch==false){
	//  				std::vector<osrf_gear::Model> allObjects=LogicalImage_1.models;
	  				// Search on Belt
	  				int BeltCase=0;
	  				for (osrf_gear::Model& detectedObject : Conveyor_Objects){
	  					orderCase=0;
	  					if(Velocity<-1 || Velocity>1){
	  						Velocity=-0.2;
	  					}
	  					for(osrf_gear::KitObject& orderPart : currentObjectList){
	  						if(detectedObject.type==orderPart.type){
	  							binType="belt";
	                			partType=orderPart.type;
	  							Target_Object=detectedObject;
	  							initialPose.pose=Target_Object.pose;
	                			geometry_msgs::TransformStamped logical_camera_to_world_frame;
	                			logical_camera_to_world_frame = tfBuffer.lookupTransform(
	                  			"world", "logical_camera_1_frame", ros::Time(0), ros::Duration(1.0) );
	                			tf2::doTransform(initialPose, initialPose, logical_camera_to_world_frame);
	  							double Y_Initial = initialPose.pose.position.y;
	  							double Time_Compare = ros::Time::now().toSec();
	  							auto Time_Initial = Conveyor_Times.at(BeltCase);
	  							double Y_Go = Y_Initial+(Velocity*(Wait_Time+Time_Compare-Time_Initial));
	//  							ROS_ERROR_STREAM(Y_Initial);
	//  							ROS_ERROR_STREAM(Time_Compare);
	//  							ROS_ERROR_STREAM(Time_Initial);
	//                			initialPose.pose.position.y=initialPose.pose.position.y+(Y_Go-Y_Initial);
	                			initialPose.pose.position.y=Y_Go;

	                			finalPose.pose=orderPart.pose;
	                			finalPose.pose.position.z=Z_Buffer;
//	                			logical_camera_to_world_frame = tfBuffer.lookupTransform(
//	                  			"world", agvs.at(currentAgv), ros::Time(0), ros::Duration(1.0) );
//	                			tf2::doTransform(finalPose, finalPose, logical_camera_to_world_frame);
	                			found=true;
	                			if(Y_Go>=0.2+Belt_Y_Buffer && Y_Go<0.85-Belt_Y_Buffer) {
	  								binNumber=1;
	  								break;
	  							}
	  							else if(Y_Go>=0.85+Belt_Y_Buffer && Y_Go<1.55-Belt_Y_Buffer) {
	  								binNumber=2;
	  								break;
	  							}
	  							else if(Y_Go>=1.55+Belt_Y_Buffer && Y_Go<2.15-Belt_Y_Buffer) {
	  								binNumber=3;
	  								break;
	  							}
	  							else if(Y_Go>=2.15+Belt_Y_Buffer && Y_Go<2.85-Belt_Y_Buffer) {
	  								binNumber=4;
	  								break;
	  							}
	  							else {
	  								found = false;
	  							}
	  						//	break;
	  						}
	  						else {
	  							orderCase++;
	  						}
	  					}
	  					if(found==true){
	  						Conveyor_Objects.erase(Conveyor_Objects.begin()+BeltCase);
	  						Conveyor_Times.erase(Conveyor_Times.begin()+BeltCase);
	  						Conveyor_Y.erase(Conveyor_Y.begin()+BeltCase);
	  						break;
	  					}
	  					BeltCase++;
	  				}
	  				beltSearch = true;
	//  				ros::spinOnce();
	  				if(beltSearch==true && found==false) {
	  					orderCase=0;
	  					for(auto i: currentObjectList){
	  						osrf_gear::GetMaterialLocations material_service_srv;
	    					material_service_srv.request.material_type=i.type;
	    					ros::service::waitForService("/ariac/material_locations");
	    					if (material_client.call(material_service_srv)){
	    						partType=i.type;
	    						for(auto location: material_service_srv.response.storage_units){
	    							if((location.unit_id).substr(0,3)=="bin"){
	    								found=true;
	    								binType="bin";
	    								if(location.unit_id=="bin5") {
	    									// Target_Object = LogicalImage_2.models[0];
	                    					// Logical_Camera_Frame="logical_camera_2_frame";
	                      					binNumber=5;
	    								}
	    								else if(location.unit_id=="bin6") {
	    									// Target_Object = LogicalImage_3.models[0];
	                      					// Logical_Camera_Frame="logical_camera_3_frame";
	                      					binNumber=6;
	    								}
	    								else if(location.unit_id=="bin7") {
	    									// Target_Object = LogicalImage_4.models[0];
	                      					// Logical_Camera_Frame="logical_camera_4_frame";
	                      					binNumber=7;
	    								}
	    								else if(location.unit_id=="bin8") {
	    									// Target_Object = LogicalImage_5.models[0];
	                      					// Logical_Camera_Frame="logical_camera_5_frame";
	                      					binNumber=8;
	    								}
	    								else {
	    									found=false;
	    								}
	    								if(found==true){
	    									ROS_INFO_STREAM(location.unit_id);

						                    // initialPose.pose=Target_Object.pose;
	    	            				    // geometry_msgs::TransformStamped logical_camera_to_world_frame;
	        	            				// logical_camera_to_world_frame = tfBuffer.lookupTransform(
	            	          				// "world", Logical_Camera_Frame, ros::Time(0), ros::Duration(1.0) );
	                	    				// tf2::doTransform(initialPose, initialPose, logical_camera_to_world_frame);
	                    					finalPose.pose=i.pose;
	                    					finalPose.pose.position.z=Z_Buffer;
//	                    					logical_camera_to_world_frame = tfBuffer.lookupTransform(
//	                      					"world", agvs.at(currentAgv), ros::Time(0), ros::Duration(1.0) );
//	                    					tf2::doTransform(finalPose, finalPose, logical_camera_to_world_frame);
		    								break;
	    								}
	    							}    		
	    						}
	    					}
	    					else {
	    						ROS_INFO_STREAM("Could Not Connect to the material_locations service");
	    					}
	    					if(found!=true){
	    						orderCase++;
	    						ros::spinOnce();
	    					}
	    					else{
	    						break;
	    					}
	    				}
	  				}
//	  				double Time_Temp_Compare=ros::Time::now().toSec();
//					if(Dead_Time!=0 && (Time_Temp_Compare-Dead_Time)>Belt_Dead_Timeout){
//						break;
//					}

	  			}
	//  			ros::spinOnce();
	  			if(found==true){
	  				auto currentOrder=currentObjectList.at(0+orderCase);
		  			ROS_INFO_STREAM(currentOrder.type);
		  			ROS_INFO_STREAM(orderCase);
		        	ReadyStateMsg.bin_number=binNumber;
		        	ReadyStateMsg.bin_type=binType;
		        	ReadyStateMsg.part_type=partType;
//		        	ReadyStateMsg.Initial_Pose=initialPose;
//		        	if(currentAgv==0){
//		        		finalPose.pose.position.y=finalPose.pose.position.y-0.2;
//		        	}
//		        	else{
//		        		finalPose.pose.position.y=finalPose.pose.position.y+0.2;
//		        	}
		        	ReadyStateMsg.Final_Pose=finalPose;
		        	ReadyStateMsg.Velocity=Velocity;
		        	ReadyStateMsg.Agv_Number=currentAgv+1;
		  			ReadyState.publish(ReadyStateMsg);
		  			ROS_INFO_STREAM("Data Sent to the armController");
		  			currentObjectList.erase(currentObjectList.begin()+orderCase);
			  		while(1) {
			  			ros::spinOnce();
			  			if(armFeedback==1){
			  				Pick_Failures=0;
			  				ROS_INFO_STREAM("Part Deposited");
			  				break;
			  			}
			  			else if(armFeedback==0 && Pick_Failures<5){
			  				Pick_Failures++;
			  				ROS_INFO_STREAM("Part Failed to Deposit");
			  				if(ReadyStateMsg.Agv_Number==currentAgv+1) {
			  					currentObjectList.push_back(currentOrder);
			  				}
			  				else{
			  					backlogKit.push_back(currentOrder);
			  				}
			  				break;
			  			}
			  			else if(armFeedback==0 && Pick_Failures>=5){
			  				Pick_Failures=0;
			  				ROS_INFO_STREAM("Part Failed to Deposit & Skipped");
			  				break;
			  			}
			  		}
			  	}
			  	else{
			  		ros::spinOnce();
			  	}
//				double Time_Temp_Compare=ros::Time::now().toSec();
//				if(Dead_Time!=0 && (Time_Temp_Compare-Dead_Time)>Belt_Dead_Timeout){
//					break;
//				}
	    	}
	    	osrf_gear::AGVControl agv_service_srv;
	    	agv_service_srv.request.kit_type=currentKitId;
	    	if(currentAgv==0){
	    		ros::service::waitForService("/ariac/agv1");
	    		if (agv1_client.call(agv_service_srv)){
	    			ROS_ERROR_STREAM(currentKitId);
	    		}
	    	}
	    	else {
	    		ros::service::waitForService("/ariac/agv2");
	    		if (agv2_client.call(agv_service_srv)){
	    		ROS_ERROR_STREAM(currentKitId);
	    		}	
	    	}

	    }
	}
}

void manager::order_manager(std::vector<osrf_gear::Order>& received_order) {
	for(osrf_gear::Order& parseOrder:received_order){
		for(osrf_gear::Kit& parseKit: parseOrder.kits){
			kitList.push_back(parseKit);
			// Call the kit-switch service or whatever
		}
	}
	switch_kit();
	received_order.erase(received_order.begin());
}

std::vector<osrf_gear::KitObject> manager::kit_manager(osrf_gear::Kit currentKit) {
	std::vector<osrf_gear::KitObject> objectList;
	for(osrf_gear::KitObject currentObject: currentKit.objects) {
		objectList.push_back(currentObject);
	}
	currentKitId=currentKit.kit_type;
	return objectList;
}

void manager::switch_kit(){
	ROS_INFO_STREAM("switch_kit");
	if(currentObjectList.size()!=0) {
		backlogKit=currentObjectList;
		backlogKitId=currentKitId;
		if(!kitList.empty()){
    	currentObjectList = kit_manager(kitList.back());
    	kitList.erase(kitList.end());
    	}
    	else{
    		currentObjectList = backlogKit;
//    		agv_switch();
    		backlogKit.clear();
    	}
		agv_switch();
	}
}

void manager::agv_switch() {
	if(currentAgv==1){
		currentAgv=0;
		ROS_INFO_STREAM("AGV Switched");
	}
	else{
		currentAgv=1;
		ROS_INFO_STREAM("AGV Switched");
	}
}

manager::~manager() {
}