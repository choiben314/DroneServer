// System Includes
#include <iostream>
#include <condition_variable>
#include <iostream>
#include <mutex>

// C Includes
#include <signal.h>

// External Includes
#include <tacopie/tacopie>

//Project Includes
#include "Drone.hpp"

namespace DroneInterface {
	RealDrone::RealDrone() : m_thread(&RealDrone::DroneMain, this), m_abort(false) {
		m_serial = "TEST_SERIAL!";
	}
	
	RealDrone::RealDrone(tacopie::tcp_client& client) : m_thread(&RealDrone::DroneMain, this), m_abort(false) {
		m_serial = "TEST_SERIAL!!";
		m_client = &client;
	}
	
	RealDrone::~RealDrone() {
		m_abort = true;
		if (m_thread.joinable())
			m_thread.join();
	}
	void RealDrone::DataReceivedHandler(const std::shared_ptr<tacopie::tcp_client>& client, const tacopie::tcp_client::read_result& res){
		m_mutex.lock();
		if (res.success) {
			for (char c : res.buffer) {
				m_packet_fragment->m_data.push_back(c);
				if (m_packet_fragment->IsFinished()) {
					uint8_t PID;
					m_packet_fragment->GetPID(PID);
					
					switch (PID) {
						case 0U: {
							Packet_CoreTelemetry packet;
							if (packet.Deserialize(*m_packet_fragment)) {
								std::cout << packet;
								this->m_packet_ct = &packet;
							}
							else {
								std::cerr << "Error: Tried to deserialize invalid Core Telemetry packet." << std::endl;
							}
							break;
						}
						case 1U: {
							Packet_ExtendedTelemetry packet;
							if (packet.Deserialize(*m_packet_fragment)) {
								std::cout << packet;
								this->m_packet_et = &packet;
							}
							else {
								std::cerr << "Error: Tried to deserialize invalid Extended Telemetry packet." << std::endl;
							}
							break;
						}
						case 2U: {
							Packet_Image packet;
							if (packet.Deserialize(*m_packet_fragment)) {
								std::cout << packet;
								//cv::imshow("Frame", packet.Frame);
								//cv::waitKey();
								this->m_packet_img = &packet;
							}
							else {
								std::cerr << "Error: Tried to deserialize invalid Image packet." << std::endl;
							}
							break;
						}
						case 3U: {
							Packet_Acknowledgment packet;
							if (packet.Deserialize(*m_packet_fragment)) {
								std::cout << packet;
								this->m_packet_ack = &packet;
							}
							else {
								std::cerr << "Error: Tried to deserialize invalid Acknowledgment packet." << std::endl;
							}
							break;
						}
						case 4U: {
							Packet_MessageString packet;
							if (packet.Deserialize(*m_packet_fragment)) {
								std::cout << packet;
								this->m_packet_ms = &packet;
							}
							else {
								std::cerr << "Error: Tried to deserialize invalid Message String packet." << std::endl;
							}
							break;
						}
					}
					m_packet_fragment->Clear();
				}
			}

			//std::vector<char> msg_buffer = res.buffer;
			//std::string msg_string(msg_buffer.begin(), msg_buffer.end());

			//std::string hostname = client->get_host();
			//uint32_t port = client->get_port();

			//std::cout << "Received a new message from " << hostname << " on port " << std::to_string(port) << ": " << msg_string << std::endl;
			////string msg_return = "Server received the following message: " + msg_string;
			//std::string msg_return = msg_string;

			//std::vector<char> return_vec(msg_return.begin(), msg_return.end());

			//client->async_write({ return_vec, nullptr });
			client->async_read({ 1024, bind(&RealDrone::DataReceivedHandler, this, client, std::placeholders::_1) });
		}
		else {
			std::cout << "Client disconnected" << std::endl;
			client->disconnect();
		}
		m_mutex.unlock();
	}
	
	//Get drone serial number as a string (should be available on construction)
	std::string RealDrone::GetDroneSerial(void) { return this->m_packet_et->DroneSerial; }
	
	//Lat & Lon (radians) and WGS84 Altitude (m)
	bool RealDrone::GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) {
		Latitude = this->m_packet_ct->Latitude;
		Latitude = this->m_packet_ct->Longitude;
		Altitude = this->m_packet_ct->Altitude;
		//Timepoint?
		return true;
	}
	
	//NED velocity vector (m/s)
	bool RealDrone::GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) {
		V_North = this->m_packet_ct->V_N;
		V_East = this->m_packet_ct->V_E;
		V_Down = this->m_packet_ct->V_D;
		return true;
	}
	
	//Yaw, Pitch, Roll (radians) using DJI definitions
	bool RealDrone::GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) {
		Yaw = this->m_packet_ct->Yaw;
		Pitch = this->m_packet_ct->Pitch;
		Roll = this->m_packet_ct->Roll;
		return true;
	}
	
	//Barometric height above ground (m) - Drone altitude minus takeoff altitude
	bool RealDrone::GetHAG(double & HAG, TimePoint & Timestamp) {
		HAG = this->m_packet_ct->HAG;
		return true;
	}
	
	//Drone Battery level (0 = Empty, 1 = Full)
	bool RealDrone::GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) {
		BattLevel = this->m_packet_et->BatLevel / 100;
		return true;
	}
	
	//Whether the drone has hit height or radius limits
	bool RealDrone::GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp) {
		MaxHAG = this->m_packet_et->MaxHeight;
		MaxDistFromHome = this->m_packet_et->MaxDist;
		return true;
	}
	
	//Wind & other vehicle warnings as strings
	bool RealDrone::GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) {
		ActiveWarnings.push_back("BatWarning: " + this->m_packet_et->BatWarning);
		ActiveWarnings.push_back("WindLevel: " + this->m_packet_et->WindLevel);
		if (this->m_packet_ms->Type == 2) {
			ActiveWarnings.push_back("Messages: " + this->m_packet_ms->Message);
		}
		return true;
	}
	
	//GNSS status (-1 for none, 0-5: DJI definitions)
	bool RealDrone::GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp) {
		SatCount = this->m_packet_et->GNSSSatCount;
		SignalLevel = this->m_packet_et->GNSSSignal;
		return true;
	}
	
	//Returns true if recognized DJI camera is present - Should be available on construction
	bool RealDrone::IsDJICamConnected(void) {
		return this->m_packet_et->DJICam == 0 || this->m_packet_et->DJICam == 1;
	}

	void RealDrone::SendPacket(DroneInterface::Packet &packet) {
		std::vector<char> ch_data(packet.m_data.begin(), packet.m_data.end());
		m_client->async_write({ ch_data, nullptr });
	}
	void RealDrone::SendPacket_EmergencyCommand(uint8_t Action) {
		DroneInterface::Packet packet;
		DroneInterface::Packet_EmergencyCommand packet_ec;

		packet_ec.Action = Action;

		packet_ec.Serialize(packet);

		this->SendPacket(packet);
	}
	void RealDrone::SendPacket_CameraControl(uint8_t Action, double TargetFPS) {
		DroneInterface::Packet packet;
		DroneInterface::Packet_CameraControl packet_cc;

		packet_cc.Action = Action;
		packet_cc.TargetFPS = TargetFPS;

		packet_cc.Serialize(packet);

		this->SendPacket(packet);
	}
	//void RealDrone::SendPacket_ExecuteWaypointMission(uint8_t LandAtEnd, uint8_t CurvedFlight, std::vector<Waypoint> Waypoints) {
	//	DroneInterface::Packet packet;
	//	DroneInterface::Packet_ExecuteWaypointMission packet_ewm;

	//	packet_ewm.LandAtEnd = LandAtEnd;
	//	packet_ewm.CurvedFlight = CurvedFlight;
	//	packet_ewm.Waypoints = Waypoints;

	//	packet_ewm.Serialize(packet);

	//	this->SendPacket(packet);

	//}
	void RealDrone::SendPacket_VirtualStickCommand(uint8_t Mode, float Yaw, float V_x, float V_y, float HAG, float timeout) {
		DroneInterface::Packet packet;
		DroneInterface::Packet_VirtualStickCommand packet_vsc;

		packet_vsc.Mode = Mode;
		packet_vsc.Yaw = Yaw;
		packet_vsc.V_x = V_x;
		packet_vsc.V_y = V_y;
		packet_vsc.HAG = HAG;
		packet_vsc.timeout = timeout;

		packet_vsc.Serialize(packet);

		this->SendPacket(packet);
	}


	//Start sending frames of live video (as close as possible to the given framerate (frame / s))
	void RealDrone::StartDJICamImageFeed(double TargetFPS) { 
		this->SendPacket_CameraControl(1, TargetFPS);
	}
	
	//Stop sending frames of live video
	void RealDrone::StopDJICamImageFeed(void) { 
		this->SendPacket_CameraControl(0, 0);
	}
	
	bool RealDrone::GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) {
		Frame = this->m_packet_img->Frame;
		//FrameNumber??

		return true;
	}
	
	//Register callback for new frames
	int RealDrone::RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) {
		std::scoped_lock lock(m_mutex);
		int token = 0;
		while (m_ImageryCallbacks.count(token) > 0U)
			token++;
		m_ImageryCallbacks[token] = Callback;
		return token;
	}

	//Unregister callback for new frames (input is token returned by RegisterCallback()
	void RealDrone::UnRegisterCallback(int Handle) {
		std::scoped_lock lock(m_mutex);
		m_ImageryCallbacks.erase(Handle);
	}
	
	//Get flight mode as a human-readable string
	bool RealDrone::GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) {
		FlightModeStr = "Flight Mode: " + this->m_packet_et->FlightMode;
		return true;
	}
	
	//Stop current mission, if running. Then load, verify, and start new waypoint mission.
	void RealDrone::ExecuteWaypointMission(WaypointMission & Mission) {
		
	}
	
	//Populate Result with whether or not a waypoint mission is currently being executed
	bool RealDrone::IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) {
		return false;
	}
	
	//Retrieve the ID of the currently running waypoint mission (if running).
	bool RealDrone::GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp) { return false; }
	

	//Put in virtualStick Mode and send command (stop mission if running)
	void RealDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) { 
		this->SendPacket_VirtualStickCommand(0, Command.Yaw, Command.V_North, Command.V_East, Command.HAG, Command.timeout);
	}
	
	//Put in virtualStick Mode and send command (stop mission if running)
	void RealDrone::IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) { 
		this->SendPacket_VirtualStickCommand(1, Command.Yaw, Command.V_Forward, Command.V_Right, Command.HAG, Command.timeout);
	}
	
	//Stop any running missions an leave virtualStick mode (if in it) and hover in place (P mode)
	void RealDrone::Hover(void) {
		this->SendPacket_EmergencyCommand(0);
	}
	
	//Initiate landing sequence immediately at current vehicle location
	void RealDrone::LandNow(void) {
		this->SendPacket_EmergencyCommand(1);
	}
	
	//Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	void RealDrone::GoHomeAndLand(void) {
		this->SendPacket_EmergencyCommand(2);
	}
	
	//Function for internal thread managing drone object
	void RealDrone::DroneMain(void) {
		//tacopie::tcp_server s;

		//// 0.0.0.0 listens on all interfaces (127.0.0.1 only accessible from localhost)
		//s.start("0.0.0.0", 3001, [this](const shared_ptr<tacopie::tcp_client>& client) -> bool {
		//	string hostname = client->get_host();
		//	uint32_t port = client->get_port();

		//	cout << "New client from " + hostname + " on port " + to_string(port) << endl;
		//	client->async_read({ 1024, bind(&RealDrone::on_new_message, this, client, placeholders::_1) });

		//	return true;
		//	});
		////signal(SIGINT, &signint_handler);

		//while (! m_abort) {
		////	m_mutex.lock();

		////	//Check to see if there is new data on the socket to process and process it.
		////	//Decode received packets and update private state		

		////	//If a new image was received and decoded, call the callbacks:
		////	/*if (newImageProcessed) {
		////		for (auto const & kv : m_ImageryCallbacks)
		////			kv.second(NewImage, NewImageTimestamp);
		////	}*/
		////	
		////	//At the end of the loop, if we did useful work do:
		////	//m_mutex.unlock();
		////	//If we did not do useful work do:
		////	m_mutex.unlock();
		//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
		//}
	}
}




