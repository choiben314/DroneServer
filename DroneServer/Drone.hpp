#pragma once

//System Includes
#include <vector>
#include <string>
#include <chrono>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include<filesystem>

//External Includes
#include <opencv2/opencv.hpp>
#include <tacopie/tacopie>

//Project Includes
#include "DroneComms.hpp"
#include "DroneDataStructures.h"

namespace DroneInterface {
	
	//Abstract class for drones - This means that an object of this type cannot actually exist... only objects of a derived type. We have two derived types:
	//RealDrone and SimulatedDrone
	class Drone {
		public:
			using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
			Drone() = default;
			virtual ~Drone() = default;
			
			//Basic hardware info (should be available on construction)
			virtual std::string GetDroneSerial(void) = 0;
			
			//Telemetry - all methods return true if at least one valid reading has been received (regardless of age) and false otherwise
			virtual bool GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) = 0; //Lat & Lon (radians) and WGS84 Altitude (m)
			virtual bool GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp) = 0; //NED velocity vector (m/s)
			virtual bool GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp) = 0; //Yaw, Pitch, Roll (radians) using DJI definitions
			virtual bool GetHAG(double & HAG, TimePoint & Timestamp) = 0; //Barometric height above ground (m) - Drone altitude minus takeoff altitude
			
			//Drone state and warnings
			virtual bool GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp) = 0; //Drone Battery level (0 = Empty, 1 = Full)
			virtual bool GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp) = 0; //Whether the drone has hit height or radius limits
			virtual bool GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) = 0; //Wind & other vehicle warnings as strings
			virtual bool GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp) = 0; //GNSS status (-1 for none, 0-5: DJI definitions)
			
			//Drone live video access - The mobile client should decode the live video feed and extract every N'th frame and send it to the server.
			//N should be detirmined when StartDJICamImageFeed() is called in order to achieve the given frame rate as closely as possible.
			//There is a frame counter that starts at 0 and increments each time a new frame is received by the server.
			virtual bool IsDJICamConnected(void) = 0; //Should be available on construction
			virtual void StartDJICamImageFeed(double TargetFPS) = 0; //Start sending frames of live video (as close as possible to the given framerate (frame / s))
			virtual void StopDJICamImageFeed(void) = 0; //Stop sending frames of live video
			virtual bool GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp) = 0;
			virtual int  RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) = 0; //Regester callback for new frames
			virtual void UnRegisterCallback(int Handle) = 0; //Unregister callback for new frames (input is token returned by RegisterCallback()
			
			//Drone Command & Control. If a method returns bool, it should return True if the requested info is known (even if very old) and set the
			//Timestamp argument to the instant of validity of the most recently received value. If the requested info is unknown, these return False.
			//There is a method provided to get an ID for the currently running waypoint mission (if one is running). An unfortunate quirk of the DJI
			//SDK design though is that it doesn't look like you can choose your own IDs when creating a mission, so to use this to see if a new mission
			//is running you need to get the mission ID, start your new mission, and then check the ID again after some time to see if the ID has changed.
			virtual bool GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp) = 0; //Get flight mode as a human-readable string
			virtual void ExecuteWaypointMission(WaypointMission & Mission) = 0; //Stop current mission, if running. Then load, verify, and start new waypoint mission.
			virtual bool IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) = 0;
			virtual bool GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp) = 0;
			virtual void IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command) = 0; //Put in virtualStick Mode and send command (stop mission if running)
			virtual void IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command) = 0; //Put in virtualStick Mode and send command (stop mission if running)
			
			virtual void Hover(void) = 0; //Stop any running missions an leave virtualStick mode (if in it) and hover in place (P mode)
			virtual void LandNow(void) = 0; //Initiate landing sequence immediately at current vehicle location
			virtual void GoHomeAndLand(void) = 0; //Initiate a Return-To-Home sequence that lands the vehicle at it's take-off location
	};
	
	//The RealDrone class provides an interface to interact with a single real drone
	class RealDrone : public Drone {
		public:
			RealDrone();
			RealDrone(tacopie::tcp_client& client);
			~RealDrone();
			
			std::string GetDroneSerial(void) override;
			
			void DataReceivedHandler(const std::shared_ptr<tacopie::tcp_client>& client, const tacopie::tcp_client::read_result& res);
			void SendPacket(Packet & packet);
			void SendPacket_EmergencyCommand(uint8_t Action);
			void SendPacket_CameraControl(uint8_t Action, double TargetFPS);
			void SendPacket_ExecuteWaypointMission(uint8_t LandAtEnd, uint8_t CurvedFlight, std::vector<Waypoint> Waypoints);
			void SendPacket_VirtualStickCommand(uint8_t Mode, float Yaw, float V_x, float V_y, float HAG, float timeout);

			bool GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) override;
			bool GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp)       override;
			bool GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp)           override;
			bool GetHAG(double & HAG, TimePoint & Timestamp)                                                  override;
			
			bool GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp)                   override;
			bool GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp)  override;
			bool GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) override;
			bool GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp)    override;

			bool IsDJICamConnected(void)                                                                            override;
			void StartDJICamImageFeed(double TargetFPS)                                                             override;
			void StopDJICamImageFeed(void)                                                                          override;
			bool GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp)             override;
			int  RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) override;
			void UnRegisterCallback(int Handle)                                                                     override;
			
			bool GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp)         override;
			void ExecuteWaypointMission(WaypointMission & Mission)                         override;
			bool IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) override;
			bool GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp)  override;
			void IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command)       override;
			void IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command)       override;
			
			void Hover(void)         override;
			void LandNow(void)       override;
			void GoHomeAndLand(void) override;
		
		private:
			//Some modules that use imagery can't handle missing frames gracefully. Thus, we use provide a callback mechanism to ensure that such a module
			//can have a guarantee that each frame received by the drone interface module will be provided downstream.
			std::unordered_map<int, std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)>> m_ImageryCallbacks;

			std::thread       m_thread;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex; //Lock in each public method for thread safety

			tacopie::tcp_client* m_client;
			std::string m_serial;

			unsigned int m_frame_num = 0;

			bool m_packet_ct_received = false;
			bool m_packet_et_received = false;
			bool m_packet_img_received = false;
			bool m_packet_ack_received = false;
			bool m_packet_ms_received = false;

			Packet* m_packet_fragment = new Packet();
			
			Packet_CoreTelemetry m_packet_ct;
			Packet_ExtendedTelemetry m_packet_et;
			Packet_Image m_packet_img;
			Packet_Acknowledgment m_packet_ack;
			Packet_MessageString m_packet_ms;

			void DroneMain(void);
	};
	
	//The SimulatedDrone class provides an interface to interact with a single virtual/simulated drone
	//Note: Right now the simulated drone is very dumb... we don't pretend to fly missions or send warnings or any of the other
	//standard things one might expect of a simulated drone. We just mimic a drone in a stationary hover (P mode) over a fixed
	//point in Lamberton, MN. We do provide meaningful implementations of all image-related functions however, so the simulated
	//drone can be used to test anything involving live drone imagery. Imagery is pulled from a video file and dispatched either
	//at real-time speed or as fast as possible, depending on configuration.
	class SimulatedDrone : public Drone {
		public:
			SimulatedDrone();
			~SimulatedDrone();
			
			std::string GetDroneSerial(void) override;
			
			bool GetPosition(double & Latitude, double & Longitude, double & Altitude, TimePoint & Timestamp) override;
			bool GetVelocity(double & V_North, double & V_East, double & V_Down, TimePoint & Timestamp)       override;
			bool GetOrientation(double & Yaw, double & Pitch, double & Roll, TimePoint & Timestamp)           override;
			bool GetHAG(double & HAG, TimePoint & Timestamp)                                                  override;
			
			bool GetVehicleBatteryLevel(double & BattLevel, TimePoint & Timestamp)                   override;
			bool GetActiveLimitations(bool & MaxHAG, bool & MaxDistFromHome, TimePoint & Timestamp)  override;
			bool GetActiveWarnings(std::vector<std::string> & ActiveWarnings, TimePoint & Timestamp) override;
			bool GetGNSSStatus(unsigned int & SatCount, int & SignalLevel, TimePoint & Timestamp)    override;
			
			bool IsDJICamConnected(void)                                                                            override;
			void StartDJICamImageFeed(double TargetFPS)                                                             override;
			void StopDJICamImageFeed(void)                                                                          override;
			bool GetMostRecentFrame(cv::Mat & Frame, unsigned int & FrameNumber, TimePoint & Timestamp)             override;
			int  RegisterCallback(std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)> Callback) override;
			void UnRegisterCallback(int Handle)                                                                     override;
			
			bool GetFlightMode(std::string & FlightModeStr, TimePoint & Timestamp)         override;
			void ExecuteWaypointMission(WaypointMission & Mission)                         override;
			bool IsCurrentlyExecutingWaypointMission(bool & Result, TimePoint & Timestamp) override;
			bool GetCurrentWaypointMissionID(uint16_t & MissionID, TimePoint & Timestamp)  override;
			void IssueVirtualStickCommand(VirtualStickCommand_ModeA const & Command)       override;
			void IssueVirtualStickCommand(VirtualStickCommand_ModeB const & Command)       override;
			
			void Hover(void)         override;
			void LandNow(void)       override;
			void GoHomeAndLand(void) override;
			
			//SimulatedDrone-specific methods
			void SetRealTime(bool Realtime); //True: Imagery will be provided at close-to-real-time rate. False: Imagery is provided as fast as possible
			void SetSourceVideoFile(std::filesystem::path const & VideoPath); //Should be set before calling StartDJICamImageFeed()
			bool IsSimVideoFinished(void); //Returns true if end of video file reached and sim is done
		private:
			//Some modules that use imagery can't handle missing frames gracefully. Thus, we use provide a callback mechanism to ensure that such a module
			//can have a guarantee that each frame received by the drone interface module will be provided downstream.
			std::unordered_map<int, std::function<void(cv::Mat const & Frame, TimePoint const & Timestamp)>> m_ImageryCallbacks;
			
			std::thread       m_thread;
			std::atomic<bool> m_abort;
			std::mutex        m_mutex; //Lock in each public method for thread safety
			
			//Additional State Data
			bool m_realtime = false;
			std::filesystem::path m_videoPath;
			double m_targetFPS = -1.0;
			cv::VideoCapture m_videoCap;
			int m_videoCap_NextFrameIndex = 0;
			int m_videoCap_NumFrames = -1;
			int m_videoCap_FPS = -1;
			bool m_imageFeedActive = false;
			cv::Mat m_Frame;            //Most recent frame
			unsigned int m_FrameNumber = 0; //Frame number of most recent frame (increments on each *used* frame)
			TimePoint m_FrameTimestamp; //Timestamp of most recent frame
			TimePoint m_VideoFeedStartTimestamp; //Timestamp of start of video feed
			bool m_videoFeedStarted = false;
			
			void DroneMain(void);
			
			//Utilities to get seconds into a video from a video file frame number, and to get the next usable video frame (skipping unused frames)
			double FrameNumToTimeIntoVid(int FrameNum) { return double(FrameNum) / double(m_videoCap_FPS); }
			bool GetNextVideoFrame(void); //Advance to and decode the next video from that needs to be used
			bool ResizeTo720p(void);      //Make sure the frame is 720p... resize if needed.
			bool Resize_4K_to_720p(void); //Drop a 4K m_frame down to 720p
	};	
}