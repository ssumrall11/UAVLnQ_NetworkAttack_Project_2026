/*--------------------------------------------------------------------------------------
 Multihop Topology + AODV Routing Protocol Baseline Implementation

 Programmed by Sarah Sumrall

 Advice & Assistance by Abdelrahman Yousef

 Important note: AI was used for programming/development assistance 
-----------------------------------------------------------------------------------------*/
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/realtime-simulator-impl.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/point-to-point-module.h"
#include "ns3/aodv-module.h"
#include "ns3/flow-monitor-module.h"
#include <common/mavlink.h>

#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <ctime>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneZmqMeshNetwork");

static std::atomic<bool> keepRunning(true);
std::mutex positionMutex;
std::queue<std::vector<uint8_t>> positionQueue;  // Changed to binary queue

NodeContainer drones;

NetDeviceContainer droneDevices;
Ipv4InterfaceContainer droneInterfaces;
std::vector<Ptr<MobilityModel>> droneMobilityModels;

// Global GPS reference point
static double s_refLat = -35.3628;
static double s_refLon = 149.1650;
static double s_refAlt = 0.0;
static double s_metersPerDegreeLat = 111320.0;
static double s_metersPerDegreeLon = s_metersPerDegreeLat * std::cos(s_refLat * M_PI / 180.0);


// Global variables for drone IPs
std::vector<Ipv4Address> droneIpAddresses;

// ZMQ context and publisher for commands
zmq::context_t g_zmqContext(1);
zmq::socket_t* g_commandPublisher = nullptr;

// Global vector for drone sockets
std::vector<Ptr<Socket>> g_droneSockets;

// MAVLink parser state - one per drone for proper parsing
std::map<uint8_t, mavlink_status_t> mavlink_status_map;

// Create MAVLink packet using official library
// This was altered to fit a multihop topology
std::vector<uint8_t> CreateMavlinkPacket(
    uint8_t system_id,
    uint8_t target_system,
    uint8_t target_component,
    float lat, float lon, float alt)
{
    mavlink_message_t msg;

    uint8_t component_id = 1;

    mavlink_mission_item_t mission_item{};
    mission_item.target_system = target_system;
    mission_item.target_component = target_component;
    mission_item.seq = 0;
    mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mission_item.command = MAV_CMD_NAV_WAYPOINT;
    mission_item.current = 0;
    mission_item.autocontinue = 1;

    mission_item.param1 = 0;
    mission_item.param2 = 0;
    mission_item.param3 = 0;
    mission_item.param4 = 0;

    mission_item.x = lat;
    mission_item.y = lon;
    mission_item.z = alt;

    mavlink_msg_mission_item_encode(system_id, component_id, &msg, &mission_item);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    return std::vector<uint8_t>(buffer, buffer + len);
}

    // Send waypoint commands directly from Drone0 to all followers
    // Waypoints format: {x, y, altitude} in meters (local coordinates)
    // Each follower has an array of waypoints sent at different times:
    //   - waypoints[0] sent at t=20s (pairIndex 0)
    //   - waypoints[1] sent at t=30s (pairIndex 1)
    //   - waypoints[2] sent at t=40s (pairIndex 2)
    // So each follower has a 3 waypoint mission send to him at different times (Can be extended and changed)
void SendWaypointPairFromDrone0(int pairIndex)
{
    if (pairIndex < 0) return;

    static const std::vector<std::vector<std::tuple<float,float,float>>> follower_waypoints = {

        // Drone 1
       // {{50,60,30},{20,60,30},{20,30,30}},

        // Drone 2
        //{{40,50,30},{30,40,30},{50,20,30}},

        // Drone 3
        //{{60,70,30},{40,60,30},{70,30,30}},

        // Drone 4
        //{{0,0,0},{20,20,20},{40,40,40}}
};

    for (uint32_t follower = 6; follower < drones.GetN(); ++follower)
    {
        uint32_t idx = follower - 6;

        if (idx >= follower_waypoints.size()) break;
        if (pairIndex >= follower_waypoints[idx].size()) continue;

        auto [x, y, z] = follower_waypoints[idx][pairIndex];

        uint8_t target_system = follower + 1;

        std::vector<uint8_t> pkt =
            CreateMavlinkPacket(1, target_system, 0, x, y, z);

        Ptr<Packet> packet = Create<Packet>(pkt.data(), pkt.size());

        InetSocketAddress dest(droneIpAddresses[follower], 20000);

        g_droneSockets[0]->SendTo(packet, 0, dest);

        NS_LOG_INFO("Leader sent waypoint to drone "
                    << follower << " via MANET routing");
    }
}

void SendHeartbeatFromDrone0()
{
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat{};
    heartbeat.type           = MAV_TYPE_QUADROTOR;
    heartbeat.autopilot      = MAV_AUTOPILOT_ARDUPILOTMEGA;
    heartbeat.base_mode      = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    heartbeat.custom_mode    = 0;
    heartbeat.system_status  = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode(1, 1, &msg, &heartbeat);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    Ptr<Packet> packet = Create<Packet>(buffer, len);

    // Send to last drone (drone 6)
    InetSocketAddress dest(droneIpAddresses[drones.GetN() - 1], 20000);
    g_droneSockets[0]->SendTo(packet, 0, dest);

    NS_LOG_INFO("Drone 0 sent HEARTBEAT to Drone 3 via AODV chain");

    // Schedule next heartbeat in 1 second (standard MAVLink rate)
}


void SendHeartbeatFromLastDrone()
{
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat{};
    heartbeat.type           = MAV_TYPE_QUADROTOR;
    heartbeat.autopilot      = MAV_AUTOPILOT_ARDUPILOTMEGA;
    heartbeat.base_mode      = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    heartbeat.custom_mode    = 0;
    heartbeat.system_status  = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode(drones.GetN(), 1, &msg, &heartbeat);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    Ptr<Packet> packet = Create<Packet>(buffer, len);

    InetSocketAddress dest(droneIpAddresses[0], 20000);
    g_droneSockets[drones.GetN() - 1]->SendTo(packet, 0, dest);

    NS_LOG_INFO("Drone " << (drones.GetN() - 1) << " sent HEARTBEAT to Drone 0 via AODV chain");
}


// Process binary MAVLink GPS_RAW_INT, SYS_STATUS, and HEARTBEAT messages that will received from ZMQ 
// ZMQ message format: [message_type (1 byte), drone_id (1 byte), MAVLink message bytes]
// Message types: 0 = GPS_RAW_INT, 1 = SYS_STATUS, 2 = HEARTBEAT
//
// ========== OPTIMIZATION: SELECTIVE FORWARDING==========
// Only forward commands from leader (Drone 0) to prevent network flooding
// GPS, HEARTBEAT, and SYS_STATUS are processed locally but NOT forwarded
// This reduces network traffic by ~90% for large swarms (10+ drones)
// This was altered to fit a multihop topology
// ========================================================
void ProcessMavlinkMessage(const std::vector<uint8_t>& data)
{
    if (data.size() < 2) return;

    uint8_t msg_type = data[0];
    uint8_t droneId = data[1];

    if (droneId >= drones.GetN()) {
        NS_LOG_WARN("Invalid drone ID: " << (int)droneId);
        return;
    }

    mavlink_message_t msg;
    mavlink_status_t* status = &mavlink_status_map[droneId];

    for (size_t i = 2; i < data.size(); i++)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, status))
        {
            // ---------------- GPS (local update only)
            if (msg_type== 0 && msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
            {
                mavlink_gps_raw_int_t gps;
                mavlink_msg_gps_raw_int_decode(&msg, &gps);

                double lat = gps.lat / 1e7;
                double lon = gps.lon / 1e7;
                double alt = gps.alt / 1000.0;

                double x = (lon - s_refLon) * s_metersPerDegreeLon;
                double y = (lat - s_refLat) * s_metersPerDegreeLat;
                double z = alt - s_refAlt;


                NS_LOG_INFO("Drone " << (int)droneId << " GPS alt raw=" << gps.alt 
               << " alt_m=" << alt << " z=" << z);

              /*  if (droneId < droneMobilityModels.size() &&
                    droneMobilityModels[droneId])
                {
                    droneMobilityModels[droneId]->SetPosition(Vector(x, y, z));
                }
            }*/

     	        /* if (droneId < droneMobilityModels.size() && droneMobilityModels[droneId]) {
                   Vector current = droneMobilityModels[droneId]->GetPosition();
                   double dist = std::sqrt(std::pow(x-current.x,2) + std::pow(y-current.y,2));
                   if (dist > 1.0) {  // only update if moved >1m
                     droneMobilityModels[droneId]->SetPosition(Vector(x, y, z));
                  }
               }*/
            }
            // ---------------- SYS_STATUS (log only)
            //else if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS)
            //{
            //    mavlink_sys_status_t sys;
            //    mavlink_msg_sys_status_decode(&msg, &sys);
            //}

            // ---------------- HEARTBEAT (log only)
            else if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
            {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
            }

            // ---------------- CONTROL MESSAGES (DO NOT FORWARD HERE)
            // IMPORTANT: no routing logic here anymore
            else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM)
            {
                NS_LOG_INFO("Mission item received by drone " << (int)droneId);
            }
            else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
            {
                NS_LOG_INFO("Command received by drone " << (int)droneId);
            }
            else if (msg.msgid == MAVLINK_MSG_ID_SET_MODE)
            {
                NS_LOG_INFO("Mode change received by drone " << (int)droneId);
            }
        }
   } 
}

// Modified ZMQ receiver to handle binary data
// This was altered to fit a multihop topology
void ZmqPositionReceiverThread()
{
    zmq::socket_t subscriber(g_zmqContext, ZMQ_SUB);

    subscriber.connect("tcp://localhost:5556");
    subscriber.set(zmq::sockopt::subscribe, "");

    NS_LOG_INFO("ZMQ subscriber connected to port 5556");

    const size_t MAX_QUEUE_SIZE = 1000;

    while (keepRunning.load())
    {
        zmq::message_t message;

        if (subscriber.recv(message, zmq::recv_flags::dontwait))
        {
            uint8_t* data =
                static_cast<uint8_t*>(message.data());

            size_t length = message.size();

            if (length < 3)
                continue;

            std::vector<uint8_t> msg_vec(data, data + length);

            {
                std::lock_guard<std::mutex> lock(positionMutex);

                if (positionQueue.size() < MAX_QUEUE_SIZE)
                {
                    positionQueue.push(std::move(msg_vec));
                }
            }

            NS_LOG_DEBUG("ZMQ MAVLink message received ("
                         << length << " bytes)");
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(2));
    }
}
// This was altered to fit a multihop topology
void UpdatePositionsFromQueue() {
    std::queue<std::vector<uint8_t>> localQueue;
    {
        std::lock_guard<std::mutex> lock(positionMutex);
        std::swap(localQueue, positionQueue);
    }

    while (!localQueue.empty()) {
        ProcessMavlinkMessage(localQueue.front());
        localQueue.pop();
    }

    Simulator::Schedule(Seconds(0.01), &UpdatePositionsFromQueue);
}
// This was altered to fit a multihop topology
void PrintDronePositions(NodeContainer nodes, double interval, double simTime) {
    Time now = Simulator::Now();
    if (now.GetSeconds() > simTime)
        return;

    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<MobilityModel> mobility = nodes.Get(i)->GetObject<MobilityModel>();
        if (mobility) {
            Vector pos = mobility->GetPosition();
            std::cout << "Time " << now.GetSeconds() << "s, Drone " << i
                      << " Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
        }
    }

    Simulator::Schedule(Seconds(interval), &PrintDronePositions, nodes, interval, simTime);
}
// This was altered to fit a multihop topology
void PacketReceived(Ptr<const Packet> p,
                    const Address& addr,
                    uint32_t droneId)
{
    InetSocketAddress inetAddr =
        InetSocketAddress::ConvertFrom(addr);

    NS_LOG_INFO("Drone "
        << droneId
        << " received HB packet from "
        << inetAddr.GetIpv4()
        << " size=" << p->GetSize());

    std::vector<uint8_t> buffer(p->GetSize());

    p->CopyData(buffer.data(), buffer.size());

    mavlink_message_t msg;
    mavlink_status_t& status = mavlink_status_map[droneId]; //debugging per Abdelrahman's advice

    for (size_t i = 0; i < buffer.size(); i++)
    {
        if (mavlink_parse_char(
                MAVLINK_COMM_0,
                buffer[i],
                &msg,
                &status))
        {
            if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM)
            {

                mavlink_mission_item_t mission;

                mavlink_msg_mission_item_decode(
                    &msg,
                    &mission);

                NS_LOG_INFO("Drone "
                    << droneId
                    << " received waypoint: "
                    << mission.x << ", "
                    << mission.y << ", "
                    << mission.z);
		if (mission.target_system == (uint8_t)(droneId + 1) && g_commandPublisher) {
                    //uint8_t type_byte = 3;
                    //std::vector<uint8_t> frame;
                    //frame.push_back(type_byte);
                    //frame.push_back((uint8_t)droneId); //debugging per Abdelrahman's advice
                    //frame.insert(frame.end(), buffer.begin(), buffer.end());
                    zmq::message_t zmqMsg(buffer.data(), buffer.size());
                    g_commandPublisher->send(zmqMsg, zmq::send_flags::none);
                    NS_LOG_INFO("Drone " << droneId << " forwarded MISSION_ITEM to ZMQ");
                }
            }
        }
    }
}

void InstallPacketSinks(NodeContainer nodes, std::vector<uint16_t> ports, double simTime) {
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        for (uint16_t port : ports) {
            PacketSinkHelper sink("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), port));
            ApplicationContainer sinkApp = sink.Install(nodes.Get(i));
            sinkApp.Start(Seconds(0.0));
            sinkApp.Stop(Seconds(simTime));

            Ptr<PacketSink> ps = sinkApp.Get(0)->GetObject<PacketSink>();
            Callback<void, std::string, Ptr<const Packet>, const Address&> cb =
                [i](std::string /*context*/, Ptr<const Packet> p, const Address& addr) {
                    PacketReceived(p, addr, i);
                };
            ps->TraceConnect("Rx", "Drone" + std::to_string(i), cb);
        }
    }
}


int main(int argc, char *argv[])
{
    uint32_t nNodes = 7;
    double simTime = 100.0;
    double txRate = 1.0; // packets/sec
    std::string outputDir = "/home/ubuntu/UAVLnQ/ns3-output"; 


    CommandLine cmd;
    cmd.AddValue("nNodes", "Number of nodes", nNodes);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("o", "Output directory for PCAP and animation files", outputDir);
    cmd.Parse(argc, argv);

    //debugging per Abdelrahman's advice
    g_commandPublisher = new zmq::socket_t(g_zmqContext, ZMQ_PUB);
    g_commandPublisher->bind("tcp://*:5555");
    Simulator::SetImplementation(CreateObject<RealtimeSimulatorImpl>());


    // -----------------------
    // 1. Create nodes
    // -----------------------
    //NodeContainer nodes;
    drones.Create(nNodes);

    // -----------------------
    // 2. Mobility (linear chain)
    // -----------------------
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();

    for (uint32_t i = 0; i < nNodes; i++)
        pos->Add(Vector(i * 80, 0, 20));

    mobility.SetPositionAllocator(pos);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(drones);
    droneMobilityModels.clear();
    for (uint32_t i = 0; i < drones.GetN(); ++i)
	    droneMobilityModels.push_back(drones.Get(i)->GetObject<MobilityModel>());

    // -----------------------
    // 3. WiFi Adhoc
    // -----------------------
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::RangePropagationLossModel",
                               "MaxRange", DoubleValue(82));

    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, drones);

    // -----------------------
    // 4. AODV ONLY
    // -----------------------
    AodvHelper aodv;
    aodv.Set("HelloInterval", TimeValue(Seconds(3.0)));
    aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(20.0)));
    aodv.Set("MyRouteTimeout", TimeValue(Seconds(20.0)));
    
    Ptr<OutputStreamWrapper> routingStream =
    Create<OutputStreamWrapper>(
        outputDir + "/aodv.routes",
        std::ios::out);

    aodv.PrintRoutingTableAllEvery(Seconds(1.0), routingStream);

    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(drones);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);
    droneIpAddresses.clear();
    for (uint32_t i = 0; i < interfaces.GetN(); ++i)
	     droneIpAddresses.push_back(interfaces.GetAddress(i));

    // -----------------------
    // 5. UDP traffic (simple chain)
    // -----------------------
    uint16_t port = 9;

    // Sink at last node
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    
    //sink.Install(drones.Get(1));
    ApplicationContainer sinkApp = sink.Install(drones.Get(nNodes - 1)); 
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simTime));

    // Source at node 0
    OnOffHelper app("ns3::UdpSocketFactory",
                    InetSocketAddress(interfaces.GetAddress(nNodes - 1), port)); 
    //app.Install(drones.Get(0));
    app.SetAttribute("DataRate", StringValue("0.5kbps"));
    app.SetAttribute("PacketSize", UintegerValue(64));
    app.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    app.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    ApplicationContainer srcApp = app.Install(drones.Get(0));
    srcApp.Start(Seconds(1.0));
    srcApp.Stop(Seconds(simTime));

    // Reverse traffic: N6 -> N0 (telemetry back to leader)
     PacketSinkHelper sinkReverse("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), 10));
     ApplicationContainer sinkRevApp = sinkReverse.Install(drones.Get(0));
    sinkRevApp.Start(Seconds(0.0));
    sinkRevApp.Stop(Seconds(simTime));

    OnOffHelper appReverse("ns3::UdpSocketFactory",
                        InetSocketAddress(interfaces.GetAddress(0), 10));
    appReverse.SetAttribute("DataRate", StringValue("0.5kbps"));
    appReverse.SetAttribute("PacketSize", UintegerValue(64));
    appReverse.SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    appReverse.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    ApplicationContainer srcRevApp = appReverse.Install(drones.Get(nNodes - 1));
    srcRevApp.Start(Seconds(1.0));
    srcRevApp.Stop(Seconds(simTime));



    g_droneSockets.clear();
    for (uint32_t i = 0; i < drones.GetN(); i++) {
	    Ptr<Socket> socket = Socket::CreateSocket(drones.Get(i), UdpSocketFactory::GetTypeId());
	    socket->Bind();
	     g_droneSockets.push_back(socket);
    }

    
    // Setup output files
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << outputDir << "/multihop-aodv"
        << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string outputPrefix = oss.str();

    std::ostringstream animOss;
    animOss << outputDir << "/multihop-aodv-anim_"
            << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".xml";
    std::string animFile = animOss.str();

    AnimationInterface anim(animFile);
    anim.SetMobilityPollInterval(Seconds(0.1));
    anim.EnablePacketMetadata(true);

    anim.UpdateNodeDescription(drones.Get(0), "Source");
    anim.UpdateNodeDescription(drones.Get(drones.GetN() - 1), "Destination");
    anim.EnableIpv4RouteTracking(outputDir + "/routes.xml", Seconds(0), Seconds(10), Seconds(5));
    anim.UpdateNodeColor(drones.Get(0), 0, 255, 0);
    anim.UpdateNodeColor(drones.Get(1), 0, 0, 255);
    anim.UpdateNodeColor(drones.Get(2), 255, 0, 0);
    anim.UpdateNodeColor(drones.Get(3), 255, 0, 0);
    anim.UpdateNodeColor(drones.Get(4), 255, 165, 0);  // orange
    anim.UpdateNodeColor(drones.Get(5), 128, 0, 128);  // purple
    anim.UpdateNodeColor(drones.Get(6), 255, 255, 0);  // yellow - destination
    // -----------------------
    // 6. Flow Monitor
    // -----------------------
    phy.EnablePcapAll(outputPrefix);

        /*Ptr<OutputStreamWrapper> routingStream =
        Create<OutputStreamWrapper>(prefix + "-routing.txt", std::ios::out);
    aodv.PrintRoutingTableAllAt(Seconds(0.5),  routingStream);
    aodv.PrintRoutingTableAllAt(Seconds(2.5),  routingStream);
    aodv.PrintRoutingTableAllAt(Seconds(25.0), routingStream);
*/
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simTime));
    
    //debugging per Abdelrahman's advice
    
    for (uint32_t i = 0; i < nNodes; ++i)
        memset(&mavlink_status_map[i], 0, sizeof(mavlink_status_t));

    //g_commandPublisher = new zmq::socket_t(g_zmqContext, ZMQ_PUB);
    //g_commandPublisher->bind("tcp://*:5555");
    NodeContainer endpoints;
    endpoints.Add(drones.Get(0));
    endpoints.Add(drones.Get(nNodes - 1));
    InstallPacketSinks(endpoints, {20000}, simTime);
    //Simulator::Schedule(Seconds(50.0), &SendWaypointPairFromDrone0, 0);
    //Simulator::Schedule(Seconds(60.0), &SendWaypointPairFromDrone0, 1);
    //Simulator::Schedule(Seconds(70.0), &SendWaypointPairFromDrone0, 2);
    Simulator::Schedule(Seconds(2.0),  &SendHeartbeatFromDrone0);
    Simulator::Schedule(Seconds(4.0),  &SendHeartbeatFromDrone0);
    Simulator::Schedule(Seconds(6.0),  &SendHeartbeatFromDrone0);
    Simulator::Schedule(Seconds(8.0),  &SendHeartbeatFromDrone0);
    //Simulator::Schedule(Seconds(10.0), &SendHeartbeatFromDrone0);
    //Simulator::Schedule(Seconds(12.0), &SendHeartbeatFromDrone0);
    Simulator::Schedule(Seconds(14.0), &SendHeartbeatFromDrone0);
    Simulator::Schedule(Seconds(16.0), &SendHeartbeatFromLastDrone);
    Simulator::Schedule(Seconds(18.0), &SendHeartbeatFromLastDrone);
    Simulator::Schedule(Seconds(20.0), &SendHeartbeatFromLastDrone);
    //Simulator::Schedule(Seconds(16.0), &SendHeartbeatFromDrone0);
    //Simulator::Schedule(Seconds(18.0), &SendHeartbeatFromDrone0);
    //Simulator::Schedule(Seconds(20.0), &SendHeartbeatFromDrone0);
    std::thread zmqThread(ZmqPositionReceiverThread);
    Simulator::Schedule(Seconds(0.01), &UpdatePositionsFromQueue);
    Simulator::Schedule(Seconds(1.0), &PrintDronePositions, drones, 1.0, simTime);

    // Add this just before Simulator::Run()
std::cout << "\n===== NODE POSITIONS & DISTANCES =====\n";
for (uint32_t i = 0; i < drones.GetN(); ++i) {
    Vector pos = drones.Get(i)->GetObject<MobilityModel>()->GetPosition();
    std::cout << "Drone " << i << ": (" << pos.x << ", " << pos.y << ", " << pos.z << ")";
    if (i > 0) {
        Vector prev = drones.Get(i-1)->GetObject<MobilityModel>()->GetPosition();
        double dist = std::sqrt(std::pow(pos.x-prev.x,2) +
                                std::pow(pos.y-prev.y,2) +
                                std::pow(pos.z-prev.z,2));
        std::cout << "  dist from Drone " << (i-1) << " = " << dist << "m";
    }
    std::cout << "\n";
}
   std::cout << "MaxRange=80m → each drone sees only immediate neighbor\n";
   std::cout << "=======================================\n\n";

    Simulator::Run();

    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

    auto stats = monitor->GetFlowStats();

    std::cout << "\n===== AODV BASELINE RESULTS =====\n";

    for (auto &flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);

        double pdr = (flow.second.txPackets > 0)
                       ? (double)flow.second.rxPackets / flow.second.txPackets
                       : 0;

        std::cout << "Flow " << flow.first
                  << " " << t.sourceAddress << " -> " << t.destinationAddress << "\n"
                  << "TX=" << flow.second.txPackets
                  << " RX=" << flow.second.rxPackets
                  << " LOST=" << flow.second.lostPackets
                  << " PDR=" << pdr
		  << "  meanDelay=" << flow.second.delaySum.GetSeconds() / 
                               std::max(flow.second.rxPackets, (uint32_t)1)
                  << "  throughput=" << flow.second.rxBytes * 8.0 / simTime
		  << "\n\n";
    }
    
    uint64_t totalDropped = 0;
    uint64_t totalTx      = 0;

    Ipv4Address n0 = interfaces.GetAddress(0);
    Ipv4Address n6 = interfaces.GetAddress(nNodes - 1);

    for (auto& flow : stats) {
       Ipv4FlowClassifier::FiveTuple ft = classifier->FindFlow(flow.first);

        if ((ft.sourceAddress == n0 && ft.destinationAddress == n6)||(ft.sourceAddress == n6 && ft.destinationAddress == n0)) {
          totalDropped += flow.second.lostPackets;
          totalTx      += flow.second.txPackets;

           std::cout << "[ATTACK FLOW] "
                  << ft.sourceAddress << " -> "
                  << ft.destinationAddress
                  << "  TX=" << flow.second.txPackets
                  << "  RX=" << flow.second.rxPackets
                  << "  LOST=" << flow.second.lostPackets
                  << "  drop rate: "
                  << (double)flow.second.lostPackets /
                     std::max(flow.second.txPackets, (uint32_t)1)
                  << "\n";
         }
    }

    std::cout << "Overall wormhole drop rate: "
          << (double)totalDropped / std::max(totalTx, (uint64_t)1)
          << "\n";

    keepRunning.store(false);
    zmqThread.join();
    delete g_commandPublisher;
    Simulator::Destroy();
    return 0;
}
