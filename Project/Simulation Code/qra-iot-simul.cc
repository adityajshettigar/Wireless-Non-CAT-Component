/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * NS-3 Simulation Script: QRA-IoT Architecture (10 Sequential Runs)
 * Includes: Ad-Hoc Random Waypoint Mobility, AODV Routing, Energy Models, 
 * FlowMonitor Analytics, and custom Overhead calculations.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/energy-module.h"
#include "ns3/netanim-module.h"
#include "ns3/aodv-module.h" 
#include <iostream>
#include <vector>
#include <tuple>
#include <string>

using namespace ns3;
using namespace ns3::energy; 

NS_LOG_COMPONENT_DEFINE ("QRA_IoT_Simulation");

// ========================================================================
// GLOBAL SIMULATION PARAMETERS
// ========================================================================
double g_initialEnergy = 10.0;        
uint32_t g_numPackets = 50;         
double g_simTime = 10.0;            

double g_threatThreshold = 0.85;      
uint32_t g_processedPackets = 0;      

// ========================================================================
// Custom Threat Filter (Simulating NetWatchdog computational load)
// ========================================================================
bool NetWatchdogFilter(Ptr<const Packet> packet) {
    g_processedPackets++; 
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    double threatScore = uv->GetValue(0.0, 1.0); 
    if (threatScore > g_threatThreshold) { return false; }
    return true; 
}

// ========================================================================
// CORE SIMULATION FUNCTION (Runs one complete instance)
// ========================================================================
void RunSimulation(int runId, uint32_t numNodes, uint32_t packetSize, double threatThreshold, double maxSpeed) 
{
    // 1. Reset dynamic globals for this specific run
    g_processedPackets = 0;
    g_threatThreshold = threatThreshold;
    
    uint32_t sourceNodeId = 0;          
    uint32_t destNodeId = numNodes - 1; 

    NS_LOG_UNCOND("\n[--- Starting Run " << runId << " | Nodes: " << numNodes 
                  << " | PktSize: " << packetSize << "B | Speed: " << maxSpeed 
                  << "m/s | Threshold: " << threatThreshold << " ---]");

    NodeContainer nodes;
    nodes.Create (numNodes);

    // Separate the gateway from the mobile nodes
    NodeContainer mobileNodes;
    NodeContainer gatewayNode;
    for (uint32_t i = 0; i < numNodes; ++i) {
        if (i == destNodeId) {
            gatewayNode.Add(nodes.Get(i));
        } else {
            mobileNodes.Add(nodes.Get(i));
        }
    }

    // ========================================================================
    // MOBILITY 1: Random Way Point for IoT Devices (Ad-Hoc)
    // ========================================================================
    MobilityHelper mobileHelper;
    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    mobileHelper.SetPositionAllocator(taPositionAlloc); 
    
    // Dynamically set the max speed based on the parameter passed
    std::string speedString = "ns3::UniformRandomVariable[Min=1.0|Max=" + std::to_string(maxSpeed) + "]";
    
    mobileHelper.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                           "Speed", StringValue (speedString),
                           "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=2.0]"),
                           "PositionAllocator", PointerValue (taPositionAlloc));
    mobileHelper.Install (mobileNodes);

    // ========================================================================
    // MOBILITY 2: Static Position for Edge Gateway 
    // ========================================================================
    MobilityHelper staticHelper;
    staticHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    staticHelper.Install(gatewayNode);
    
    // Place gateway in the exact center of the 50x50 grid
    gatewayNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(25.0, 25.0, 0.0));

    // ========================================================================
    // PHY & MAC LAYER CONFIGURATION 
    // ========================================================================
    WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_80211g);
    
    YansWifiPhyHelper wifiPhy; 
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel"); 
    
    wifiPhy.SetChannel (wifiChannel.Create ());

    WifiMacHelper wifiMac;
    wifiMac.SetType ("ns3::AdhocWifiMac"); 
    NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

    // ========================================================================
    // ENERGY MODEL
    // ========================================================================
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (g_initialEnergy));
    EnergySourceContainer sources = basicSourceHelper.Install (nodes);

    WifiRadioEnergyModelHelper radioEnergyHelper;
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, sources);

    // ========================================================================
    // INTERNET STACK & ROUTING (AODV)
    // ========================================================================
    AodvHelper aodv; 
    Ipv4StaticRoutingHelper staticRouting;
    Ipv4ListRoutingHelper list;
    list.Add (staticRouting, 0);
    list.Add (aodv, 10); 

    InternetStackHelper stack;
    stack.SetRoutingHelper (list); 
    stack.Install (nodes);

    Ipv4AddressHelper address;
    address.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign (devices);

    // ========================================================================
    // APPLICATION TRAFFIC
    // ========================================================================
    uint16_t port = 9; 
    
    UdpServerHelper server (port);
    ApplicationContainer serverApp = server.Install (nodes.Get (destNodeId));
    serverApp.Start (Seconds (0.0));
    serverApp.Stop (Seconds (g_simTime));

    UdpClientHelper client (interfaces.GetAddress (destNodeId), port);
    client.SetAttribute ("MaxPackets", UintegerValue (g_numPackets));
    client.SetAttribute ("Interval", TimeValue (MilliSeconds (100))); 
    client.SetAttribute ("PacketSize", UintegerValue (packetSize));

    ApplicationContainer clientApp = client.Install (nodes.Get (sourceNodeId));
    clientApp.Start (Seconds (2.0));
    clientApp.Stop (Seconds (g_simTime));

    // Background traffic
    for (uint32_t i = 1; i < numNodes - 1; ++i) {
        UdpClientHelper bgClient (interfaces.GetAddress (destNodeId), port);
        bgClient.SetAttribute ("MaxPackets", UintegerValue (g_numPackets / 20)); 
        bgClient.SetAttribute ("Interval", TimeValue (Seconds (10.0))); 
        bgClient.SetAttribute ("PacketSize", UintegerValue (packetSize));
        ApplicationContainer bgApp = bgClient.Install (nodes.Get (i));
        bgApp.Start (Seconds (10.0));
        bgApp.Stop (Seconds (g_simTime));
    }

    // ========================================================================
    // START SIMULATION & FLOW MONITOR
    // ========================================================================
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // ========================================================================
    // NETANIM CONFIGURATION (Unique file per run)
    // ========================================================================
    std::string animFile = "qra-iot-animation-run-" + std::to_string(runId) + ".xml";
    AnimationInterface anim(animFile);
    anim.SetMaxPktsPerTraceFile(5000000); 
        
    for (uint32_t i = 0; i < numNodes; ++i) {
        uint32_t nodeId = nodes.Get(i)->GetId(); 

        if (i == destNodeId) {
            anim.UpdateNodeDescription(nodes.Get(i), "Edge Gateway");
            anim.UpdateNodeColor(nodeId, 255, 0, 0); 
            anim.UpdateNodeSize(nodeId, 5.0, 5.0);
        } else if (i == sourceNodeId) {
            anim.UpdateNodeDescription(nodes.Get(i), "URLLC Source");
            anim.UpdateNodeColor(nodeId, 0, 255, 0);
            anim.UpdateNodeSize(nodeId, 3.0, 3.0);
        } else {
            anim.UpdateNodeColor(nodeId, 0, 0, 255);
            anim.UpdateNodeSize(nodeId, 2.0, 2.0);
        }
    }
    anim.EnablePacketMetadata(true);

    // Execute the Simulator
    Simulator::Stop (Seconds (g_simTime));
    Simulator::Run ();

    // ========================================================================
    // METRICS CALCULATION
    // ========================================================================
    monitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

    double totalDelay = 0.0;
    uint32_t totalTxPackets = 0;
    uint32_t totalRxPackets = 0;
    uint64_t totalTxBytes = 0;
    uint64_t totalRxBytes = 0;
    int validFlows = 0;

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {
        if (i->second.rxPackets > 0) {
            totalDelay += i->second.delaySum.GetSeconds() / i->second.rxPackets;
            validFlows++;
        }
        totalTxPackets += i->second.txPackets;
        totalRxPackets += i->second.rxPackets;
        totalTxBytes += i->second.txBytes;
        totalRxBytes += i->second.rxBytes;
        
        // Pass nullptr instead of 0 for modern C++ standards
        NetWatchdogFilter(nullptr); 
    }

    double pdr = (totalTxPackets > 0) ? ((double)totalRxPackets / totalTxPackets) * 100.0 : 0;
    double avgDelay = (validFlows > 0) ? (totalDelay / validFlows) : 0;
    double throughput = (totalRxBytes * 8.0) / (g_simTime * 1000.0); 
    
    double payloadBytes = totalTxPackets * packetSize;
    double commOverhead = (payloadBytes > 0) ? ((double)(totalTxBytes - payloadBytes) / payloadBytes) * 100.0 : 0;

    double totalEnergyConsumed = 0.0;
    for (uint32_t i = 0; i < sources.GetN(); ++i) {
        Ptr<BasicEnergySource> basicSource = DynamicCast<BasicEnergySource>(sources.Get(i));
        if (basicSource) {
            double remainingEnergy = basicSource->GetRemainingEnergy();
            totalEnergyConsumed += (g_initialEnergy - remainingEnergy);
        }
    }
    double avgEnergyConsumed = totalEnergyConsumed / numNodes;
    double compOverheadMs = g_processedPackets * 0.5; 

    // ========================================================================
    // PRINT RESULTS FOR THIS RUN
    // ========================================================================
    std::cout << "================================================================\n";
    std::cout << "   RUN " << runId << " SUMMARY: " << numNodes << " Nodes | " 
              << packetSize << "B Pkts | " << maxSpeed << "m/s | Thr: " << threatThreshold << "\n";
    std::cout << "================================================================\n";
    std::cout << "[+] Packet Delivery Ratio   : " << pdr << " %\n";
    std::cout << "[+] Average End-to-End Delay: " << (avgDelay * 1000.0) << " ms\n";
    std::cout << "[+] Network Throughput      : " << throughput << " Kbps\n";
    std::cout << "[+] Avg Energy Consumed     : " << avgEnergyConsumed << " Joules / node\n";
    std::cout << "[+] Communication Overhead  : " << commOverhead << " %\n";
    std::cout << "[+] Computational Overhead  : " << compOverheadMs << " ms\n";
    std::cout << "================================================================\n";

    // CRITICAL: Clear the simulation context completely so the next run starts fresh
    Simulator::Destroy();
}

int main (int argc, char *argv[])
{
    // Global NS-3 setups
    Time::SetResolution (Time::NS);
    LogComponentEnable ("QRA_IoT_Simulation", LOG_LEVEL_INFO);

    // ========================================================================
    // DEFINE 10 UNIQUE SIMULATION RUNS
    // Parameter format: { numNodes, packetSize, threatThreshold, maxMobilitySpeed }
    // ========================================================================
    std::vector<std::tuple<uint32_t, uint32_t, double, double>> simulationRuns = {
        {20, 64,  0.95, 1.0},  // Run 1: Light network, small packets, slow movement
        {25, 128, 0.90, 2.0},  // Run 2
        {30, 256, 0.85, 3.0},  // Run 3
        {35, 256, 0.80, 4.0},  // Run 4
        {40, 512, 0.85, 5.0},  // Run 5: Medium network, standard speed
        {45, 512, 0.75, 6.0},  // Run 6
        {50, 256, 0.85, 5.0},  // Run 7: The baseline architecture
        {55, 1024,0.70, 7.0},  // Run 8: Heavy network, large packets, fast
        {60, 1024,0.85, 8.0},  // Run 9
        {65, 256, 0.90, 10.0}  // Run 10: Extreme density and high mobility
    };

    // Iterate through all 10 configurations
    for (size_t i = 0; i < simulationRuns.size(); ++i) {
        uint32_t nodes     = std::get<0>(simulationRuns[i]);
        uint32_t pktSize   = std::get<1>(simulationRuns[i]);
        double   threshold = std::get<2>(simulationRuns[i]);
        double   speed     = std::get<3>(simulationRuns[i]);

        RunSimulation(i + 1, nodes, pktSize, threshold, speed);
    }

    return 0;
}
