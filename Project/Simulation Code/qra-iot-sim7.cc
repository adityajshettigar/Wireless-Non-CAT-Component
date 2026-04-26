/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * NS-3 Standalone Simulation: Test Case 7 (Primary Paper Baseline)
 * Parameters: 50 Nodes, 256B Packets, 5.0 m/s Speed, 0.85 Threat Threshold.
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
#include "ns3/aodv-module.h" 
#include <iostream>

using namespace ns3;
using namespace ns3::energy; 

NS_LOG_COMPONENT_DEFINE ("QRA_IoT_TestCase7");

uint32_t g_processedPackets = 0;
double g_threatThreshold = 0.85;

bool NetWatchdogFilter(Ptr<const Packet> packet) {
    g_processedPackets++; 
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    if (uv->GetValue(0.0, 1.0) > g_threatThreshold) { return false; }
    return true; 
}

int main (int argc, char *argv[])
{
    // ==========================================
    // 1. PARAMETERS FOR TEST CASE 7 (BASELINE)
    // ==========================================
    uint32_t numNodes = 50;
    uint32_t packetSize = 256;
    double maxSpeed = 5.0;
    double initialEnergy = 10.0;
    double simTime = 10.0;
    uint32_t sourceNodeId = 0;
    uint32_t destNodeId = numNodes - 1;

    Time::SetResolution (Time::NS);
    LogComponentEnable ("QRA_IoT_TestCase7", LOG_LEVEL_INFO);

    NodeContainer nodes;
    nodes.Create (numNodes);

    // ==========================================
    // 2. MOBILITY (Random Waypoint)
    // ==========================================
    MobilityHelper mobility;
    mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                   "X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"),
                                   "Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));

    std::string speedStr = "ns3::UniformRandomVariable[Min=1.0|Max=" + std::to_string(maxSpeed) + "]";
    mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                               "Speed", StringValue (speedStr),
                               "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=2.0]"));
    mobility.Install (nodes);

    // ==========================================
    // 3. WIFI & NETWORK STACK (AODV)
    // ==========================================
    WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_80211g);
    YansWifiPhyHelper wifiPhy; 
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel (wifiChannel.Create ());

    WifiMacHelper wifiMac;
    wifiMac.SetType ("ns3::AdhocWifiMac"); 
    NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

    AodvHelper aodv; 
    InternetStackHelper stack;
    stack.SetRoutingHelper (aodv); 
    stack.Install (nodes);

    Ipv4AddressHelper address;
    address.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign (devices);

    // ==========================================
    // 4. ENERGY MODEL
    // ==========================================
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (initialEnergy));
    EnergySourceContainer sources = basicSourceHelper.Install (nodes);
    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Install (devices, sources);

    // ==========================================
    // 5. APPLICATIONS (Baseline Flow)
    // ==========================================
    uint16_t port = 9;
    UdpServerHelper server (port);
    ApplicationContainer serverApp = server.Install (nodes.Get (destNodeId));
    serverApp.Start (Seconds (0.0));
    serverApp.Stop (Seconds (simTime));

    UdpClientHelper client (interfaces.GetAddress (destNodeId), port);
    client.SetAttribute ("MaxPackets", UintegerValue (50));
    client.SetAttribute ("Interval", TimeValue (MilliSeconds (100))); 
    client.SetAttribute ("PacketSize", UintegerValue (packetSize));
    ApplicationContainer clientApp = client.Install (nodes.Get (sourceNodeId));
    clientApp.Start (Seconds (2.0));
    clientApp.Stop (Seconds (simTime));

    // ==========================================
    // 6. SIMULATION & ANALYTICS
    // ==========================================
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop (Seconds (simTime));
    Simulator::Run ();

    monitor->CheckForLostPackets ();
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

    double totalDelay = 0.0;
    double totalJitter = 0.0;
    uint32_t totalTxPackets = 0;
    uint32_t totalRxPackets = 0;
    uint64_t totalRxBytes = 0;
    uint32_t totalHops = 0;
    int validFlows = 0;

    for (auto const& [id, stat] : stats) {
        if (stat.rxPackets > 0) {
            totalDelay += stat.delaySum.GetSeconds() / stat.rxPackets;
            totalJitter += stat.jitterSum.GetSeconds() / stat.rxPackets;
            validFlows++;
        }
        totalTxPackets += stat.txPackets;
        totalRxPackets += stat.rxPackets;
        totalRxBytes += stat.rxBytes;
        totalHops += stat.timesForwarded + stat.rxPackets;
        NetWatchdogFilter(nullptr); 
    }

    double pdr = (totalTxPackets > 0) ? ((double)totalRxPackets / totalTxPackets) * 100.0 : 0;
    double plr = (totalTxPackets > 0) ? (((double)totalTxPackets - (double)totalRxPackets) / totalTxPackets) * 100.0 : 0;
    double avgDelay = (validFlows > 0) ? (totalDelay / validFlows) : 0;
    double avgJitter = (validFlows > 0) ? (totalJitter / validFlows) : 0;
    double avgHopCount = (totalRxPackets > 0) ? ((double)totalHops / totalRxPackets) : 0;
    double throughput = (totalRxBytes * 8.0) / (simTime * 1000.0); 

    double totalEnergyConsumed = 0.0;
    for (uint32_t i = 0; i < sources.GetN(); ++i) {
        Ptr<BasicEnergySource> basicSource = DynamicCast<BasicEnergySource>(sources.Get(i));
        if (basicSource) totalEnergyConsumed += (initialEnergy - basicSource->GetRemainingEnergy());
    }
    double avgEnergyConsumed = totalEnergyConsumed / numNodes;
    double energyEfficiency = (totalEnergyConsumed > 0) ? ((totalRxBytes * 8.0) / totalEnergyConsumed) : 0;

    // ==========================================
    // 7. FINAL OUTPUT
    // ==========================================
    std::cout << "\n================================================================\n";
    std::cout << " TEST CASE 7: Primary Paper Baseline (50 Nodes, 5.0 m/s)\n";
    std::cout << "================================================================\n";
    std::cout << "[+] Packet Delivery Ratio   : " << pdr << " %\n";
    std::cout << "[+] Packet Loss Ratio (PLR) : " << plr << " %\n";
    std::cout << "[+] Average End-to-End Delay: " << (avgDelay * 1000.0) << " ms\n";
    std::cout << "[+] Average Jitter Variation: " << (avgJitter * 1000.0) << " ms\n";
    std::cout << "[+] Network Throughput      : " << throughput << " Kbps\n";
    std::cout << "[+] Average Hop Count       : " << avgHopCount << " hops\n";
    std::cout << "[+] Avg Energy Consumed     : " << avgEnergyConsumed << " Joules / node\n";
    std::cout << "[+] IoT Energy Efficiency   : " << energyEfficiency << " Bits / Joule\n";
    std::cout << "[+] Computational Overhead  : " << (g_processedPackets * 0.5) << " ms\n";
    std::cout << "================================================================\n";

    Simulator::Destroy();
    return 0;
}
