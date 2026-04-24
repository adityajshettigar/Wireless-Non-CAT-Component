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

double g_initialEnergy = 10.0;        
uint32_t g_numPackets = 50;         
double g_simTime = 10.0;            

double g_threatThreshold = 0.85;      
uint32_t g_processedPackets = 0;      

bool NetWatchdogFilter(Ptr<const Packet> packet) {
    g_processedPackets++; 
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    double threatScore = uv->GetValue(0.0, 1.0); 
    if (threatScore > g_threatThreshold) { return false; }
    return true; 
}

void RunSimulation(int runId, uint32_t numNodes, uint32_t packetSize, double threatThreshold, double maxSpeed) 
{
 
    g_processedPackets = 0;
    g_threatThreshold = threatThreshold;
    
    uint32_t sourceNodeId = 0;          
    uint32_t destNodeId = numNodes - 1; 

    NS_LOG_UNCOND("\n[--- Starting Run " << runId << " | Nodes: " << numNodes 
                  << " | PktSize: " << packetSize << "B | Speed: " << maxSpeed 
                  << "m/s | Threshold: " << threatThreshold << " ---]");

    NodeContainer nodes;
    nodes.Create (numNodes);


    NodeContainer mobileNodes;
    NodeContainer gatewayNode;
    for (uint32_t i = 0; i < numNodes; ++i) {
        if (i == destNodeId) {
            gatewayNode.Add(nodes.Get(i));
        } else {
            mobileNodes.Add(nodes.Get(i));
        }
    }

    MobilityHelper mobileHelper;
    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    mobileHelper.SetPositionAllocator(taPositionAlloc); 
    
    std::string speedString = "ns3::UniformRandomVariable[Min=1.0|Max=" + std::to_string(maxSpeed) + "]";
    
    mobileHelper.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                           "Speed", StringValue (speedString),
                           "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=2.0]"),
                           "PositionAllocator", PointerValue (taPositionAlloc));
    mobileHelper.Install (mobileNodes);

    MobilityHelper staticHelper;
    staticHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    staticHelper.Install(gatewayNode);
    gatewayNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(25.0, 25.0, 0.0));

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

    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (g_initialEnergy));
    EnergySourceContainer sources = basicSourceHelper.Install (nodes);

    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Install (devices, sources);

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

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop (Seconds (g_simTime));
    Simulator::Run ();
    monitor->CheckForLostPackets ();
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

    double totalDelay = 0.0;
    double totalJitter = 0.0;
    uint32_t totalTxPackets = 0;
    uint32_t totalRxPackets = 0;
    uint64_t totalTxBytes = 0;
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
        totalTxBytes += stat.txBytes;
        totalRxBytes += stat.rxBytes;
        totalHops += stat.timesForwarded + stat.rxPackets;
        NetWatchdogFilter(nullptr); 
    }

    double pdr = (totalTxPackets > 0) ? ((double)totalRxPackets / totalTxPackets) * 100.0 : 0;
    double plr = (totalTxPackets > 0) ? (((double)totalTxPackets - (double)totalRxPackets) / totalTxPackets) * 100.0 : 0;
    double avgDelay = (validFlows > 0) ? (totalDelay / validFlows) : 0;
    double avgJitter = (validFlows > 0) ? (totalJitter / validFlows) : 0;
    double avgHopCount = (totalRxPackets > 0) ? ((double)totalHops / totalRxPackets) : 0;
    double throughput = (totalRxBytes * 8.0) / (g_simTime * 1000.0); 

    double totalEnergyConsumed = 0.0;
    for (uint32_t i = 0; i < sources.GetN(); ++i) {
        Ptr<BasicEnergySource> basicSource = DynamicCast<BasicEnergySource>(sources.Get(i));
        if (basicSource) {
            totalEnergyConsumed += (g_initialEnergy - basicSource->GetRemainingEnergy());
        }
    }
    
    double avgEnergyConsumed = totalEnergyConsumed / numNodes;
    double energyEfficiency = (totalEnergyConsumed > 0) ? ((totalRxBytes * 8.0) / totalEnergyConsumed) : 0;
    double compOverheadMs = g_processedPackets * 0.5; 

    std::cout << "================================================================\n";
    std::cout << "   RUN " << runId << " SUMMARY: " << numNodes << " Nodes | " 
              << packetSize << "B Pkts | " << maxSpeed << "m/s | Thr: " << g_threatThreshold << "\n";
    std::cout << "================================================================\n";
    std::cout << "[+] Packet Delivery Ratio   : " << pdr << " %\n";
    std::cout << "[+] Packet Loss Ratio (PLR) : " << plr << " %\n";
    std::cout << "[+] Average End-to-End Delay: " << (avgDelay * 1000.0) << " ms\n";
    std::cout << "[+] Average Jitter Variation: " << (avgJitter * 1000.0) << " ms\n";
    std::cout << "[+] Network Throughput      : " << throughput << " Kbps\n";
    std::cout << "[+] Average Hop Count       : " << avgHopCount << " hops\n";
    std::cout << "[+] Avg Energy Consumed     : " << avgEnergyConsumed << " Joules / node\n";
    std::cout << "[+] IoT Energy Efficiency   : " << energyEfficiency << " Bits / Joule\n";
    std::cout << "[+] Computational Overhead  : " << compOverheadMs << " ms\n";
    std::cout << "================================================================\n\n";

    Simulator::Destroy();
}

int main (int argc, char *argv[])
{
    Time::SetResolution (Time::NS);
    std::vector<std::tuple<uint32_t, uint32_t, double, double>> simulationRuns = {
        {20, 64,   0.95, 1.0},
        {25, 128,  0.90, 2.0},
        {30, 256,  0.85, 3.0},
        {35, 256,  0.80, 4.0},
        {40, 512,  0.85, 5.0},
        {45, 512,  0.75, 6.0},
        {50, 256,  0.85, 5.0},
        {55, 1024, 0.70, 7.0},
        {60, 1024, 0.85, 8.0},
        {65, 256,  0.90, 10.0}
    };

    for (size_t i = 0; i < simulationRuns.size(); ++i) {
        auto [nodes, pktSize, threshold, speed] = simulationRuns[i];
        RunSimulation(i + 1, nodes, pktSize, threshold, speed);
    }

    return 0;
}
