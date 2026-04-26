#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/energy-module.h"
#include <iostream>

using namespace ns3;

uint32_t numNodes = 20;
uint32_t packetSize = 64;
double threatThreshold = 0.95;
double maxSpeed = 1.0;
double initialEnergy = 10.0;
double simTime = 10.0;
uint32_t processedPackets = 0;

bool NetWatchdogFilter(Ptr<const Packet> packet) {
    processedPackets++;
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    return (uv->GetValue(0.0, 1.0) <= threatThreshold);
}

int main(int argc, char *argv[]) {
    Time::SetResolution(Time::NS);
    NodeContainer nodes; nodes.Create(numNodes);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator", 
        "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
        "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=1.0]"),
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"));
    mobility.Install(nodes);

    WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211g);
    YansWifiPhyHelper phy; YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    phy.SetChannel(channel.Create());
    WifiMacHelper mac; mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    BasicEnergySourceHelper energyHelper; energyHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(initialEnergy));
    EnergySourceContainer sources = energyHelper.Install(nodes);
    WifiRadioEnergyModelHelper radioEnergy; radioEnergy.Install(devices, sources);

    AodvHelper aodv; InternetStackHelper stack; stack.SetRoutingHelper(aodv); stack.Install(nodes);
    Ipv4AddressHelper address; address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    UdpServerHelper server(9); ApplicationContainer sApp = server.Install(nodes.Get(numNodes - 1));
    sApp.Start(Seconds(0.0)); sApp.Stop(Seconds(simTime));
    UdpClientHelper client(interfaces.GetAddress(numNodes - 1), 9);
    client.SetAttribute("MaxPackets", UintegerValue(50));
    client.SetAttribute("Interval", TimeValue(MilliSeconds(100)));
    client.SetAttribute("PacketSize", UintegerValue(packetSize));
    ApplicationContainer cApp = client.Install(nodes.Get(0));
    cApp.Start(Seconds(2.0)); cApp.Stop(Seconds(simTime));

    FlowMonitorHelper flowmon; Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    Simulator::Stop(Seconds(simTime)); Simulator::Run();

    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    uint32_t tx = 0, rx = 0; double delay = 0;
    for (auto const& [id, stat] : stats) { 
        tx += stat.txPackets; rx += stat.rxPackets; 
        if(stat.rxPackets > 0) delay += stat.delaySum.GetSeconds() / stat.rxPackets;
        NetWatchdogFilter(nullptr);
    }

    std::cout << "TEST CASE 1 RESULTS\nNodes: 20, PLR: " << (tx > 0 ? (double)(tx - rx) / tx * 100 : 0) << "%, Delay: " << delay * 1000 << "ms\n";
    Simulator::Destroy(); return 0;
}
