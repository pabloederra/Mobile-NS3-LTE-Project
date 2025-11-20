#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteProjectSimulation");

int main(int argc, char *argv[])
{
    double simTime = 20.0;

    CommandLine cmd;
    cmd.Parse(argc);

    // LTE + EPC
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    InternetStackHelper internet;

    // -------------------------------
    // REMOTE HOSTS
    // -------------------------------
    NodeContainer remoteHosts;
    remoteHosts.Create(2);
    internet.Install(remoteHosts);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));

    for (uint32_t i = 0; i < 2; i++)
    {
        NetDeviceContainer link = p2ph.Install(pgw, remoteHosts.Get(i));
        Ipv4AddressHelper addr;
        addr.SetBase(("1." + std::to_string(i+1) + ".0.0").c_str(), "255.255.0.0");
        addr.Assign(link);
    }

    // -------------------------------
    // NODES: eNB + UEs
    // -------------------------------
    NodeContainer enbs;
    enbs.Create(4);

    NodeContainer ues;
    ues.Create(10);
    internet.Install(ues);

    // -------------------------------
    // MOBILITY FOR ENBs
    // -------------------------------
    MobilityHelper enbMobility;
    Ptr<ListPositionAllocator> enbPositions = CreateObject<ListPositionAllocator>();
    enbPositions->Add(Vector(0, 0, 20));
    enbPositions->Add(Vector(200, 0, 20));
    enbPositions->Add(Vector(0, 200, 20));
    enbPositions->Add(Vector(200, 200, 20));

    enbMobility.SetPositionAllocator(enbPositions);
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.Install(enbs);

    // -------------------------------
    // MOBILITY FOR UEs
    // -------------------------------
    MobilityHelper ueMobility;
    ueMobility.SetPositionAllocator(
        "ns3::RandomRectanglePositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=200]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=200]")
    );

    // First 5 UEs: RandomWalk
    for (int i = 0; i < 5; i++)
    {
        ueMobility.SetMobilityModel(
            "ns3::RandomWalk2dMobilityModel",
            "Speed", StringValue("ns3::ConstantRandomVariable[Constant=5]"),
            "Bounds", RectangleValue(Rectangle(0, 200, 0, 200))
        );
        ueMobility.Install(ues.Get(i));
    }

    // Next 5 UEs: Waypoint (car-like movement)
    for (int i = 5; i < 10; i++)
    {
        ueMobility.SetMobilityModel("ns3::WaypointMobilityModel");
        ueMobility.Install(ues.Get(i));

        Ptr<WaypointMobilityModel> wp = ues.Get(i)->GetObject<WaypointMobilityModel>();
        wp->AddWaypoint(Waypoint(Seconds(0), Vector(10, 10, 0)));
        wp->AddWaypoint(Waypoint(Seconds(5), Vector(150, 10, 0)));
        wp->AddWaypoint(Waypoint(Seconds(10), Vector(150, 150, 0)));
        wp->AddWaypoint(Waypoint(Seconds(15), Vector(10, 150, 0)));
    }

    // -------------------------------
    // INSTALL LTE DEVICES
    // -------------------------------
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbs);
    NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ues);

    Ipv4InterfaceContainer ueIps = epcHelper->AssignUeIpv4Addresses(ueDevs);

    for (uint32_t i = 0; i < ues.GetN(); i++)
        lteHelper->Attach(ueDevs.Get(i), enbDevs.Get(i % 4));

    // -------------------------------
    // TRAFFIC APPS
    // -------------------------------
    uint16_t bulkPort = 9000;
    uint16_t webPort = 8000;

    // BulkSend → UEs 0–4 from RemoteHost 0
    for (int i = 0; i < 5; i++)
    {
        BulkSendHelper bulk("ns3::TcpSocketFactory",
            InetSocketAddress(ueIps.GetAddress(i), bulkPort));
        bulk.SetAttribute("MaxBytes", UintegerValue(0));

        ApplicationContainer send = bulk.Install(remoteHosts.Get(0));
        send.Start(Seconds(1.0));
        send.Stop(Seconds(simTime));

        PacketSinkHelper sink("ns3::TcpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), bulkPort));
        sink.Install(ues.Get(i)).Start(Seconds(0.5));
    }

    // OnOff → UEs 5–9 from RemoteHost 1
    for (int i = 5; i < 10; i++)
    {
        OnOffHelper onoff("ns3::UdpSocketFactory",
            InetSocketAddress(ueIps.GetAddress(i), webPort));
        onoff.SetConstantRate(DataRate("5Mbps"));
        onoff.SetAttribute("PacketSize", UintegerValue(1024));

        ApplicationContainer app = onoff.Install(remoteHosts.Get(1));
        app.Start(Seconds(2.0));
        app.Stop(Seconds(simTime));

        PacketSinkHelper sink("ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), webPort));
        sink.Install(ues.Get(i)).Start(Seconds(1.5));
    }

    // -------------------------------
    // RADIO ENVIRONMENT MAP
    // -------------------------------
    Ptr<RadioEnvironmentMapHelper> rem = CreateObject<RadioEnvironmentMapHelper>();
    rem->SetAttribute("OutputFile", StringValue("rem-output.dat"));
    rem->SetAttribute("XMin", DoubleValue(0));
    rem->SetAttribute("XMax", DoubleValue(200));
    rem->SetAttribute("YMin", DoubleValue(0));
    rem->SetAttribute("YMax", DoubleValue(200));
    rem->SetAttribute("XRes", UintegerValue(200));
    rem->SetAttribute("YRes", UintegerValue(200));
    rem->Install();

    // -------------------------------
    // FLOW MONITOR
    // -------------------------------
    FlowMonitorHelper fm;
    Ptr<FlowMonitor> monitor = fm.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->SerializeToXmlFile("flowmon-lte.xml", true, true);
    Simulator::Destroy();

    return 0;
}

