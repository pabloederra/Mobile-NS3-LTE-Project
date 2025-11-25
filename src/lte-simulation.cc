/* lte-simulation-ns339.cc
 *
 * Fully compatible with ns-3.39
 *
 * Requirements implemented:
 *  - 4 eNBs
 *  - 10 UEs
 *  - 2 remote hosts:
 *      - remoteHosts.Get(0) -> generates BulkSend/TCP traffic to a subset of UEs
 *      - remoteHosts.Get(1) -> generates OnOff/UDP ("web-like") traffic to another subset of UEs
 *  - EPC + PGW configured correctly
 *  - correct IPv4 addressing for remote-host links (avoids collisions with EPC 7.0.0.0/8)
 *  - UE IPs assigned using the PointToPointEpcHelper API for ns-3.39
 *  - default routes on UEs and static routes on remote hosts so traffic flows via PGW
 *  - FlowMonitor enabled and result serialized to flowmon-lte.xml
 *
 * Save into: ns-3.39/scratch/lte-simulation-ns339.cc
 * Build: from ns-3.39 directory run: cmake .. ; make -j4  (or use the existing build configuration)
 * Run:  from ns-3.39/build (or ns-3.39 root depending your setup) ./ns3 run "scratch/lte-simulation-ns339"
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteProjectSimulation_ns339");

int
main(int argc, char *argv[])
{
    double simTime = 20.0; // seconds
    CommandLine cmd;
    cmd.Parse(argc, argv);

    // Create LTE + EPC helpers
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    // PGW node (EPC)
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Install internet stack on PGW (so it can route) and on remote hosts / UEs later
    InternetStackHelper internet;
    internet.Install(pgw); // PGW must have IP stack

    // ----------------------------
    // Remote hosts (2): one for BulkSend, one for OnOff (web-like)
    // ----------------------------
    NodeContainer remoteHosts;
    remoteHosts.Create(2);
    internet.Install(remoteHosts);

    // Connect remote hosts to PGW via point-to-point links using distinct subnets
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));

    // We'll store PGW side addresses to configure routes on remote hosts later
    std::vector<Ipv4Address> pgwAddressesForRemote;
    Ipv4AddressHelper remoteAddrHelper;

    for (uint32_t i = 0; i < remoteHosts.GetN(); ++i)
    {
        NetDeviceContainer link = p2p.Install(pgw, remoteHosts.Get(i));
        // Use 10.(i+1).0.0/16 for remote links to avoid colliding with EPC 7.0.0.0/8
        std::ostringstream base;
        base << "10." << (i + 1) << ".0.0";
        remoteAddrHelper.SetBase(base.str().c_str(), "255.255.0.0");
        Ipv4InterfaceContainer ifc = remoteAddrHelper.Assign(link);
        // ifc.GetAddress(0) -> PGW side, ifc.GetAddress(1) -> remote host side
        pgwAddressesForRemote.push_back(ifc.GetAddress(0));
    }

    // ----------------------------
    // eNBs and UEs
    // ----------------------------
    NodeContainer enbs;
    enbs.Create(4);

    NodeContainer ues;
    ues.Create(10);

    // Install internet stack on UEs (they will get IPs from EPC)
    internet.Install(ues);

    // Mobility for eNBs (static positions)
    MobilityHelper enbMobility;
    Ptr<ListPositionAllocator> enbPos = CreateObject<ListPositionAllocator>();
    enbPos->Add(Vector(0.0, 0.0, 20.0));
    enbPos->Add(Vector(200.0, 0.0, 20.0));
    enbPos->Add(Vector(0.0, 200.0, 20.0));
    enbPos->Add(Vector(200.0, 200.0, 20.0));
    enbMobility.SetPositionAllocator(enbPos);
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.Install(enbs);

    // Mobility for UEs:
    // First 5: RandomWalk2dMobilityModel within 0..200 rectangle
    MobilityHelper ueMobility;
    for (int i = 0; i < 5; ++i)
    {
        MobilityHelper uh;
        uh.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=200]"),
                                "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=200]"));
        uh.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                            "Speed", StringValue("ns3::ConstantRandomVariable[Constant=5]"),
                            "Bounds", RectangleValue(Rectangle(0, 200, 0, 200)));
        uh.Install(ues.Get(i));
    }
    // Next 5: Waypoint mobility (car-like waypoints)
    for (int i = 5; i < 10; ++i)
    {
        MobilityHelper wh;
        wh.SetMobilityModel("ns3::WaypointMobilityModel");
        wh.Install(ues.Get(i));
        Ptr<WaypointMobilityModel> wp = ues.Get(i)->GetObject<WaypointMobilityModel>();
        wp->AddWaypoint(Waypoint(Seconds(0.0), Vector(10, 10, 0)));
        wp->AddWaypoint(Waypoint(Seconds(5.0), Vector(150, 10, 0)));
        wp->AddWaypoint(Waypoint(Seconds(10.0), Vector(150, 150, 0)));
        wp->AddWaypoint(Waypoint(Seconds(15.0), Vector(10, 150, 0)));
    }

    // ----------------------------
    // Install LTE devices and assign UE IP addresses via EPC helper (ns-3.39)
    // ----------------------------
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbs);
    NetDeviceContainer ueDevs  = lteHelper->InstallUeDevice(ues);

    // In ns-3.39 use AssignUeIpv4Address (singular) with a NetDeviceContainer
    Ipv4InterfaceContainer ueIfaces = epcHelper->AssignUeIpv4Address(ueDevs);

    // Attach UEs to eNBs (round-robin)
    for (uint32_t i = 0; i < ues.GetN(); ++i)
    {
        lteHelper->Attach(ueDevs.Get(i), enbDevs.Get(i % enbs.GetN()));
    }

    // ----------------------------
    // Configure routing:
    //  - set default route on each UE to EPC/PGW gateway
    //  - add static routes on remote hosts to reach UE network via PGW link address
    // ----------------------------
    Ipv4StaticRoutingHelper staticRoutingHelper;

    // UE default route -> EPC gateway
    Ipv4Address ueGateway = epcHelper->GetUeDefaultGatewayAddress();
    for (uint32_t i = 0; i < ues.GetN(); ++i)
    {
        Ptr<Node> ueNode = ues.Get(i);
        Ptr<Ipv4> ipv4 = ueNode->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> ueStatic = staticRoutingHelper.GetStaticRouting(ipv4);
        ueStatic->SetDefaultRoute(ueGateway, 1);
    }

    // Remote hosts: add route to UE network (7.0.0.0/8) via PGW side address of their link
    Ipv4Address ueNetwork("7.0.0.0");
    Ipv4Mask ueMask("255.0.0.0");

    for (uint32_t i = 0; i < remoteHosts.GetN(); ++i)
    {
        Ptr<Ipv4> ipv4 = remoteHosts.Get(i)->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> rhStatic = staticRoutingHelper.GetStaticRouting(ipv4);

        rhStatic->AddNetworkRouteTo(
            ueNetwork,
            ueMask,
            pgwAddressesForRemote[i],
            1   // ✅ outgoing interface toward PGW
        );
    }


    // ----------------------------
    // Applications: BulkSend (TCP) and OnOff (UDP)
    // - BulkSend from remoteHosts[0] to UEs 0..4
    // - OnOff (web-like) from remoteHosts[1] to UEs 5..9
    // ----------------------------
    uint16_t bulkPort = 9000;
    uint16_t webPort  = 8000;

    // Install sinks on UEs first, then senders on remote hosts

    // BulkSend sinks on UEs 0..4
    for (int i = 0; i < 5; ++i)
    {
        PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), bulkPort));
        ApplicationContainer sinks = sinkHelper.Install(ues.Get(i));
        sinks.Start(Seconds(0.5));

        // BulkSend from remoteHosts[0] to this UE
        BulkSendHelper bulk("ns3::TcpSocketFactory",
                            InetSocketAddress(ueIfaces.GetAddress(i), bulkPort));
        bulk.SetAttribute("MaxBytes", UintegerValue(0)); // unlimited
        ApplicationContainer apps = bulk.Install(remoteHosts.Get(0));
        apps.Start(Seconds(1.0));
        apps.Stop(Seconds(simTime));
    }

    // OnOff (UDP) sinks on UEs 5..9 and OnOff apps on remoteHosts[1]
    for (int i = 5; i < 10; ++i)
    {
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), webPort));
        ApplicationContainer sinks = sinkHelper.Install(ues.Get(i));
        sinks.Start(Seconds(1.5));

        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(ueIfaces.GetAddress(i), webPort));
        // make it continuously ON for web-like traffic
        onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        onoff.SetConstantRate(DataRate("5Mbps"));
        onoff.SetAttribute("PacketSize", UintegerValue(1024));
        ApplicationContainer apps = onoff.Install(remoteHosts.Get(1));
        apps.Start(Seconds(2.0));
        apps.Stop(Seconds(simTime));
    }

    // ----------------------------
    // Flow monitor to collect metrics
    // ----------------------------
    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> flowmon = fmHelper.InstallAll();

    // Run simulation
    Simulator::Stop(Seconds(simTime));
    // ----------------------------
    // Flow monitor to collect metrics
    // ----------------------------
    FlowMonitorHelper fm;
    Ptr<FlowMonitor> monitor = fm.InstallAll();

    // Run simulation
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    monitor->SerializeToXmlFile("flowmon-lte.xml", true, true);

    Simulator::Destroy();
    return 0;
}

