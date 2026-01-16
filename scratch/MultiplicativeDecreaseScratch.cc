#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LedbatMdTest");

int
main (int argc, char *argv[])
{
  LogComponentEnable ("LedbatMdTest", LOG_LEVEL_INFO);
  LogComponentEnable ("TcpLedbat", LOG_LEVEL_ALL);

  NS_LOG_INFO ("LEDBAT + Cubic simulation started");

  NodeContainer nodes;
  nodes.Create (2); //Sender and Receiver

  PointToPointHelper p2p; //Setting Channel bandwidth and delay, in a way to increase delay as much as possible
  p2p.SetDeviceAttribute ("DataRate", StringValue ("500Kbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("100ms"));

  NetDeviceContainer devices = p2p.Install (nodes);

  InternetStackHelper internet;
  internet.Install (nodes); // Adding TCP, IP, ... required protocols

  TrafficControlHelper tch;
  tch.SetRootQueueDisc ("ns3::PfifoFastQueueDisc",
                        "MaxSize", StringValue ("10p")); // 10 packets in one queue
  tch.Install (devices);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  TypeId cubicTid  = TypeId::LookupByName ("ns3::TcpCubic");
  TypeId ledbatTid = TypeId::LookupByName ("ns3::TcpLedbat");

  Config::Set ("/NodeList/0/$ns3::TcpL4Protocol/SocketType",
               TypeIdValue (cubicTid)); // Cubic initialisation

  uint16_t cubicPort = 6000;

  BulkSendHelper cubicSender (
      "ns3::TcpSocketFactory",
      InetSocketAddress (interfaces.GetAddress (1), cubicPort));

  cubicSender.SetAttribute ("MaxBytes", UintegerValue (0));

  ApplicationContainer cubicApp = cubicSender.Install (nodes.Get (0));
  cubicApp.Start (Seconds (0.5));
  cubicApp.Stop (Seconds (10.0));

  PacketSinkHelper cubicSink (
      "ns3::TcpSocketFactory",
      InetSocketAddress (Ipv4Address::GetAny (), cubicPort));

  cubicSink.Install (nodes.Get (1));

  Config::Set ("/NodeList/0/$ns3::TcpL4Protocol/SocketType",
               TypeIdValue (ledbatTid)); // LEDBAT initialisation

  uint16_t ledbatPort = 7000;

  BulkSendHelper ledbatSender (
      "ns3::TcpSocketFactory",
      InetSocketAddress (interfaces.GetAddress (1), ledbatPort));

  ledbatSender.SetAttribute ("MaxBytes", UintegerValue (0));

  ApplicationContainer ledbatApp = ledbatSender.Install (nodes.Get (0));
  ledbatApp.Start (Seconds (2.0));   // starts after Cubic fills queue
  ledbatApp.Stop (Seconds (10.0));

  PacketSinkHelper ledbatSink (
      "ns3::TcpSocketFactory",
      InetSocketAddress (Ipv4Address::GetAny (), ledbatPort));

  ledbatSink.Install (nodes.Get (1));

  Simulator::Stop (Seconds (10.0));
  Simulator::Run ();
  Simulator::Destroy ();

  NS_LOG_INFO ("Simulation finished");
  return 0;
}
