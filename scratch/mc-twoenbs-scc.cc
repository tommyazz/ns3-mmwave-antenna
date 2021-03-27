/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/* *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Tommy Azzino <tommy.azzino@gmail.com>
 */

#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/buildings-module.h>
#include <ns3/random-variable-stream.h>
#include <ns3/lte-ue-net-device.h>
#include "ns3/isotropic-antenna-model.h"

#include <iostream>
#include <ctime>
#include <stdlib.h>
#include <list>

using namespace ns3;
using namespace mmwave;

NS_LOG_COMPONENT_DEFINE ("McTwoEnbsScc");

uint32_t timeRes = 100;
Ptr<OutputStreamWrapper> stream1, stream2, stream3, stream4;

static void
ComputeStatistics (ApplicationContainer sinkApps, std::vector<uint32_t> lastRxBytes, std::vector<Ptr<MobilityModel>> mobilityModels)
{
  uint32_t totRxBytes, rxBytes;
  Ptr<PacketSink> sink;
  Vector pos;
  double now = Simulator::Now ().GetSeconds ();
  std::cout << now << "\t";
  *stream1->GetStream () << now;
  for (uint32_t i = 0; i < sinkApps.GetN (); ++i)
  {
    sink = StaticCast<PacketSink> (sinkApps.Get (i));
    totRxBytes = sink->GetTotalRx ();
    rxBytes = totRxBytes - lastRxBytes.at (i);
    lastRxBytes.at (i) = totRxBytes;
    double thr = rxBytes * 8.0 / (timeRes * 1e-3) / 1e6;  // Throughput in Mbps
    pos = mobilityModels.at (i)->GetPosition ();
    std::cout << thr << "\t";
    *stream1->GetStream () << "\t" << thr << "\t" << pos.x << "\t" << pos.y;
  }  
  std::cout << "Mbps" << std::endl;
  *stream1->GetStream () << std::endl;

  Simulator::Schedule (MilliSeconds (timeRes), &ComputeStatistics, sinkApps, lastRxBytes, mobilityModels);
}

static void
Rx (uint32_t appId, Ptr<const Packet> pkt, const Address &rxAddr, const Address &txAddr, const SeqTsSizeHeader& hdr)
{
  NS_LOG_DEBUG ("Rx packet with size: " << pkt->GetSize () << "; appId: " << appId); 
  Time now = Simulator::Now ();
  Time txTime = hdr.GetTs ();
  double pktDelay = now.GetSeconds () - txTime.GetSeconds ();
  NS_LOG_DEBUG ("Delay for packet with seq=" << hdr.GetSeq () << " is: " << pktDelay*1e3 << " ms");
  *stream2->GetStream () << appId << "\t" << Simulator::Now ().GetSeconds () << "\t" << hdr.GetSeq () << "\t" << pkt->GetSize () << "\t" << pktDelay << std::endl;
}

static void
Tx (uint32_t appId, Ptr<const Packet> pkt, const Address &rxAddr, const Address &txAddr, const SeqTsSizeHeader& hdr)
{
  NS_LOG_DEBUG ("Tx packet with size: " << pkt->GetSize () << "; appId: " << appId);
  *stream4->GetStream () << appId << "\t" << Simulator::Now ().GetSeconds () << "\t" << hdr.GetSeq () << "\t" << pkt->GetSize () << std::endl;
}

static void
Sinr (uint32_t ueId, uint64_t imsi, SpectrumValue& oldSinr, SpectrumValue& newSinr)
{
  double sinr = Sum (newSinr) / (newSinr.GetSpectrumModel ()->GetNumBands ());
  NS_LOG_DEBUG (Simulator::Now ().GetSeconds () << "\t" << 10*log10 (sinr) << " dB");
  *stream3->GetStream () << ueId << "\t" << Simulator::Now ().GetSeconds () << "\t" << sinr << std::endl;
}

int
main (int argc, char *argv[])
{
  bool rlcAmEnabled = true;
  bool harqEnabled = true;
  bool fixedTti = false;
  double bandwidth = 100e6; // bandwidth of the simulation [MHz]
  uint32_t appPacketSize = 1460; // application layer packet size [bytes]
  uint16_t enbAntennaNum = 64; // number of antenna elements at the BS
  uint16_t ueAntennaNum = 16; // number of antenna elements at the UE
  double frequency = 28e9; // operating frequency [Hz]
  double txPow = 30.0; // tx power [dBm]
  double noiseFigure = 9.0; // noise figure [dB]
  uint32_t simTime = 10; // simulation time [s]
  uint32_t updatePeriod = 100; // channel/channel condition update period [ms]
  uint16_t nonSelfBlocking = 4; // number of self-blocking components for the blockage model
  uint32_t remoteHostDelay = 10; // delay from PGW to Remote host [ms]
  uint32_t numUes = 1; // number of UEs in the simulation
  std::string outputFolder = ""; // path to the main output folder for the results
  std::string scenario = "UMi-StreetCanyon"; // 3GPP propagation scenario (Urban-Micro)
  bool isBlockage = false; // enable blockage modeling
  bool isMc = false; // whether to add multi-connectivity with eNodeB (LTE) in the simulation
  uint32_t bufferSize = 20; // buffer size [MByte] 
  uint8_t hoMode = 3; // handover mode
  double outThreshold = -5.0; // outage threshold [dB]
  uint32_t reportTablePeriodicity = 1600; // report table periodicity 
  bool enableLog = false;

  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("rlcAmEnabled", "Enable RLC AM mode at RLC layer", rlcAmEnabled);
  cmd.AddValue ("harqEnabled", "Enable HARQ at the MAC layer", harqEnabled);
  cmd.AddValue ("bandwidth", "The bandwidth of the simulation", bandwidth);
  cmd.AddValue ("updatePeriod", "Channel/channel condition update periodicity [ms]", updatePeriod);
  cmd.AddValue ("blockage", "Enable blockage model A of the 3GPP channel model", isBlockage);
  cmd.AddValue ("nonSelfBlocking", "Number of non self-blocking components", nonSelfBlocking);
  cmd.AddValue ("numUes", "Number of UEs in the simulation", numUes);
  cmd.AddValue ("mc", "Enable multi-connectivity in the simulation", isMc);
  cmd.AddValue ("simTime", "Simulation time [s]", simTime);
  cmd.Parse (argc, argv);

  if (enableLog)
  {
    LogComponentEnable ("MmWaveHelper", LOG_LEVEL_ALL);
  } 

  // Variables for the RT
  int windowForTransient = 150; // number of samples for the vector to use in the filter
  int ReportTablePeriodicity = (int)reportTablePeriodicity; // in microseconds
  if (ReportTablePeriodicity == 1600)
    {
      windowForTransient = 150;
    }
  else if (ReportTablePeriodicity == 25600)
    {
      windowForTransient = 50;
    }
  else if (ReportTablePeriodicity == 12800)
    {
      windowForTransient = 100;
    }
  else
    {
      NS_ASSERT_MSG (false, "Unrecognized");
    }
  int vectorTransient = windowForTransient * ReportTablePeriodicity;
  double transientDuration = double(vectorTransient) / 1000000;

  std::cout << "rlcAmEnabled: " << rlcAmEnabled << std::endl;
  std::cout << "harqEnabled: " << harqEnabled << std::endl;
  std::cout << "multiConnectivity: " << isMc << std::endl;
  std::cout << "updatePeriod: " << updatePeriod << std::endl;
  std::cout << "blockage: " << isBlockage << std::endl;
  std::cout << "nonSelfBlocking: " << nonSelfBlocking << std::endl;
  std::cout << "numUes: " << numUes << std::endl;
  std::cout << "simTime: " << simTime << std::endl;
  std::cout << "transientDuration: " << transientDuration << std::endl;
  std::cout << "Seed: " << ns3::RngSeedManager::GetSeed () << std::endl;
  std::cout << "Run: " << ns3::RngSeedManager::GetRun () << std::endl;
  
  Config::SetDefault ("ns3::MmWaveUeMac::UpdateUeSinrEstimatePeriod", DoubleValue (0));
  // setting power and noise figure
  Config::SetDefault ("ns3::MmWaveEnbPhy::TxPower", DoubleValue (txPow));
  Config::SetDefault ("ns3::MmWaveEnbPhy::NoiseFigure", DoubleValue (noiseFigure));
  Config::SetDefault ("ns3::MmWaveUePhy::TxPower", DoubleValue (txPow));
  Config::SetDefault ("ns3::MmWaveUePhy::NoiseFigure", DoubleValue (noiseFigure));

  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::FixedTti", BooleanValue (fixedTti));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::SymPerSlot", UintegerValue (6));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::TbDecodeLatency", UintegerValue (200.0));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue (100));
  Config::SetDefault ("ns3::LteEnbRrc::SystemInformationPeriodicity", TimeValue (MilliSeconds (5.0)));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
  Config::SetDefault ("ns3::LteEnbRrc::FirstSibTime", UintegerValue (2));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkDelay", TimeValue (MicroSeconds (500)));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkDataRate", DataRateValue (DataRate ("1000Gb/s")));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkMtu",  UintegerValue (10000));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::S1uLinkDelay", TimeValue (MicroSeconds (1000)));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::S1apLinkDelay", TimeValue (MicroSeconds (10000)));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcAm::StatusProhibitTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (frequency));

  // handover and RT related params
  switch (hoMode)
    {
    case 1:
      Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", EnumValue (LteEnbRrc::THRESHOLD));
      break;
    case 2:
      Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", EnumValue (LteEnbRrc::FIXED_TTT));
      break;
    case 3:
      Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", EnumValue (LteEnbRrc::DYNAMIC_TTT));
      break;
    }

  Config::SetDefault ("ns3::LteEnbRrc::FixedTttValue", UintegerValue (150));
  Config::SetDefault ("ns3::LteEnbRrc::CrtPeriod", IntegerValue (ReportTablePeriodicity));
  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (outThreshold));
  Config::SetDefault ("ns3::MmWaveEnbPhy::UpdateSinrEstimatePeriod", IntegerValue (ReportTablePeriodicity));
  Config::SetDefault ("ns3::MmWaveEnbPhy::Transient", IntegerValue (vectorTransient));
  Config::SetDefault ("ns3::MmWaveEnbPhy::NoiseAndFilter", BooleanValue (false));
  Config::SetDefault ("ns3::McUePdcp::LteUplink", BooleanValue (false));

  // settings for the 3GPP the channel
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (updatePeriod)));
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod", TimeValue (MilliSeconds (updatePeriod)));
  Config::SetDefault ("ns3::ThreeGppChannelModel::Blockage", BooleanValue (isBlockage)); // use blockage or not
  Config::SetDefault ("ns3::ThreeGppChannelModel::PortraitMode", BooleanValue (false)); // use blockage model with UT in portrait mode
  Config::SetDefault ("ns3::ThreeGppChannelModel::NumNonselfBlocking", IntegerValue (nonSelfBlocking)); // number of non-self blocking obstacles
  
  // set to false to use the 3GPP radiation pattern (proper configuration of the bearing and downtilt angles is needed) 
  Config::SetDefault ("ns3::PhasedArrayModel::AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ())); 

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();

  // create and configure the factories for the channel condition and propagation loss models
  ObjectFactory propagationLossModelFactory;
  ObjectFactory channelConditionModelFactory;
  if (scenario == "RMa")
  {
    mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppRmaPropagationLossModel");
    mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppRmaChannelConditionModel");
  }
  else if (scenario == "UMa")
  {
    mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmaPropagationLossModel");
    mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppUmaChannelConditionModel");
  }
  else if (scenario == "UMi-StreetCanyon")
  {
    mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
    mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");
  }
  else
  {
    NS_FATAL_ERROR ("Unknown scenario");
  }

  // set the number of antennas in the devices
  mmwaveHelper->SetUePhasedArrayModelAttribute ("NumColumns" , UintegerValue (sqrt(ueAntennaNum)));
  mmwaveHelper->SetUePhasedArrayModelAttribute ("NumRows" , UintegerValue (sqrt(ueAntennaNum)));
  mmwaveHelper->SetEnbPhasedArrayModelAttribute ("NumColumns" , UintegerValue (sqrt(enbAntennaNum)));
  mmwaveHelper->SetEnbPhasedArrayModelAttribute ("NumRows" , UintegerValue (sqrt(enbAntennaNum)));

  Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);
  mmwaveHelper->SetHarqEnabled (harqEnabled);
  mmwaveHelper->Initialize ();

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults ();

  // parse again so you can override default values from the command line
  cmd.Parse (argc, argv);

  // Get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet by connecting remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (remoteHostDelay)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // create LTE, mmWave eNB nodes and UE node
  NodeContainer ueNodes;
  NodeContainer mmWaveEnbNodes;
  NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;
  mmWaveEnbNodes.Create (1);
  allEnbNodes.Add (mmWaveEnbNodes);
  if (isMc)
  {
    lteEnbNodes.Create (1);
    allEnbNodes.Add (lteEnbNodes);
  }
  ueNodes.Create (numUes);

  // install mobility model for the BSs (if eNodeB is present, it has the same location as the gNodeB)
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  enbPositionAlloc->Add (Vector (0, 0, 10));
  enbPositionAlloc->Add (Vector (0, 0, 10));
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (allEnbNodes);

  // UEs mobility
  Ptr<UniformRandomVariable> uniform = CreateObject<UniformRandomVariable> ();
  double x,y;
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
  {
    Ptr<MobilityModel> ueMobility = CreateObject<RandomWalk2dOutdoorMobilityModel> (); // 2D random walk mobility model for each UE
    ueMobility->SetAttribute ("Mode", EnumValue (RandomWalk2dOutdoorMobilityModel::MODE_DISTANCE)); // updating mode for UE direction and speed
    // ueMobility->SetAttribute ("Time", TimeValue (Seconds (10.0)));
    // ueMobility->SetAttribute ("Distance", DoubleValue (10.0)); // update UE direction and speed every VALUE meters walked
    
    x = 200.0 * uniform->GetValue () - 100.0;
    y = 200.0 * uniform->GetValue () - 100.0; 
    ueMobility->SetAttribute ("Bounds", RectangleValue (Rectangle (-100.0, 100.0, -100.0, 100.0))); // intended UEs random walk bounds     
    ueMobility->SetPosition (Vector (x, y, 1.6));
    ueNodes.Get (i)->AggregateObject (ueMobility);
  }

  // Install mmWave, lte, mc Devices to the nodes
  NetDeviceContainer lteEnbDevs, mmWaveEnbDevs, ueDevs;
  if (isMc)
  {
    lteEnbDevs = mmwaveHelper->InstallLteEnbDevice (lteEnbNodes);
    mmWaveEnbDevs = mmwaveHelper->InstallEnbDevice (mmWaveEnbNodes);
    ueDevs = mmwaveHelper->InstallMcUeDevice (ueNodes);
  }
  else
  {
    mmWaveEnbDevs = mmwaveHelper->InstallEnbDevice (mmWaveEnbNodes);
    ueDevs = mmwaveHelper->InstallUeDevice (ueNodes);
  }

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));
  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  if (isMc)
  {
    mmwaveHelper->AddX2Interface (lteEnbNodes, mmWaveEnbNodes);
    mmwaveHelper->AttachToClosestEnb (ueDevs, mmWaveEnbDevs, lteEnbDevs);
  }
  else
  {
    mmwaveHelper->AttachToClosestEnb (ueDevs, mmWaveEnbDevs);
  }

  // Install and start applications on UEs and remote host
  uint16_t ulPort = 2000;
  std::vector<double> appStartTime (ueNodes.GetN ());
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  // for each UE create a CBR application @100 Mbps (uplink traffic from the UE to the remote host) 
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
  {
    ++ulPort;
    // create the sink 
    PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
    ulPacketSinkHelper.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true)); // enable SeqTs header to measure end-to-end delay
    // create the CBR application
    OnOffHelper ulOnOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (remoteHostAddr, ulPort));
    ulOnOffHelper.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true)); // enable SeqTs header to measure end-to-end delay
    ulOnOffHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=100000.0]"));
    ulOnOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
    ulOnOffHelper.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
    ulOnOffHelper.SetAttribute ("PacketSize", UintegerValue (appPacketSize));
    // randomize the starting time 
    appStartTime.at (i) = transientDuration + uniform->GetValue (0.1, 0.2);
    ulOnOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (appStartTime.at (i))));
    
    serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
    clientApps.Add (ulOnOffHelper.Install (ueNodes.Get (i)));
  }

  mmwaveHelper->EnableTraces ();

  AsciiTraceHelper asciiTraceHelper;
  stream1 = asciiTraceHelper.CreateFileStream (outputFolder+"thr-mobility.csv");
  stream2 = asciiTraceHelper.CreateFileStream (outputFolder+"rx-packet-trace.csv");
  stream3 = asciiTraceHelper.CreateFileStream (outputFolder+"sinr-trace.csv");
  stream4 = asciiTraceHelper.CreateFileStream (outputFolder+"tx-packet-trace.csv");

  Ptr<MmWaveUePhy> uePhy;
  for (uint32_t i = 0; i < ueDevs.GetN (); ++i)
  {
    if (isMc)
    {
      uePhy = StaticCast <McUeNetDevice> (ueDevs.Get (i))->GetMmWavePhy ();
    }
    else
    {
      uePhy = StaticCast <MmWaveUeNetDevice> (ueDevs.Get (i))->GetPhy ();
    }
    uePhy->TraceConnectWithoutContext ("ReportCurrentCellRsrpSinr", MakeBoundCallback (&Sinr, i));
  }

  // App layer traces: collect statistics of intended UEs
  for (uint32_t i = 0; i < serverApps.GetN (); ++i)
  {
    serverApps.Get (i)->TraceConnectWithoutContext ("RxWithSeqTsSize", MakeBoundCallback (&Rx, i));
  }

  for (uint32_t i = 0; i < clientApps.GetN (); ++i)
  {
    clientApps.Get (i)->TraceConnectWithoutContext ("TxWithSeqTsSize", MakeBoundCallback (&Tx, i));
  }

  // collect mobility traces of the UEs
  std::vector<Ptr<MobilityModel>> mobilityModels (ueNodes.GetN ());
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
  {
    mobilityModels.at (i) = ueNodes.Get (i)->GetObject<MobilityModel> ();
  }

  double maxAppStartTime = *std::max_element (appStartTime.begin (), appStartTime.end ());
  std::vector<uint32_t> lastRxBytes (serverApps.GetN (), 0);
  Simulator::Schedule (Seconds (maxAppStartTime), &ComputeStatistics, serverApps, lastRxBytes, mobilityModels);
  
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
