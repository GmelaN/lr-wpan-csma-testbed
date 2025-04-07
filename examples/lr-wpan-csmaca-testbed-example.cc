/*
 * Copyright (c) 2025 Gyeongsang National University, Jinju, South Korea.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */

 #include <ns3/constant-position-mobility-model.h>
 #include <ns3/core-module.h>
 #include <ns3/log.h>
 #include <ns3/lr-wpan-csmaca-noba.h>
 #include <ns3/lr-wpan-csmaca-standard.h>
 #include <ns3/lr-wpan-csmaca-sw-noba.h>
 #include <ns3/lr-wpan-csmaca-swpr-noba.h>
 #include <ns3/lr-wpan-delay-tag.h>
 #include <ns3/lr-wpan-module.h>
 #include <ns3/lr-wpan-priority-tag.h>
 #include <ns3/lr-wpan-retransmission-tag.h>
 #include <ns3/multi-model-spectrum-channel.h>
 #include <ns3/packet.h>
 #include <ns3/propagation-delay-model.h>
 #include <ns3/propagation-loss-model.h>
 #include <ns3/simulator.h>
 #include <ns3/single-model-spectrum-channel.h>
 
 #include <fstream>
 #include <iostream>
 
 #include <numeric>
 
 // #include "configuration.h"
 
 #define CSMA_CA_BEB 0
 #define CSMA_CA_NOBA 1
 #define CSMA_CA_SW_NOBA 2
 #define CSMA_CA_STANDARD 3
 #define CSMA_CA_SWPR_NOBA 4
 
 #define CSMA_CA CSMA_CA_SW_NOBA
 
 #define NODE_COUNT TP_COUNT * 10 + 1 //  8 priorities, 10 nodes per priority, EACH PAN, INCLUDING COORDINATOR
 #define PACKET_SIZE 20
 #define PAN_ID 5
 #define COORD_ADDR 1
 
 #define SIM_TIME 10
 
 using namespace ns3;
 using namespace ns3::lrwpan;
 
 static uint32_t requestTX[TP_COUNT];
 static uint32_t sentTX[TP_COUNT];
 static uint32_t successTX[TP_COUNT];
 static uint32_t collisions[TP_COUNT];
 
 static uint32_t failTX[TP_COUNT];
 static uint32_t successRX[TP_COUNT];
 static uint32_t failRX[TP_COUNT];
 
 // static uint32_t txEnqueue[TP_COUNT];
 static uint32_t txDequeue[TP_COUNT];
 
 static std::vector<uint32_t> retransmissionCount[NODE_COUNT + 1];
 
 static std::vector<double> rxDelay[TP_COUNT];
 
 
 void
 progress()
 {
     static int counter = 0;
     NS_LOG_UNCOND(counter << "%");
     counter++;
     Simulator::Schedule(
         Seconds(SIM_TIME / 100.0),
         progress
     );
 }
 
 
 void
 SetupLogComponents()
 {
     LogComponentEnableAll(LOG_PREFIX_TIME);
     LogComponentEnableAll(LOG_PREFIX_FUNC);
     LogComponentEnable("LrWpanMac", LOG_ALL);
     // LogComponentEnable("LrWpanPhy", LOG_LEVEL_DEBUG);
     // LogComponentEnable("LrWpanCsmaCaSwNoba", LOG_LOGIC);
     // LogComponentEnable("LrWpanCsmaCaSwprNoba", LOG_LOGIC);
     // LogComponentEnable("LrWpanCsmaCaNoba", LOG_LEVEL_DEBUG);
     // LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_DEBUG);
 }
 
 
 void
 BeaconIndication(MlmeBeaconNotifyIndicationParams params)
 {
     // NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " secs | Received BEACON packet of size ");
 }
 
 void
 DataIndication(McpsDataIndicationParams params, Ptr<Packet> p)
 {
     // NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                   // << " secs | Received DATA packet of size " << p->GetSize());
 }
 
 void
 TransEndIndication(McpsDataConfirmParams params)
 {
     // In the case of transmissions with the Ack flag activated, the transaction is only
     // successful if the Ack was received.
     // if (params.m_status == MacStatus::SUCCESS)
      // {
          // NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " secs | Transmission successfully sent");
      // }
      // if (params.m_status == MacStatus::NO_ACK)
      // {
         // NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " secs | TRANSMISSION ENDED WITH NO ACK");
      // }
 }
  
 void
 DataIndicationCoordinator(McpsDataIndicationParams params, Ptr<Packet> p)
  {
      // NS_LOG_UNCOND(Simulator::Now().GetSeconds()
      //               << "s Coordinator Received DATA packet (size " << p->GetSize() << " bytes)");
  }
  
 void
 StartConfirm(MlmeStartConfirmParams params)
  {
     // if (params.m_status == MacStatus::SUCCESS)
     // {
     //     NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "Beacon status SUCCESSFUL");
     // }
 }
 
 void
 CsmaCaCollision(uint8_t TP, uint32_t collision)
 {
     // NS_LOG_UNCOND("\t\tPRIORITY " << (uint32_t) TP << " GOT COLLISION: " << collisions[TP]);
     collisions[TP]++;
 }
 
 void
 PhyRxDrop(Ptr<const Packet> pkt, uint8_t TP) // PhyRxDrop: fail RX
 {
     // count: RX new packet if PHY in RX state(packet collision)
     failRX[TP]++;
 }
 
 void
 MacRx(Ptr<const Packet> p, uint8_t TP) // MacRx: success RX
 {
     successRX[TP]++;
 
     LrWpanDelayTag tag1;
     p->PeekPacketTag(tag1);
 
     LrWpanPriorityTag tag2;
     p->PeekPacketTag(tag2);
 
 
     double issuedTime = tag1.Get();
     double current = Simulator::Now().GetMilliSeconds();
 
     uint8_t senderTP = tag2.Get();
     // std::cout << "issuedTime: " << (double) issuedTime << std::endl;
     // std::cout << "current: " << (double) current << std::endl;
 
     double delay = current - issuedTime;
     // std::cout << "DELAY: " << (double) delay << std::endl;
 
     rxDelay[senderTP].push_back(delay);
 }
 
 void
 MacTxReq(Ptr<const Packet> pkt, uint8_t TP) // MacTx: total requested TX
 {
     requestTX[TP]++;
 }
 
 void
 MacTxSent(Ptr<const Packet> pkt, uint8_t TP)
 {
     sentTX[TP]++;
 }
 
 void
 MacTxOk(Ptr<const Packet> pkt, uint8_t TP) // MacTxOk: success TX, received ACK
 {
     LrWpanRetransmissionTag txTag;
     pkt->PeekPacketTag(txTag);
 
     uint32_t reTxCount = txTag.Get();
 
     LrWpanMacHeader header;
     pkt->PeekHeader(header);
     retransmissionCount[header.GetShortSrcAddr().ConvertToInt()].push_back(reTxCount);
     successTX[TP]++;
 }
 
 void
 MacTxDrop(Ptr<const Packet> pkt, uint8_t TP) // MacTxDrop: fail TX
 {
     failTX[TP]++;
 }
 
 // void
 // TxEnqueue(Ptr<const Packet> pkt, uint8_t TP)
 // {
 //     txEnqueue[TP]++;
 // }
 
 void
 TxDequeue(Ptr<const Packet> p, uint8_t TP)
 {
     txDequeue[TP]++;
 }
 
 void
 GenerateTraffic(NetDeviceContainer devices, double interval)
 {
     static uint8_t msduHandle = 0;
 
     for(auto i = devices.Begin(); i < devices.End(); i++)
     {
         if(i == devices.Begin())
         {
             continue; // first one is coordinator
         }
  
         Ptr<LrWpanNetDevice> dev = DynamicCast<LrWpanNetDevice>(*i);
         Ptr<Packet> p = Create<Packet>(PACKET_SIZE);
 
         LrWpanDelayTag tag1;
         tag1.Set(Simulator::Now().GetMilliSeconds());
         p->AddPacketTag(tag1);
 
         LrWpanPriorityTag tag2;
         tag2.Set(dev->GetMac()->GetPriority());
         p->AddPacketTag(tag2);
 
 
         McpsDataRequestParams params2;
         params2.m_dstPanId = PAN_ID;
         params2.m_srcAddrMode = SHORT_ADDR;
         params2.m_dstAddrMode = SHORT_ADDR;
         params2.m_dstAddr = Mac16Address(COORD_ADDR);
         msduHandle = (msduHandle + 1) % (UINT8_MAX + 1);
         params2.m_msduHandle = msduHandle;
         params2.m_txOptions = TX_OPTION_ACK;  // Enable direct transmission with Ack
  
         Simulator::ScheduleNow(
             &LrWpanMac::McpsDataRequest,
             dev->GetMac(),
             params2,
             p
         );
     }
  
     Simulator::Schedule(
         Seconds(interval),
         GenerateTraffic,
         devices,
         interval
     );
 }
  
 int
 main(int argc, char* argv[])
 {
     ns3::RngSeedManager::SetSeed(42);
 
     SetupLogComponents();
 
     // LrWpanHelper lrWpanHelper;
     NodeContainer nodes;
 
     ////////////////////////////// 1. SETUP HELPER //////////////////////////////
     Ptr<SingleModelSpectrumChannel> channel = Create<SingleModelSpectrumChannel>();
     Ptr<LogDistancePropagationLossModel> lossModel = Create<LogDistancePropagationLossModel>();
 
     Ptr<ConstantSpeedPropagationDelayModel> delayModel =
         Create<ConstantSpeedPropagationDelayModel>();
 
     channel->SetPropagationDelayModel(delayModel);
     channel->AddPropagationLossModel(lossModel);
 
     NetDeviceContainer devices;
 
 
     nodes.Create(NODE_COUNT); // first one is coordinator
     uint16_t coordAddr = COORD_ADDR;
 
 
     uint8_t priority = 7;
     uint16_t address = coordAddr + 1;
     // Vector center(0, 0, 0);
     // double radius = 5;
     for (int i = 0; i < NODE_COUNT; i++)
     {
         //////////////////// MAKE NEW NETDEVICE ////////////////////
         Ptr<LrWpanNetDevice> dev = CreateObject<LrWpanNetDevice>();
 
         //////////////////// SETUP MOBILITY ////////////////////
         // Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
         //
         // if (i == 0) // coordinator
         // {
         //     mob->SetPosition(center);
         // } else {
         //     // 원형으로 배치
         //     int angle = (int) 2.0 * M_PI * (i - 1) / (NODE_COUNT - 1); // 각도 계산
         //     int x = (int) center.x + radius * cos(angle);
         //     int y = (int) center.y + radius * sin(angle);
         //
         //     std::cout << "NODE " << i << " POSITION: " << x << ", " << y << std::endl;
         //     mob->SetPosition(Vector(x, y, 0.0));
         // }
         //
         // dev->GetPhy()->SetMobility(mob);
 
         dev->SetChannel(channel);
         dev->SetNode(nodes.Get(i));
 
         nodes.Get(i)->AddDevice(dev);
 
         ////////// SET ADDRESS AND ASSOCIATED COORDINATOR //////////
         ///// this setting must prior CSMA/CA
         dev->GetMac()->SetPanId(PAN_ID);
         dev->GetMac()->SetAssociatedCoor(Mac16Address(COORD_ADDR));
         if (i == 0) // coordinator
         {
             dev->SetAddress(Mac16Address(COORD_ADDR));
             dev->GetMac()->setPriority(7);
         }
         else
         {
             dev->SetAddress(Mac16Address(address++));
             dev->GetMac()->setPriority(priority % 8);
         }
 
         //////////////////// SETUP CSMA/CA ////////////////////
         Ptr<LrWpanCsmaCaCommon> csma;
         #if CSMA_CA == CSMA_CA_BEB
         csma = CreateObject<LrWpanCsmaCa>(priority % 8);
         dev->GetMac()->SetCsmaCaOption(CSMA_ORIGINAL);
         #elif CSMA_CA == CSMA_CA_NOBA
         csma = CreateObject<LrWpanCsmaCaNoba>(priority % 8);
         dev->GetMac()->SetCsmaCaOption(CSMA_NOBA);
         #elif CSMA_CA == CSMA_CA_SW_NOBA
         csma = CreateObject<LrWpanCsmaCaSwNoba>(priority % 8);
         dev->GetMac()->SetCsmaCaOption(CSMA_SW_NOBA);
         #elif CSMA_CA == CSMA_CA_STANDARD
         csma = CreateObject<LrWpanCsmaCaStandard>(priority % 8);
         dev->GetMac()->SetCsmaCaOption(CSMA_STANDARD);
         #elif CSMA_CA == CSMA_CA_SWPR_NOBA
         csma = CreateObject<LrWpanCsmaCaSwprNoba>(priority % 8);
         dev->GetMac()->SetCsmaCaOption(CSMA_SWPR_NOBA);
         #else
         NS_ASSERT_MSG(false, "UNKNOWN CSMA CA");
         #endif
 
         if (i != 0)
         {
             // node
             dev->SetCsmaCa(csma);
         }
         else
         {
             // coordinator
             Ptr<LrWpanCsmaCa> csmaa = CreateObject<LrWpanCsmaCa>(7);
             dev->SetCsmaCa(csmaa);
         }
 
         //////////////////// SET CALLBACKS ////////////////////
         MlmeStartConfirmCallback cb0;
         cb0 = MakeCallback(&StartConfirm);
         dev->GetMac()->SetMlmeStartConfirmCallback(cb0);
 
         McpsDataConfirmCallback cb1;
         cb1 = MakeCallback(&TransEndIndication);
         dev->GetMac()->SetMcpsDataConfirmCallback(cb1);
 
         MlmeBeaconNotifyIndicationCallback cb3;
         cb3 = MakeCallback(&BeaconIndication);
         dev->GetMac()->SetMlmeBeaconNotifyIndicationCallback(cb3);
 
         if (i == 0) // coordinator
         {
             McpsDataIndicationCallback cb5;
             cb5 = MakeCallback(&DataIndicationCoordinator);
             dev->GetMac()->SetMcpsDataIndicationCallback(cb5);
 
             MlmeStartRequestParams params;
             params.m_panCoor = true;
             params.m_PanId = PAN_ID;
             params.m_bcnOrd = 5;
             params.m_sfrmOrd = 5;
             Simulator::ScheduleWithContext(1,
                                            Seconds(0.01),
                                            &LrWpanMac::MlmeStartRequest,
                                            dev->GetMac(),
                                            params);
         }
         else
         {
             McpsDataIndicationCallback cb4;
             cb4 = MakeCallback(&DataIndication);
             dev->GetMac()->SetMcpsDataIndicationCallback(cb4);
         }
         dev->GetPhy()->TraceConnectWithoutContext("PhyRxDrop",
                                                   MakeCallback(&PhyRxDrop)); // dropped RX
         dev->GetMac()->TraceConnectWithoutContext("MacRx",
                                                   MakeCallback(&MacRx)); // received RX(DATA ONLY)
         dev->GetMac()->TraceConnectWithoutContext("MacTxEnqueue",
                                                   MakeCallback(&MacTxReq)); // requested TX
         dev->GetMac()->TraceConnectWithoutContext("MacTxOk",
                                                   MakeCallback(&MacTxOk)); // sent TX & received ACK
         dev->GetMac()->TraceConnectWithoutContext("MacTx", MakeCallback(&MacTxSent)); // sent TX
         dev->GetMac()->TraceConnectWithoutContext(
             "MacTxDrop",
             MakeCallback(&MacTxDrop)); // dropped TX (ACK timeout | channel access failure)
         dev->GetMac()->TraceConnectWithoutContext(
             "MacTxDequeue", MakeCallback(&TxDequeue)
             );
 
         if (i == 0)
         {
             DynamicCast<LrWpanCsmaCa>(dev->GetCsmaCa())
                 ->TraceConnectWithoutContext("csmaCaCollisionTrace",
                                              MakeCallback(&CsmaCaCollision));
         }
         else
         {
             #if CSMA_CA == CSMA_CA_BEB
             DynamicCast<LrWpanCsmaCa>(dev->GetCsmaCa())
                 ->TraceConnectWithoutContext("csmaCaBebCollisionTrace",
                                              MakeCallback(&CsmaCaCollision));
             #elif CSMA_CA == CSMA_CA_NOBA
             DynamicCast<LrWpanCsmaCaNoba>(dev->GetCsmaCa())
                 ->TraceConnectWithoutContext("csmaCaNobaCollisionTrace",
                                              MakeCallback(&CsmaCaCollision));
             #elif CSMA_CA == CSMA_CA_SW_NOBA
             DynamicCast<LrWpanCsmaCaSwNoba>(dev->GetCsmaCa())
                 ->TraceConnectWithoutContext("csmaCaSwNobaCollisionTrace",
                                              MakeCallback(&CsmaCaCollision));
             #elif CSMA_CA == CSMA_CA_STANDARD
             DynamicCast<LrWpanCsmaCaStandard>(dev->GetCsmaCa())
                 ->TraceConnectWithoutContext("csmaCaStandardCollisionTrace",
                                              MakeCallback(&CsmaCaCollision));
             #elif CSMA_CA == CSMA_CA_SWPR_NOBA
             DynamicCast<LrWpanCsmaCaSwprNoba>(dev->GetCsmaCa())
                 ->TraceConnectWithoutContext("csmaCaSwprNobaCollisionTrace",
                                             MakeCallback(&CsmaCaCollision));
             #else
             NS_ASSERT_MSG(false, "Unknown CSMA CA");
             #endif
         }
         // ADD DEVICE TO CONTAINER
         devices.Add(dev);
         priority++;
     }
 
     ////////////////////////////// 5. DATA TRANSMISSION //////////////////////////////
     Simulator::Schedule(Seconds(1.0), &GenerateTraffic, devices, 1);
 
     Simulator::Schedule(
         Seconds(SIM_TIME - 0.00001),
         MakeEvent(
             [nodes] () mutable -> void
             {
                 std::cout << std::endl << std::endl << std::endl;
 
                 std::string str;
                 switch (CSMA_CA)
                 {
                     case CSMA_CA_STANDARD:
                         str = "CSMA/CA STANDARD";
                         break;
                     case CSMA_CA_BEB:
                         str = "CSMA/CA BEB";
                         break;
                     case CSMA_CA_NOBA:
                         str = "CSMA/CA NOBA";
                         break;
                     case CSMA_CA_SW_NOBA:
                         str = "CSMA/CA SW-NOBA";
                         break;
                     case CSMA_CA_SWPR_NOBA:
                         str = "CSMA/CA SWPR-NOBA";
                         break;
                     default:
                         str = "UNKNOWN";
                         break;
                 }
                 std::cout << "SIMULATION DURATION: " << SIM_TIME << std::endl;
                 std::cout << "CSMA/CA CONFIGURATION: " << str << std::endl;
                 std::cout << "NODE COUNT: " << NODE_COUNT << std::endl;
                 std::cout << "PACKET SIZE: " << PACKET_SIZE << std::endl;
 
                 for (int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << "TP" << i << "\t\t";
                 }
                 std::cout << std::endl << "REQ TX(ENQUEUED)\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << requestTX[i] << "\t";
                 }
                 std::cout << "\nSUCCESS TX\t\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << successTX[i] << "\t";
                 }
                 std::cout << "\nFAILED TX\t\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << failTX[i] << "\t\t";
                 }
                 std::cout << "\nSUCCESS RX\t\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << successRX[i] << "\t\t";
                 }
                 std::cout << "\nFAILED RX\t\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << failRX[i] << "\t";
                 }
                 std::cout << "\nCSMA/CA COLLISIONS\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << collisions[i] << "\t\t";
                 }
                 std::cout << "\nMAX DELAYS\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << *std::max_element(rxDelay[i].begin(), rxDelay[i].end()) << "\t\t";
                 }
                 std::cout << "\nAVG DELAYS\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout
                         << std::accumulate(rxDelay[i].begin(), rxDelay[i].end(), 0) / rxDelay[i].size()
                         << "\t\t";
                 }
                 std::cout << "\nMAC TX COUNTS\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << sentTX[i] << "\t\t";
                 }
                 std::cout << "\nMAC TX DEQUEUE COUNTS\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << txDequeue[i] << "\t\t";
                 }
                 std::cout << "\nMAX RETX COUNTS\t";
                 std::vector<uint32_t> maxRetxPerTp(TP_COUNT, 0);
                 for(int i = 0; i < NODE_COUNT; i++)
                 {
                     if (retransmissionCount[i].empty())
                     {
                         continue;
                     }
                     uint8_t tp = DynamicCast<LrWpanNetDevice>(nodes.Get(i)->GetDevice(0))->GetMac()->GetPriority();
                     uint32_t maxReTx = *std::max_element(retransmissionCount[i].begin(), retransmissionCount[i].end());
                     if (maxReTx > maxRetxPerTp[tp])
                     {
                         maxRetxPerTp[tp] = maxReTx;
                     }
                 }
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << maxRetxPerTp[i] << "\t\t";
                 }
                 std::cout << std::endl;
 
                 std::ofstream out("result.csv");
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     out << "TP" << i << ", ";
                     for (auto k = rxDelay[i].begin(); k != rxDelay[i].end(); k++)
                     {
                         out << static_cast<double>(*k) << ", ";
                     }
                     out << std::endl;
                 }
 
                 std::ofstream out2("result_retx.csv");
                 for(int i = 0; i < NODE_COUNT; i++)
                 {
                     uint8_t tp = DynamicCast<LrWpanNetDevice>(nodes.Get(i)->GetDevice(0))->GetMac()->GetPriority();
                     out2 << "NODE " << i << " (TP "  << (int) tp << "), ";
                     for (auto k = retransmissionCount[i].begin(); k != retransmissionCount[i].end(); k++)
                     {
                         out2 << *k << ", ";
                     }
                     out2 << std::endl;
                 }
             }
         )
     );
 
 
     Simulator::Schedule(
         Seconds(0.1),
         progress
     );
 
     ////////////////////////////// PRINT NODE INFORMATION //////////////////////////////
     std::cout << "\n\n==========NODE INFORMATION==========\n\n";
     for (auto d = devices.Begin(); d != devices.End(); d++)
     {
         Ptr<LrWpanNetDevice> dev = DynamicCast<LrWpanNetDevice>(*d);
 
         uint16_t panId = dev->GetMac()->GetPanId();
         Mac16Address addr = dev->GetMac()->GetShortAddress();
         priority = dev->GetMac()->GetPriority();
 
         std::cout
         << "PAN ID:\t" << panId
         << "\nADDRESS:\t" << addr
         << "\nPRIORITY:\t" << (uint32_t) priority
         << std::endl;
     }
     std::cout << "\n\n==================================\n\n";
 
 
     Simulator::Stop(Seconds(SIM_TIME));
     Simulator::Run();
 
     Simulator::Destroy();
     return 0;
 }
  