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
 #include <ns3/lr-wpan-csmaca-gnu-noba.h>
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


 #define CSMA_CA_BEB 0
 #define CSMA_CA_NOBA 1
 #define CSMA_CA_SW_NOBA 2
 #define CSMA_CA_STANDARD 3
 #define CSMA_CA_GNU_NOBA 4
 #define PAN_ID 5
 #define COORD_ADDR 1

int CSMA_CA = CSMA_CA_STANDARD;
int PACKET_SIZE = 50;
int MAX_RETX = 0;
int SIM_TIME = 3600 + 30;
int ncount = 9;

int BEACON_ORDER = 4;

 using namespace ns3;
 using namespace ns3::lrwpan;

#define MAX_NODE_COUNT_PER_TP 2
static std::vector<int> NODE_COUNT_PER_TP;

static uint32_t NODE_COUNT;

 static uint32_t requestTX[TP_COUNT];
 static uint32_t sentTX[TP_COUNT];
 static uint32_t successTX[TP_COUNT];
 static uint32_t collisions[TP_COUNT];

 static uint32_t failTX[TP_COUNT];
 static uint32_t successRX[TP_COUNT];
 static uint32_t failRX[TP_COUNT];

 static uint32_t dynamicFailure[TP_COUNT];

 // static uint32_t txEnqueue[TP_COUNT];
 static uint32_t txDequeue[TP_COUNT];

 static std::vector<std::vector<uint32_t>> retransmissionCount;

 static std::vector<double> rxDelay[TP_COUNT];

static NodeContainer nodes;
static NetDeviceContainer devices;

 void
 progress()
 {
     static int counter = 0;
     NS_LOG_UNCOND(counter << "%");
     counter++;
     Simulator::Schedule(
         Seconds((SIM_TIME + 30) / 100.0),
         progress
     );
 }

std::vector<int> fillArray(int n)
{
     std::vector<int> arr(8, 0); // 초기 배열: [0, 0, 0, 0, 0, 0, 0, 0]
     // int value = 1;         // 넣을 값 (1부터 시작)
     // int remaining = n;
     //
     // while (remaining > 0) {
     //     for (int i = 7; i >= 0 && remaining > 0; --i) {
     //         if (arr[i] == value - 1) {
     //             arr[i] = value;
     //             remaining--;
     //         }
     //     }
     //     value++; // 한 사이클 끝나면 다음 값으로 증가
     // }
     arr[3] = 5;
     arr[6] = 2;
     arr[7] = 2;

     return arr;
 }

 void
 SetupLogComponents()
 {
     LogComponentEnableAll(LOG_PREFIX_TIME);
     LogComponentEnableAll(LOG_PREFIX_FUNC);
     // LogComponentEnable("LrWpanMac", LOG_ALL);
     // LogComponentEnable("LrWpanPhy", LOG_ALL);
     // LogComponentEnable("LrWpanCsmaCaSwNoba", LOG_ALL);
     // LogComponentEnable("LrWpanCsmaCaGnuNoba", LOG_ALL);
     // LogComponentEnable("LrWpanCsmaCaNoba", LOG_ALL);
     // LogComponentEnable("LrWpanCsmaCa", LOG_ALL);
     // LogComponentEnable("MultiModelSpectrumChannel", LOG_ALL);
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
 DynamicFailure(uint8_t TP)
 {
     dynamicFailure[TP]++;
 }

 void
 MacTxReq(Ptr<const Packet> pkt, uint8_t TP) // MacTx: total requested TX
 {
     // std::cout << Simulator::Now().As(Time::MS) << "\tTP: " << (int) TP << " REQUESTED PACKET." << std::endl;
     requestTX[TP]++;
 }

 void
 MacTxSent(Ptr<const Packet> pkt, uint8_t TP)
 {
     // std::cout << Simulator::Now().As(Time::MS) << "\tTP: " << (int) TP << " SENT PACKET." << std::endl;
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

 void
 TxDequeue(Ptr<const Packet> p, uint8_t TP)
 {
     txDequeue[TP]++;
 }

 void
 GenerateTraffic(SequenceNumber8 macBsn)
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
            // &LrWpanMac::McpsDataRequest,
            &LrWpanMacBase::McpsDataRequest,
            dev->GetMac(),
            params2,
            p
        );
     }
 }

 int
 main(int argc, char* argv[])
 {
     CommandLine cmd;

     cmd.AddValue("csmaCa", "CSMA-CA algorithm (0: BEB, 1: NOBA, 2: SW_NOBA, 3: STANDARD, 4: GNU_NOBA)", CSMA_CA);
     cmd.AddValue("packetSize", "Size of the packet", PACKET_SIZE);
     cmd.AddValue("maxRetx", "Maximum number of retransmissions", MAX_RETX);
     // cmd.AddValue("beaconOrder", "Beacon order value", BEACON_ORDER);
     cmd.AddValue("simTime", "Simulation time (in seconds)", SIM_TIME);
     cmd.AddValue("ncount", "node count", ncount);

     cmd.Parse(argc, argv);

    NODE_COUNT = ncount + 1;
    for(uint32_t i = 0; i < NODE_COUNT + 1; i++)
    {
        retransmissionCount.push_back(std::vector<uint32_t>());
    }

     NODE_COUNT_PER_TP = fillArray(ncount);

     Callback<void, SequenceNumber8> cb;
     ns3::RngSeedManager::SetSeed(42);

     SetupLogComponents();

     // LrWpanHelper lrWpanHelper;

     ////////////////////////////// 1. SETUP HELPER //////////////////////////////
     Ptr<SingleModelSpectrumChannel> channel = Create<SingleModelSpectrumChannel>();
     Ptr<LogDistancePropagationLossModel> lossModel = Create<LogDistancePropagationLossModel>();

     Ptr<ConstantSpeedPropagationDelayModel> delayModel =
         Create<ConstantSpeedPropagationDelayModel>();

     channel->SetPropagationDelayModel(delayModel);
     channel->AddPropagationLossModel(lossModel);


     nodes.Create(NODE_COUNT); // first one is coordinator
     uint16_t coordAddr = COORD_ADDR;


     uint8_t priority = 7;
     uint8_t tpCount = 0;
     uint16_t address = coordAddr + 1;
     // Vector center(0, 0, 0);
     // double radius = 5;
     for (uint32_t i = 0; i < NODE_COUNT; i++)
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

         // dev->SetMac(CreateObject<LrWpanEapMac>());

         dev->SetChannel(channel);
         dev->SetNode(nodes.Get(i));

         nodes.Get(i)->AddDevice(dev);

         ////////// SET ADDRESS AND ASSOCIATED COORDINATOR //////////
         ///// this setting must prior CSMA/CA
         dev->GetMac()->SetMacMaxFrameRetries(MAX_RETX);
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
         if (CSMA_CA == CSMA_CA_BEB)
         {
             csma = CreateObject<LrWpanCsmaCa>(priority % 8);
             dev->GetMac()->SetCsmaCaOption(CSMA_ORIGINAL);
         }
         else if (CSMA_CA == CSMA_CA_NOBA)
         {
             csma = CreateObject<LrWpanCsmaCaNoba>(priority % 8);
             dev->GetMac()->SetCsmaCaOption(CSMA_NOBA);
             // csma->TraceConnectWithoutContext("csmaCaNobaMKViolationTrace", MakeCallback(&DynamicFailure));
         }
         else if (CSMA_CA == CSMA_CA_SW_NOBA)
         {
             csma = CreateObject<LrWpanCsmaCaSwNoba>(priority % 8);
             dev->GetMac()->SetCsmaCaOption(CSMA_SW_NOBA);
             csma->TraceConnectWithoutContext("csmaCaSwNobaMKViolationTrace", MakeCallback(&DynamicFailure));
         }
         else if (CSMA_CA == CSMA_CA_STANDARD)
         {
             csma = CreateObject<LrWpanCsmaCaStandard>(priority % 8);
             dev->GetMac()->SetCsmaCaOption(CSMA_STANDARD);
             csma->TraceConnectWithoutContext("csmaCaStandardMKViolationTrace", MakeCallback(&DynamicFailure));
         }
         else if (CSMA_CA == CSMA_CA_GNU_NOBA)
         {
             csma = CreateObject<LrWpanCsmaCaGnuNoba>(priority % 8);
             dev->GetMac()->SetCsmaCaOption(CSMA_GNU_NOBA);
             csma->TraceConnectWithoutContext("csmaCaGnuNobaMKViolationTrace", MakeCallback(&DynamicFailure));
         }
         else
         {
             NS_ASSERT_MSG(false, "UNKNOWN CSMA CA");
         }

         if (i != 0)
         {
             // node
             dev->SetCsmaCa(csma);
             dev->GetMac()->TraceConnectWithoutContext("MacTx", MakeCallback(&MacTxSent)); // sent TX
         }
         else
         {
             // coordinator
             Ptr<LrWpanCsmaCa> csmaa = CreateObject<LrWpanCsmaCa>(7);
             dev->SetCsmaCa(csmaa);
         }

         //////////////////// SET CALLBACKS ////////////////////
         if (i == 0) // coordinator
         {
             MlmeStartRequestParams params;
             params.m_panCoor = true;
             params.m_PanId = PAN_ID;
             params.m_bcnOrd = BEACON_ORDER;
             params.m_sfrmOrd = BEACON_ORDER;
             Simulator::ScheduleWithContext(1,
                                            Seconds(0.01),
                                            // &LrWpanMac::MlmeStartRequest,
                                            &LrWpanMacBase::MlmeStartRequest,
                                            dev->GetMac(),
                                            params);
             cb = MakeCallback(&GenerateTraffic);
             dev->GetMac()->TraceConnectWithoutContext("BeaconStart",
                                                   cb); // received RX(DATA ONLY)
         }

         dev->GetPhy()->TraceConnectWithoutContext("PhyRxDrop",
                                                   MakeCallback(&PhyRxDrop)); // dropped RX
         dev->GetMac()->TraceConnectWithoutContext("MacRx",
                                                   MakeCallback(&MacRx)); // received RX(DATA ONLY)
         dev->GetMac()->TraceConnectWithoutContext("MacTxEnqueue",
                                                   MakeCallback(&MacTxReq)); // requested TX
         dev->GetMac()->TraceConnectWithoutContext("MacTxOk",
                                                   MakeCallback(&MacTxOk)); // sent TX & received ACK
         // dev->GetMac()->TraceConnectWithoutContext("MacTx", MakeCallback(&MacTxSent)); // sent TX
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
             if (CSMA_CA == CSMA_CA_BEB)
             {
                 DynamicCast<LrWpanCsmaCa>(dev->GetCsmaCa())
                     ->TraceConnectWithoutContext("csmaCaBebCollisionTrace",
                                              MakeCallback(&CsmaCaCollision));
             }
             else if (CSMA_CA == CSMA_CA_NOBA)
             {
                 DynamicCast<LrWpanCsmaCaNoba>(dev->GetCsmaCa())
                     ->TraceConnectWithoutContext("csmaCaNobaCollisionTrace",
                                                  MakeCallback(&CsmaCaCollision));
             }
             else if (CSMA_CA == CSMA_CA_SW_NOBA)
             {
                 DynamicCast<LrWpanCsmaCaSwNoba>(dev->GetCsmaCa())
                     ->TraceConnectWithoutContext("csmaCaSwNobaCollisionTrace",
                                                  MakeCallback(&CsmaCaCollision));
             }
             else if (CSMA_CA == CSMA_CA_STANDARD)
             {
                 DynamicCast<LrWpanCsmaCaStandard>(dev->GetCsmaCa())
                     ->TraceConnectWithoutContext("csmaCaStandardCollisionTrace",
                                                  MakeCallback(&CsmaCaCollision));
             }
             else if (CSMA_CA == CSMA_CA_GNU_NOBA)
             {
                 DynamicCast<LrWpanCsmaCaGnuNoba>(dev->GetCsmaCa())
                     ->TraceConnectWithoutContext("csmaCaGnuNobaCollisionTrace",
                                                 MakeCallback(&CsmaCaCollision));
             }
             else
             {
                 NS_ASSERT_MSG(false, "Unknown CSMA CA");
             }
         }
         // ADD DEVICE TO CONTAINER
         devices.Add(dev);

         if (i != 0)
         {
             tpCount++;
         }

         if (tpCount == NODE_COUNT_PER_TP[priority])
         {
             std::cout << "TP " << int(priority) << ": " << (int) tpCount << std::endl;
             while (NODE_COUNT_PER_TP[--priority] == 0)
                 ;
             tpCount = 0;
         }
     }

     ////////////////////////////// 5. DATA TRANSMISSION //////////////////////////////
     // Simulator::Schedule(Seconds(0), &GenerateTraffic, devices, INTERVAL);

     Simulator::Schedule(
         Seconds(SIM_TIME),
         MakeEvent(
             [cb] () mutable -> void
             {
                 DynamicCast<LrWpanNetDevice>(devices.Get(0))->GetMac()
                        ->TraceDisconnectWithoutContext("BeaconStart", cb);
             }
        )
    );


     Simulator::Schedule(
         Seconds(SIM_TIME + 30 - 0.00001),
         MakeEvent(
             [] () mutable -> void
             {
                 std::string str;
                 std::cout << std::endl << std::endl << std::endl;

                 std::ostringstream filenameStream;

                 switch (CSMA_CA)
                {
                    case CSMA_CA_STANDARD:
                        str = "std";
                        break;
                    case CSMA_CA_BEB:
                        str = "beb";
                        break;
                    case CSMA_CA_NOBA:
                        str = "noba";
                        break;
                    case CSMA_CA_SW_NOBA:
                        str = "swnoba";
                        break;
                    case CSMA_CA_GNU_NOBA:
                        str = "gnunoba";
                        break;
                    default:
                        str = "unknown";
                        break;
                }

                 filenameStream << str
                                << "_ncount" << ncount
                                << "_pkt" << PACKET_SIZE
                                << "_sim" << SIM_TIME
                                << "_retx" << MAX_RETX
                                << "_bo" << BEACON_ORDER << ".txt";
                 std::string filename = filenameStream.str();
                 std::ofstream outFile(filename);
                 std::ostream& out = outFile;  // 또는 std::cout 으로 쉽게 전환 가능

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
                     case CSMA_CA_GNU_NOBA:
                         str = "CSMA/CA GNU-NOBA";
                         break;
                     default:
                         str = "UNKNOWN";
                         break;
                 }

                 std::string s;
                 s += "{";
                 for (auto i = NODE_COUNT_PER_TP.begin(); i != NODE_COUNT_PER_TP.end(); i++)
                 {
                     s += std::to_string(*i);
                     s += ", ";
                 }
                 s += "}";

                 std::cout << "SIMULATION DURATION: " << SIM_TIME << std::endl;
                 std::cout << "NODE COUNT: " << s << std::endl;
                 std::cout << "PACKET SIZE: " << PACKET_SIZE << std::endl;
                 std::cout << "CSMA/CA CONFIGURATION: " << str << std::endl;

                 out << "SIMULATION DURATION: " << SIM_TIME << std::endl;
                 out << "NODE COUNT: " << s << std::endl;
                 out << "PACKET SIZE: " << PACKET_SIZE << std::endl;
                 out << "CSMA/CA CONFIGURATION: " << str << std::endl;

                 std::cout << "\t\t\t";
                 out << "\t\t\t";
                 for (int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << "TP" << i << "\t";
                     out << "TP" << i << "\t";
                 }
                 std::cout << std::endl << "ENQUEUED TX\t\t";
                 out << std::endl << "ENQUEUED TX\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << requestTX[i] << "\t";
                     out << requestTX[i] << "\t";
                 }
                 std::cout << "\nDEQUEUED TX\t\t";
                 out << "\nDEQUEUED TX\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << txDequeue[i] << "\t";
                     out << txDequeue[i] << "\t";
                 }
                 std::cout << "\nSUCCESS TX\t\t";
                 out << "\nSUCCESS TX\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << successTX[i] << "\t";
                     out << successTX[i] << "\t";
                 }
                 std::cout << "\nFAILED TX\t\t";
                 out << "\nFAILED TX\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << failTX[i] << "\t";
                     out << failTX[i] << "\t";
                 }
                 std::cout << "\nDYNAMIC FAILURE\t\t";
                 out << "\nDYNAMIC FAILURE\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << dynamicFailure[i] << "\t";
                     out << dynamicFailure[i] << "\t";
                 }
                 // std::cout << "\nFAILED RX\t\t";
                 // for(int i = 0; i < TP_COUNT; i++)
                 // {
                 //     std::cout << failRX[i] << "\t";
                 // }
                 std::cout << "\nSUCCESS RX\t\t";
                 out << "\nSUCCESS RX\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << successRX[i] << "\t";
                     out << successRX[i] << "\t";
                 }
                 std::cout << "\nCSMA/CA COLLISIONS\t";
                 out << "\nCSMA/CA COLLISIONS\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << collisions[i] << "\t";
                     out << collisions[i] << "\t";
                 }
                 std::cout << "\nMAX DELAYS\t\t";
                 out << "\nMAX DELAYS\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     if (NODE_COUNT_PER_TP[i] == 0)
                     {
                         std::cout << 0 << "\t";
                         out << 0 << "\t";
                         continue;
                     }
                     std::cout << *std::max_element(rxDelay[i].begin(), rxDelay[i].end()) << "\t";
                     out << *std::max_element(rxDelay[i].begin(), rxDelay[i].end()) << "\t";
                 }
                 std::cout << "\nAVG DELAYS\t\t";
                 out << "\nAVG DELAYS\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     if (NODE_COUNT_PER_TP[i] == 0)
                     {
                         std::cout << 0 << "\t";
                         out << 0 << "\t";
                         continue;
                     }
                     std::cout
                         << std::accumulate(rxDelay[i].begin(), rxDelay[i].end(), 0) / rxDelay[i].size()
                         << "\t";
                     out
                         << std::accumulate(rxDelay[i].begin(), rxDelay[i].end(), 0) / rxDelay[i].size()
                         << "\t";
                 }
                 std::cout << "\nMAC TX COUNTS\t\t";
                 out << "\nMAC TX COUNTS\t\t";
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     std::cout << sentTX[i] << "\t";
                     out << sentTX[i] << "\t";
                 }
                 std::cout << "\nMAX RETX COUNTS\t\t";
                 out << "\nMAX RETX COUNTS\t\t";
                 std::vector<uint32_t> maxRetxPerTp(TP_COUNT, 0);
                 for(uint32_t i = 0; i < NODE_COUNT; i++)
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
                     std::cout << maxRetxPerTp[i] << "\t";
                     out << maxRetxPerTp[i] << "\t";
                 }
                 std::cout << std::endl;
                 out << std::endl;

                 std::ostringstream filenameStream2;
                filenameStream2 << str
                              << "_ncount" << ncount
                              << "_pkt" << PACKET_SIZE
                              << "_sim" << SIM_TIME
                              << "_retx" << MAX_RETX
                              << "_bo" << BEACON_ORDER << ".csv";
                std::ofstream ou(filenameStream2.str());
                std::ostream& out2 = ou;
                 for(int i = 0; i < TP_COUNT; i++)
                 {
                     out2 << "TP" << i << ", ";
                     for (auto k = rxDelay[i].begin(); k != rxDelay[i].end(); k++)
                     {
                         out2 << static_cast<double>(*k) << ", ";
                     }
                     out2 << std::endl;
                 }
             }
         )
     );


     Simulator::Schedule(
         Seconds(0),
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


     Simulator::Stop(Seconds(SIM_TIME + 30));
     Simulator::Run();

     Simulator::Destroy();
     return 0;
 }
