/*
 * Copyright (c) 2025 Gyeongsang National University, Jinju, South Korea.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */

#include "../src/spectrum/model/single-model-spectrum-channel.h"

#include <ns3/constant-position-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/log.h>
#include <ns3/lr-wpan-csmaca-noba.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/packet.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>

#define CSMA_CA_BEB 0
#define CSMA_CA_NOBA 1
#define CSMA_CA_SW_NOBA 2

#define CSMA_CA CSMA_CA_NOBA




#include <iostream>

#define NODE_COUNT 9 // 8 priorities, 10 nodes per priority, INCLUDING COORDINATOR
#define PACKET_SIZE 30
#define PAN_ID 5
#define COORD_ADDR 1

#define SIM_TIME 2

using namespace ns3;
using namespace ns3::lrwpan;

static uint32_t requestTX[TP_COUNT] = {0, };
static uint32_t sentTX[TP_COUNT] = {0, };
static uint32_t successTX[TP_COUNT] = {0, };
static uint32_t collisions[TP_COUNT] = {0, };

static uint32_t failTX[TP_COUNT] = {0, };
static uint32_t successRX[TP_COUNT] = {0, };
static uint32_t failRX[TP_COUNT] = {0, };

void
BeaconIndication(MlmeBeaconNotifyIndicationParams params)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " secs | Received BEACON packet of size ");
}

void
DataIndication(McpsDataIndicationParams params, Ptr<Packet> p)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << " secs | Received DATA packet of size " << p->GetSize());
}

void
TransEndIndication(McpsDataConfirmParams params)
{
    // In the case of transmissions with the Ack flag activated, the transaction is only
    // successful if the Ack was received.
    if (params.m_status == MacStatus::SUCCESS)
     {
         NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " secs | Transmission successfully sent");
     }
     if (params.m_status == MacStatus::NO_ACK)
     {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " secs | TRANSMISSION ENDED WITH NO ACK");
     }

}

void
DataIndicationCoordinator(McpsDataIndicationParams params, Ptr<Packet> p)
 {
     NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                   << "s Coordinator Received DATA packet (size " << p->GetSize() << " bytes)");
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
    collisions[TP]++;
    // std::cout << "\t\tPRIORITY " << TP << "GOT COLLISION: " << collisions[TP] << std::endl;
}


void
MacRxDrop(Ptr<const Packet> pkt, uint8_t TP) // MacRxDrop: fail RX
{
    // includes wrong MAC address
    failRX[TP]++;
}


void
MacRx(Ptr<const Packet> pkt, uint8_t TP) // MacRx: success RX
{
    successRX[TP]++;
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
    successTX[TP]++;
}

void
MacTxDrop(Ptr<const Packet> pkt, uint8_t TP) // MacTxDrop: fail TX
{
    failTX[TP]++;
}

static uint32_t a = 0;

void
TxEnqueue(Ptr<const Packet> pkt)
{
    a++;
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
        McpsDataRequestParams params2;
        params2.m_dstPanId = PAN_ID;
        params2.m_srcAddrMode = SHORT_ADDR;
        params2.m_dstAddrMode = SHORT_ADDR;
        params2.m_dstAddr = Mac16Address(COORD_ADDR);
        params2.m_msduHandle = (msduHandle++) % (UINT8_MAX + 1);
        params2.m_txOptions = TX_OPTION_ACK;  // Enable direct transmission with Ack

        Simulator::ScheduleNow(
            &LrWpanMac::McpsDataRequest,
            dev->GetMac(),
            params2,
            p
        );
    }

    Simulator::ScheduleWithContext(
        2,
        Simulator::Now() + Time(interval),
        GenerateTraffic,
        devices,
        interval
    );
}

int
main(int argc, char* argv[])
{
    ns3::RngSeedManager::SetSeed(42);

    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnable("LrWpanMac", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanPhy", LOG_LEVEL_DEBUG);
    LogComponentEnable("LrWpanCsmaCaNoba", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_ALL);

    // LrWpanHelper lrWpanHelper;
    NodeContainer nodes;
    nodes.Create(NODE_COUNT); // first one is coordinator


    ////////////////////////////// 1. SETUP HELPER //////////////////////////////
    Ptr<SingleModelSpectrumChannel> channel = Create<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> lossModel = Create<LogDistancePropagationLossModel>();

    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        Create<ConstantSpeedPropagationDelayModel>();

    channel->SetPropagationDelayModel(delayModel);
    channel->AddPropagationLossModel(lossModel);

    NetDeviceContainer devices;


    uint8_t priority = 7;
    uint16_t address = 1;
    Vector center(0, 0, 0);
    double radius = 5;
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


        //////////////////// SETUP CSMA/CA ////////////////////
        #if CSMA_CA == CSMA_CA_BEB
            Ptr<LrWpanCsmaCa> csma = CreateObject<LrWpanCsmaCa>(priority % 8);
        #endif
        #if CSMA_CA == CSMA_CA_NOBA
            Ptr<LrWpanCsmaCaNoba> csma = CreateObject<LrWpanCsmaCaNoba>(priority % 8);
        #endif

        dev->SetCsmaCa(csma);
        dev->SetChannel(channel);
        dev->SetNode(nodes.Get(i));
        nodes.Get(i)->AddDevice(dev);


        ////////// SET ADDRESS AND ASSOCIATED COORDINATOR //////////
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

        if(i == 0) // coordinator
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
        dev->GetMac()->TraceConnectWithoutContext("MacRxDrop", MakeCallback(&MacRxDrop));   // dropped RX
        dev->GetMac()->TraceConnectWithoutContext("MacRx", MakeCallback(&MacRx));           // received RX(DATA ONLY)
        dev->GetMac()->TraceConnectWithoutContext("MacTxEnqueue", MakeCallback(&MacTxReq)); // requested TX
        dev->GetMac()->TraceConnectWithoutContext("MacTxOk", MakeCallback(&MacTxOk));       // sent TX & received ACK
        dev->GetMac()->TraceConnectWithoutContext("MacTx", MakeCallback(&MacTxSent));       // sent TX
        dev->GetMac()->TraceConnectWithoutContext("MacTxDrop", MakeCallback(&MacTxDrop));   // dropped TX (ACK timeout | channel access failure)

#if CSMA_CA == CSMA_CA_BEB
        DynamicCast<LrWpanCsmaCa>(dev->GetCsmaCa())->TraceConnectWithoutContext("csmaCaCollisionTrace", MakeCallback(&CsmaCaCollision));
#endif
#if CSMA_CA == CSMA_CA_NOBA
        DynamicCast<LrWpanCsmaCaNoba>(dev->GetCsmaCa())->TraceConnectWithoutContext("csmaCaNobaCollisionTrace", MakeCallback(&CsmaCaCollision));
#endif

        // ADD DEVICE TO CONTAINER
        devices.Add(dev);

        priority++;
    }


    ////////////////////////////// 5. DATA TRANSMISSION //////////////////////////////
    Simulator::Schedule(
        Seconds(1.0),
        &GenerateTraffic,
        devices,
        0.5
    );

    Simulator::Schedule(
        Seconds(SIM_TIME - 0.001),
        MakeEvent(
            [] () mutable -> void
            {
                std::cout << std::endl << std::endl << std::endl;

                std::cout << "TRAFFIC PRIORITY\t";
                for (int i = 0; i < TP_COUNT; i++)
                {
                    std::cout << "TP" << i << "\t";
                }
                std::cout << std::endl << "REQUESTED TX\t\t";
                for(int i = 0; i < TP_COUNT; i++)
                {
                    std::cout << requestTX[i] << '\t';
                }
                std::cout << "\nSUCCESS TX\t\t\t";
                for(int i = 0; i < TP_COUNT; i++)
                {
                    std::cout << successTX[i] << '\t';
                }
                std::cout << "\nFAILED TX\t\t\t";
                for(int i = 0; i < TP_COUNT; i++)
                {
                    std::cout << failTX[i] << '\t';
                }
                std::cout << "\nSUCCESS RX\t\t\t";
                for(int i = 0; i < TP_COUNT; i++)
                {
                    std::cout << successRX[i] << '\t';
                }
                std::cout << "\nFAILED RX\t\t\t";
                for(int i = 0; i < TP_COUNT; i++)
                {
                    std::cout << failRX[i] << '\t';
                }
                std::cout << "\nCSMA/CA COLLISIONS\t";
                for(int i = 0; i < TP_COUNT; i++)
                {
                    std::cout << collisions[i] << '\t';
                }
                std::cout << std::endl;
            }
        )
    );

    ////////////////////////////// PRINT NODE INFORMATION //////////////////////////////
    std::cout << "\n\n==========NODE INFORMATION==========\n\n";
    for (auto d = devices.Begin(); d != devices.End(); d++)
    {
        Ptr<LrWpanNetDevice> dev = DynamicCast<LrWpanNetDevice>(*d);

        uint16_t panId = dev->GetMac()->GetPanId();
        Mac16Address addr = dev->GetMac()->GetShortAddress();
        priority = dev->GetMac()->getPriority();

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
