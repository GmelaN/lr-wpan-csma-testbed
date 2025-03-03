/*
 * Copyright (c) 2019 Ritsumeikan University, Shiga, Japan.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:  Alberto Gallegos Ramonet <ramonet@fc.ritsumei.ac.jp>
 */

/*
 *   Coordinator              End Device
 *       N0   <----------------  N1
 *      (dev0)                 (dev1)
 *
 * This example demonstrate the usage of the MAC primitives involved in
 * direct transmissions for the beacon enabled mode of IEEE 802.15.4-2011.
 * A single packet is sent from an end device to the coordinator during the CAP
 * of the first incoming superframe.
 *
 * This example do not demonstrate a full protocol stack usage.
 * For full protocol stack usage refer to 6lowpan examples.
 *
 */

#include <ns3/constant-position-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/log.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/packet.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/multi-model-spectrum-channel.h>

#include <ns3/lr-wpan-csmaca-noba.h>

#include <iostream>

#define NODE_COUNT 80 // 8 priorities, 10 nodes per priority
#define PACKET_SIZE 30
#define PAN_ID 5
#define COORD_ADDR 1

using namespace ns3;
using namespace ns3::lrwpan;

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
    if (params.m_status == MacStatus::SUCCESS)
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "Beacon status SUCCESSFUL");
    }
}

void GenerateTraffic(NetDeviceContainer devices, double start, double interval)
{
    for(auto i = devices.Begin(); i < devices.End(); i++)
    {
        if(i == devices.Begin()) continue; // first one is coordinator

        Ptr<LrWpanNetDevice> dev = DynamicCast<LrWpanNetDevice>(*i);
        Ptr<Packet> p = Create<Packet>(PACKET_SIZE);
        McpsDataRequestParams params2;
        params2.m_dstPanId = PAN_ID;
        params2.m_srcAddrMode = SHORT_ADDR;
        params2.m_dstAddrMode = SHORT_ADDR;
        params2.m_dstAddr = Mac16Address(COORD_ADDR);
        params2.m_msduHandle = 0;
        params2.m_txOptions = TX_OPTION_ACK;  // Enable direct transmission with Ack

        Simulator::ScheduleWithContext(
            1,
            Seconds(start),
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
        start,
        interval
    );
}

int
main(int argc, char* argv[])
{
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnable("LrWpanMac", LOG_LEVEL_LOGIC);
    LogComponentEnable("LrWpanCsmaCaNoba", LOG_LEVEL_ALL);

    LrWpanHelper lrWpanHelper;
    NodeContainer nodes;

    ////////////////////////////// 0. SETUP HELPER //////////////////////////////
    nodes.Create(NODE_COUNT + 1); // first one is coordinator
    Ptr<MultiModelSpectrumChannel> channel = Create<MultiModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> lossModel = Create<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel = Create<ConstantSpeedPropagationDelayModel>();
    channel->SetPropagationDelayModel(delayModel);
    channel->AddPropagationLossModel(lossModel);
    lrWpanHelper.SetChannel(channel); 

    NetDeviceContainer devices = lrWpanHelper.Install(nodes);


    ////////////////////////////// 1. SETUP COORDINATOR //////////////////////////////
    Ptr<LrWpanNetDevice> coordDev = DynamicCast<LrWpanNetDevice>(nodes.Get(0)->GetDevice(0));
    coordDev->SetAddress(Mac16Address(COORD_ADDR));
    coordDev->GetMac()->setPriority(0);
    
    MlmeStartRequestParams params;
    params.m_panCoor = true;
    params.m_PanId = PAN_ID;
    params.m_bcnOrd = 14;
    params.m_sfrmOrd = 6;
    Simulator::ScheduleWithContext(1,
                                   Seconds(2.0),
                                   &LrWpanMac::MlmeStartRequest,
                                   coordDev->GetMac(),
                                   params);


    ////////////////////////////// 2. SETUP PRIORITY //////////////////////////////
    uint16_t address = 2;
    uint8_t priority = 0;
    for(int i = 1; i <= NODE_COUNT; i++)
    {
        // SET PRIORITY
        Ptr<LrWpanNetDevice> dev = DynamicCast<LrWpanNetDevice>(nodes.Get(i)->GetDevice(0));
        dev->GetMac()->setPriority(priority);
        Ptr<LrWpanCsmaCaNoba> csma = CreateObject<LrWpanCsmaCaNoba>(priority);
        dev->SetCsmaCa(csma);
        
        if(i % 10 == 0) priority++;

        // SET ADDRESS
        dev->SetAddress(Mac16Address(address++));

        // SET PAN ID
        dev->GetMac()->SetPanId(PAN_ID);
        dev->GetMac()->SetAssociatedCoor(Mac16Address(COORD_ADDR));
    }

    ////////////////////////////// 3. SETUP MOBILITY //////////////////////////////
    double radius = 10.0;
    Vector center(0.0, 0.0, 0.0);
    for (uint32_t i = 0; i < NODE_COUNT; ++i) {
        Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
    
        if (i == 0) { 
            // 코디네이터는 중앙에 배치
            mob->SetPosition(center);
        } else {
            // 원형으로 배치
            double angle = 2.0 * M_PI * (i - 1) / (NODE_COUNT - 1); // 각도 계산
            double x = center.x + radius * cos(angle);
            double y = center.y + radius * sin(angle);
            mob->SetPosition(Vector(x, y, 0.0));
        }
    
        nodes.Get(i)->AggregateObject(mob);
    }


    ////////////////////////////// 4. SETUP CALLBACKS //////////////////////////////
    for(auto i = devices.Begin(); i < devices.End(); i++)
    {
        Ptr<LrWpanNetDevice> dev = DynamicCast<LrWpanNetDevice>(*i);

        MlmeStartConfirmCallback cb0;
        cb0 = MakeCallback(&StartConfirm);
        dev->GetMac()->SetMlmeStartConfirmCallback(cb0);

        McpsDataConfirmCallback cb1;
        cb1 = MakeCallback(&TransEndIndication);
        dev->GetMac()->SetMcpsDataConfirmCallback(cb1);

        MlmeBeaconNotifyIndicationCallback cb3;
        cb3 = MakeCallback(&BeaconIndication);
        dev->GetMac()->SetMlmeBeaconNotifyIndicationCallback(cb3);

        if(i == devices.Begin()) {
            McpsDataIndicationCallback cb5;
            cb5 = MakeCallback(&DataIndicationCoordinator);
            dev->GetMac()->SetMcpsDataIndicationCallback(cb5);
        }
        else
        {
            McpsDataIndicationCallback cb4;
            cb4 = MakeCallback(&DataIndication);
            dev->GetMac()->SetMcpsDataIndicationCallback(cb4);
        }
    }


    ////////////////////////////// 5. DATA TRANSMISSION //////////////////////////////
    GenerateTraffic(devices, 2, 0.5);

    Simulator::Stop(Seconds(600));
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
