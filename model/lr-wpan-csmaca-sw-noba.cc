/*
 * Copyright (c) 2025 jshyeon
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */

#include "lr-wpan-csmaca-sw-noba.h"
#include "lr-wpan-constants.h"

#include <ns3/log.h>
#include <ns3/random-variable-stream.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>

/*
#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT                                                                      \
     std::clog << "[" << m_mac->GetShortAddress() << "] ";
*/
namespace ns3
{
namespace lrwpan
{

NS_LOG_COMPONENT_DEFINE("LrWpanCsmaCaSwNoba");
NS_OBJECT_ENSURE_REGISTERED(LrWpanCsmaCaSwNoba);


uint32_t LrWpanCsmaCaSwNoba::SW[TP_COUNT]; // each TP
std::pair<uint32_t, uint32_t> LrWpanCsmaCaSwNoba::CW[TP_COUNT]; // each TP
uint32_t LrWpanCsmaCaSwNoba::WL[TP_COUNT]; // each TP
uint32_t LrWpanCsmaCaSwNoba::COLLISION_COUNT[TP_COUNT]; // each TP
uint32_t LrWpanCsmaCaSwNoba::SUCCESS_COUNT[TP_COUNT]; // each TP


TypeId
LrWpanCsmaCaSwNoba::GetTypeId()
{
    static TypeId tid = TypeId("ns3::lrwpan::LrWpanCsmaCaSwNoba")
                            .AddDeprecatedName("ns3::LrWpanCsmaCaSwNoba")
                            .SetParent<LrWpanCsmaCaCommon>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<LrWpanCsmaCaSwNoba>()
                            .AddTraceSource("csmaCaSwNobaCollisionTrace",
                                            "CSMA/CA-SW-NOBA collision count trace",
                                            MakeTraceSourceAccessor(&LrWpanCsmaCaSwNoba::m_csmaCaCollisionTrace),
                                            "ns3::TracedCallback");
    return tid;
}

void
LrWpanCsmaCaSwNoba::InitializeGlobals()
{
    for(int i = 0; i < TP_COUNT; i++)
    {
        LrWpanCsmaCaSwNoba::SW[i] = 1;
    }
    LrWpanCsmaCaSwNoba::CW[7].first = 1;

    for(int i = TP_COUNT - 1; i >= 0; i--) {
        LrWpanCsmaCaSwNoba::WL[i] = 8 * (8 - i); // 8, 16, 24, 32, 40, 48, 56, 64
    }

    for(int i = TP_COUNT - 1; i >= 0; i--) {
        if(i > 0) {
            LrWpanCsmaCaSwNoba::CW[i].second = // CW_max,i
                std::min(LrWpanCsmaCaSwNoba::CW[i].first + LrWpanCsmaCaSwNoba::SW[i], LrWpanCsmaCaSwNoba::WL[i]);

            LrWpanCsmaCaSwNoba::CW[i-1].first = // CW_min,i-1
                LrWpanCsmaCaSwNoba::CW[i].second + 1;
        }
        else {
            LrWpanCsmaCaSwNoba::CW[i].second =
                std::min(LrWpanCsmaCaSwNoba::CW[i].first + LrWpanCsmaCaSwNoba::SW[i], LrWpanCsmaCaSwNoba::WL[i]);
        }
    }

    NS_LOG_DEBUG(
        "CSMA/CA-NOBA: Initializing SW, CW, WL...\n"
        <<
        "SW: " << SW[0] << "\n" << SW[1] << "\n" << SW[2] << "\n" << SW[3] << "\n" << SW[4] << "\n" << SW[5] << "\n" << SW[6] << "\n"  << SW[7]
        <<
        '\n'
        <<
        "CW: "
        << "[0]: " << CW[0].first << " ~ " << CW[0].second << "\n"
        << "[1]: " << CW[1].first << " ~ " << CW[1].second << "\n"
        << "[2]: " << CW[2].first << " ~ " << CW[2].second << "\n"
        << "[3]: " << CW[3].first << " ~ " << CW[3].second << "\n"
        << "[4]: " << CW[4].first << " ~ " << CW[4].second << "\n"
        << "[5]: " << CW[5].first << " ~ " << CW[5].second << "\n"
        << "[6]: " << CW[6].first << " ~ " << CW[6].second << "\n"
        << "[7]: " << CW[7].first << " ~ " << CW[7].second << "\n"
        <<
        "WL: " << WL[0] << "\n" << WL[1] << "\n" << WL[2] << "\n" << WL[3] << "\n" << WL[4] << "\n" << WL[5] << "\n" << WL[6] << "\n"  << WL[7]
    );
    return;
}


LrWpanCsmaCaSwNoba::LrWpanCsmaCaSwNoba(uint8_t priority)
{
    NS_ASSERT(priority >= 0 && priority <= 7);

    for(int i = 0; i < TP_COUNT; i++)
    {
        LrWpanCsmaCaSwNoba::SUCCESS_COUNT[i] = 0;
        LrWpanCsmaCaSwNoba::COLLISION_COUNT[i] = 0;
    }

    LrWpanCsmaCaSwNoba::InitializeGlobals();

    m_isSlotted = true;
    m_macBattLifeExt = false;
    m_random = CreateObject<UniformRandomVariable>();
    m_ccaRequestRunning = false;
    m_randomBackoffPeriodsLeft = 0;
    m_coorDest = false;

    m_TP = priority;
    m_collisions = 0;
    m_backoffCount = 0;
    m_freezeBackoff = false;
}

LrWpanCsmaCaSwNoba::LrWpanCsmaCaSwNoba()
{
    NS_ASSERT_MSG(false, "nodeCount, priority missing.");
}

LrWpanCsmaCaSwNoba::~LrWpanCsmaCaSwNoba()
{
    m_mac = nullptr;
}

void
LrWpanCsmaCaSwNoba::DoDispose()
{
    m_lrWpanMacStateCallback = MakeNullCallback<void, MacState>();
    m_lrWpanMacTransCostCallback = MakeNullCallback<void, uint32_t>();

    Cancel();
    m_mac = nullptr;
}

void
LrWpanCsmaCaSwNoba::SetMac(Ptr<LrWpanMac> mac)
{
    m_mac = mac;
}

Ptr<LrWpanMac>
LrWpanCsmaCaSwNoba::GetMac()
{
    return m_mac;
}

void
LrWpanCsmaCaSwNoba::SetSlottedCsmaCa()
{
    m_isSlotted = true;
}

void
LrWpanCsmaCaSwNoba::SetUnSlottedCsmaCa()
{
    NS_ASSERT_MSG(false, "cannot set unslotted CSMA/CA NOBA.");
}

bool
LrWpanCsmaCaSwNoba::IsSlottedCsmaCa()
{
    return m_isSlotted;
}

bool
LrWpanCsmaCaSwNoba::IsUnSlottedCsmaCa()
{
    return !m_isSlotted;
}

Time
LrWpanCsmaCaSwNoba::GetTimeToNextSlot() const
{
    NS_LOG_FUNCTION(this);

    // The reference for the beginning of the SUPERFRAME (the active period) changes depending
    // on the data packet being sent from the Coordinator/outgoing frame (Tx beacon time reference)
    // or other device/incoming frame (Rx beacon time reference ).

    Time elapsedSuperframe; // (i.e  The beacon + the elapsed CAP)
    Time currentTime = Simulator::Now();
    double symbolsToBoundary;
    Time nextBoundary;
    uint64_t elapsedSuperframeSymbols;
    uint64_t symbolRate =
        (uint64_t)m_mac->GetPhy()->GetDataOrSymbolRate(false); // symbols per second
    Time timeAtBoundary;

    if (m_coorDest)
    {
        // Take the Incoming Frame Reference
        elapsedSuperframe = currentTime - m_mac->m_macBeaconRxTime;

        Time beaconTime [[maybe_unused]] = Seconds((double)m_mac->m_rxBeaconSymbols / symbolRate);
        Time elapsedCap [[maybe_unused]] = elapsedSuperframe - beaconTime;
        NS_LOG_DEBUG("Elapsed incoming CAP symbols: " << (elapsedCap.GetSeconds() * symbolRate)
                                                      << " (" << elapsedCap.As(Time::S) << ")");
    }
    else
    {
        // Take the Outgoing Frame Reference
        elapsedSuperframe = currentTime - m_mac->m_macBeaconTxTime;
    }

    // get a close value to the the boundary in symbols
    elapsedSuperframeSymbols = elapsedSuperframe.GetSeconds() * symbolRate;
    symbolsToBoundary = lrwpan::aUnitBackoffPeriod -
                        std::fmod((double)elapsedSuperframeSymbols, lrwpan::aUnitBackoffPeriod);

    timeAtBoundary = Seconds((double)(elapsedSuperframeSymbols + symbolsToBoundary) / symbolRate);

    // get the exact time boundary
    nextBoundary = timeAtBoundary - elapsedSuperframe;

    NS_LOG_DEBUG("Elapsed Superframe symbols: " << elapsedSuperframeSymbols << " ("
                                                << elapsedSuperframe.As(Time::S) << ")");

    NS_LOG_DEBUG("Next backoff period boundary in approx. "
                 << nextBoundary.GetSeconds() * symbolRate << " symbols ("
                 << nextBoundary.As(Time::S) << ")");

    return nextBoundary;
}

void
LrWpanCsmaCaSwNoba::Start()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(m_isSlotted, "only slotted CSMA-CA supported.");

    m_collisions = 0; // collision counter C
    m_backoffCount = m_random->GetInteger(LrWpanCsmaCaSwNoba::CW[m_TP].first, LrWpanCsmaCaSwNoba::CW[m_TP].second); // backoff counter B
    NS_LOG_DEBUG("Using CSMA-CA NOBA, bakcoff count is: " << m_backoffCount);

    // m_coorDest to decide between incoming and outgoing superframes times
    m_coorDest = m_mac->IsCoordDest();

    // Locate backoff period boundary. (i.e. a time delay to align with the next backoff period
    // boundary)
    Time backoffBoundary = GetTimeToNextSlot();
    m_randomBackoffEvent =
        Simulator::Schedule(backoffBoundary, &LrWpanCsmaCaSwNoba::RandomBackoffDelay, this);
}

void
LrWpanCsmaCaSwNoba::Cancel()
{
    m_randomBackoffEvent.Cancel();
    m_requestCcaEvent.Cancel();
    m_canProceedEvent.Cancel();
    if(m_mac) {
        m_mac->GetPhy()->CcaCancel();
    }
}

void
LrWpanCsmaCaSwNoba::RandomBackoffDelay()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(m_isSlotted, "only slotted CSMA/CA is supported.");

    Time randomBackoff;
    uint64_t symbolRate;
    Time timeLeftInCap;

    symbolRate = (uint64_t)m_mac->GetPhy()->GetDataOrSymbolRate(false); // symbols per second

    // We should not recalculate the random backoffPeriods if we are in a slotted CSMA-CA and the
    // transmission was previously deferred (m_randomBackoffPeriods != 0) or ACK not receiveed
    if (m_backoffCount == 0 || m_freezeBackoff)
    {
        m_backoffCount = m_random->GetInteger(LrWpanCsmaCaSwNoba::CW[m_TP].first, LrWpanCsmaCaSwNoba::CW[m_TP].second);
    }

    randomBackoff =
        Seconds((double)(m_backoffCount * lrwpan::aUnitBackoffPeriod) / symbolRate);

        // We must make sure there is enough time left in the CAP, otherwise we continue in
        // the CAP of the next superframe after the transmission/reception of the beacon (and the
        // IFS)
        timeLeftInCap = GetTimeLeftInCap();

        NS_LOG_DEBUG("CSMA/CA-NOBA: proceeding after random backoff of "
                     << m_backoffCount << " periods ("
                     << (randomBackoff.GetSeconds() * symbolRate) << " symbols or "
                     << randomBackoff.As(Time::S) << ")");

        NS_LOG_DEBUG("Backoff periods left in CAP: "
                     << ((timeLeftInCap.GetSeconds() * symbolRate) / lrwpan::aUnitBackoffPeriod)
                     << " (" << (timeLeftInCap.GetSeconds() * symbolRate) << " symbols or "
                     << timeLeftInCap.As(Time::S) << ")");

        if (randomBackoff >= timeLeftInCap)
        {
            uint32_t usedBackoffs =
                (double)(timeLeftInCap.GetSeconds() * symbolRate) / lrwpan::aUnitBackoffPeriod;
            m_backoffCount -= usedBackoffs;
            NS_LOG_DEBUG("No time in CAP to complete backoff delay, deferring to the next CAP");
            m_endCapEvent =
                Simulator::Schedule(timeLeftInCap, &LrWpanCsmaCaSwNoba::DeferCsmaTimeout, this);
        }
        else
        {
            m_canProceedEvent = Simulator::Schedule(randomBackoff, &LrWpanCsmaCaSwNoba::CanProceed, this);
        }
}

Time
LrWpanCsmaCaSwNoba::GetTimeLeftInCap()
{
    Time currentTime;
    uint64_t capSymbols;
    Time endCapTime;
    uint64_t activeSlot;
    uint64_t symbolRate;
    Time rxBeaconTime;

    // At this point, the currentTime should be aligned on a backoff period boundary
    currentTime = Simulator::Now();
    symbolRate = (uint64_t)m_mac->GetPhy()->GetDataOrSymbolRate(false); // symbols per second

    if (m_coorDest)
    { // Take Incoming frame reference
        activeSlot = m_mac->m_incomingSuperframeDuration / 16;
        capSymbols = activeSlot * (m_mac->m_incomingFnlCapSlot + 1);
        endCapTime = m_mac->m_macBeaconRxTime + Seconds((double)capSymbols / symbolRate);
    }
    else
    { // Take Outgoing frame reference
        activeSlot = m_mac->m_superframeDuration / 16;
        capSymbols = activeSlot * (m_mac->m_fnlCapSlot + 1);
        endCapTime = m_mac->m_macBeaconTxTime + Seconds((double)capSymbols / symbolRate);
    }

    return (endCapTime - currentTime);
}

void
LrWpanCsmaCaSwNoba::CanProceed()
{
    NS_LOG_FUNCTION(this);

    Time timeLeftInCap;
    uint16_t ccaSymbols;
    uint32_t transactionSymbols;
    Time transactionTime;
    uint64_t symbolRate;

    ccaSymbols = 0;
    m_randomBackoffPeriodsLeft = 0;
    symbolRate = (uint64_t)m_mac->GetPhy()->GetDataOrSymbolRate(false);
    timeLeftInCap = GetTimeLeftInCap();

    // TODO: On the 950 Mhz Band (Japanese Band)
    //       only a single CCA check is performed;
    //       the CCA check duration time is:
    //
    //       CCA symbols = phyCCADuration * m_CW (1)
    //       other PHYs:
    //       CCA symbols = 8 * m_CW(2)
    //
    //       note: phyCCADuration & 950Mhz band PHYs are
    //             not currently implemented in ns-3.
    ccaSymbols += 8 * m_backoffCount;

    // The MAC sublayer shall proceed if the remaining CSMA-CA algorithm steps
    // can be completed before the end of the CAP.
    // See IEEE 802.15.4-2011 (Sections 5.1.1.1 and 5.1.1.4)
    // Transaction = 2 CCA + frame transmission (SHR+PHR+PPDU) + turnaroudtime*2 (Rx->Tx & Tx->Rx) +
    // IFS (LIFS or SIFS) and Ack time (if ack flag true)

    transactionSymbols = ccaSymbols + m_mac->GetTxPacketSymbols();

    if (m_mac->IsTxAckReq())
    {
        NS_LOG_DEBUG("ACK duration symbols: " << m_mac->GetMacAckWaitDuration());
        transactionSymbols += m_mac->GetMacAckWaitDuration();
    }
    else
    {
        // time the PHY takes to switch from Rx to Tx and Tx to Rx
        transactionSymbols += (lrwpan::aTurnaroundTime * 2);
    }
    transactionSymbols += m_mac->GetIfsSize();

    // Report the transaction cost
    if (!m_lrWpanMacTransCostCallback.IsNull())
    {
        m_lrWpanMacTransCostCallback(transactionSymbols);
    }

    transactionTime = Seconds((double)transactionSymbols / symbolRate);
    NS_LOG_DEBUG("Total required transaction: " << transactionSymbols << " symbols ("
                                                << transactionTime.As(Time::S) << ")");

    if (transactionTime > timeLeftInCap)
    {
        NS_LOG_DEBUG("Transaction of "
                     << transactionSymbols << " symbols "
                     << "cannot be completed in CAP, deferring transmission to the next CAP");

        NS_LOG_DEBUG("Symbols left in CAP: " << (timeLeftInCap.GetSeconds() * symbolRate) << " ("
                                             << timeLeftInCap.As(Time::S) << ")");

        m_endCapEvent = Simulator::Schedule(timeLeftInCap, &LrWpanCsmaCaSwNoba::DeferCsmaTimeout, this);
    }
    else
    {
        m_requestCcaEvent = Simulator::ScheduleNow(&LrWpanCsmaCaSwNoba::RequestCCA, this);
    }
}

void
LrWpanCsmaCaSwNoba::RequestCCA()
{
    NS_LOG_FUNCTION(this);
    m_ccaRequestRunning = true;
    m_mac->GetPhy()->PlmeCcaRequest();
}

void
LrWpanCsmaCaSwNoba::DeferCsmaTimeout()
{
    NS_LOG_FUNCTION(this);
    m_lrWpanMacStateCallback(MAC_CSMA_DEFERRED);
}

void
LrWpanCsmaCaSwNoba::PlmeCcaConfirm(PhyEnumeration status)
{
    NS_LOG_FUNCTION(this << status);

    // Only react on this event, if we are actually waiting for a CCA.
    // If the CSMA algorithm was canceled, we could still receive this event from
    // the PHY. In this case we ignore the event.
    if (m_ccaRequestRunning)
    {
        m_ccaRequestRunning = false;
        if (status == IEEE_802_15_4_PHY_IDLE)
        {
            // channel is idle
            m_backoffCount--;
            if (m_backoffCount == 0)
            {
                // inform MAC channel is idle
                if (!m_lrWpanMacStateCallback.IsNull())
                {
                    NS_LOG_LOGIC("Notifying MAC of idle channel");
                    m_lrWpanMacStateCallback(CHANNEL_IDLE);
                }
            }
            else
            {
                NS_LOG_LOGIC("Perform CCA again, backoff count = " << m_backoffCount);
                m_requestCcaEvent = Simulator::ScheduleNow(&LrWpanCsmaCaSwNoba::RequestCCA,
                                                            this); // Perform CCA again
            }
        }
        else
        {
            // m_csmaCaCollisionTrace(m_TP, m_collisions);
            // freeze backoff counter and retry
            NS_LOG_DEBUG("Perform another backoff; freeze backoff count: " << m_backoffCount);
            m_freezeBackoff = true;
            m_randomBackoffEvent =
                Simulator::ScheduleNow(&LrWpanCsmaCaSwNoba::RandomBackoffDelay, this);
        }
    }
}

void
LrWpanCsmaCaSwNoba::SetLrWpanMacTransCostCallback(LrWpanMacTransCostCallback c)
{
    NS_LOG_FUNCTION(this);
    m_lrWpanMacTransCostCallback = c;
}

void
LrWpanCsmaCaSwNoba::SetLrWpanMacStateCallback(LrWpanMacStateCallback c)
{
    NS_LOG_FUNCTION(this);
    m_lrWpanMacStateCallback = c;
}

void
LrWpanCsmaCaSwNoba::SetBatteryLifeExtension(bool batteryLifeExtension)
{
    m_macBattLifeExt = batteryLifeExtension;
}

int64_t
LrWpanCsmaCaSwNoba::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this);
    m_random->SetStream(stream);
    return 1;
}

uint8_t
LrWpanCsmaCaSwNoba::GetNB()
{
    return m_collisions;
}

bool
LrWpanCsmaCaSwNoba::GetBatteryLifeExtension()
{
    return m_macBattLifeExt;
}

void
LrWpanCsmaCaSwNoba::SetBackoffCounter()
{
    LrWpanCsmaCaSwNoba::COLLISION_COUNT[m_TP]++;
    m_collisions++;
    m_csmaCaCollisionTrace(m_TP, m_collisions);

    if(LrWpanCsmaCaSwNoba::COLLISION_COUNT[m_TP] == 0)
    {
        LrWpanCsmaCaSwNoba::SW[m_TP] = 1;
    }
    else
    {
        LrWpanCsmaCaSwNoba::SW[m_TP] =
            pow(2, COLLISION_COUNT[m_TP])
            - std::min(round(std::tgamma(COLLISION_COUNT[m_TP] + 1)), pow(2, COLLISION_COUNT[m_TP]))
        ;
    }
    this->AdjustCW();

    if(LrWpanCsmaCaSwNoba::CW[m_TP].second > LrWpanCsmaCaSwNoba::WL[m_TP]) {
        m_backoffCount = m_random->GetInteger(LrWpanCsmaCaSwNoba::CW[m_TP].first, LrWpanCsmaCaSwNoba::WL[m_TP]);
    }
    else {
        m_backoffCount = m_random->GetInteger(LrWpanCsmaCaSwNoba::CW[m_TP].first, LrWpanCsmaCaSwNoba::CW[m_TP].second);
    }

    NS_LOG_DEBUG("MODIFIED backoff count is: " << m_backoffCount);
}

void
LrWpanCsmaCaSwNoba::AdjustSW()
{
    // adjust SW if three success TX occured
    if(COLLISION_COUNT[m_TP] == 0)
    {
        LrWpanCsmaCaSwNoba::SW[m_TP] = 1;
        return;
    }
    LrWpanCsmaCaSwNoba::SW[m_TP] =
        pow(2, COLLISION_COUNT[m_TP]) - (uint32_t) std::round(std::tgamma(COLLISION_COUNT[m_TP] - 1 + 1));
}

void
LrWpanCsmaCaSwNoba::AdjustCW()
{
    for (int i = m_TP - 1; i >= 0; i--)  // Adjust lower TPs
    {
        CW[i].first = CW[i + 1].second + 1;
        CW[i].second = std::min(CW[i].first + SW[i], WL[i]);
    }
}

} // namespace lrwpan
} // namespace ns3
