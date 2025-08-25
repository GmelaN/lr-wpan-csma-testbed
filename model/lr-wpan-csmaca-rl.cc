/*
 * Copyright (c) 2025 jshyeon
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */

#include "lr-wpan-csmaca-rl.h"
#include "lr-wpan-constants.h"

#include <ns3/log.h>
#include <ns3/random-variable-stream.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <vector>
#include <cmath>

#define TP_COUNT 8

#define LEARNING_RATE 0.2
#define DISCOUNT_FACTOR 0.99

#define DECREASE 0
#define KEEP 1
#define INCREASE 2


/*
#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT                                                                      \
     std::clog << "[" << m_mac->GetShortAddress() << "] ";
*/
namespace ns3
{
namespace lrwpan
{

NS_LOG_COMPONENT_DEFINE("LrWpanCsmaCaRl");
NS_OBJECT_ENSURE_REGISTERED(LrWpanCsmaCaRl);


std::pair<uint32_t, uint32_t> LrWpanCsmaCaRl::CW[TP_COUNT]; // each TP


uint32_t LrWpanCsmaCaRl::TP_M[TP_COUNT] = {6, 6, 7, 7, 8, 8, 9, 10}; // each TP
uint32_t LrWpanCsmaCaRl::TP_K[TP_COUNT] = {10, 10, 10, 10, 10, 10, 10, 10}; // each TP

std::vector<double[3]> LrWpanCsmaCaRl::q_table[TP_COUNT];

TypeId
LrWpanCsmaCaRl::GetTypeId()
{
    static TypeId tid = TypeId("ns3::lrwpan::LrWpanCsmaCaRl")
                            .AddDeprecatedName("ns3::LrWpanCsmaCaRl")
                            .SetParent<LrWpanCsmaCaCommon>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<LrWpanCsmaCaRl>()
                            .AddTraceSource("csmaCaRlCollisionTrace",
                                            "CSMA/CA RL collision count trace",
                                            MakeTraceSourceAccessor(&LrWpanCsmaCaRl::m_csmaCaRlCollisionTrace),
                                            "ns3::TracedCallback")
                            .AddTraceSource("csmaCaRlMKViolationTrace",
                                        "CSMA/CA RL m, k violation trace",
                                        MakeTraceSourceAccessor(&LrWpanCsmaCaRl::m_csmaCaRlMKViolationTrace),
                                        "ns3::TracedCallback")
;

    return tid;
}


LrWpanCsmaCaRl::LrWpanCsmaCaRl(uint8_t priority)
{
    NS_ASSERT(priority >= 0 && priority <= 7);

    CW[0].first = 16; CW[1].first = 16;
    CW[2].first = 8;  CW[3].first = 8;
    CW[4].first = 4;  CW[5].first = 4;
    CW[6].first = 2;  CW[7].first = 1;

    CW[0].second = 64; CW[1].second = 32;
    CW[2].second = 32; CW[3].second = 16;
    CW[4].second = 16; CW[5].second = 8;
    CW[6].second = 8;  CW[7].second = 4;

    // Initialize Q-table only once
    if (q_table[0].empty())
    {
        for (size_t up = 0; up < TP_COUNT; up++)
        {
            for (size_t i = CW[up].first; i <= CW[up].second; i++)
            {
                double q_dec = 0.0;
                double q_keep = 0.0;
                double q_inc = 0.0;
                if (i == CW[up].first)
                    q_dec = -100.0; // Cannot decrease further
                if (i == CW[up].second)
                    q_inc = -100.0; // Cannot increase further
                q_table[up].push_back({q_dec, q_keep, q_inc});
            }
        }
    }
    m_backoffCount = CW[priority].first;

    m_isSlotted = true;
    m_macBattLifeExt = false;
    m_random = CreateObject<UniformRandomVariable>();
    m_ccaRequestRunning = false;
    m_randomBackoffPeriodsLeft = 0;
    m_coorDest = false;

    m_TP = priority;
    m_collisions = 0;
    m_freezeBackoff = false;
    m_previousBackoffCount = m_backoffCount;

    m_action = KEEP;

    for (uint32_t i = 0; i < TP_K[m_TP]; i++)
    {
        m_resultQueue.push_back(true);
    }
}

LrWpanCsmaCaRl::LrWpanCsmaCaRl()
{
    NS_ASSERT_MSG(false, "nodeCount, priority missing.");
}

LrWpanCsmaCaRl::~LrWpanCsmaCaRl()
{
    m_mac = nullptr;
}

void
LrWpanCsmaCaRl::DoDispose()
{
    m_lrWpanMacStateCallback = MakeNullCallback<void, MacState>();
    m_lrWpanMacTransCostCallback = MakeNullCallback<void, uint32_t>();

    Cancel();
    m_mac = nullptr;
}

void
LrWpanCsmaCaRl::SetMac(Ptr<LrWpanMac> mac)
{
    m_mac = mac;
}

Ptr<LrWpanMac>
LrWpanCsmaCaRl::GetMac()
{
    return m_mac;
}

void
LrWpanCsmaCaRl::SetSlottedCsmaCa()
{
    m_isSlotted = true;
}

void
LrWpanCsmaCaRl::SetUnSlottedCsmaCa()
{
    NS_ASSERT_MSG(false, "cannot set unslotted CSMA/CA RL.");
}

bool
LrWpanCsmaCaRl::IsSlottedCsmaCa()
{
    return m_isSlotted;
}

bool
LrWpanCsmaCaRl::IsUnSlottedCsmaCa()
{
    return !m_isSlotted;
}

Time
LrWpanCsmaCaRl::GetTimeToNextSlot() const
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
LrWpanCsmaCaRl::Start()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(m_isSlotted, "only slotted CSMA-CA supported.");

    m_collisions = 0; // collision counter C

    m_previousBackoffCount = m_backoffCount;

    // The state is the current backoff counter.
    // We choose an action based on the Q-table for the current state.
    uint32_t state_index = m_previousBackoffCount - CW[m_TP].first;
    auto& q_values = q_table[m_TP][state_index];

    // Find the maximum Q-value.
    double max_q = *std::max_element(std::begin(q_values), std::end(q_values));

    // Find all actions with the maximum Q-value (to handle ties).
    std::vector<uint8_t> best_actions;
    for (uint8_t i = 0; i < 3; ++i)
    {
        // Use a small tolerance for floating point comparison
        if (std::abs(q_values[i] - max_q) < 1e-9)
        {
            best_actions.push_back(i);
        }
    }
    uint32_t random_index = m_random->GetInteger(0, best_actions.size() - 1);
    m_action = best_actions[random_index];

    // Update backoff counter based on the chosen action.
    if (m_action == DECREASE)
    {
        if (m_backoffCount > CW[m_TP].first)
        {
            m_backoffCount--;
        }
    }
    else if (m_action == INCREASE)
    {
        if (m_backoffCount < CW[m_TP].second)
        {
            m_backoffCount++;
        }
    }
    // If action is KEEP, m_backoffCount remains unchanged.

    NS_LOG_DEBUG("Using CSMA-CA RL, bakcoff count is: " << m_backoffCount);
    m_backoff = m_backoffCount;

    // m_coorDest to decide between incoming and outgoing superframes times
    m_coorDest = m_mac->IsCoordDest();

    // Locate backoff period boundary. (i.e. a time delay to align with the next backoff period
    // boundary)
    Time backoffBoundary = GetTimeToNextSlot();
    m_randomBackoffEvent =
        Simulator::Schedule(backoffBoundary, &LrWpanCsmaCaRl::RandomBackoffDelay, this);
}

void
LrWpanCsmaCaRl::Cancel()
{
    m_randomBackoffEvent.Cancel();
    m_requestCcaEvent.Cancel();
    m_canProceedEvent.Cancel();
    if(m_mac)
    {
        m_mac->GetPhy()->CcaCancel();
    }
}

void
LrWpanCsmaCaRl::RandomBackoffDelay()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(m_isSlotted, "only slotted CSMA/CA is supported.");

    Time randomBackoff;
    uint64_t symbolRate;
    Time timeLeftInCap;

    symbolRate = (uint64_t)m_mac->GetPhy()->GetDataOrSymbolRate(false); // symbols per second

    // We should not recalculate the random backoffPeriods if we are in a slotted CSMA-CA and the
    // transmission was previously deferred (m_randomBackoffPeriods != 0) or ACK not received
    // if (m_backoff == 0 || m_freezeBackoff)
    // {
    //     m_backoff = m_random->GetInteger(CW[m_TP].first, CW[m_TP].second);

    // }

    randomBackoff =
        Seconds((double)(m_backoff * lrwpan::aUnitBackoffPeriod) / symbolRate);

        // We must make sure there is enough time left in the CAP, otherwise we continue in
        // the CAP of the next superframe after the transmission/reception of the beacon (and the
        // IFS)
        timeLeftInCap = GetTimeLeftInCap();

        NS_LOG_DEBUG("CSMA/CA-RL: proceeding after random backoff of "
                     << m_backoff << " periods ("
                     << (randomBackoff.GetSeconds() * symbolRate) << " symbols or "
                     << randomBackoff.As(Time::S) << ")");

        NS_LOG_DEBUG("Backoff periods left in CAP: "
                     << ((timeLeftInCap.GetSeconds() * symbolRate) / lrwpan::aUnitBackoffPeriod)
                     << " (" << (timeLeftInCap.GetSeconds() * symbolRate) << " symbols or "
                     << timeLeftInCap.As(Time::S) << ")");

        if (randomBackoff >= timeLeftInCap)
        {
            // std::cout << (int) m_TP <<  ": PACKET DEFERRED" << std::endl;

            uint32_t usedBackoffs =
                (double)(timeLeftInCap.GetSeconds() * symbolRate) / lrwpan::aUnitBackoffPeriod;
            m_backoff -= usedBackoffs;
            NS_LOG_DEBUG("No time in CAP to complete backoff delay, deferring to the next CAP");
            if (timeLeftInCap < Seconds(0))
            {
                m_endCapEvent =
                    Simulator::ScheduleNow(&LrWpanCsmaCaRl::DeferCsmaTimeout, this);
                return;
            }
        }
        else
        {
            m_canProceedEvent = Simulator::Schedule(randomBackoff, &LrWpanCsmaCaRl::CanProceed, this);
        }
}

void
LrWpanCsmaCaRl::TxSucceed()
{
    // update (m, k) queue
    NS_ASSERT(m_resultQueue.size() == TP_K[m_TP]);
    m_resultQueue.pop_front();
    m_resultQueue.push_back(true);
    NS_ASSERT(m_resultQueue.size() == TP_K[m_TP]);

    // update Q-table
    double reward = 1.0;
    uint32_t old_state_index = m_backoffCount - CW[m_TP].first;
    auto& old_q_values = q_table[m_TP][old_state_index];

    // calculate max Q value of next state.
    double max_q_decrease = -std::numeric_limits<double>::infinity();
    if (old_state_index > 0) {
        auto& q_vals_decrease = q_table[m_TP][old_state_index - 1];
        max_q_decrease = std::max_element(q_vals_decrease[0], q_vals_decrease[1], q_vals_decrease[2]);
    }

    auto& q_vals_keep = q_table[m_TP][old_state_index];
    double max_q_keep = std::max_element(q_vals_keep[0], q_vals_keep[1], q_vals_keep[2]);

    double max_q_increase = -std::numeric_limits<double>::infinity();
    if (old_state_index + 1 < q_table[m_TP].size()) { 
        auto& q_vals_increase = q_table[m_TP][old_state_index + 1];
        max_q_increase = std::max_element(q_vals_increase[0], q_vals_increase[1], q_vals_increase[2]);
    }

    // choose the biggest q-value.
    double best_future_q = std::max({max_q_decrease, max_q_keep, max_q_increase});

    old_q_values[m_action] =
        (1-LEARNING_RATE) * old_q_values[m_action]
        + LEARNING_RATE * (reward + DISCOUNT_FACTOR * best_future_q);
}

void
LrWpanCsmaCaRl::AckTimeout()
{
    // update (m, k) queue
    NS_ASSERT(m_resultQueue.size() == TP_K[m_TP]);
    m_resultQueue.pop_front();
    m_resultQueue.push_back(false);
    NS_ASSERT(m_resultQueue.size() == TP_K[m_TP]);

    // update Q-table
    double reward = -1.0;
    uint32_t old_state_index = m_backoffCount - CW[m_TP].first;
    auto& old_q_values = q_table[m_TP][old_state_index];

    // calculate max Q value of next state.
    double max_q_decrease = -std::numeric_limits<double>::infinity();
    if (old_state_index > 0) {
        auto& q_vals_decrease = q_table[m_TP][old_state_index - 1];
        max_q_decrease = std::max_element(q_vals_decrease[0], q_vals_decrease[1], q_vals_decrease[2]);
    }

    auto& q_vals_keep = q_table[m_TP][old_state_index];
    double max_q_keep = std::max_element(q_vals_keep[0], q_vals_keep[1], q_vals_keep[2]);

    double max_q_increase = -std::numeric_limits<double>::infinity();
    if (old_state_index + 1 < q_table[m_TP].size()) { 
        auto& q_vals_increase = q_table[m_TP][old_state_index + 1];
        max_q_increase = std::max_element(q_vals_increase[0], q_vals_increase[1], q_vals_increase[2]);
    }

    // choose the biggest q-value.
    double best_future_q = std::max({max_q_decrease, max_q_keep, max_q_increase});

    old_q_values[m_action] =
        (1-LEARNING_RATE) * old_q_values[m_action]
        + LEARNING_RATE * (reward + DISCOUNT_FACTOR * best_future_q);

    // (m, k) rule check
    uint32_t failCount = std::count(m_resultQueue.begin(), m_resultQueue.end(), false);
    if (failCount > TP_K[m_TP] - TP_M[m_TP])
    {
        // (m, k) rule violation detected
        m_csmaCaRlMKViolationTrace(m_TP);
        m_resultQueue.clear();
        m_resultQueue.insert(m_resultQueue.begin(), TP_K[m_TP], true);  // 전부 meet 처리
    }
}



Time
LrWpanCsmaCaRl::GetTimeLeftInCap()
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
LrWpanCsmaCaRl::CanProceed()
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

        m_endCapEvent = Simulator::Schedule(timeLeftInCap, &LrWpanCsmaCaRl::DeferCsmaTimeout, this);
    }
    else
    {
        m_requestCcaEvent = Simulator::ScheduleNow(&LrWpanCsmaCaRl::RequestCCA, this);
    }
}

void
LrWpanCsmaCaRl::RequestCCA()
{
    NS_LOG_FUNCTION(this);
    m_ccaRequestRunning = true;
    m_mac->GetPhy()->PlmeCcaRequest();
}

void
LrWpanCsmaCaRl::DeferCsmaTimeout()
{
    NS_LOG_FUNCTION(this);
    m_lrWpanMacStateCallback(MAC_CSMA_DEFERRED);
}

void
LrWpanCsmaCaRl::PlmeCcaConfirm(PhyEnumeration status)
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
                m_requestCcaEvent = Simulator::ScheduleNow(&LrWpanCsmaCaRl::RequestCCA,
                                                            this); // Perform CCA again
            }
        }
        else
        {
            // m_csmaCaRlCollisionTrace(m_TP, m_collisions);
            // freeze backoff counter and retry
            NS_LOG_DEBUG("Perform another backoff; freeze backoff count: " << m_backoffCount);
            m_freezeBackoff = true;
            m_randomBackoffEvent =
                Simulator::ScheduleNow(&LrWpanCsmaCaRl::RandomBackoffDelay, this);
        }
    }
}

void
LrWpanCsmaCaRl::SetLrWpanMacTransCostCallback(LrWpanMacTransCostCallback c)
{
    NS_LOG_FUNCTION(this);
    m_lrWpanMacTransCostCallback = c;
}

void
LrWpanCsmaCaRl::SetLrWpanMacStateCallback(LrWpanMacStateCallback c)
{
    NS_LOG_FUNCTION(this);
    m_lrWpanMacStateCallback = c;
}

void
LrWpanCsmaCaRl::SetBatteryLifeExtension(bool batteryLifeExtension)
{
    m_macBattLifeExt = batteryLifeExtension;
}

int64_t
LrWpanCsmaCaRl::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this);
    m_random->SetStream(stream);
    return 1;
}

uint8_t
LrWpanCsmaCaRl::GetNB()
{
    return m_collisions;
}

bool
LrWpanCsmaCaRl::GetBatteryLifeExtension()
{
    return m_macBattLifeExt;
}

void
LrWpanCsmaCaRl::SetBackoffCounter()
{
    m_collisions++;
    m_csmaCaRlCollisionTrace(m_TP, m_collisions);

    this->AckTimeout();

    // m_previousBackoffCount = m_backoffCount;

    // The state is the current backoff counter.
    // We choose an action based on the Q-table for the current state.
    uint32_t state_index = m_backoffCount - CW[m_TP].first;
    auto& q_values = q_table[m_TP][state_index];

    // Find the maximum Q-value.
    double max_q = *std::max_element(std::begin(q_values), std::end(q_values));

    // Find all actions with the maximum Q-value (to handle ties).
    std::vector<uint8_t> best_actions;
    for (uint8_t i = 0; i < 3; ++i)
    {
        // Use a small tolerance for floating point comparison
        if (std::abs(q_values[i] - max_q) < 1e-9)
        {
            best_actions.push_back(i);
        }
    }
    uint32_t random_index = m_random->GetInteger(0, best_actions.size() - 1);
    m_action = best_actions[random_index];

    // Update backoff counter based on the chosen action.
    if (m_action == DECREASE)
    {
        if (m_backoffCount > CW[m_TP].first)
        {
            m_backoffCount--;
        }
    }
    else if (m_action == INCREASE)
    {
        if (m_backoffCount < CW[m_TP].second)
        {
            m_backoffCount++;
        }
    }
    // If action is KEEP, m_backoffCount remains unchanged.

    NS_LOG_DEBUG("MODIFIED backoff count is: " << m_backoffCount);
    m_backoff = m_backoffCount;
}


} // namespace lrwpan
} // namespace ns3
