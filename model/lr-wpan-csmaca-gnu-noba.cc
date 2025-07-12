/*
 * Copyright (c) 2025 jshyeon
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */

#include "lr-wpan-csmaca-gnu-noba.h"
#include "lr-wpan-constants.h"

#include <ns3/log.h>
#include <ns3/random-variable-stream.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <bitset>


#define K 5

#define ALPHA_INCREASE_STEP 0.2
#define ALPHA_DECREASE_STEP 0.1
#define MIN_ALPHA 0.8
#define MAX_ALPHA 1.7

#define WINDOW_COUNT 5

/*
#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT                                                                      \
     std::clog << "[" << m_mac->GetShortAddress() << "] ";
*/

namespace ns3
{
namespace lrwpan
{

NS_LOG_COMPONENT_DEFINE("LrWpanCsmaCaGnuNoba");
NS_OBJECT_ENSURE_REGISTERED(LrWpanCsmaCaGnuNoba);


uint32_t LrWpanCsmaCaGnuNoba::SW[TP_COUNT]; // each TP
std::pair<uint32_t, uint32_t> LrWpanCsmaCaGnuNoba::CW[TP_COUNT]; // each TP
uint32_t LrWpanCsmaCaGnuNoba::WL[TP_COUNT] = {64, 56, 48, 40, 32, 24, 16, 10}; // each TP
uint32_t LrWpanCsmaCaGnuNoba::SUCCESS_COUNT[TP_COUNT] = {0, }; // each TP
std::deque<uint32_t> LrWpanCsmaCaGnuNoba::SUCCESS_WINDOW[TP_COUNT];

uint32_t LrWpanCsmaCaGnuNoba::TP_M[TP_COUNT] = {6, 6, 7, 7, 8, 8, 9, 10}; // each TP
uint32_t LrWpanCsmaCaGnuNoba::TP_K[TP_COUNT] = {10, 10, 10, 10, 10, 10, 10, 10}; // each TP


TypeId
LrWpanCsmaCaGnuNoba::GetTypeId()
{
    static TypeId tid = TypeId("ns3::lrwpan::LrWpanCsmaCaGnuNoba")
                            .AddDeprecatedName("ns3::LrWpanCsmaCaGnuNoba")
                            .SetParent<LrWpanCsmaCaCommon>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<LrWpanCsmaCaGnuNoba>()
                            .AddTraceSource("csmaCaGnuNobaCollisionTrace",
                                            "CSMA/CA GNU-NOBA collision count trace",
                                            MakeTraceSourceAccessor(&LrWpanCsmaCaGnuNoba::m_csmaCaGnuNobaCollisionTrace),
                                            "ns3::TracedCallback")
                            .AddTraceSource("csmaCaGnuNobaMKViolationTrace",
                                            "CSMA/CA ",
                                            MakeTraceSourceAccessor(&LrWpanCsmaCaGnuNoba::m_csmaCaGnuNobaMKViolationTrace),
                                            "ns3::TracedCallback")
                            ;
    return tid;
}

void
LrWpanCsmaCaGnuNoba::InitializeAggregations()
{
    // In context just before new beacon start,
    // we have to initialize aggregation variables.

    for (uint32_t i = 0; i < TP_COUNT; i++)
    {
        SUCCESS_COUNT[i] = 0;
    }
}
void
LrWpanCsmaCaGnuNoba::CalculateCWRanges()
{
    // In context just before new beacon start,
    // we have to calculate and deploy new CW range.

    // calculate deltas for each TP first.
    for(int tp = 0; tp < TP_COUNT; tp++)
    {
        // calcuate average success count
        uint64_t average = 0;
        int delta = 0;
        for(auto i = SUCCESS_WINDOW[tp].begin(); i < SUCCESS_WINDOW[tp].end(); i++)
        {
            average += (*i);
        }
        average /= static_cast<double>(SUCCESS_WINDOW->size());

        // and get delta value.
        delta = SUCCESS_COUNT[tp] - average;


        // update success window.
        NS_ASSERT(SUCCESS_WINDOW[tp].size() == WINDOW_COUNT);
        SUCCESS_WINDOW[tp].pop_front();
        SUCCESS_WINDOW[tp].push_back(SUCCESS_COUNT[tp]);
        NS_ASSERT(SUCCESS_WINDOW[tp].size() == WINDOW_COUNT);

        // TODO: 구체적인 구간 산정 필요
        // and then control CW range by delta value.
        // n^2 - n (n >= 2)
        // std::cout << delta << std::endl;
        if(delta > 10) // default: 1
        {
            SW[tp] = 1;
        }
        else if(delta > 8) // n = 2
        {
            SW[tp] = 2;
        }
        else if(delta > 4) // n = 3
        {
            SW[tp] = 6;
        }
        else if(delta > 2) // n = 4
        {
            SW[tp] = 12;
        }
        else // delta < 0, n = 5
        {
            SW[tp] = 20;
        }
    }
}

void
LrWpanCsmaCaGnuNoba::UpdateCW()
{
    CalculateCWRanges();
    // In context of before transmit first beacon, after adjust SW by aggregated statistics.
    // update CW ranges by NOBA-like method, CW + SW.

    CW[TP_COUNT-1].first = 1;
    CW[TP_COUNT-1].second = std::min(CW[TP_COUNT-1].first + SW[TP_COUNT-1], WL[TP_COUNT-1]);

    for (int i = TP_COUNT - 2; i >= 0; i--) // Adjust lower TPs
    {
        CW[i].first = CW[i + 1].second + 1;
        CW[i].second = std::min(CW[i].first + SW[i], WL[i]);
    }

    NS_LOG_DEBUG("CSMA/CA GNU-NOBA: MODIFIED SW, CW: \n"
                 << "SW: " << SW[0] << "\t"
                 << SW[1] << "\t"
                 << SW[2] << "\t"
                 << SW[3] << "\t"
                 << SW[4] << "\t"
                 << SW[5] << "\t"
                 << SW[6] << "\t"
                 << SW[7] << '\n'
                 << "CW: "
                 << "[0]: " << CW[0].first << " ~ " << CW[0].second << "\n"
                 << "[1]: " << CW[1].first << " ~ " << CW[1].second << "\n"
                 << "[2]: " << CW[2].first << " ~ " << CW[2].second << "\n"
                 << "[3]: " << CW[3].first << " ~ " << CW[3].second << "\n"
                 << "[4]: " << CW[4].first << " ~ " << CW[4].second << "\n"
                 << "[5]: " << CW[5].first << " ~ " << CW[5].second << "\n"
                 << "[6]: " << CW[6].first << " ~ " << CW[6].second << "\n"
                 << "[7]: " << CW[7].first << " ~ " << CW[7].second << "\n");

    InitializeAggregations();
}

void
LrWpanCsmaCaGnuNoba::AckTimeout()
{
    // In context of source node transmitted packet and didn't receive ACK.
    // Update (m, k) queue and modify alpha, beta.

    // update (m, k) queue
    NS_ASSERT(m_resultQueue.size() == m_K);
    m_resultQueue.pop_front();
    m_resultQueue.push_back(false);
    NS_ASSERT(m_resultQueue.size() == m_K);

    ModifyAlpha(true);
}

void
LrWpanCsmaCaGnuNoba::SetBackoffCounter()
{
    m_collisions++;
    m_csmaCaGnuNobaCollisionTrace(m_TP, 0);
    m_backoffCount = BetaMappedRandom(m_alpha, m_beta, CW[m_TP].first, CW[m_TP].second);
}

void
LrWpanCsmaCaGnuNoba::TransmissionSucceed()
{
    // in context of sink node successfully received data and sent ACK.
    // we have to include this success transmission to aggregation.
    SUCCESS_COUNT[m_TP]++;

    // update (m, k) queue
    NS_ASSERT(m_resultQueue.size() == m_K);
    m_resultQueue.pop_front();
    m_resultQueue.push_back(true);
    NS_ASSERT(m_resultQueue.size() == m_K);

    ModifyAlpha(false);
}

void
LrWpanCsmaCaGnuNoba::ModifyAlpha(bool isFailure)
{
    // modify alpha and beta according to DBP.
    uint32_t meetCount = 0;
    int l = m_resultQueue.size();  // initial value: last index.
    int k = m_resultQueue.size();
    int distBasedPriority = 0;
    for (int i = k - 1; i >= 0; --i)
    {
        if (m_resultQueue[i]) {
            meetCount++;
            if (meetCount == TP_M[m_TP]) {
                l = i;  // m번째 meet의 위치
                break;
            }
        }
    }

    if (meetCount < TP_M[m_TP]) // worst case.
    {
        distBasedPriority = k + 1;
    }
    else
    {
        distBasedPriority = k - l + 1;
    }

    uint32_t failCount = std::count(m_resultQueue.begin(), m_resultQueue.end(), false);
    if (failCount > TP_K[m_TP] - TP_M[m_TP])
    {
        NS_ASSERT(isFailure);
        // (m, k) rule violation detected
        m_csmaCaGnuNobaMKViolationTrace(m_TP);
        m_alpha = MIN_ALPHA;
        m_resultQueue.clear();
        m_resultQueue.insert(m_resultQueue.begin(), TP_K[m_TP], true);  // 전부 meet 처리
    }
    // else
    // {
    //     double alpha = 1.7 - (distBasedPriority * distBasedPriority - distBasedPriority) * 0.1;
    //     m_alpha = std::max(MIN_ALPHA, std::min(alpha, MAX_ALPHA));
    // }

    // 계산: 현재 DBP
    double decayFactor = (distBasedPriority * distBasedPriority - distBasedPriority);

    // 조절 강도 변화: DBP 클수록 감소 효과 줄고, 작을수록 크다
    double alphaDecay = decayFactor * 0.12;   // aggressive decay
    double alphaBase = 1.65;

    // soft clipping 증가
    double alpha = alphaBase - alphaDecay;

    // soft 상승 제어: 안정 시 천천히
    if (m_alpha < alpha) {
        // 증가 시에는 매우 느리게
        m_alpha = std::min(m_alpha + 0.02, alpha);
    } else {
        // 감소는 바로 반영
        m_alpha = alpha;
    }

    // 범위 제한
    m_alpha = std::max(MIN_ALPHA, std::min(m_alpha, MAX_ALPHA));



}

LrWpanCsmaCaGnuNoba::LrWpanCsmaCaGnuNoba(uint8_t priority)
{
    NS_ASSERT(priority <= 7);
    
    m_alpha = 1.7;

    // TODO: 최초 성공 카운트 설정

    // EACH NODE: initialize m, k model.
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

    // M, K model initialization
    m_M = TP_M[m_TP];
    m_K = TP_K[m_TP];
    NS_LOG_DEBUG("LR-WPAN GNU-NOBA: M, K = " << m_M << ",\t" << m_K);
    for (uint32_t i = 0; i < m_K; i++)
    {
        m_resultQueue.push_back(true);
    }


    // COORDINATION: initialize success count
    for(int i = 0; i < TP_COUNT; i++) { SUCCESS_COUNT[i] = 0; }

    // COORDINATION: initialize SW, CW ranges
    for(int i = 0; i < TP_COUNT; i++)
    {
        SW[i] = 1;

        if (SUCCESS_WINDOW[i].empty())
        {
            for (int j = 0; j < WINDOW_COUNT; j++)
            {
                SUCCESS_WINDOW[i].push_back(9999);
            }
        }
        NS_ASSERT(SUCCESS_WINDOW[i].size() == WINDOW_COUNT);
    }


    CW[TP_COUNT-1].first = 1;
    CW[TP_COUNT-1].second = std::min(CW[TP_COUNT-1].first + SW[TP_COUNT-1], WL[TP_COUNT-1]);
    for (int i = TP_COUNT - 2; i >= 0; i--) // Adjust lower TPs
    {
        CW[i].first = CW[i + 1].second + 1;
        CW[i].second = std::min(CW[i].first + SW[i], WL[i]);
    }
}

LrWpanCsmaCaGnuNoba::LrWpanCsmaCaGnuNoba()
{
    NS_ASSERT_MSG(false, "nodeCount, priority missing.");
}

LrWpanCsmaCaGnuNoba::~LrWpanCsmaCaGnuNoba()
{
    m_mac = nullptr;
}

void
LrWpanCsmaCaGnuNoba::DoDispose()
{
    m_lrWpanMacStateCallback = MakeNullCallback<void, MacState>();
    m_lrWpanMacTransCostCallback = MakeNullCallback<void, uint32_t>();

    Cancel();
    m_mac = nullptr;
}

void
LrWpanCsmaCaGnuNoba::SetMac(Ptr<LrWpanMac> mac)
{
    m_mac = mac;
}

Ptr<LrWpanMac>
LrWpanCsmaCaGnuNoba::GetMac()
{
    return m_mac;
}

void
LrWpanCsmaCaGnuNoba::SetSlottedCsmaCa()
{
    m_isSlotted = true;
}

void
LrWpanCsmaCaGnuNoba::SetUnSlottedCsmaCa()
{
    NS_ASSERT_MSG(false, "cannot set unslotted CSMA/CA NOBA.");
}

bool
LrWpanCsmaCaGnuNoba::IsSlottedCsmaCa()
{
    return m_isSlotted;
}

bool
LrWpanCsmaCaGnuNoba::IsUnSlottedCsmaCa()
{
    return !m_isSlotted;
}

Time
LrWpanCsmaCaGnuNoba::GetTimeToNextSlot() const
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
LrWpanCsmaCaGnuNoba::Start()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(m_isSlotted, "only slotted CSMA-CA supported.");

    m_collisions = 0; // collision counter C

    m_backoffCount = BetaMappedRandom(m_alpha, m_beta, CW[m_TP].first, CW[m_TP].second);
    // m_backoffCount = m_random->GetInteger(1 + CW[m_TP].first, CW[m_TP].second); // backoff counter B
    NS_LOG_DEBUG("Using CSMA CA GNU-NOBA, bakcoff count is: " << m_backoffCount);

    // m_coorDest to decide between incoming and outgoing superframes times
    m_coorDest = m_mac->IsCoordDest();

    // Locate backoff period boundary. (i.e. a time delay to align with the next backoff period
    // boundary)
    Time backoffBoundary = GetTimeToNextSlot();
    m_randomBackoffEvent =
        Simulator::Schedule(backoffBoundary, &LrWpanCsmaCaGnuNoba::RandomBackoffDelay, this);
}

void
LrWpanCsmaCaGnuNoba::Cancel()
{
    m_randomBackoffEvent.Cancel();
    m_requestCcaEvent.Cancel();
    m_canProceedEvent.Cancel();
    if(m_mac) {
        m_mac->GetPhy()->CcaCancel();
    }
}

void
LrWpanCsmaCaGnuNoba::RandomBackoffDelay()
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
        m_backoffCount = BetaMappedRandom(m_alpha, m_beta, CW[m_TP].first, CW[m_TP].second);
        // m_backoffCount = m_random->GetInteger(CW[m_TP].first, CW[m_TP].second);
    }

    randomBackoff =
        Seconds(static_cast<double>(m_backoffCount * lrwpan::aUnitBackoffPeriod) / symbolRate);

        // We must make sure there is enough time left in the CAP, otherwise we continue in
        // the CAP of the next superframe after the transmission/reception of the beacon (and the
        // IFS)
        timeLeftInCap = GetTimeLeftInCap();

        NS_LOG_DEBUG("CSMA/CA GNU-NOBA: proceeding after random backoff of "
                     << m_backoffCount << " periods ("
                     << (randomBackoff.GetSeconds() * symbolRate) << " symbols or "
                     << randomBackoff.As(Time::S) << ")");

        NS_LOG_DEBUG("Backoff periods left in CAP: "
                     << ((timeLeftInCap.GetSeconds() * symbolRate) / lrwpan::aUnitBackoffPeriod)
                     << " (" << (timeLeftInCap.GetSeconds() * symbolRate) << " symbols or "
                     << timeLeftInCap.As(Time::S) << ")");

        if (randomBackoff >= timeLeftInCap)
        {
            std::cout << (int) m_TP <<  ": PACKET DEFERRED" << std::endl;
            uint32_t usedBackoffs =
                (double)(timeLeftInCap.GetSeconds() * symbolRate) / lrwpan::aUnitBackoffPeriod;
            m_backoffCount -= usedBackoffs;
            NS_LOG_DEBUG("No time in CAP to complete backoff delay, deferring to the next CAP");
            m_endCapEvent =
                Simulator::Schedule(timeLeftInCap, &LrWpanCsmaCaGnuNoba::DeferCsmaTimeout, this);
        }
        else
        {
            m_canProceedEvent = Simulator::Schedule(randomBackoff, &LrWpanCsmaCaGnuNoba::CanProceed, this);
        }
}

Time
LrWpanCsmaCaGnuNoba::GetTimeLeftInCap()
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
LrWpanCsmaCaGnuNoba::CanProceed()
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

        m_endCapEvent = Simulator::Schedule(timeLeftInCap, &LrWpanCsmaCaGnuNoba::DeferCsmaTimeout, this);
    }
    else
    {
        m_requestCcaEvent = Simulator::ScheduleNow(&LrWpanCsmaCaGnuNoba::RequestCCA, this);
    }
}

void
LrWpanCsmaCaGnuNoba::RequestCCA()
{
    NS_LOG_FUNCTION(this);
    m_ccaRequestRunning = true;
    m_mac->GetPhy()->PlmeCcaRequest();
}

void
LrWpanCsmaCaGnuNoba::DeferCsmaTimeout()
{
    NS_LOG_FUNCTION(this);
    m_lrWpanMacStateCallback(MAC_CSMA_DEFERRED);
}

void
LrWpanCsmaCaGnuNoba::PlmeCcaConfirm(PhyEnumeration status)
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
                    NS_LOG_DEBUG("Notifying MAC of idle channel");
                    m_lrWpanMacStateCallback(CHANNEL_IDLE);
                }
            }
            else
            {
                NS_LOG_DEBUG("Perform CCA again, backoff count = " << m_backoffCount);
                m_requestCcaEvent = Simulator::ScheduleNow(&LrWpanCsmaCaGnuNoba::RequestCCA,
                                                            this); // Perform CCA again
            }
        }
        else
        {
            // m_csmaCaSwprNobaCollisionTrace(m_TP, m_collisions);
            // freeze backoff counter and retry
            NS_LOG_DEBUG("Perform another backoff; freeze backoff count: " << m_backoffCount);
            m_freezeBackoff = true;
            m_randomBackoffEvent =
                Simulator::ScheduleNow(&LrWpanCsmaCaGnuNoba::RandomBackoffDelay, this);
        }
    }
}

void
LrWpanCsmaCaGnuNoba::SetLrWpanMacTransCostCallback(LrWpanMacTransCostCallback c)
{
    NS_LOG_FUNCTION(this);
    m_lrWpanMacTransCostCallback = c;
}

void
LrWpanCsmaCaGnuNoba::SetLrWpanMacStateCallback(LrWpanMacStateCallback c)
{
    NS_LOG_FUNCTION(this);
    m_lrWpanMacStateCallback = c;
}

void
LrWpanCsmaCaGnuNoba::SetBatteryLifeExtension(bool batteryLifeExtension)
{
    m_macBattLifeExt = batteryLifeExtension;
}

int64_t
LrWpanCsmaCaGnuNoba::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this);
    m_random->SetStream(stream);
    return 1;
}

uint8_t
LrWpanCsmaCaGnuNoba::GetNB()
{
    return m_collisions;
}

bool
LrWpanCsmaCaGnuNoba::GetBatteryLifeExtension()
{
    return m_macBattLifeExt;
}

uint32_t
LrWpanCsmaCaGnuNoba::BetaMappedRandom(const double alpha, const double beta, uint32_t x, uint32_t y)
{
    Ptr<GammaRandomVariable> gammaAlpha = CreateObject<GammaRandomVariable>();
    Ptr<GammaRandomVariable> gammaBeta = CreateObject<GammaRandomVariable>();

    gammaAlpha->SetAttribute("Alpha", DoubleValue(alpha));
    gammaAlpha->SetAttribute("Beta", DoubleValue(1.0));

    gammaBeta->SetAttribute("Alpha", DoubleValue(beta));
    gammaBeta->SetAttribute("Beta", DoubleValue(1.0));

    double a = gammaAlpha->GetValue();
    double b = gammaBeta->GetValue();

    double z = a / (a + b); // in [0,1]

    return static_cast<uint32_t>(x + (y - x) * z); // map to [x,y]
}

} // namespace lrwpan
} // namespace ns3
