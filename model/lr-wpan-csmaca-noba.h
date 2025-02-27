/*
 * Copyright (c) 2011 The Boeing Company
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  kwong yin <kwong-sang.yin@boeing.com>
 *  Sascha Alexander Jopen <jopen@cs.uni-bonn.de>
 *  Alberto Gallegos Ramonet <ramonet@fc.ritsumei.ac.jp>
 */

#ifndef LR_WPAN_CSMACA_NOBA_H
#define LR_WPAN_CSMACA_NOBA_H

#include "lr-wpan-mac.h"
#include "lr-wpan-csmaca.h"

#include <ns3/event-id.h>
#include <ns3/object.h>

namespace ns3
{

class UniformRandomVariable;

namespace lrwpan
{

/**
 * \ingroup lr-wpan
 *
 * This method informs the MAC whether the channel is idle or busy.
 */
typedef Callback<void, MacState> LrWpanMacStateCallback;
/**
 * \ingroup lr-wpan
 *
 * This method informs the transaction cost in a slotted CSMA-CA data transmission.
 * i.e. Reports number of symbols (time) it would take slotted CSMA-CA to process the current
 * transaction. 1 Transaction = 2 CCA + frame transmission (PPDU) + turnaroudtime or Ack time
 * (optional) + IFS See IEEE 802.15.4-2011 (Sections 5.1.1.1 and 5.1.1.4)
 */
typedef Callback<void, uint32_t> LrWpanMacTransCostCallback;

/**
 * \ingroup lr-wpan
 *
 * This class is a helper for the LrWpanMac to manage the Csma/CA
 * state machine according to IEEE 802.15.4-2006, section 7.5.1.4.
 */
class LrWpanCsmaCaNoba : public LrWpanCsmaCa
{
  public:
    LrWpanCsmaCaNoba();
    ~LrWpanCsmaCaNoba() override = default;


    void SetTP(uint8_t TP)
    {
      m_TP = TP;
    }

    uint8_t GetTP()
    {
      return m_TP;
    }

  
  private:
    void DoDispose() override;
    /**
     * \brief Get the time left in the CAP portion of the Outgoing or Incoming superframe.
     * \return the time left in the CAP
     */
    Time GetTimeLeftInCap();
    /**
     * The callback to inform the cost of a transaction in slotted CSMA-CA.
     */
    LrWpanMacTransCostCallback m_lrWpanMacTransCostCallback;
    /**
     * The callback to inform the configured MAC of the CSMA/CA result.
     */
    LrWpanMacStateCallback m_lrWpanMacStateCallback;
    /**
     * Beacon-enabled slotted or nonbeacon-enabled unslotted CSMA-CA.
     */
    bool m_isSlotted;
    /**
     * The MAC instance for which this CSMA/CA implementation is configured.
     */
    Ptr<LrWpanMac> m_mac;
    /**
     * Number of backoffs for the current transmission.
     */
    uint8_t m_NB;
    /**
     * Traffic Priority
     */
    uint8_t m_TP;
    /**
     * Contention window length (used in slotted ver only).
     */
    uint8_t m_CW;
    /**
     * Backoff exponent.
     */
    uint8_t m_BE;
    /**
     * Battery Life Extension.
     */
    bool m_macBattLifeExt;
    /**
     * Minimum backoff exponent. 0 - macMaxBE, default 3
     */
    uint8_t m_macMinBE;
    /**
     * Maximum backoff exponent. 3 - 8, default 5
     */
    uint8_t m_macMaxBE;
    /**
     * Maximum number of backoffs. 0 - 5, default 4
     */
    uint8_t m_macMaxCSMABackoffs;
    /**
     * Count the number of remaining random backoff periods left to delay.
     */
    uint64_t m_randomBackoffPeriodsLeft;
    /**
     * Uniform random variable stream.
     */
    Ptr<UniformRandomVariable> m_random;
    /**
     * Scheduler event for the start of the next random backoff/slot.
     */
    EventId m_randomBackoffEvent;
    /**
     * Scheduler event for the end of the current CAP
     */
    EventId m_endCapEvent;
    /**
     * Scheduler event when to start the CCA after a random backoff.
     */
    EventId m_requestCcaEvent;
    /**
     * Scheduler event for checking if we can complete the transmission before the
     * end of the CAP.
     */
    EventId m_canProceedEvent;
    /**
     * Flag indicating that the PHY is currently running a CCA. Used to prevent
     * reporting the channel status to the MAC while canceling the CSMA algorithm.
     */
    bool m_ccaRequestRunning;
    /**
     * Indicates whether the CSMA procedure is targeted for a message to be sent to the coordinator.
     * Used to run slotted CSMA/CA on the incoming or outgoing superframe
     * according to the target.
     */
    bool m_coorDest;
};

} // namespace lrwpan
} // namespace ns3

// namespace ns-3

#endif /* LR_WPAN_CSMACA_H */
