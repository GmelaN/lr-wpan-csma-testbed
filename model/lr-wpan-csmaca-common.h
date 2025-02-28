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

#ifndef LR_WPAN_CSMACA_COMMON_H
#define LR_WPAN_CSMACA_COMMON_H

#include "lr-wpan-mac.h"

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
class LrWpanCsmaCaCommon : public Object
{
  public:
    /**
     * Get the type ID.
     *
     * \return the object TypeId
     */
    static TypeId GetTypeId();
    /**
     * Default constructor.
     */
    // LrWpanCsmaCaCommon();
    ~LrWpanCsmaCaCommon() override;


    /**
     * Set the MAC to which this CSMA/CA implementation is attached to.
     *
     * \param mac the used MAC
     */
    virtual void SetMac(Ptr<LrWpanMac> mac) = 0;
    /**
     * Get the MAC to which this CSMA/CA implementation is attached to.
     *
     * \return the used MAC
     */
    virtual Ptr<LrWpanMac> GetMac() = 0; // const

    /**
     * Configure for the use of the slotted CSMA/CA version.
     */
    virtual void SetSlottedCsmaCa() = 0;
    /**
     * Configure for the use of the unslotted CSMA/CA version.
     */
    virtual void SetUnSlottedCsmaCa() = 0;
    /**
     * Check if the slotted CSMA/CA version is being used.
     *
     * \return true, if slotted CSMA/CA is used, false otherwise.
     */
    virtual bool IsSlottedCsmaCa() = 0; // const
    /**
     * Check if the unslotted CSMA/CA version is being used.
     *
     * \return true, if unslotted CSMA/CA is used, false otherwise.
     */
    virtual bool IsUnSlottedCsmaCa() = 0; // const
    /**
     * Start CSMA-CA algorithm (step 1), initialize NB, BE for both slotted and unslotted
     * CSMA-CA. For slotted CSMA-CA initializes CW and starts the backoff slot count.
     */
    virtual void Start() = 0;
    /**
     * Cancel CSMA-CA algorithm.
     */
    virtual void Cancel() = 0;
    /**
     * IEEE 802.15.4-2006 section 6.2.2.2
     * PLME-CCA.confirm status
     * \param status TRX_OFF, BUSY or IDLE
     *
     * When Phy has completed CCA, it calls back here which in turn execute the final steps
     * of the CSMA-CA algorithm.
     * It checks to see if the Channel is idle, if so check the Contention window  before
     * permitting transmission (step 5). If channel is busy, either backoff and perform CCA again or
     * treat as channel access failure (step 4).
     */
    virtual void PlmeCcaConfirm(PhyEnumeration status) = 0;
    /**
     * Set the value of the Battery Life Extension
     *
     * \param batteryLifeExtension the Battery Life Extension value active or inactive
     */
    virtual void SetBatteryLifeExtension(bool batteryLifeExtension) = 0;
    /**
     * Get the value of the Battery Life Extension
     *
     * \returns  true or false to Battery Life Extension support
     */
    virtual bool GetBatteryLifeExtension() = 0; // const
    /**
     * Get the number of CSMA retries
     *
     * \returns the number of CSMA retries
     */
    virtual uint8_t GetNB() = 0; // const
    /**
     * Assign a fixed random variable stream number to the random variables
     * used by this model.  Return the number of streams that have been assigned.
     *
     * \param stream first stream index to use
     * \return the number of stream indices assigned by this model
     */
    virtual int64_t AssignStreams(int64_t stream) = 0;
    /**
     * Set the callback function to the MAC. Used at the end of a Channel Assessment, as part of the
     * interconnections between the CSMA-CA and the MAC. The callback
     * lets MAC know a channel is either idle or busy.
     *
     * \param macState the mac state callback
     */
    virtual void SetLrWpanMacStateCallback(LrWpanMacStateCallback c) = 0;
  protected:
    virtual void DoDispose() = 0;
    /**
     * \brief Get the time left in the CAP portion of the Outgoing or Incoming superframe.
     * \return the time left in the CAP
     */
    virtual Time GetTimeLeftInCap() = 0;
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
