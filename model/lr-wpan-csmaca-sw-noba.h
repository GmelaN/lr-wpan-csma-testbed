/*
 * Copyright (c) 2025 jshyeon
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */

#ifndef LR_WPAN_CSMACA_SW_NOBA_H
#define LR_WPAN_CSMACA_SW_NOBA_H

#define TP_COUNT 8

#include "lr-wpan-mac.h"
#include "lr-wpan-csmaca-common.h"

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
 * This class is a helper for the LrWpanMac to manage the Csma/CA
 * state machine according to IEEE 802.15.4-2006, section 7.5.1.4.
 */
class LrWpanCsmaCaSwNoba : public LrWpanCsmaCaCommon
{
  static uint32_t SW[TP_COUNT]; // each TP
  static std::pair<uint32_t, uint32_t> CW[TP_COUNT]; // each TP
  static uint32_t WL[TP_COUNT]; // each TP
  static uint32_t TP_M[TP_COUNT]; // each TP
  static uint32_t TP_K[TP_COUNT]; // each TP

  public:
    static uint32_t COLLISION_COUNT[TP_COUNT]; // each TP
    static uint32_t SUCCESS_COUNT[TP_COUNT]; // each TP
    /**
       * Update SWs by given equation when three successful transmission condition.
       */
    void AdjustSW();
    /**
     * @param init bool, true when init situration
     */
    static void InitializeGlobals(bool init);
    /**
    * Get the type ID.
    *
    * \return the object TypeId
    */
    static TypeId GetTypeId();
    /**
    * Default constructor.
    */
    LrWpanCsmaCaSwNoba();
    ~LrWpanCsmaCaSwNoba() override;

    LrWpanCsmaCaSwNoba(uint8_t priority);
    // Delete copy constructor and assignment operator to avoid misuse
    LrWpanCsmaCaSwNoba(const LrWpanCsmaCaSwNoba&) = delete;
    LrWpanCsmaCaSwNoba& operator=(const LrWpanCsmaCaSwNoba&) = delete;
    /**
      * Set the MAC to which this CSMA/CA implementation is attached to.
      *
      * \param mac the used MAC
    */
    void SetMac(Ptr<LrWpanMac> mac) override;
    /**
    * Get the MAC to which this CSMA/CA implementation is attached to.
    *
    * \return the used MAC
    */
    Ptr<LrWpanMac> GetMac() override;
    /**
    * Configure for the use of the slotted CSMA/CA version.
    */
    void SetSlottedCsmaCa() override;
    /**
    * Configure for the use of the unslotted CSMA/CA version.
    */
    void SetUnSlottedCsmaCa() override;
    /**
    * Check if the slotted CSMA/CA version is being used.
    *
    * \return true, if slotted CSMA/CA is used, false otherwise.
    */
    bool IsSlottedCsmaCa() override;
    /**
    * Check if the unslotted CSMA/CA version is being used.
    *
    * \return true, if unslotted CSMA/CA is used, false otherwise.
    */
    bool IsUnSlottedCsmaCa() override;
    /**
    * Set the minimum backoff exponent value.
    * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
    *
    * \param macMinBE the minimum backoff exponent value
    */
    void SetMacMinBE(uint8_t macMinBE);
    /**
    * Get the minimum backoff exponent value.
    * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
    *
    * \return the minimum backoff exponent value
    */
    uint8_t GetMacMinBE() const;
    /**
    * Set the maximum backoff exponent value.
    * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
    *
    * \param macMaxBE the maximum backoff exponent value
    */
    void SetMacMaxBE(uint8_t macMaxBE);
    /**
    * Get the maximum backoff exponent value.
    * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
    *
    * \return the maximum backoff exponent value
    */
    uint8_t GetMacMaxBE() const;
    /**
    * Set the maximum number of backoffs.
    * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
    *
    * \param macMaxCSMABackoffs the maximum number of backoffs
    */
    void SetMacMaxCSMABackoffs(uint8_t macMaxCSMABackoffs);

    /**
    * Get the maximum number of backoffs.
    * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
    *
    * \return the maximum number of backoffs
    */
    uint8_t GetMacMaxCSMABackoffs() const;
    /**
    * Locates the time to the next backoff period boundary in the SUPERFRAME
    * and returns the amount of time left to this moment.
    *
    * \return time offset to the next slot
    */
    Time GetTimeToNextSlot() const;
    /**
    * Start CSMA-CA algorithm (step 1), initialize NB, BE for both slotted and unslotted
    * CSMA-CA. For slotted CSMA-CA initializes CW and starts the backoff slot count.
    */
    void Start() override;
    /**
    * Cancel CSMA-CA algorithm.
    */
    void Cancel() override;
    /**
    * In step 2 of the CSMA-CA, perform a random backoff in the range of 0 to 2^BE -1
    */
    void RandomBackoffDelay();
    /**
    * In the slotted CSMA-CA, after random backoff, determine if the remaining
    * CSMA-CA operation can proceed, i.e. can the entire transactions can be
    * transmitted before the end of the CAP. This step is performed between step
    * 2 and 3. This step is NOT performed for the unslotted CSMA-CA. If it can
    * proceed function RequestCCA() is called.
    */
    void CanProceed();
    /**
    * Request the Phy to perform CCA (Step 3)
    */
    void RequestCCA();
    /**
    * The CSMA algorithm call this function at the end of the CAP to return the MAC state
    * back to to IDLE after a transmission was deferred due to the lack of time in the CAP.
    */
    void DeferCsmaTimeout();
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
    void PlmeCcaConfirm(PhyEnumeration status) override;
    /**
    * Set the callback function to report a transaction cost in slotted CSMA-CA. The callback is
    * triggered in CanProceed() after calculating the transaction cost (2 CCA checks,transmission
    * cost, turnAroundTime, ifs) in the boundary of an Active Period.
    *
    * \param trans the transaction cost callback
    */
    void SetLrWpanMacTransCostCallback(LrWpanMacTransCostCallback trans);
    /**
    * Set the callback function to the MAC. Used at the end of a Channel Assessment, as part of the
    * interconnections between the CSMA-CA and the MAC. The callback
    * lets MAC know a channel is either idle or busy.
    *
    * \param macState the mac state callback
    */
    void SetLrWpanMacStateCallback(LrWpanMacStateCallback macState) override;
    /**
    * Set the value of the Battery Life Extension
    *
    * \param batteryLifeExtension the Battery Life Extension value active or inactive
    */
    void SetBatteryLifeExtension(bool batteryLifeExtension) override;
    /**
    * Assign a fixed random variable stream number to the random variables
    * used by this model.  Return the number of streams that have been assigned.
    *
    * \param stream first stream index to use
    * \return the number of stream indices assigned by this model
    */
    int64_t AssignStreams(int64_t stream) override;
    /**
    * Get the number of CSMA retries
    *
    * \returns the number of CSMA retries
    */
    uint8_t GetNB() override;
    /**
    * Get the value of the Battery Life Extension
    *
    * \returns  true or false to Battery Life Extension support
    */
    bool GetBatteryLifeExtension() override;
    void SetTP(uint8_t TP)
    {
      m_TP = TP;
    }

    uint8_t GetTP()
    {
      return m_TP;
    }

  /**
   * when ACK timeout occured, modify CW, SW and get backoff counter value
   */
  void SetBackoffCounter();

  void AckTimeout();


private:
  /**
   * Traffic Priority
  */
  uint8_t m_TP;
  /**
   * Collision Count
   */
  uint32_t m_collisions;
  /**
   * backoff count
   */
  uint32_t m_backoffCount;
  /**
   * should we freeze backoff?
   */
  bool m_freezeBackoff;
  void DoDispose() override;
  Time GetTimeLeftInCap() override;
  /**
   * Update CW
   */
  void AdjustCW();
  /**
  * The trace source fired when collision occurs.
  */
  TracedCallback<uint8_t, uint32_t> m_csmaCaSwNobaCollisionTrace;
  /**
  * The trace source fired when dynamic failure occurs.
  */
  TracedCallback<uint8_t> m_csmaCaSwNobaMKViolationTrace;
  std::deque<bool> m_resultQueue;
};

} // namespace lrwpan
} // namespace ns3

#endif /* LR_WPAN_CSMACA_SW_NOBA_H */
