/*
* Copyright (c) 2025 jshyeon, Gyeonsang National University
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */
#ifndef LR_WPAN_DELAY_TAG_H
#define LR_WPAN_DELAY_TAG_H

#include <ns3/tag.h>


namespace ns3
{
namespace lrwpan
{

/**
 * \ingroup lr-wpan
 * Represent the issued time(for calculate delay).
 *
 * The delay Tag is added to each received packet, and can be
 * used by upper layers to estimate the channel conditions.
 *
 */
class LrWpanDelayTag : public Tag
{
public:
    /**
     * Get the type ID.
     *
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    TypeId GetInstanceTypeId() const override;

    /**
     * Create a LrWpanDelayTag with the default issuedTime 0.
     */
    LrWpanDelayTag();

    /**
     * Create a LrWpanDelayTag with the given issuedTime value.
     * \param issuedTime The issued time.
     */
    LrWpanDelayTag(double issuedTime);

    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;

    void Set(double issuedTime);
    double Get() const;

private:
    /**
     * The issued time of the tag.
     */
    double m_issuedTime;
};

} // namespace lrwpan
} // namespace ns3
#endif /* LR_WPAN_DELAY_TAG_H */
