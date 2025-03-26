/*
* Copyright (c) 2025 jshyeon, Gyeonsang National University
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */
#ifndef LR_WPAN_RETRANSMISSION_TAG_H
#define LR_WPAN_RETRANSMISSION_TAG_H

#include <ns3/tag.h>


namespace ns3
{
namespace lrwpan
{

/**
 * \ingroup lr-wpan
 * Represent the retransmission count for this packet.
 *
 * The retransmission Tag is added to each received packet.
 *
 */
class LrWpanRetransmissionTag : public Tag
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
     * Create a LrWpanRetransmissionTag with the default count 0.
     */
    LrWpanRetransmissionTag();

    /**
     * Create a LrWpanRetransmissionTag with the given value.
     * \param retransmissionCount The retransmission count.
     */
    LrWpanRetransmissionTag(uint32_t retransmissionCount);

    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;

    void Set(uint32_t retransmissionCount);
    uint32_t Get() const;

private:
    /**
     * The retransmission count of the tag.
     */
    uint32_t m_retransmissionCount;
};

} // namespace lrwpan
} // namespace ns3
#endif /* LR_WPAN_RETRANSMISSION_TAG_H */
