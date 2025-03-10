/*
* Copyright (c) 2025 jshyeon, Gyeonsang National University
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */
#ifndef LR_WPAN_PRIORITY_TAG_H
#define LR_WPAN_PRIORITY_TAG_H

#include <ns3/tag.h>


namespace ns3
{
namespace lrwpan
{

/**
 * \ingroup lr-wpan
 * Represent the priority.
 *
 * The priority Tag is added to each received packet, and can be
 * used by upper layers to estimate the channel conditions.
 *
 */
class LrWpanPriorityTag : public Tag
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
     * Create a LrWpanPriorityTag with the default priority 0.
     */
    LrWpanPriorityTag();

    /**
     * Create a LrWpanPriorityTag with the given priority value.
     * \param priority source node priority.
     */
    LrWpanPriorityTag(uint8_t priority);

    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;

    void Set(uint8_t priority);
    uint8_t Get() const;

private:
    /**
     * The priority of issued node.
     */
    uint8_t m_priority;
};

} // namespace lrwpan
} // namespace ns3
#endif /* LR_WPAN_PRIORITY_TAG_H */
