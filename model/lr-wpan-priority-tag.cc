/*
* Copyright (c) 2025 jshyeon, Gyeonsang National University
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */
#include "lr-wpan-priority-tag.h"

#include "../../core/model/integer.h"

#include <ns3/integer.h>

namespace ns3
{
namespace lrwpan
{

NS_OBJECT_ENSURE_REGISTERED(LrWpanPriorityTag);

TypeId
LrWpanPriorityTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::lrwpan::LrWpanPriorityTag")
                            .AddDeprecatedName("ns3::LrWpanPriorityTag")
                            .SetParent<Tag>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<LrWpanPriorityTag>()
                            .AddAttribute("issuedTime",
                                          "packet issued time",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&LrWpanPriorityTag::Get),
                                          MakeIntegerChecker<uint8_t>());
    return tid;
}

TypeId
LrWpanPriorityTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

LrWpanPriorityTag::LrWpanPriorityTag()
    : m_priority(0)
{
}

uint32_t
LrWpanPriorityTag::GetSerializedSize() const
{
    return sizeof(uint8_t);
}

void
LrWpanPriorityTag::Serialize(TagBuffer i) const
{
    i.WriteU8(m_priority);
}

void
LrWpanPriorityTag::Deserialize(TagBuffer i)
{
    m_priority = i.ReadU8();
}

void
LrWpanPriorityTag::Print(std::ostream& os) const
{
    os << "issued node's priority = " << m_priority;
}


uint8_t
LrWpanPriorityTag::Get() const
{
    return m_priority;
}

void
LrWpanPriorityTag::Set(uint8_t priority)
{
     m_priority = priority;
}


} // namespace lrwpan
} // namespace ns3
