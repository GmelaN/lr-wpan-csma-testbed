/*
* Copyright (c) 2025 jshyeon, Gyeonsang National University
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */
#include "lr-wpan-retransmission-tag.h"
#include <ns3/integer.h>

namespace ns3
{
namespace lrwpan
{

NS_OBJECT_ENSURE_REGISTERED(LrWpanRetransmissionTag);

TypeId
LrWpanRetransmissionTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::lrwpan::LrWpanRetransmissionTag")
                            .AddDeprecatedName("ns3::LrWpanRetransmissionTag")
                            .SetParent<Tag>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<LrWpanRetransmissionTag>()
                            .AddAttribute("retransmissionCount",
                                          "retransmission count",
                                          IntegerValue(0),
                                          MakeIntegerAccessor(&LrWpanRetransmissionTag::Get),
                                          MakeIntegerChecker<uint32_t>());
    return tid;
}

TypeId
LrWpanRetransmissionTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

LrWpanRetransmissionTag::LrWpanRetransmissionTag()
    : m_retransmissionCount(0)
{
}

LrWpanRetransmissionTag::LrWpanRetransmissionTag(uint32_t retransmissionCount)
    : m_retransmissionCount(retransmissionCount)
{
}

uint32_t
LrWpanRetransmissionTag::GetSerializedSize() const
{
    return sizeof(uint32_t);
}

void
LrWpanRetransmissionTag::Serialize(TagBuffer i) const
{
    i.WriteU32(m_retransmissionCount);
}

void
LrWpanRetransmissionTag::Deserialize(TagBuffer i)
{
    m_retransmissionCount = i.ReadU32();
}

void
LrWpanRetransmissionTag::Print(std::ostream& os) const
{
    os << "retransmission count = " << m_retransmissionCount;
}

void
LrWpanRetransmissionTag::Set(uint32_t retransmissionCount)
{
    m_retransmissionCount = retransmissionCount;
}

uint32_t
LrWpanRetransmissionTag::Get() const
{
    return m_retransmissionCount;
}


} // namespace lrwpan
} // namespace ns3
