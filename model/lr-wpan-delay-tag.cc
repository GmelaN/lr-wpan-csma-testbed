/*
* Copyright (c) 2025 jshyeon, Gyeonsang National University
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author:
 *  Jo Seoung Hyeon <gmelan@gnu.ac.kr>
 */
#include "lr-wpan-delay-tag.h"
#include <ns3/double.h>
#include <ns3/integer.h>

namespace ns3
{
namespace lrwpan
{

NS_OBJECT_ENSURE_REGISTERED(LrWpanDelayTag);

TypeId
LrWpanDelayTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::lrwpan::LrWpanDelayTag")
                            .AddDeprecatedName("ns3::LrWpanDelayTag")
                            .SetParent<Tag>()
                            .SetGroupName("LrWpan")
                            .AddConstructor<LrWpanDelayTag>()
                            .AddAttribute("issuedTime",
                                          "packet issued time",
                                          DoubleValue(0.0),
                                          MakeDoubleAccessor(&LrWpanDelayTag::Get),
                                          MakeDoubleChecker<double>());
    return tid;
}

TypeId
LrWpanDelayTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

LrWpanDelayTag::LrWpanDelayTag()
    : m_issuedTime(0.0)
{
}

LrWpanDelayTag::LrWpanDelayTag(double issuedTime)
    : m_issuedTime(issuedTime)
{
}

uint32_t
LrWpanDelayTag::GetSerializedSize() const
{
    return sizeof(double);
}

void
LrWpanDelayTag::Serialize(TagBuffer i) const
{
    i.WriteDouble(m_issuedTime);
}

void
LrWpanDelayTag::Deserialize(TagBuffer i)
{
    m_issuedTime = i.ReadDouble();
}

void
LrWpanDelayTag::Print(std::ostream& os) const
{
    os << "issued time = " << m_issuedTime;
}

void
LrWpanDelayTag::Set(double issuedTime)
{
    m_issuedTime = issuedTime;
}

double
LrWpanDelayTag::Get() const
{
    return m_issuedTime;
}


} // namespace lrwpan
} // namespace ns3
