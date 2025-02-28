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

#include "lr-wpan-csmaca-common.h"

#include "lr-wpan-constants.h"

#include <ns3/log.h>
#include <ns3/random-variable-stream.h>
#include <ns3/simulator.h>

#include <algorithm>

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT                                                                      \
    std::clog << "[" << m_mac->GetShortAddress() << " | " << m_mac->GetExtendedAddress() << "] ";

namespace ns3
{
namespace lrwpan
{

NS_LOG_COMPONENT_DEFINE("LrWpanCsmaCaCommon");
NS_OBJECT_ENSURE_REGISTERED(LrWpanCsmaCaCommon);

TypeId
LrWpanCsmaCaCommon::GetTypeId()
{
    static TypeId tid = TypeId("ns3::lrwpan::LrWpanCsmaCaCommon")
                            .AddDeprecatedName("ns3::LrWpanCsmaCaCommon")
                            .SetParent<Object>()
                            .SetGroupName("LrWpan");
                            // .AddConstructor<LrWpanCsmaCaCommon>();
    return tid;
}

LrWpanCsmaCaCommon::~LrWpanCsmaCaCommon()
{
}
}
}