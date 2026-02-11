/*
 * Copyright (c) 2016 NITK Surathkal
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Ankit Deepak <adadeepak8@gmail.com>
 *
 */

#include "tcp-ledbat-plus-plus.h"

#include "tcp-socket-state.h"

#include "ns3/log.h"
#include "ns3/simulator.h" // Now ()

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TcpLedbatPlusPlus");
NS_OBJECT_ENSURE_REGISTERED(TcpLedbatPlusPlus);

TypeId
TcpLedbatPlusPlus::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TcpLedbatPlusPlus")
            .SetParent<TcpNewReno>()
            .AddConstructor<TcpLedbatPlusPlus>()
            .SetGroupName("Internet")
            .AddAttribute("TargetDelay",
                          "Targeted Queue Delay",
                          TimeValue(MilliSeconds(60)),
                          MakeTimeAccessor(&TcpLedbatPlusPlus::m_target),
                          MakeTimeChecker())
            .AddAttribute("BaseHistoryLen",
                          "Number of Base delay samples",
                          UintegerValue(10),
                          MakeUintegerAccessor(&TcpLedbatPlusPlus::m_baseHistoLen),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("NoiseFilterLen",
                          "Number of Current delay samples",
                          UintegerValue(4),
                          MakeUintegerAccessor(&TcpLedbatPlusPlus::m_noiseFilterLen),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("Gain",
                          "Offset Gain",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&TcpLedbatPlusPlus::m_gain),
                          MakeDoubleChecker<double>(1e-6))
            .AddAttribute("SSParam",
                          "Possibility of Slow Start",
                          EnumValue(DO_SLOWSTART),
                          MakeEnumAccessor<SlowStartType>(&TcpLedbatPlusPlus::SetDoSs),
                          MakeEnumChecker(DO_SLOWSTART, "yes", DO_NOT_SLOWSTART, "no"))
            .AddAttribute("MinCwnd",
                          "Minimum cWnd for Ledbat",
                          UintegerValue(2),
                          MakeUintegerAccessor(&TcpLedbatPlusPlus::m_minCwnd),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("AllowedIncrease",
                          "Allowed Increase",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&TcpLedbatPlusPlus::m_allowedIncrease),
                          MakeDoubleChecker<double>(1e-6));
    return tid;
}

void
TcpLedbatPlusPlus::SetDoSs(SlowStartType doSS)
{
    NS_LOG_FUNCTION(this << doSS);
    m_doSs = doSS;
    if (m_doSs)
    {
        m_flag |= LEDBAT_CAN_SS;
    }
    else
    {
        m_flag &= ~LEDBAT_CAN_SS;
    }
}

TcpLedbatPlusPlus::TcpLedbatPlusPlus()
    : TcpNewReno()
{
    NS_LOG_FUNCTION(this);
    InitCircBuf(m_baseHistory);
    InitCircBuf(m_noiseFilter);
    m_lastRollover = Seconds(0);
    m_sndCwndCnt = 0;
    m_flag = LEDBAT_CAN_SS;
}

void
TcpLedbatPlusPlus::InitCircBuf(OwdCircBuf& buffer)
{
    NS_LOG_FUNCTION(this);
    buffer.buffer.clear();
    buffer.min = 0;
}

TcpLedbatPlusPlus::TcpLedbatPlusPlus(const TcpLedbatPlusPlus& sock)
    : TcpNewReno(sock)
{
    NS_LOG_FUNCTION(this);
    m_target = sock.m_target;
    m_gain = sock.m_gain;
    m_doSs = sock.m_doSs;
    m_baseHistoLen = sock.m_baseHistoLen;
    m_noiseFilterLen = sock.m_noiseFilterLen;
    m_baseHistory = sock.m_baseHistory;
    m_noiseFilter = sock.m_noiseFilter;
    m_lastRollover = sock.m_lastRollover;
    m_sndCwndCnt = sock.m_sndCwndCnt;
    m_flag = sock.m_flag;
    m_minCwnd = sock.m_minCwnd;
    m_allowedIncrease = sock.m_allowedIncrease;
}

TcpLedbatPlusPlus::~TcpLedbatPlusPlus()
{
    NS_LOG_FUNCTION(this);
}

Ptr<TcpCongestionOps>
TcpLedbatPlusPlus::Fork()
{
    return CopyObject<TcpLedbatPlusPlus>(this);
}

std::string
TcpLedbatPlusPlus::GetName() const
{
    return "TcpLedbatPlusPlus";
}

uint32_t
TcpLedbatPlusPlus::MinCircBuf(OwdCircBuf& b)
{
    NS_LOG_FUNCTION_NOARGS();
    if (b.buffer.empty())
    {
        return ~0U;
    }
    else
    {
        return b.buffer[b.min];
    }
}

uint32_t
TcpLedbatPlusPlus::CurrentDelay(FilterFunction filter)
{
    NS_LOG_FUNCTION(this);
    return filter(m_noiseFilter);
}

uint32_t
TcpLedbatPlusPlus::BaseDelay()
{
    NS_LOG_FUNCTION(this);
    return MinCircBuf(m_baseHistory);
}

double
TcpLedbatPlusPlus::ComputeGain ()
{
    uint64_t base_delay = BaseDelay();

    if (base_delay == 0 || base_delay == ~0U)
    {
        return 1.0;
    }

    double base = static_cast<double>(base_delay);
    double target = static_cast<double>(m_target.GetMilliSeconds());

    double ratio = std::ceil(2.0 * target / base);
    double denom = std::min(16.0, ratio);

    return 1.0 / denom;
}


void
TcpLedbatPlusPlus::IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
    NS_LOG_FUNCTION(this << tcb << segmentsAcked);
    if (tcb->m_cWnd.Get() <= tcb->m_segmentSize && !(tcb->m_initialSs || tcb->m_waitingForInitialSlowdown || tcb->m_inSlowdown || tcb->m_slowdownRecovery))
    {
        m_flag |= LEDBAT_CAN_SS;
    } 

    uint32_t queueDelay = 0;

    if (tcb->m_initialSs && (m_flag & LEDBAT_VALID_OWD))
    {
        uint32_t currentDelay = CurrentDelay(&TcpLedbatPlusPlus::MinCircBuf);
        uint32_t baseDelay = BaseDelay();

        queueDelay = currentDelay > baseDelay ? currentDelay - baseDelay : 0;

        if (static_cast<double>(queueDelay) > 0.75 * static_cast<double>(m_target.GetMilliSeconds()))       //after exiting initial slow start, we wait for 2 RTT to enter into initial slowdown
        {
			NS_LOG_INFO("Exiting initial slow start due to exceeding 3/4 of target delay...");
            NS_LOG_INFO("Queue delay: "<< queueDelay<<" Target delay: "<< m_target.GetMilliSeconds());
            tcb->m_initialSs = false;
            m_flag &= ~LEDBAT_CAN_SS;
            tcb->m_initialSsExitTime = Simulator::Now();
            tcb->m_waitingForInitialSlowdown = true;
        }
        else
        {
            m_flag |= LEDBAT_CAN_SS;
        }
    }else if(!(tcb->m_waitingForInitialSlowdown || tcb->m_inSlowdown || tcb->m_slowdownRecovery) && (Simulator::Now() >= tcb->m_nextSlowdownTime)){     //checking if time for next periodic slowdown
        tcb->m_inSlowdown = true;
        tcb->m_inSlowdown = true;
        m_flag |= LEDBAT_CAN_SS;
        tcb->m_slowdownStartTime = Simulator::Now();
        tcb->m_ssThresh = tcb->m_cWnd;                                      // set ssthresh to current cwnd
        tcb->m_cWnd = 2 * tcb->m_segmentSize;                               // set cwnd to two packets for 2 RTT
    }

    if(tcb->m_waitingForInitialSlowdown){                                       // checking if initial slowdown has not occured yet
        if(m_flag & LEDBAT_VALID_OWD){
            uint32_t currentDelay = CurrentDelay(&TcpLedbatPlusPlus::MinCircBuf);
            uint32_t baseDelay = BaseDelay();
            queueDelay = currentDelay > baseDelay ? currentDelay - baseDelay : 0;
            NS_LOG_INFO("Queue delay: "<< queueDelay<<" Target delay: "<< m_target.GetMilliSeconds()<<" Base delay: "<< baseDelay);
        }
        if(Simulator::Now() - tcb->m_initialSsExitTime >= 2*tcb->m_srtt){       // waited for 2 RTT and now entering initial slowdown
            tcb->m_waitingForInitialSlowdown = false;
            tcb->m_inSlowdown = true;
            tcb->m_slowdownStartTime = Simulator::Now();
            tcb->m_ssThresh = tcb->m_cWnd;                                      // set ssthresh to current cwnd
            tcb->m_cWnd = 2 * tcb->m_segmentSize;                               // set cwnd to two packets for 2 RTT
            m_flag |= LEDBAT_CAN_SS;
            NS_LOG_INFO("Holding cwnd at 2 packets for 2 RTT...");
        }else{                                                                  // 2 RTT have not passed and still waiting for slowdown
            NS_LOG_INFO("Waiting 2 RTT for initial slowdown period to start...");
            m_flag &= ~LEDBAT_CAN_SS;
            CongestionAvoidance(tcb, segmentsAcked);
        }
    }
    else if(tcb->m_inSlowdown && (m_flag & LEDBAT_CAN_SS)){                                                 // checking if in slowdown phase
        if(m_flag & LEDBAT_VALID_OWD){
            uint32_t currentDelay = CurrentDelay(&TcpLedbatPlusPlus::MinCircBuf);
            uint32_t baseDelay = BaseDelay();
            queueDelay = currentDelay > baseDelay ? currentDelay - baseDelay : 0;
            NS_LOG_INFO("Queue delay: "<< queueDelay<<" Target delay: "<< m_target.GetMilliSeconds()<<" Base delay: "<< baseDelay);
        }
        if(Simulator::Now() - tcb->m_slowdownStartTime >= 2*tcb->m_srtt){       // checking if 2 RTT have passed after holding cwnd at 2 packets
            tcb->m_inSlowdown = false;
            tcb->m_slowdownRecovery = true;                                     // grow cwnd according to SlowStart
            m_flag |= LEDBAT_CAN_SS;
            NS_LOG_INFO("2 RTT over, growing cwnd to ssthresh through slow start...");
            SlowStart(tcb, segmentsAcked);
        }else{                                                                  // continue to hold cwnd at 2 packets until 2 RTT passes
            NS_LOG_INFO("Holding cwnd at 2 packets for 2 RTT...");
        }
    }else if(tcb->m_slowdownRecovery){                                          // checking if in slowdown recovery, growing cwnd to ssthresh 
        if(m_flag & LEDBAT_VALID_OWD){
            uint32_t currentDelay = CurrentDelay(&TcpLedbatPlusPlus::MinCircBuf);
            uint32_t baseDelay = BaseDelay();
            queueDelay = currentDelay > baseDelay ? currentDelay - baseDelay : 0;
            NS_LOG_INFO("Queue delay: "<< queueDelay<<" Target delay: "<< m_target.GetMilliSeconds()<<" Base delay: "<< baseDelay);
        }
        if(tcb->m_cWnd >= tcb->m_ssThresh){                                     // checking if cwnd has regrown to ssthresh
            tcb->m_slowdownRecovery = false;
            tcb->m_slowdownDuration = Simulator::Now() - tcb->m_slowdownStartTime;
            tcb->m_nextSlowdownTime = Simulator::Now() + 9*tcb->m_slowdownDuration;
            m_flag &= ~LEDBAT_CAN_SS;
            CongestionAvoidance(tcb, segmentsAcked);
        }else{                                                                  // waiting for cwnd to regrow to ssthresh
            NS_LOG_INFO("Growing cwnd to ssthresh through slow start...");
            m_flag |= LEDBAT_CAN_SS;
            SlowStart(tcb, segmentsAcked);
        }
    }
    else if (m_doSs == DO_SLOWSTART && tcb->m_cWnd <= tcb->m_ssThresh && (m_flag & LEDBAT_CAN_SS))
    {
        if((m_flag & LEDBAT_VALID_OWD)){
            NS_LOG_INFO("Queue delay: "<< queueDelay<<" Target delay: "<< m_target.GetMilliSeconds());
        }
        SlowStart(tcb, segmentsAcked);
    }
    else
    {
        if(tcb->m_initialSs){
            tcb->m_initialSs = false;
            tcb->m_initialSsExitTime = Simulator::Now();
            tcb->m_waitingForInitialSlowdown = true;
        }
        m_flag &= ~LEDBAT_CAN_SS;
        CongestionAvoidance(tcb, segmentsAcked);
    }
}

void
TcpLedbatPlusPlus::CongestionAvoidance(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
    NS_LOG_FUNCTION(this << tcb << segmentsAcked);
    if ((m_flag & LEDBAT_VALID_OWD) == 0)
    {
        TcpNewReno::CongestionAvoidance(
            tcb,
            segmentsAcked); // letting it fall to TCP behaviour if no timestamps
        return;
    }
    uint32_t queueDelay;
    uint32_t cwnd = (tcb->m_cWnd.Get());
    uint32_t maxCwnd;
    uint32_t currentDelay = CurrentDelay(&TcpLedbatPlusPlus::MinCircBuf);
    uint32_t baseDelay = BaseDelay();
    uint32_t segmentSize = tcb->m_segmentSize;

    double ackFactor = static_cast<double>(segmentSize) / cwnd;
    double W = static_cast<double>(cwnd) / segmentSize;

    queueDelay = currentDelay > baseDelay ? currentDelay - baseDelay : 0;

    double delayRatio = static_cast<double>(queueDelay) / m_target.GetMilliSeconds();

  
    if(delayRatio < 1.0){
        double gain = ComputeGain();
        W += gain * ackFactor;
    } else {
        double md = m_gain - W * (delayRatio - 1.0);
        W += std::max(md * ackFactor, -W * ackFactor / 2.0);
    }

    W = std::max(W, 2.0);
    cwnd = static_cast<uint32_t>(W * segmentSize);
    NS_LOG_INFO("base_delay: " << baseDelay
                << " curr_delay: " << currentDelay
                << " queue_delay: " << queueDelay
                << " delay_ratio: " << delayRatio
                << " W: " << W);
    NS_LOG_INFO("W=" << W << " ratio=" << delayRatio);

    uint32_t flightSizeBeforeAck = tcb->m_bytesInFlight.Get() + (segmentsAcked * segmentSize);
    maxCwnd = flightSizeBeforeAck + static_cast<uint32_t>(m_allowedIncrease * segmentSize);
    cwnd = std::min(cwnd, maxCwnd);
    cwnd = std::max(cwnd, m_minCwnd * segmentSize);
    tcb->m_cWnd = cwnd;

    if (tcb->m_cWnd <= tcb->m_ssThresh)
    {
        tcb->m_ssThresh = tcb->m_cWnd - 1;
    }
}

void
TcpLedbatPlusPlus::AddDelay(OwdCircBuf& cb, uint32_t owd, uint32_t maxlen)
{
    NS_LOG_FUNCTION(this << owd << maxlen << cb.buffer.size());
    if (cb.buffer.empty())
    {
        NS_LOG_LOGIC("First Value for queue");
        cb.buffer.push_back(owd);
        cb.min = 0;
        return;
    }
    cb.buffer.push_back(owd);
    if (cb.buffer[cb.min] > owd)
    {
        cb.min = cb.buffer.size() - 1;
    }
    if (cb.buffer.size() >= maxlen)
    {
        NS_LOG_LOGIC("Queue full" << maxlen);
        cb.buffer.erase(cb.buffer.begin());
        auto bufferStart = cb.buffer.begin();
        cb.min = std::distance(bufferStart, std::min_element(bufferStart, cb.buffer.end()));
        NS_LOG_LOGIC("Current min element" << cb.buffer[cb.min]);
    }
}

void
TcpLedbatPlusPlus::UpdateBaseDelay(uint32_t owd)
{
    NS_LOG_FUNCTION(this << owd);
    if (m_baseHistory.buffer.empty())
    {
        AddDelay(m_baseHistory, owd, m_baseHistoLen);
        return;
    }
    Time timestamp = Simulator::Now();

    if (timestamp - m_lastRollover > Seconds(60))
    {
        m_lastRollover = timestamp;
        AddDelay(m_baseHistory, owd, m_baseHistoLen);
    }
    else
    {
        size_t last = m_baseHistory.buffer.size() - 1;
        if (owd < m_baseHistory.buffer[last])
        {
            m_baseHistory.buffer[last] = owd;
            if (owd < m_baseHistory.buffer[m_baseHistory.min])
            {
                m_baseHistory.min = last;
            }
        }
    }
}

void
TcpLedbatPlusPlus::PktsAcked(Ptr<TcpSocketState> tcb,
                     uint32_t segmentsAcked,
                     const Time& rtt)
{
    NS_LOG_FUNCTION(this << tcb << segmentsAcked << rtt);

    if (rtt.IsPositive() && tcb->m_rcvTimestampValue >= tcb->m_rcvTimestampEchoReply)
    {
        // RTT based delay signal 
        m_flag |= LEDBAT_VALID_OWD;

        uint32_t rttMs = rtt.GetMilliSeconds();

        AddDelay(m_noiseFilter, rttMs, m_noiseFilterLen);
        UpdateBaseDelay(rttMs);
    }
    else
    {
        m_flag &= ~LEDBAT_VALID_OWD;
    }
}

} // namespace ns3