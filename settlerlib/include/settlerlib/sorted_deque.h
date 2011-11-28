/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <deque>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <ros/time.h>
#include <ros/console.h>

#ifndef SETTLERLIB_SORTED_DEQUE_H_
#define SETTLERLIB_SORTED_DEQUE_H_

#define DEQUE_DEBUG(fmt, ...) \
    ROS_DEBUG_NAMED(logger_.c_str(), fmt,##__VA_ARGS__)

namespace settlerlib
{

/**
 * \brief Adds helper routines to the STL Deque for holding messages with headers
 *
 * Provides functionality to store a deque of messages that are sorted by timestamp. Users can
 * then extract specific intervals of messages.  Old messages fall off the back of the deque
 */
template <class M>
class SortedDeque : public std::deque<M>
{
public:

  using std::deque<M>::size;
  using std::deque<M>::front;
  using std::deque<M>::back;
  using std::deque<M>::pop_front;
  using std::deque<M>::begin;
  using std::deque<M>::end;
  using std::deque<M>::rbegin;
  using std::deque<M>::rend;
  using std::deque<M>::insert;
  using std::deque<M>::at;
  using std::deque<M>::erase;

  /**
   * \brief Assumes that '.header.stamp' can be used to get the stamp used for sorting
   * \param logger name of rosconsole logger to display debugging output from this deque
   */
  SortedDeque(std::string logger = "deque") : std::deque<M>(), logger_(logger)
  {
    getStamp = &SortedDeque<M>::getStructStamp;
    max_size_ = 1;
  }

  /**
   * \brief Advanced constructor, allowing user to specific the timestamp getter method
   * \param getStampFunc Function pointer specific how to get timestamp from the contained datatype
   * \param logger name of rosconsole logger to display debugging output from this deque
   */
  SortedDeque(boost::function<const ros::Time&(const M&)> getStampFunc,
              std::string logger = "deque") : std::deque<M>(), logger_(logger)
  {
    getStamp = getStampFunc;
    max_size_ = 1;
  }

  /**
   * \brief Set the maximum # of elements this deque can hold.  Older elems are popped once the length is exeeded
   * \param max_size The maximum # of elems to hold. Passing in 0 implies that the queue can grow indefinitely (which is a recipe for a memory leak)
   */
  void setMaxSize(unsigned int max_size)
  {
    max_size_ = max_size ;
  }

  /**
   * \brief Add a new element to the deque, correctly sorted by timestamp
   * \param msg the element to add
   */
  void add(const M& msg)
  {
    DEQUE_DEBUG("Called add()");
    DEQUE_DEBUG_STATS("   ");
    if (max_size_ != 0)
    {
      while (size() >= max_size_)                // Keep popping off old data until we have space for a new msg
      {
        pop_front() ;                            // The front of the deque has the oldest elem, so we can get rid of it
        DEQUE_DEBUG("   Popping element");
        DEQUE_DEBUG_STATS("   ");
      }
    }

    typename std::deque<M>::reverse_iterator rev_it = rbegin();

    // Keep walking backwards along deque until we hit the beginning,
    //   or until we find a timestamp that's smaller than (or equal to) msg's timestamp
    while(rev_it != rend() && getStamp(*rev_it) > getStamp(msg))
      rev_it++;

    // Add msg to the cache
    insert(rev_it.base(), msg);
    DEQUE_DEBUG("   Done inserting");
    DEQUE_DEBUG_STATS("   ");
  }

  /**
   * \brief Extract all the elements that occur in the interval between the start and end times
   * \param start The start of the interval
   * \param end The end of the interval
   */
  std::vector<M> getInterval(const ros::Time& start, const ros::Time& end)
  {
    // Find the starting index. (Find the first index after [or at] the start of the interval)
    unsigned int start_index = 0 ;
    while(start_index < size() &&
          getStamp(at(start_index)) < start)
    {
      start_index++ ;
    }

    // Find the ending index. (Find the first index after the end of interval)
    unsigned int end_index = start_index ;
    while(end_index < size() &&
          getStamp(at(end_index)) <= end)
    {
      end_index++ ;
    }

    std::vector<M> interval_elems ;
    interval_elems.reserve(end_index - start_index) ;
    for (unsigned int i=start_index; i<end_index; i++)
    {
      interval_elems.push_back(at(i)) ;
    }

    return interval_elems ;
  }

  /**
   * Retrieve the smallest interval of messages that surrounds an interval from start to end.
   * If the messages in the cache do not surround (start,end), then this will return the interval
   * that gets closest to surrounding (start,end)
   */
  std::vector<M> getSurroundingInterval(const ros::Time& start, const ros::Time& end)
  {
    // Find the starting index. (Find the first index after [or at] the start of the interval)
    unsigned int start_index = size()-1;
    while(start_index > 0 &&
          getStamp(at(start_index)) > start)
    {
      start_index--;
    }
    unsigned int end_index = start_index;
    while(end_index < size()-1 &&
          getStamp(at(end_index)) < end)
    {
      end_index++;
    }

    std::vector<M> interval_elems;
    interval_elems.reserve(end_index - start_index + 1) ;
    for (unsigned int i=start_index; i<=end_index; i++)
    {
      interval_elems.push_back(at(i)) ;
    }

    return interval_elems;
  }

  /**
  * \brief Grab the oldest element that occurs right before the specified time.
  * \param time The time that must occur after the elem
  * \param out Output: Stores the extracted elem
  * \return False there are no elems before the specified time
  **/
  bool getElemBeforeTime(const ros::Time& time, M& out) const
  {
    unsigned int i=0 ;
    int elem_index = -1 ;
    while (i<size() &&
           getStamp(at(i)) < time)
    {
      elem_index = i ;
      i++ ;
    }

    if (elem_index >= 0)
    {
      out = at(elem_index);
      return true;
    }
    //out = M();
    return false;
  }

  /**
   * \brief Grab the oldest element that occurs right after the specified time.
   * \param time The time that must occur before the elem
   * \param out Output: Stores the extracted elem
   * \return False there are no elems after the specified time
   */
  bool getElemAfterTime(const ros::Time& time, M& out) const
  {
    int i=size()-1 ;
    int elem_index = -1 ;
    while (i>=0 &&
           getStamp(at(i)) > time)
    {
      elem_index = i ;
      i-- ;
    }

    if (elem_index >= 0)
    {
      out = at(elem_index);
      return true;
    }
    //out = M();
    return false;
  }

  /**
   * \brief Get the elem that occurs closest to the specified time
   * \param time The time that we want to closest elem to
   * \param out Output: Stores the extracted elem
   * \return False if the queue is empty
   */
  bool getClosestElem(const ros::Time& time, M& out)
  {
    if (size() == 0)
      return false;

    typename std::deque<M>::iterator it = begin();
    typename std::deque<M>::iterator best = it;

    double best_diff = fabs( (time - getStamp(*best)).toSec());

    while (it != end())
    {
      double cur_diff = fabs( (time - getStamp(*it)).toSec());
      if (cur_diff < best_diff)
      {
        best_diff = cur_diff;
        best = it;
      }
      ++it;
    }
    out = *best;
    return true;
  }


  /**
   * \brief Removes all elements that occur before the specified time
   * \param time All elems that occur before this are removed from the deque
   */
  void removeAllBeforeTime(const ros::Time& time)
  {
    DEQUE_DEBUG("Called removeAllBeforeTime()");
    DEQUE_DEBUG("   Erasing all elems before time: %u %u", time.sec, time.nsec);
    typename std::deque<M>::iterator it = begin();

    while (size() > 0 && getStamp(front()) < time)
    {
      DEQUE_DEBUG("   Erasing elem at time: %u, %u", getStamp(front()).sec, getStamp(front()).nsec);
      pop_front();
      DEQUE_DEBUG("   Erased an elem");
      DEQUE_DEBUG_STATS("   ");
    }
    DEQUE_DEBUG("   Done erasing elems");
  }

  static const ros::Time& getPtrStamp(const M& m)
  {
    return m->header.stamp;
  }

  static const ros::Time& getStructStamp(const M& m)
  {
    return m.header.stamp;
  }

  static const ros::Time& getHeaderStamp(const M& m)
  {
    return m.stamp;
  }
private:
  unsigned int max_size_;
  std::string logger_;


  /*const ros::Time& getStamp(const M& m)
  {
    return getStructStamp(m);
  }*/

  boost::function<const ros::Time&(const M&)> getStamp;

  inline void DEQUE_DEBUG_STATS(const std::string& prefix)
  {
    DEQUE_DEBUG("%sdeque.size(): %u   max_size: %u", prefix.c_str(), (unsigned int) size(), max_size_);
  }
};

}

#undef DEQUE_DEBUG
#undef DEQUE_WARN

#endif
