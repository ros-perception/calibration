/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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


#include <gtest/gtest.h>

#include "settlerlib/sorted_deque.h"

using namespace std;
using namespace settlerlib;

struct Header
{
  ros::Time stamp ;
} ;


struct Msg
{
  Header header ;
  int data ;
} ;

void fillEasy(SortedDeque<Msg>& sd, unsigned int start, unsigned int end)
{
  for (unsigned int i=start; i < end; i++)
  {
    Msg msg ;
    msg.data = i ;
    msg.header.stamp.fromSec(i*10) ;

    sd.add(msg) ;
  }
}

TEST(SortedDeque, easyInterval)
{
  SortedDeque<Msg> sd;
  sd.setMaxSize(10);

  fillEasy(sd, 0, 5);

  vector<Msg> interval_data = sd.getInterval(ros::Time().fromSec(5), ros::Time().fromSec(35)) ;

  EXPECT_EQ(interval_data.size(), (unsigned int) 3) ;
  EXPECT_EQ(interval_data[0].data, 1) ;
  EXPECT_EQ(interval_data[1].data, 2) ;
  EXPECT_EQ(interval_data[2].data, 3) ;

  // Look for an interval past the end of the cache
  interval_data = sd.getInterval(ros::Time().fromSec(55), ros::Time().fromSec(65)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 0) ;

  // Look for an interval that fell off the back of the cache
  fillEasy(sd, 5, 20) ;
  interval_data = sd.getInterval(ros::Time().fromSec(5), ros::Time().fromSec(35)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 0) ;
}

TEST(SortedDeque, easySurroundingInterval)
{
  SortedDeque<Msg> cache;
  cache.setMaxSize(10);
  fillEasy(cache, 1, 6);

  vector<Msg> interval_data;
  interval_data = cache.getSurroundingInterval(ros::Time(15,0), ros::Time(35,0)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 4);
  EXPECT_EQ(interval_data[0].data, 1);
  EXPECT_EQ(interval_data[1].data, 2);
  EXPECT_EQ(interval_data[2].data, 3);
  EXPECT_EQ(interval_data[3].data, 4);

  interval_data = cache.getSurroundingInterval(ros::Time(0,0), ros::Time(35,0)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 4);
  EXPECT_EQ(interval_data[0].data, 1);

  interval_data = cache.getSurroundingInterval(ros::Time(35,0), ros::Time(35,0)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 2);
  EXPECT_EQ(interval_data[0].data, 3);
  EXPECT_EQ(interval_data[1].data, 4);

  interval_data = cache.getSurroundingInterval(ros::Time(55,0), ros::Time(55,0)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 1);
  EXPECT_EQ(interval_data[0].data, 5);
}

Msg buildMsg(double time, int data)
{
  Msg msg;
  msg.data = data;
  msg.header.stamp.fromSec(time);
  return msg;
}

TEST(SortedDeque, easyUnsorted)
{
  SortedDeque<Msg> cache;
  cache.setMaxSize(10);

  cache.add(buildMsg(10.0, 1)) ;
  cache.add(buildMsg(30.0, 3)) ;
  cache.add(buildMsg(70.0, 7)) ;
  cache.add(buildMsg( 5.0, 0)) ;
  cache.add(buildMsg(20.0, 2)) ;

  vector<Msg> interval_data = cache.getInterval(ros::Time().fromSec(3), ros::Time().fromSec(15)) ;

  EXPECT_EQ(interval_data.size(), (unsigned int) 2) ;
  EXPECT_EQ(interval_data[0].data, 0) ;
  EXPECT_EQ(interval_data[1].data, 1) ;

  // Grab all the data
  interval_data = cache.getInterval(ros::Time().fromSec(0), ros::Time().fromSec(80)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 5) ;
  EXPECT_EQ(interval_data[0].data, 0) ;
  EXPECT_EQ(interval_data[1].data, 1) ;
  EXPECT_EQ(interval_data[2].data, 2) ;
  EXPECT_EQ(interval_data[3].data, 3) ;
  EXPECT_EQ(interval_data[4].data, 7) ;
}

TEST(SortedDeque, easyElemBeforeAfter)
{
  SortedDeque<Msg> cache;
  cache.setMaxSize(10);
  Msg elem;

  fillEasy(cache, 5, 10) ;

  bool result;
  result = cache.getElemAfterTime( ros::Time().fromSec(85.0), elem);

  ASSERT_TRUE(result);
  EXPECT_EQ(elem.data, 9);

  result = cache.getElemBeforeTime( ros::Time().fromSec(85.0), elem);
  ASSERT_TRUE(result) ;
  EXPECT_EQ(elem.data, 8) ;

  result = cache.getElemBeforeTime( ros::Time().fromSec(45.0), elem);
  EXPECT_FALSE(result) ;
}

TEST(SortedDeque, easyRemoval)
{
  SortedDeque<Msg> sd;
  sd.setMaxSize(20);

  fillEasy(sd, 1, 10);

  vector<Msg> interval_data;
  interval_data = sd.getInterval(ros::Time().fromSec(5), ros::Time().fromSec(105)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 9) ;

  sd.removeAllBeforeTime(ros::Time().fromSec(35));
  interval_data = sd.getInterval(ros::Time().fromSec(5), ros::Time().fromSec(105)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 6);
  EXPECT_EQ(interval_data[0].data, 4) ;
  EXPECT_EQ(interval_data[1].data, 5) ;
  EXPECT_EQ(interval_data[2].data, 6) ;
  EXPECT_EQ(interval_data[3].data, 7) ;
  EXPECT_EQ(interval_data[4].data, 8) ;
  EXPECT_EQ(interval_data[5].data, 9) ;
}

TEST(SortedDeque, easyClosest)
{
  SortedDeque<Msg> sd;
  sd.setMaxSize(20);

  fillEasy(sd, 1, 10);

  bool found;
  Msg elem;

  found = sd.getClosestElem(ros::Time().fromSec(5), elem) ;
  EXPECT_TRUE(found) ;
  EXPECT_EQ(elem.data, 1) ;

  found = sd.getClosestElem(ros::Time().fromSec(20), elem) ;
  EXPECT_TRUE(found) ;
  EXPECT_EQ(elem.data, 2) ;
}



TEST(SortedDeque, easyPointer)
{
  SortedDeque<boost::shared_ptr<Msg> > sd(SortedDeque<boost::shared_ptr<Msg> >::getPtrStamp);
  sd.setMaxSize(20);

  boost::shared_ptr<Msg> msg_ptr(new Msg);
  msg_ptr->header.stamp = ros::Time(10,0);
  sd.add(msg_ptr);

  msg_ptr.reset(new Msg);
  msg_ptr->header.stamp = ros::Time(20,0);
  sd.add(msg_ptr);

  msg_ptr.reset(new Msg);
  msg_ptr->header.stamp = ros::Time(30,0);
  sd.add(msg_ptr);


  boost::shared_ptr<Msg> found_elem;
  bool found;
  found = sd.getClosestElem(ros::Time().fromSec(22), found_elem);
  ASSERT_TRUE(found);
  EXPECT_EQ(found_elem->header.stamp, ros::Time(20,0));
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
