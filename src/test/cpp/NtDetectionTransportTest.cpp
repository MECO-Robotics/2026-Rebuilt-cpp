#include <base/ObjectPerception.h>
#include <frc/Timer.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <gtest/gtest.h>
#include <chrono>
#include <array>
#include <string>
#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>

// Real NT4 loopback transport; synthetic detection results, not camera inference.
TEST(NtDetectionTransport, FastProfilePreservesMoreCameraFrames) {
  using Clock=std::chrono::steady_clock;
  struct Result { int sent=0, received=0; double meanAgeMs=0, maxAgeMs=0; };
  auto run=[](double period,int port) {
    auto server=nt::NetworkTableInstance::GetDefault();
    server.StopServer();server.StartServer("","127.0.0.1",0,port);
    const auto topic=std::string{"/Test/Detections/"}+std::to_string(port);
    auto receiver=base::MakeNtObjectDetectionIO(topic,units::second_t{period});
    auto client=nt::NetworkTableInstance::Create();
    client.StartClient4("detection-rate-probe");client.SetServer("127.0.0.1",port);
    nt::PubSubOptions options;options.periodic=period;
    auto publisher=client.GetDoubleArrayTopic(topic).Publish(options);
    const auto deadline=Clock::now()+std::chrono::seconds(3);
    while(!client.IsConnected() && Clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_TRUE(client.IsConnected());
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    Result result;auto start=Clock::now(),next=start;
    while(Clock::now()-start<std::chrono::seconds(4)) {
      auto now=Clock::now();
      if(now>=next) {
        ++result.sent;
        const std::array<double,7> packet{1,static_cast<double>(result.sent),0,1,0,0,1};
        publisher.Set(packet);next+=std::chrono::microseconds(16667);
      }
      if(auto frame=receiver->ReadFrame()) {
        ++result.received;
        const double age=(frc::Timer::GetFPGATimestamp()-frame->captureTime).value()*1000;
        result.meanAgeMs+=age;result.maxAgeMs=std::max(result.maxAgeMs,age);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if(result.received)result.meanAgeMs/=result.received;
    publisher={};receiver.reset();client.StopClient();nt::NetworkTableInstance::Destroy(client);server.StopServer();
    return result;
  };
  const auto baseline=run(.1,15810),fast=run(.005,15820);
  std::cout<<"DETECTION_TRANSPORT baseline_sent="<<baseline.sent<<" baseline_received="<<baseline.received
    <<" baseline_mean_age_ms="<<baseline.meanAgeMs<<" fast_sent="<<fast.sent<<" fast_received="<<fast.received
    <<" fast_mean_age_ms="<<fast.meanAgeMs<<" fast_max_age_ms="<<fast.maxAgeMs<<"\n";
  EXPECT_GT(baseline.received,10);
  EXPECT_GT(fast.received,baseline.received*3);
  EXPECT_GT(fast.received,fast.sent*.75);
}
