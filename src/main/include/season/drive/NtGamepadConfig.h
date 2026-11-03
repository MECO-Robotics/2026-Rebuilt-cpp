#pragma once
#include <cstdint>
namespace season::drive::ntgamepadconfig
{
  inline constexpr double kDefaultPeriodMs = 5.0, kDefaultStaleMs = 50.0;
  inline constexpr int64_t kMaxTimestampDisagreementUs = 20000, kMaxFutureOffsetUs = 5000;
}
