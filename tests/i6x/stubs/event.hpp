#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <vector>

namespace LibXR
{
template <typename T> struct Callback
{
  template <typename F, typename P>
  static auto Create(F function, P pointer)
  {
    return [function, pointer](bool in_isr, T value)
    {
      function(in_isr, pointer, value);
    };
  }
};

struct Event
{
  using Handler = std::function<void(bool, uint32_t)>;
  std::map<uint32_t, std::vector<Handler>> handlers;

  void Register(uint32_t id, Handler handler) { handlers[id].push_back(handler); }
  void Active(uint32_t id, bool in_isr = false)
  {
    const auto callbacks = handlers[id];
    for (const auto& callback : callbacks)
    {
      callback(in_isr, id);
    }
  }
  void Bind(Event& source, uint32_t source_id, uint32_t target_id)
  {
    source.Register(source_id, [this, target_id](bool in_isr, uint32_t)
                    { Active(target_id, in_isr); });
  }
};
}
