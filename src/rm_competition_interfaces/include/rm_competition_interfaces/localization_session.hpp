#pragma once
#include <algorithm>
#include <cstdint>
#include <set>
#include <string>

namespace rm_competition_interfaces
{
// Admission for one explicitly selected producer. A ticket is a work snapshot,
// never an authorization to commit after a newer notice/epoch/clock boundary.
class LocalizationSession
{
public:
  explicit LocalizationSession(std::int64_t now) : clock_high_(now), source_high_(now), revoked_(now) {}
  bool select(const std::string & id, std::uint64_t minimum_epoch, std::int64_t now = -1)
  {
    if (id.empty() || retired_.count(id)) {return false;}
    if (id == id_) {
      if (minimum_epoch < epoch_) {return false;}
      if (minimum_epoch == epoch_) {return true;}
      epoch_ = minimum_epoch;
    } else {
      if (!id_.empty()) {retired_.insert(id_);}
      id_ = id; epoch_ = minimum_epoch; sequence_ = 0; generation_ = 0; revision_ = 0;
    }
    if (now >= 0) {source_high_ = std::max(source_high_, now);}
    ++boundary_; ++ticket_; revoke();
    return true;
  }
  bool clock(std::int64_t now)
  {
    if (now < clock_high_ && !time_barrier_) {
      time_barrier_ = true; clock_epoch_ = epoch_; clock_id_ = id_;
      ++boundary_; ++ticket_; revoke();
    }
    clock_high_ = std::max(clock_high_, now);
    return !time_barrier_;
  }
  bool observe(const std::string & id, std::uint64_t epoch, std::uint64_t sequence,
    std::uint64_t generation, std::uint64_t revision, std::int64_t stamp,
    bool sample, bool valid, std::int64_t now)
  {
    clock(now);
    if (id_.empty() && !select(id, epoch)) {return false;}
    if (id != id_ || epoch < epoch_ || sequence == 0 || sequence <= sequence_ ||
      (epoch == epoch_ && (generation < generation_ || revision < revision_))) {return false;}
    if (epoch > epoch_) {epoch_ = epoch; ++boundary_; source_high_ = std::max(source_high_, now); revoke();}
    if (time_barrier_ && (id != clock_id_ || epoch > clock_epoch_) && now >= clock_high_) {
      time_barrier_ = false;
    }
    // Rejected stale samples do not ratchet sequence/version or cancel newer
    // in-flight work. An authoritative epoch boundary above still fences it.
    if (sample && (time_barrier_ || stamp <= source_high_ || stamp <= revoked_)) {return false;}
    sequence_ = sequence; generation_ = generation; revision_ = revision; ++ticket_;
    if (!sample) {if (!valid) {revoke();} return true;}
    source_high_ = stamp;
    if (!valid) {revoke();}
    return true;
  }
  void revoke() {++ticket_; revoked_ = std::max(revoked_, source_high_);}
  bool current(std::uint64_t ticket) const {return ticket == ticket_ && !time_barrier_;}
  std::uint64_t ticket() const {return ticket_;}
  std::uint64_t boundary() const {return boundary_;}
  std::uint64_t epoch() const {return epoch_;}
  const std::string & id() const {return id_;}
  bool timeBarrier() const {return time_barrier_;}
private:
  std::string id_, clock_id_;
  std::set<std::string> retired_;
  std::uint64_t epoch_{0}, sequence_{0}, generation_{0}, revision_{0};
  std::uint64_t boundary_{0}, ticket_{0}, clock_epoch_{0};
  std::int64_t clock_high_, source_high_, revoked_;
  bool time_barrier_{false};
};
}  // namespace rm_competition_interfaces
