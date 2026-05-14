// Copyright 2026 TIER IV, Inc.

#pragma once

#include "vehicle_can_decoder/signal_router.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>

namespace vehicle_can_decoder
{

/// Result of alias resolution and domain routing for a single raw signal.
struct SignalResolution
{
  std::string name;    ///< Resolved (aliased) signal name
  std::string domain;  ///< Domain name, or SignalRouter::kUnassignedDomain

  bool is_assigned() const noexcept { return domain != SignalRouter::kUnassignedDomain; }
};

/// Resolve alias and determine routing domain for a raw signal name.
///
/// Tries the compound key "CAN{can_id}_{raw_name}" first so that signals with
/// the same DBC name in different messages can be aliased independently.
/// Falls back to the bare signal name alias, then to CAN-ID-based routing.
inline SignalResolution resolve_signal(
  const std::string & raw_name,
  uint32_t can_id,
  const SignalRouter & router,
  const std::unordered_map<std::string, std::string> & signal_to_domain)
{
  // Compound key disambiguation: "CAN{id}_{signal}" takes precedence over
  // bare signal name so two DBC messages sharing a signal name can be routed
  // independently via aliases.
  const std::string compound_key = "CAN" + std::to_string(can_id) + "_" + raw_name;
  const std::string & compound_alias = router.apply_alias(compound_key);
  const std::string & resolved =
    (compound_alias != compound_key) ? compound_alias : router.apply_alias(raw_name);

  std::string domain;
  if (!signal_to_domain.empty()) {
    const auto it = signal_to_domain.find(resolved);
    domain = (it != signal_to_domain.end()) ? it->second
                                            : std::string(SignalRouter::kUnassignedDomain);
  } else {
    domain = router.domain_for_id(can_id);
  }

  return {std::string(resolved), std::move(domain)};
}

}  // namespace vehicle_can_decoder
