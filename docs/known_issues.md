# Known Issues and Limitations

---

## KI-001: PACMod v3 cannot map `body.lights.hazard`

**Status:** Open  
**Affected config:** `config/pacmod_v3.yaml`  
**Related signals:** `body.lights.turn_signal`, `body.lights.hazard`

### Background

The schema defines two separate signals for lighting state:

| Canonical name            | Type | Meaning                          |
| ------------------------- | ---- | -------------------------------- |
| `body.lights.turn_signal` | enum | OFF=0, LEFT=1, RIGHT=2, HAZARD=3 |
| `body.lights.hazard`      | bool | 0=inactive, 1=active             |

Toyota platforms expose these as two independent bits in `BLINKERS_STATE` (BO_1556):
`TURN_SIGNALS` (2-bit enum) and `HAZARD_LIGHT` (1-bit bool).
Both are mapped directly via `aliases.*` in `config/toyota_nodsu_pt_hybrid.yaml`.

PACMod v3, however, encodes all four states in a **single signal**
`TURN_RPT(560).OUTPUT_VALUE` (0=RIGHT, 1=NONE, 2=LEFT, 3=HAZARD).
There is no separate hazard bit in the PACMod DBC.

### Current behaviour

The alias system enforces a **1 DBC signal → 1 canonical name** mapping.
It is not possible to fan-out one physical signal to two canonical slots using
config-level aliases and transforms alone.

For PACMod configs:

- `body.lights.turn_signal` — correctly populated; enum value `3` represents HAZARD.
- `body.lights.hazard` — always `STATUS_INITIAL` (not published).

### Workaround

Cross-platform consumers that need to detect hazard on both platforms should evaluate:

```text
hazard_active = (body.lights.turn_signal == 3) OR (body.lights.hazard == 1)
```

### Proposed fix

Add a **derived signal** feature to the config schema that allows one canonical
signal to produce a second canonical value through an independent transform expression.
Example syntax (not yet implemented):

```yaml
derived_signals:
  - source: body.lights.turn_signal # evaluated after alias transform
    target: body.lights.hazard
    expression: "x == 3 ? 1 : 0"
```

This would let PACMod configs derive `body.lights.hazard` from the already-transformed
`turn_signal` value without modifying the DBC decoder layer.
