# Calibration session fixtures

JSON captures used by `DeviceSessionCalibrationTests`.

## Sources

| `source` | Meaning |
|---|---|
| `device` | Raw dump from the demo app (`CalibrationSessionExporter`) |
| `device-log-reconstructed` | Synthetic arc rebuilt from Quest logcat summaries (no full point cloud in logs) |

Known good device captures:

- `quest-20260715-160854-device-contentAlign-ok.json` — 0.1.12 always-contentAlign success (mid-tilt, worldUp phase)
- `quest-20260715-155630-device.json` — 0.1.10 pre-fix session (same capture pipeline)

## Pull real sessions from Quest

After a successful calibrate, the app writes:

`/sdcard/Android/data/<package>/files/CalibrationSessions/quest-*.json`

```bash
adb pull /sdcard/Android/data/com.izmtechlab.xrsteeringdemo/files/CalibrationSessions/ ./Fixtures/
```

Then add optional expectation fields (same names as in the reconstructed fixture) and reference the file from a `[TestCase("…json")]`.

## Schema (v1)

- `handlePosition` / `headPosition`: float[3]
- `handleRotation` / `headRotation`: float[4] (x,y,z,w)
- `points`: flat float[] of xyz triplets
- optional `expected*` fields for regression asserts
