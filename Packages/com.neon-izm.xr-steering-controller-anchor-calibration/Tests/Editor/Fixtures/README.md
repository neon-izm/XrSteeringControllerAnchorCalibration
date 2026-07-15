# Calibration session fixtures

JSON captures used by `DeviceSessionCalibrationTests`.

Phase / roll always use **world up** (`Vector3.up`) in the library; fixtures only assert geometric expectations (radius, normalDotUp, handle-local roll, driver seat).

## Sources

| `source` | Meaning |
|---|---|
| `device` | Raw dump from the demo app (`CalibrationSessionExporter`) |
| `device-log-reconstructed` | Synthetic arc rebuilt from Quest logcat summaries (no full point cloud in logs) |

Known good device captures:

- `quest-20260715-160854-device-contentAlign-ok.json` — contentAlign success (mid-tilt)
- `quest-20260715-155630-device.json` — earlier device session
- `quest-20260716-midtilt-log-reconstructed.json` — log-reconstructed mid-tilt

## Pull real sessions from Quest

After a successful calibrate, the app writes:

`/sdcard/Android/data/<package>/files/CalibrationSessions/quest-*.json`

```bash
adb pull /sdcard/Android/data/com.izmtechlab.xrsteeringdemo/files/CalibrationSessions/ ./Fixtures/
```

Then add optional expectation fields and reference the file from a `[TestCase("…json")]`.

## Schema (v1)

- `handlePosition` / `headPosition`: float[3]
- `handleRotation` / `headRotation`: float[4] (x,y,z,w)
- `points`: flat float[] of xyz triplets
- optional `expectedCircleRadiusApprox`, `expectedNormalDotUpApprox`, `expectedMaxHandleLocalRollDeg`, …
