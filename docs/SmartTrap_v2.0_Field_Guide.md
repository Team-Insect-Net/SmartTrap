# SmartTrap v2.0 — Field Deployment & Maintenance Guide

In v2.0 all field interaction is through the **SmartTrap Android app** over
Bluetooth — there is no button or LCD on the device.

**Android app (APK):**
<https://drive.google.com/file/d/1rnNRD0PhtQwLzY0d07APNC46B7DXw_hL/view?usp=sharing>

> Install by enabling "Install unknown apps" for your browser/file manager, then
> open the downloaded APK. Grant Bluetooth (and Location, if prompted) permissions.

---

## 1. Field Deployment

### Site Selection

- Place the trap at the field edge or within the crop canopy
- Avoid direct sunlight on the electronics
- Ensure each IR beam path is clear across the trap throat
- Keep the phone within BLE range (~10m) when checking on the trap

### Installation Checklist

- [ ] SD card formatted FAT32 (≤32GB)
- [ ] Power bank fully charged
- [ ] Phone paired with the trap (passkey **123456**)
- [ ] RTC time set from the app (`SETTIME`)
- [ ] Fresh pheromone lure
- [ ] All 4 IR beams aligned (`DIAG` shows `ir=CLEAR`)
- [ ] Enclosure sealed

### Expected Battery Life

| Power Bank | Capacity | Runtime |
| --- | --- | --- |
| Small | 10,000 mAh | 3–4 weeks |
| Medium | 20,000 mAh | 6–8 weeks |

---

## 2. Maintenance Schedule

### Weekly

- Check power bank level (app `STATUS` → uptime)
- Clear the trap of accumulated insects
- Verify IR beams with `DIAG` (`ir=CLEAR`) or watch for `BEAM:BLOCKED` pushes

### Bi-Weekly

- Replace pheromone lure (2–4 weeks)
- Clean the camera lens
- Inspect for moisture

### Monthly

- Full image + log export (WiFi image download, or USB drive mode)
- `RESET` from the app to clear data and zero the count (optional)
- Confirm RTC time; re-send `SETTIME` if it has drifted

---

## 3. Data Collection

### Files on the SD card

| Path | Contents |
| --- | --- |
| /events/YYYYMMDD/img_*.jpg | 10-frame JPEG burst per detection |
| /daily/YYYYMMDD_HH.jpg | Scheduled daily photo(s) |
| /logs/detections.csv | Detection events: timestamp, count, event dir, frames |
| /logs/beam_health.csv | IR beam block/restore events |
| /logs/daily_photos.csv | Scheduled-photo log |

> v2.0 records images and counts only — there is **no environmental (temperature/
> humidity/soil) logging**.

### Method A — Fast image download over WiFi (recommended for photos)

1. In the app, send **`WIFI:ON`** — the trap starts a WiFi hotspot
   (SSID `SmartTrap_001`, password `trap12345`) and reports its URL.
2. Join that WiFi from the phone and download the JPEGs over HTTP.
3. Send **`WIFI:OFF`** when done to save power.

### Method B — File browse/download over BLE (good for logs)

1. Connect and pair in the app.
2. Browse with `LIST` / `CD`, then `GET` the CSV logs or individual images.

### Method C — USB drive mode (bulk offload at a bench)

1. In the app, send **`USB`** — the SD card mounts as a USB Mass Storage drive.
2. Plug the trap into a computer with USB-C and copy `/events`, `/daily`, `/logs`.
3. Safely eject and unplug.

### Method D — SD card reader (fastest bulk transfer)

1. Power off the trap, remove the microSD card.
2. Copy the folders using a USB card reader, then re-insert and power on.

### Resetting the detection count

The count is derived from `detections.csv`. To reset: send **`RESET`** over BLE
(wipes `/events` + `/logs`, count → 0), or delete those folders in USB drive mode
and reboot.

---

## 4. Common Field Issues

| Issue | Solution |
| --- | --- |
| False detections | Re-align IR beams; shield from wind and stray IR/sunlight |
| Missed detections | Clean IR emitters/receivers; confirm `ir=CLEAR` in `DIAG` |
| `BEAM:BLOCKED` alerts | Clear debris from the beam path; re-align LED/receiver |
| Can't connect | Re-pair (passkey 123456); confirm the trap has power |
| Moisture damage | Improve sealing, add desiccant |
| SD card full | Download data, then `RESET` or delete folders |

---

## 5. Support

**Project PI:** Dr. Edward Idun Amoah (Penn State)
**Ghana PI:** Dr. Kofi Frimpong-Anin (CSIR-CRI)

---

*SmartTrap v2.0 — Penn State & CSIR-CRI collaboration. Licensed CC BY-NC-SA 4.0.*
