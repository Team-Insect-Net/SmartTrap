# SmartTrap v1.0 - Field Deployment & Maintenance Guide

## 1. Field Deployment

### Site Selection

- Place trap at field edge or within crop canopy
- Avoid direct sunlight on electronics
- Ensure IR sensor path is clear
- Position soil sensors in representative soil

### Installation Checklist

- [ ] SD card formatted FAT32
- [ ] Power bank fully charged
- [ ] RTC time verified
- [ ] Fresh pheromone lure
- [ ] IR sensor aligned
- [ ] Enclosure sealed

### Expected Battery Life

| Power Bank | Capacity | Runtime |
| --- | --- | --- |
| Small | 10,000 mAh | 3-4 weeks |
| Medium | 20,000 mAh | 6-8 weeks |

## 2. Maintenance Schedule

### Weekly Tasks

- Check power bank level
- Clear trap of insects
- Verify IR sensor alignment

### Bi-Weekly Tasks

- Replace pheromone lure (2-4 weeks)
- Clean camera lens
- Inspect for moisture

### Monthly Tasks

- Full data export via USB Drive Mode
- Delete /logs and /events folders to reset count
- RTC stays accurate (~1 min/year drift); re-flash firmware to re-sync if needed

## 3. Data Collection

### Files to Download

The system creates two separate CSV files for better analysis.

| File | Contents |
| --- | --- |
| /logs/environment.csv | Periodic readings: timestamp, air_temp, humidity, soil_temp, soil_moisture |
| /logs/detections.csv | Detection events: timestamp, detection_num, env data, video/audio paths |
| /events/YYYYMMDD/ | Video (.avi) and audio (.wav) recordings by date |

### Data Collection via USB Drive Mode (Primary Method)

To download data via USB, press the BUTTON during startup:
1. Plug USB-C cable into your computer
2. Press BUTTON within 10 seconds when you see the countdown
3. SD card appears as a removable USB drive
4. Copy /logs/ and /events/ folders to your computer
5. Safely eject the drive and unplug

**Note:** If you don't press the button, the device enters Normal Mode (monitoring). This is intentional for field deployment with power banks!

### Resetting the Detection Count

The moth detection count is derived from the data on the SD card. To reset:
1. Connect via USB Drive Mode
2. Delete the `/logs/` and `/events/` folders
3. Reboot the device — count starts at 0

No BLE connection or special commands needed.

### Data Offloading via SD Card Reader (Alternative)

For faster bulk transfers, remove the SD card and use a USB card reader:
1. Power off the device
2. Gently remove the microSD card from the expansion board slot
3. Insert into a USB microSD card reader
4. Copy /logs/ and /events/ folders to your computer
5. Re-insert card and power on device

## 4. Common Field Issues

| Issue | Solution |
| --- | --- |
| False detections | Adjust IR alignment, shield from wind |
| Missed detections | Clean IR sensor, verify LED working |
| Moisture damage | Improve sealing, add desiccant |
| SD card full | Download data via USB, delete /logs and /events |

## 5. Support

**Project PI:** Dr. Edward Idun Amoah (Penn State)
**Ghana PI:** Dr. Kofi Frimpong-Anin (CSIR-CRI)
