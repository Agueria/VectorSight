# VectorSight

**Author:** Cem Berk Çakır  
**Purpose:** Field-ready camera-to-coordinate estimation for PX4-based systems  
**Core idea:** Preserve and strengthen the original focal-length, reachability, and coordinate math designed by the project author

VectorSight is a field-ready vision-to-coordinate system that converts camera
detections, focal-length calibration, PX4 telemetry, and reachability logic into
reliable target position estimates. It is built around the original mathematical
approach of the project author, then organized into a configurable, testable,
and field-validation-ready Python package.

The system is intentionally dependency-light. The core coordinate logic does not
require a heavy geodesy library. Optional dependencies are only used where they
are actually needed: OpenCV/NumPy for vision, pymavlink for PX4 telemetry, and
Raspberry Pi GPIO/I2C libraries for hardware output.

## English

### What VectorSight Does

VectorSight takes a detected object in a camera frame and estimates where that
object is relative to the aircraft and, when telemetry is available, where it is
on the earth as latitude/longitude.

It combines:

- camera target detection
- focal-length calibration from a known object size
- distance estimation from apparent pixel size
- roll/pitch-aware aircraft projection inside the image
- pixel-to-ground conversion using altitude and field of view
- yaw-based body-frame to north/east rotation
- local meter offset to global GPS coordinate conversion
- reachability gating before any coordinate output is trusted
- optional GPIO/I2C output for field hardware
- JSONL logging for replay and post-test inspection

### Why It Exists

The project started from a practical coordinate calculation idea: use what the
camera sees, combine it with focal length and aircraft telemetry, and turn that
into a usable field coordinate estimate. VectorSight keeps that original math
instead of replacing it with a black-box dependency.

The goal is not just to detect an object. The goal is to produce a controlled,
explainable, testable coordinate estimate that can be inspected before field use.

### Calculation Pipeline

1. **Detect the target**
   - The frame is filtered by configured HSV ranges.
   - The largest valid contour is selected.
   - The target center and radius are estimated using the original extreme-point
     contour logic.

2. **Calibrate focal length**
   - A known real object width, known distance, and observed pixel width are used
     to calculate `focal_px`.
   - This keeps calibration tied to the real camera and target setup.

3. **Estimate distance**
   - The configured real target size and detected pixel size are used to estimate
     camera-to-target distance.

4. **Project aircraft attitude into the image**
   - Roll and pitch are converted into a projected aircraft point in pixel space.
   - This preserves the original idea that the camera center is not always the
     true aircraft reference point when the platform tilts.

5. **Apply reachability logic**
   - Both the detected target and the projected aircraft point must be inside the
     center-half shooting zone.
   - If either point is outside that zone, VectorSight returns `NOT_REACHABLE`
     and does not emit local/global coordinates.

6. **Convert pixels to meters**
   - Altitude, horizontal FOV, vertical FOV, and selected resolution are used to
     calculate meter-per-pixel scale.
   - The system supports both the preserved legacy geometry mode and a refined
     FOV-based mode.

7. **Rotate into north/east coordinates**
   - The body-relative offset is rotated by yaw.
   - This produces a local north/east meter offset.

8. **Estimate global position**
   - The local north/east offset is converted into latitude/longitude around the
     aircraft GPS position.
   - This avoids adding a heavy geodesy dependency while keeping the calculation
     transparent and testable.

### Output Statuses

VectorSight returns explicit status values instead of silent failures:

| Status | Meaning |
|---|---|
| `OK` | Target is reachable and local/global coordinates were produced. |
| `NO_TARGET` | No valid target was detected in the frame. |
| `UNCALIBRATED` | `focal_px` is missing; calibration is required first. |
| `NO_GPS` | Global position telemetry is missing or stale. |
| `NO_ATTITUDE` | Attitude telemetry is missing or stale. |
| `BAD_ALTITUDE` | Relative altitude is zero or invalid. |
| `NOT_REACHABLE` | Target or aircraft projection is outside the configured safe zone. |

### Resolution Presets

Resolution is selected from config instead of being hardcoded across the code:

| Preset | Size |
|---|---|
| `HD_720` | `1280 x 720` |
| `FHD_1080` | `1920 x 1080` |
| `LEGACY_1440_1080` | `1440 x 1080` |
| `CUSTOM` | User-provided width and height |

### Hardware Behavior

VectorSight keeps the legacy field behavior through optional adapters:

- GPIO reachable LED on BCM pin `21`
- I2C target reporter using the old 8-byte float payload
- found-signal GPIO pin `3`
- hardware disabled by default for safe development
- no hardware dependency required unless the hardware extra is installed

### Visual Overlay

The live preview can draw:

- center crosshair
- center-half shooting zone
- detected target center
- target radius
- contour diagonals
- aircraft-to-target reachable line
- current status and distance/coordinate text

This makes field testing easier because the operator can see why a target is
accepted, rejected, or marked unreachable.

### Installation

From the project directory:

```powershell
cd C:\Users\Valo-\Desktop\proj\field_coordinate_system
python -m pip install -e .
```

For camera and PX4 telemetry:

```powershell
python -m pip install -e ".[vision,px4]"
```

For Raspberry Pi GPIO/I2C output:

```powershell
python -m pip install -e ".[hardware]"
```

For development and tests:

```powershell
python -m pip install -e ".[dev]"
```

### Commands

Calibrate focal length:

```powershell
python -m field_coordinate calibrate --config config/default.json --known-width-m 0.07 --distance-m 1.05 --observed-width-px 70
```

Run live camera/PX4 estimation:

```powershell
python -m field_coordinate run --config config/default.json
```

Replay a JSONL session log:

```powershell
python -m field_coordinate replay --log logs/session.jsonl
```

### Configuration

Main config file:

```text
config/default.json
```

Important fields:

| Field | Purpose |
|---|---|
| `resolution_preset` | Selects camera resolution preset. |
| `geometry_mode` | Selects `LEGACY` or `REFINED` geometry. |
| `horizontal_fov_deg` | Camera horizontal field of view. |
| `vertical_fov_deg` | Camera vertical field of view. |
| `target_real_size_m` | Real size of the detected target/object. |
| `focal_px` | Calibrated focal length in pixels. |
| `hsv_ranges` | Color ranges used for detection. |
| `mavlink_ports` | Candidate PX4 serial ports. |
| `gpio_enabled` | Enables/disables GPIO output. |
| `i2c_enabled` | Enables/disables I2C target reporting. |

The default config intentionally keeps `focal_px` as `null`. Field use should
start with calibration for the actual camera, lens, target, and resolution.

### Project Structure

```text
field_coordinate_system/
  config/                 Default runtime configuration
  docs/testing/           TDD evidence and validation notes
  src/field_coordinate/   VectorSight source package
  tests/                  Unit and integration tests
```

Important modules:

| Module | Responsibility |
|---|---|
| `camera_geometry.py` | Focal length, distance, pixel scale, aircraft projection. |
| `coordinates.py` | Local north/east rotation and GPS conversion. |
| `vision.py` | HSV filtering, contour selection, detection, overlay. |
| `telemetry.py` | MAVLink unit normalization and stale telemetry cache. |
| `pipeline.py` | End-to-end target estimate generation. |
| `hardware.py` | Optional GPIO LED and I2C reporter adapters. |
| `state.py` | Attitude change filtering. |
| `cli.py` | Calibrate, run, and replay commands. |

### Test Status

Current validation:

```text
45 passed
90% coverage
```

Run locally:

```powershell
python -m pytest tests --cov=src/field_coordinate --cov-report=term-missing
```

### Ownership

Copyright (c) 2026 Cem Berk Çakır.

The VectorSight coordinate estimation logic, focal-length-based targeting math,
reachability behavior, and project direction are authored by Cem Berk Çakır.
If this repository is published with an open-source license, keep the copyright
notice and author attribution intact.

### License

VectorSight is licensed under the Apache License 2.0. See `LICENSE` for the full
license text. The license allows use, modification, and distribution under its
terms while preserving copyright and attribution notices.

### Field Use Notice

VectorSight is a calculation and field-validation system. Before real field use,
verify camera calibration, resolution, FOV values, target size, telemetry units,
hardware wiring, and output behavior in a controlled environment.

---

## Türkçe

### VectorSight Nedir?

VectorSight, kamera görüntüsündeki bir hedefi algılayıp bu hedefin araca göre
yerel konumunu ve uygun telemetri varsa GPS koordinatını tahmin eden saha
hazırlıklı bir görüntüden koordinata hesaplama sistemidir.

Sistem şunları birleştirir:

- kamera üzerinden hedef algılama
- bilinen gerçek boyuttan focal length kalibrasyonu
- pikselde görünen boyuttan mesafe hesabı
- roll/pitch değerlerine göre görüntü içinde araç izdüşümü
- altitude ve FOV ile pikselden metreye dönüşüm
- yaw ile gövde ekseninden kuzey/doğu eksenine dönüşüm
- yerel metre ofsetinden GPS koordinatı üretimi
- koordinat üretmeden önce erişilebilirlik kontrolü
- saha donanımı için opsiyonel GPIO/I2C çıkışı
- test sonrası inceleme için JSONL loglama

### Neden Var?

Bu proje pratik bir koordinat hesabı fikrinden doğdu: kameranın gördüğü hedefi,
focal length hesabı ve uçuş telemetrisiyle birleştirip kullanılabilir bir saha
koordinatına dönüştürmek.

VectorSight bu matematik fikrini dışarıdan hazır bir kara kutuyla değiştirmez.
Tam tersine, Cem Berk Çakır tarafından yazılan orijinal matematik yaklaşımını
korur, düzenler, test eder ve saha testine hazır bir paket haline getirir.

Amaç sadece bir nesneyi bulmak değildir. Amaç, neden doğru veya yanlış olduğunu
görebileceğin, test edilebilir, açıklanabilir ve kontrollü bir koordinat tahmini
üretmektir.

### Hesaplama Akışı

1. **Hedef algılanır**
   - Görüntü ayarlı HSV aralıklarıyla filtrelenir.
   - Geçerli en büyük contour seçilir.
   - Hedef merkezi ve yarıçapı, eski koddaki uç nokta mantığı korunarak
     hesaplanır.

2. **Focal length kalibre edilir**
   - Gerçek hedef genişliği, bilinen mesafe ve görüntüdeki piksel genişliğiyle
     `focal_px` hesaplanır.
   - Böylece hesap gerçek kamera, lens, hedef ve çözünürlük kurulumuna bağlanır.

3. **Mesafe tahmini yapılır**
   - Ayarlanan gerçek hedef boyutu ve görüntüdeki piksel boyutu kullanılarak
     kamera-hedef mesafesi tahmin edilir.

4. **Aracın görüntüdeki izdüşümü hesaplanır**
   - Roll ve pitch değerleri görüntüdeki bir aircraft point değerine çevrilir.
   - Bu, platform eğildiğinde kamera merkezinin her zaman doğru referans noktası
     olmadığı fikrini korur.

5. **Erişilebilirlik kontrolü yapılır**
   - Hem hedef noktası hem de aracın izdüşüm noktası görüntünün orta yarısındaki
     güvenli bölge içinde olmalıdır.
   - İkisinden biri dışarıdaysa sistem `NOT_REACHABLE` döndürür ve koordinat
     üretmez.

6. **Piksel metreye çevrilir**
   - Altitude, yatay FOV, dikey FOV ve seçilen çözünürlük ile metre/piksel
     ölçeği hesaplanır.
   - Sistem hem korunmuş `LEGACY` modu hem de daha rafine `REFINED` modu
     destekler.

7. **Kuzey/doğu eksenine döndürülür**
   - Gövdeye göre hesaplanan ofset yaw değeriyle döndürülür.
   - Sonuç kuzey/doğu metre ofsetidir.

8. **GPS koordinatı tahmin edilir**
   - Yerel kuzey/doğu ofseti, aracın GPS konumu etrafında latitude/longitude
     değerine dönüştürülür.
   - Bu yaklaşım ağır geodesy bağımlılığı eklemeden hesabı açık ve test
     edilebilir tutar.

### Durum Kodları

VectorSight sessiz hata üretmez; her durumda açık bir status döndürür:

| Status | Anlamı |
|---|---|
| `OK` | Hedef erişilebilir ve koordinat üretildi. |
| `NO_TARGET` | Görüntüde geçerli hedef bulunamadı. |
| `UNCALIBRATED` | `focal_px` eksik; önce kalibrasyon gerekir. |
| `NO_GPS` | GPS telemetrisi yok veya eski. |
| `NO_ATTITUDE` | Attitude telemetrisi yok veya eski. |
| `BAD_ALTITUDE` | Relative altitude sıfır veya geçersiz. |
| `NOT_REACHABLE` | Hedef veya araç izdüşümü güvenli bölge dışında. |

### Çözünürlük Seçenekleri

Çözünürlük artık kod içine dağılmış sabit değerlerden değil config üzerinden
seçilir:

| Preset | Boyut |
|---|---|
| `HD_720` | `1280 x 720` |
| `FHD_1080` | `1920 x 1080` |
| `LEGACY_1440_1080` | `1440 x 1080` |
| `CUSTOM` | Kullanıcı tarafından verilen genişlik ve yükseklik |

### Donanım Davranışı

Eski saha davranışları opsiyonel adaptörlerle korunur:

- BCM pin `21` üzerinde reachable LED çıkışı
- eski 8-byte float payload ile uyumlu I2C hedef bildirimi
- found sinyali için GPIO pin `3`
- güvenli geliştirme için donanım varsayılan olarak kapalıdır
- donanım kütüphaneleri sadece hardware extra kurulursa gerekir

### Canlı Görsel Overlay

Canlı önizleme şunları çizebilir:

- merkez crosshair
- orta yarı shooting zone
- algılanan hedef merkezi
- hedef yarıçapı
- contour diagonalleri
- aircraft-to-target erişilebilirlik çizgisi
- mevcut status, mesafe ve koordinat yazısı

Bu sayede saha testinde hedefin neden kabul edildiği, reddedildiği veya
erişilemez sayıldığı ekranda görülebilir.

### Kurulum

Proje klasöründen:

```powershell
cd C:\Users\Valo-\Desktop\proj\field_coordinate_system
python -m pip install -e .
```

Kamera ve PX4 telemetrisi için:

```powershell
python -m pip install -e ".[vision,px4]"
```

Raspberry Pi GPIO/I2C çıkışı için:

```powershell
python -m pip install -e ".[hardware]"
```

Geliştirme ve test için:

```powershell
python -m pip install -e ".[dev]"
```

### Komutlar

Focal length kalibrasyonu:

```powershell
python -m field_coordinate calibrate --config config/default.json --known-width-m 0.07 --distance-m 1.05 --observed-width-px 70
```

Canlı kamera/PX4 hesabı:

```powershell
python -m field_coordinate run --config config/default.json
```

JSONL session log replay:

```powershell
python -m field_coordinate replay --log logs/session.jsonl
```

### Config

Ana config dosyası:

```text
config/default.json
```

Önemli alanlar:

| Alan | Görevi |
|---|---|
| `resolution_preset` | Kamera çözünürlük presetini seçer. |
| `geometry_mode` | `LEGACY` veya `REFINED` geometrisini seçer. |
| `horizontal_fov_deg` | Kameranın yatay görüş açısı. |
| `vertical_fov_deg` | Kameranın dikey görüş açısı. |
| `target_real_size_m` | Hedefin gerçek boyutu. |
| `focal_px` | Piksel cinsinden kalibre focal length. |
| `hsv_ranges` | Algılama için renk aralıkları. |
| `mavlink_ports` | Denenecek PX4 seri portları. |
| `gpio_enabled` | GPIO çıkışını açar/kapatır. |
| `i2c_enabled` | I2C hedef bildirimini açar/kapatır. |

Varsayılan config içinde `focal_px` bilinçli olarak `null` bırakılmıştır. Saha
kullanımından önce gerçek kamera, lens, hedef ve çözünürlükle kalibrasyon
yapılmalıdır.

### Proje Yapısı

```text
field_coordinate_system/
  config/                 Varsayılan çalışma ayarları
  docs/testing/           TDD kanıtları ve doğrulama notları
  src/field_coordinate/   VectorSight kaynak paketi
  tests/                  Unit ve integration testleri
```

Önemli modüller:

| Modül | Görev |
|---|---|
| `camera_geometry.py` | Focal length, mesafe, piksel ölçeği, aircraft projection. |
| `coordinates.py` | Kuzey/doğu dönüşümü ve GPS koordinat hesabı. |
| `vision.py` | HSV filtreleme, contour seçimi, detection, overlay. |
| `telemetry.py` | MAVLink unit normalization ve stale telemetry cache. |
| `pipeline.py` | Uçtan uca target estimate üretimi. |
| `hardware.py` | Opsiyonel GPIO LED ve I2C reporter adaptörleri. |
| `state.py` | Attitude change filter. |
| `cli.py` | Calibrate, run ve replay komutları. |

### Test Durumu

Mevcut doğrulama:

```text
45 passed
90% coverage
```

Yerelde çalıştırmak için:

```powershell
python -m pytest tests --cov=src/field_coordinate --cov-report=term-missing
```

### Sahiplik

Copyright (c) 2026 Cem Berk Çakır.

VectorSight koordinat tahmin mantığı, focal length tabanlı hedef matematiği,
erişilebilirlik yaklaşımı ve proje yönü Cem Berk Çakır tarafından yazılmış ve
tasarlanmıştır. Bu repository açık kaynak lisansıyla yayınlanırsa, copyright
notu ve yazar atfı korunmalıdır.

### Lisans

VectorSight, Apache License 2.0 ile lisanslanmıştır. Tam lisans metni için
`LICENSE` dosyasına bakın. Bu lisans; kullanım, değiştirme ve dağıtıma izin
verirken copyright ve atıf notlarının korunmasını ister.

### Saha Kullanım Notu

VectorSight bir hesaplama ve saha doğrulama sistemidir. Gerçek saha kullanımına
geçmeden önce kamera kalibrasyonu, çözünürlük, FOV değerleri, hedef boyutu,
telemetri birimleri, donanım bağlantıları ve çıkış davranışı kontrollü ortamda
doğrulanmalıdır.
