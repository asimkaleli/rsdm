# orientation.py
# MPU-9250/9255 (accel+gyro) + AK8963 (mag) — NO MOCK
# QThread tabanlı worker: orientation(roll, pitch, yaw, temp) yayımlar.
# Gövde eksenleri: +X=ön, +Y=sağ, +Z=aşağı (aerospace convention)
# Pitch: yukarı negatif, aşağı pozitif. Roll: sağa pozitif. Yaw: saat yönü pozitif.

import time
import math
from PySide2.QtCore import QThread, Signal

try:
    from smbus2 import SMBus
except Exception:
    SMBus = None

# ---------------- I2C / Adresler ----------------
I2C_BUS_NUM = 1
MPU_ADDRS       = (0x68, 0x69)
MPU_WHO_AM_I    = 0x75
WHO_VALID       = (0x68, 0x71, 0x73)  # 6050=0x68, 9250/55=0x71/0x73

# MPU kayıtları
MPU_PWR_MGMT_1   = 0x6B
MPU_USER_CTRL    = 0x6A
MPU_INT_PIN_CFG  = 0x37
MPU_ACCEL_XOUT_H = 0x3B
MPU_TEMP_OUT_H   = 0x41
MPU_GYRO_XOUT_H  = 0x43

# Ölçekler (reset sonrası varsayılanlar)
ACC_LSB_PER_G     = 16384.0      # ±2g
GYRO_LSB_PER_DPS  = 131.0        # ±250 dps

# AK8963 (manyetometre)
AK_ADDR   = 0x0C
AK_WIA    = 0x00  # WHO_AM_I -> 0x48
AK_ST1    = 0x02  # bit0: DRDY
AK_HXL    = 0x03  # HXL..HZH (6B) + ST2
AK_ST2    = 0x09  # bit3: HOFL (overflow)
AK_CNTL1  = 0x0A
AK_CNTL2  = 0x0B
AK_ASAX   = 0x10  # ASA kalibrasyon (x,y,z)

AK_MODE_POWER_DOWN        = 0x00
AK_MODE_FUSE_ROM          = 0x0F
AK_MODE_CONT_100HZ_16BIT  = 0x16

AK_WHO_EXPECT = 0x48
AK_16BIT_uT_PER_LSB = 0.15  # 16-bit çözünürlükte 0.15 µT/LSB

# ---- AK8963 -> Gövde eksen dönüşümü (datasheet yönlerine göre)
# Mag eksenleri, accel/gyro ile hizalı DEĞİL:
#   mx_c = my_raw
#   my_c = mx_raw
#   mz_c = -mz_raw
def _map_mag_axes(mx_uT, my_uT, mz_uT):
    return (my_uT, mx_uT, -mz_uT)

# ---------------- Yardımcılar ----------------
def _twos16(hi, lo):
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v

def _deg_wrap(a):
    a %= 360.0
    return a + 360.0 if a < 0 else a

# ---------------- Worker ----------------
class OrientationWorker(QThread):
    """
    Signals:
        orientation(roll, pitch, yaw, temp)
        error(str)
    """
    orientation = Signal(float, float, float, float)
    error = Signal(str)

    def __init__(self,
                 i2c_addr=0x68,
                 hz=50,
                 gyro_bias_samples=300,
                 alpha=0.98,           # roll/pitch complementary
                 use_mag=True,
                 yaw_alpha=0.6,        # yaw füzyon: 1.0 → gyro baskın, 0.0 → mag baskın
                 declination_deg=6.0,  # İstanbul ~ +5..6°
                 parent=None):
        super().__init__(parent)
        self.addr = i2c_addr
        self.hz = max(1, int(hz))
        self.dt = 1.0 / float(self.hz)

        self.bias_samples = max(50, int(gyro_bias_samples))
        self.alpha = float(alpha)

        self.use_mag = bool(use_mag)
        self.yaw_alpha = float(yaw_alpha)
        self.declination_deg = float(declination_deg)

        self._run = False
        self._bus = None

        # Gyro bias (°/s)
        self._bx = self._by = self._bz = 0.0

        # Açı durumları
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

        # Mag durumu
        self._have_mag = False
        self._mag_adj = (1.0, 1.0, 1.0)  # ASA düzeltmeleri

    # ---------- I2C ----------
    def _open_bus(self):
        if SMBus is None:
            raise RuntimeError("smbus2 kurulu değil (pip install smbus2).")
        return SMBus(I2C_BUS_NUM)

    def _detect_mpu(self, bus):
        addrs = (self.addr,) + tuple(a for a in MPU_ADDRS if a != self.addr)
        for a in addrs:
            try:
                who = bus.read_byte_data(a, MPU_WHO_AM_I)
                if who in WHO_VALID:
                    return a
            except Exception:
                pass
        return None

    def _mpu_wake(self, bus, addr):
        bus.write_byte_data(addr, MPU_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

    def _enable_bypass(self, bus, addr):
        # USER_CTRL[I2C_MST_EN]=0 ve INT_PIN_CFG[BYPASS_EN]=1
        try:
            uc = bus.read_byte_data(addr, MPU_USER_CTRL)
            uc &= ~(1 << 5)  # I2C_MST_EN=0
            bus.write_byte_data(addr, MPU_USER_CTRL, uc)
            time.sleep(0.005)
        except Exception:
            pass
        try:
            ipc = bus.read_byte_data(addr, MPU_INT_PIN_CFG)
            ipc |= (1 << 1)  # BYPASS_EN=1
            bus.write_byte_data(addr, MPU_INT_PIN_CFG, ipc)
            time.sleep(0.005)
        except Exception:
            pass

    # ---------- AK8963 ----------
    def _ak_soft_reset(self):
        try:
            self._bus.write_byte_data(AK_ADDR, AK_CNTL2, 0x01)
            time.sleep(0.01)
        except Exception:
            pass

    def _ak_read_asa(self):
        try:
            self._bus.write_byte_data(AK_ADDR, AK_CNTL1, AK_MODE_POWER_DOWN)
            time.sleep(0.01)
            self._bus.write_byte_data(AK_ADDR, AK_CNTL1, AK_MODE_FUSE_ROM)
            time.sleep(0.01)
            asa = self._bus.read_i2c_block_data(AK_ADDR, AK_ASAX, 3)
            self._bus.write_byte_data(AK_ADDR, AK_CNTL1, AK_MODE_POWER_DOWN)
            time.sleep(0.01)
            adj = [((a - 128) / 256.0) + 1.0 for a in asa]
            return tuple(adj)
        except Exception:
            return (1.0, 1.0, 1.0)

    def _ak_set_mode(self):
        try:
            self._bus.write_byte_data(AK_ADDR, AK_CNTL1, AK_MODE_CONT_100HZ_16BIT)
            time.sleep(0.01)
            return True
        except Exception:
            return False

    def _init_mag(self, bus, mpu_addr):
        if not self.use_mag:
            self._have_mag = False
            return
        self._enable_bypass(bus, mpu_addr)
        time.sleep(0.01)
        try:
            wia = bus.read_byte_data(AK_ADDR, AK_WIA)
        except Exception:
            wia = None
        if wia != AK_WHO_EXPECT:
            self._have_mag = False
            return
        self._ak_soft_reset()
        self._mag_adj = self._ak_read_asa()
        if not self._ak_set_mode():
            self._have_mag = False
            return
        self._have_mag = True

    def _ak_read_raw(self):
        try:
            st1 = self._bus.read_byte_data(AK_ADDR, AK_ST1)
            if (st1 & 0x01) == 0:
                return None
            raw = self._bus.read_i2c_block_data(AK_ADDR, AK_HXL, 7)
            mx = _twos16(raw[1], raw[0])
            my = _twos16(raw[3], raw[2])
            mz = _twos16(raw[5], raw[4])
            st2 = raw[6]
            if (st2 & 0x08) != 0:  # overflow
                return None
            # ASA düzeltmesi + ölçek (µT)
            mx_uT = mx * self._mag_adj[0] * AK_16BIT_uT_PER_LSB
            my_uT = my * self._mag_adj[1] * AK_16BIT_uT_PER_LSB
            mz_uT = mz * self._mag_adj[2] * AK_16BIT_uT_PER_LSB
            return (mx_uT, my_uT, mz_uT)
        except Exception:
            return None

    # ---------- Matematik ----------
    @staticmethod
    def _acc_to_angles(ax_g, ay_g, az_g):
        # Aerospace konv.: burnu yukarı (nose up) → pitch negatif
        roll  = math.degrees(math.atan2(ay_g, az_g))
        pitch = math.degrees(math.atan2(-ax_g, math.sqrt(ay_g*ay_g + az_g*az_g)))
        return roll, pitch

    # ---------- Başlangıç ----------
    def _calibrate_bias(self, bus, addr):
        bx = by = bz = 0.0
        for _ in range(self.bias_samples):
            g = bus.read_i2c_block_data(addr, MPU_GYRO_XOUT_H, 6)
            gx = _twos16(g[0], g[1]) / GYRO_LSB_PER_DPS
            gy = _twos16(g[2], g[3]) / GYRO_LSB_PER_DPS
            gz = _twos16(g[4], g[5]) / GYRO_LSB_PER_DPS
            bx += gx; by += gy; bz += gz
            time.sleep(0.002)
        n = float(self.bias_samples)
        self._bx, self._by, self._bz = bx/n, by/n, bz/n

    def _init_angles(self, bus, addr):
        a = bus.read_i2c_block_data(addr, MPU_ACCEL_XOUT_H, 6)
        ax = _twos16(a[0], a[1]) / ACC_LSB_PER_G
        ay = _twos16(a[2], a[3]) / ACC_LSB_PER_G
        az = _twos16(a[4], a[5]) / ACC_LSB_PER_G
        self._roll, self._pitch = self._acc_to_angles(ax, ay, az)
        self._yaw = 0.0  # gyro-only başlangıç

    # ---------- Thread ----------
    def run(self):
        self._run = True
        try:
            self._bus = self._open_bus()
            real_addr = self._detect_mpu(self._bus)
            if real_addr is None:
                raise RuntimeError("IMU bulunamadı (WHO_AM_I eşleşmedi).")
            self.addr = real_addr
            self._mpu_wake(self._bus, self.addr)
            self._init_mag(self._bus, self.addr)  # varsa aç
            self._calibrate_bias(self._bus, self.addr)
            self._init_angles(self._bus, self.addr)
        except Exception as e:
            self._cleanup()
            self.error.emit(str(e))
            return

        next_t = time.time()
        while self._run:
            try:
                # --- ACC (g) ---
                ab = self._bus.read_i2c_block_data(self.addr, MPU_ACCEL_XOUT_H, 6)
                ax = _twos16(ab[0], ab[1]) / ACC_LSB_PER_G
                ay = _twos16(ab[2], ab[3]) / ACC_LSB_PER_G
                az = _twos16(ab[4], ab[5]) / ACC_LSB_PER_G

                # --- TEMP (°C) ---
                tb = self._bus.read_i2c_block_data(self.addr, MPU_TEMP_OUT_H, 2)
                temp_raw = _twos16(tb[0], tb[1])
                temp_c = (temp_raw / 340.0) + 36.53

                # --- GYRO (°/s) (bias düşülmüş) ---
                gb = self._bus.read_i2c_block_data(self.addr, MPU_GYRO_XOUT_H, 6)
                gx = (_twos16(gb[0], gb[1]) / GYRO_LSB_PER_DPS) - self._bx
                gy = (_twos16(gb[2], gb[3]) / GYRO_LSB_PER_DPS) - self._by
                gz = (_twos16(gb[4], gb[5]) / GYRO_LSB_PER_DPS) - self._bz

                # --- Gyro entegrasyonu (tahmin) ---
                roll_g  = self._roll  + gx * self.dt
                pitch_g = self._pitch + gy * self.dt
                yaw_g   = self._yaw   + gz * self.dt  # gyro yaw

                # --- Acc referansı (roll/pitch) ---
                roll_acc, pitch_acc = self._acc_to_angles(ax, ay, az)

                # --- Complementary (roll/pitch) ---
                a = self.alpha
                self._roll  = a * roll_g  + (1.0 - a) * roll_acc
                self._pitch = a * pitch_g + (1.0 - a) * pitch_acc

                # --- Mag (tilt-compensated heading) ---
                mag_heading = None
                if self._have_mag:
                    mr = self._ak_read_raw()
                    if mr:
                        mx_uT, my_uT, mz_uT = mr

                        # Datasheet’e göre eksen dönüştürmesi
                        mx_c, my_c, mz_c = _map_mag_axes(mx_uT, my_uT, mz_uT)

                        # Tilt compensation
                        rr = math.radians(self._roll)
                        pr = math.radians(self._pitch)
                        Xh = mx_c * math.cos(pr) + mz_c * math.sin(pr)
                        Yh = (mx_c * math.sin(rr) * math.sin(pr)
                              + my_c * math.cos(rr)
                              - mz_c * math.sin(rr) * math.cos(pr))
                        head = math.degrees(math.atan2(Yh, Xh))
                        mag_heading = _deg_wrap(head + self.declination_deg)

                # --- Yaw füzyonu ---
                if mag_heading is not None:
                    ayw = self.yaw_alpha
                    gyaw = _deg_wrap(yaw_g)
                    mgaw = mag_heading
                    diff = (mgaw - gyaw + 540.0) % 360.0 - 180.0  # en kısa fark
                    self._yaw = _deg_wrap(gyaw + (1.0 - ayw) * diff)
                else:
                    self._yaw = _deg_wrap(yaw_g)

                # Yayınla
                self.orientation.emit(float(self._roll),
                                      float(self._pitch),
                                      float(self._yaw),
                                      float(temp_c))

                # Zamanlama
                next_t += self.dt
                sleep_t = next_t - time.time()
                if sleep_t > 0:
                    time.sleep(sleep_t)
                else:
                    next_t = time.time()
            except Exception as e:
                self.error.emit(str(e))
                break

        self._cleanup()

    # ---------- API ----------
    def stop(self):
        self._run = False

    def zero_yaw(self):
        """İsteğe bağlı: o anki yönü 0 kabul et (relative heading)."""
        self._yaw = 0.0

    # ---------- Temizlik ----------
    def _cleanup(self):
        try:
            if self._bus:
                self._bus.close()
        except Exception:
            pass
        self._bus = None
