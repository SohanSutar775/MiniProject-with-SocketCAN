"""Unit + integration tests (virtual-time simulation).  Run: python3 -m unittest -v test_vms"""
import unittest
from can_matrix import *
from sim_bus import build_scenario

class Codec(unittest.TestCase):
    def test_roundtrip(self):
        s, r, c, _ = unpack_motion(pack_motion(87.65, 3012.25, 7)); self.assertAlmostEqual(s, 87.65, 2); self.assertAlmostEqual(r, 3012.25, 2); self.assertEqual(c, 7)
        self.assertAlmostEqual(unpack_coolant(pack_coolant(-12.3)), -12.3, 1)
        self.assertAlmostEqual(unpack_battery(pack_battery(13.8)), 13.8, 2)
        self.assertAlmostEqual(unpack_fuel(pack_fuel(74.5)), 74.5, 1)
    def test_dlc(self):
        for cid, m in MESSAGES.items(): pass
        self.assertEqual(len(pack_motion(0, 0, 0)), 6); self.assertEqual(len(pack_act_status(0, 0, 0, 0, 0)), 5)
    def test_saturation(self):
        self.assertAlmostEqual(unpack_battery(pack_battery(1000)), 655.35, 2)   # saturates, no exception
    def test_matrix_requirements(self):
        self.assertGreaterEqual(len(MESSAGES), 8)
        kinds = [m["kind"] for m in MESSAGES.values()]
        self.assertGreaterEqual(sum(k.startswith("Periodic") for k in kinds), 2); self.assertGreaterEqual(kinds.count("Event"), 2)

class Scenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.s = build_scenario().run(60.0); cls.ev = cls.s.events()
    def t(self, key, t0=0):
        for e in self.ev:
            if e["t"] >= t0 and e["key"] == key: return e["t"]
    def test_c1_normal(self):
        smp = [x for x in self.s.samples if 2 <= x["t"] < 10]
        self.assertTrue(all(x["mode"] == 0 and not x["dtcs"] for x in smp)); self.assertEqual(self.s.diag.lost_frames, 0)
    def test_c2_missing(self):
        self.assertLessEqual(self.t("DTC_SET:0xc100", 10) - 10.0, 0.075); self.assertIsNotNone(self.t("MODE:SAFE", 10)); self.assertIsNotNone(self.t("DTC_CLR:0xc100", 14))
    def test_c3_invalid(self):
        self.assertIsNotNone(self.t("DTC_SET:0xb101", 18)); self.assertIsNotNone(self.t("MODE:SAFE", 18)); self.assertIsNotNone(self.t("DTC_SET:0xb102", 24)); self.assertIsNotNone(self.t("MODE:DEGRADED", 24))
    def test_c4_c5_actuator(self):
        self.assertIsNotNone(self.t("DTC_SET:0xc300", 32)); self.assertIsNotNone(self.t("DTC_CLR:0xc300", 38)); self.assertIsNotNone(self.t("MODE:NORMAL", 38))
    def test_controller_loss_local_safe(self):
        self.assertIsNotNone(self.t("LOCAL_SAFE_ON", 42)); self.assertIsNotNone(self.t("LOCAL_SAFE_OFF", 46))
    def test_thresholds(self):
        self.assertIsNotNone(self.t("ALARM_SET:0x10", 50)); self.assertIsNotNone(self.t("ALARM_SET:0x11", 53))
    def test_bus_load(self):
        self.assertLess(100 * self.s.bus.busy / 60, 5.0)

if __name__ == "__main__":
    unittest.main(verbosity=2)
