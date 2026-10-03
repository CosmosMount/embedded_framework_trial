"""Run from the repository root: python -m unittest discover -s .agents/tests -v."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="pnx-config-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        shutil.copytree(ROOT / "configs", self.root / "configs")
        self.boards = sorted(p.parent.name for p in (self.root / "configs/boards").glob("*/board.cmake"))
        self.assertTrue(self.boards)

    def cmake(self, script, board, *args):
        return subprocess.run(
            ["cmake", f"-DPNX_BOARD={board}", *args, "-P", f"configs/cmake/{script}"],
            cwd=self.root, text=True, capture_output=True,
        )

    def test_generation_and_editor_without_generated_boards(self):
        self.assertFalse((self.root / "boards").exists())
        for board in self.boards:
            with self.subTest(board=board):
                output = self.root / "build" / board
                result = self.cmake("configure.cmake", board, f"-DOUT_DIR={output}")
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                for name in ("config.hpp", "robot_config.hpp", "bsp_bindings.cpp", "cubemx.script"):
                    self.assertGreater((output / name).stat().st_size, 0)
                result = self.cmake("export_editor_context.cmake", board, f"-DPNX_EDITOR_OUTPUT={output}/editor.json")
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                context = json.loads((output / "editor.json").read_text())
                self.assertEqual(context["board"], board)
                for file in context["files"].values():
                    path = Path(file).resolve()
                    self.assertTrue(path.is_relative_to(self.root / "configs"), file)
                    self.assertTrue(path.is_file(), file)
        self.assertFalse((self.root / "boards").exists())

    def test_led_parameters(self):
        board = "h723_dmmc02"
        path = self.root / "configs/boards" / board / "params.json"
        data = json.loads(path.read_text())
        for key, value in (("brightness", -1), ("brightness", 256), ("brightness", 1.5),
                           ("spi_timeout_ms", 0), ("blink_half_period_calls", 32768)):
            with self.subTest(key=key, value=value):
                data["led"] = {key: value}
                path.write_text(json.dumps(data))
                result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/led")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("params.led", result.stdout + result.stderr)
        data["led"] = {"brightness": 0, "blink_half_period_calls": 32767}
        path.write_text(json.dumps(data))
        result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/led")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_unsupported_motor_options_rejected(self):
        board = "h723_dmmc02"
        path = self.root / "configs/boards" / board / "robot.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        for key, value in (("control_mode", "multi"), ("model", "dji_unsupported")):
            with self.subTest(key=key):
                invalid = json.loads(json.dumps(data))
                invalid["devices"]["motors"]["list"][0][key] = value
                path.write_text(json.dumps(invalid), encoding="utf-8")
                result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/unsupported")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(key, result.stdout + result.stderr)

    def test_xv2_existing_models_are_selectable(self):
        for board in self.boards:
            path = self.root / "configs/boards" / board / "robot.json"
            robot = json.loads(path.read_text())
            bus = "fdcan2" if board == "h723_dmmc02" else "can1"
            for model in ("xv2_x42", "xv2_y42"):
                with self.subTest(board=board, model=model):
                    robot["devices"]["motors"]["list"] = [{
                        "name": "axis", "model": model, "can_bus": bus,
                        "can_type": "classic", "can_id": "0x100"}]
                    path.write_text(json.dumps(robot))
                    result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/xv2")
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_fdcan_tdc_configuration(self):
        board = "h723_dmmc02"
        path = self.root / "configs/boards" / board / "params.json"
        original = json.loads(path.read_text())
        for bus, offset, window in (("fdcan1", -1, 13), ("fdcan1", 128, 13),
                                    ("fdcan1", 13, 1.5), ("fdcan1", "13", 13),
                                    ("fdcan2", 13, 13)):
            with self.subTest(bus=bus, offset=offset, window=window):
                params = json.loads(json.dumps(original))
                params["can"][bus].update(tdc_offset=offset, tdc_filter=window)
                path.write_text(json.dumps(params))
                result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/tdc")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("TDC" if bus == "fdcan2" else "tdc_", result.stdout + result.stderr)
        params = json.loads(json.dumps(original))
        del params["can"]["fdcan1"]["tdc_filter"]
        path.write_text(json.dumps(params))
        result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/tdc")
        self.assertNotEqual(result.returncode, 0)

    def test_numeric_parameter_contracts(self):
        board = "h723_dmmc02"
        path = self.root / "configs/boards" / board / "params.json"
        original = json.loads(path.read_text())
        robot_path = path.with_name("robot.json")
        robot = json.loads(robot_path.read_text())
        for section, key, value in (
                ("ahrs", "imu_thread_priority", 1.5),
                ("ahrs", "imu_thread_priority", "11"),
                ("ahrs", "target_temp", "40"),
                ("bmi088", "spi_timeout_ms", -1),
                ("bmi088", "spi_timeout_ms", 4294967296),
                ("dmimu", "offline_timeout_ticks", 2147483648),
                ("dmimu", "receive_wait_ticks", 0),
                ("dmimu", "request_period_ticks", 1.5),
                ("remoter", "rx_timeout_ticks", 0),
                ("remoter", "rx_timeout_ticks", 2147483648)):
            with self.subTest(section=section, key=key, value=value):
                robot["devices"]["dmimu"]["enabled"] = section == "dmimu"
                robot["devices"]["bmi088"]["enabled"] = section != "dmimu"
                robot_path.write_text(json.dumps(robot))
                data = dict(original)
                data[section] = {key: value}
                if section == "remoter":
                    data[section]["source"] = "dr16"
                path.write_text(json.dumps(data))
                result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/invalid-parameter")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(key, result.stdout + result.stderr)

    def test_motor_capacity_rejects_invalid_integer_range(self):
        board = "h723_dmmc02"
        path = self.root / "configs/boards" / board / "robot.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        for capacity in (0, -1, 65536, 1.5):
            with self.subTest(capacity=capacity):
                data["devices"]["motors"].setdefault("dm", {})["max_motors"] = capacity
                path.write_text(json.dumps(data), encoding="utf-8")
                result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/invalid-capacity")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("max_motors", result.stdout + result.stderr)

    def test_dual_remote_configuration(self):
        for board in self.boards:
            path = self.root / "configs/boards" / board / "params.json"
            data = json.loads(path.read_text(encoding="utf-8"))
            data["remoter"] = {"source": "vt03_dr16"}
            data["bindings"]["referee_uart"] = "none"
            path.write_text(json.dumps(data), encoding="utf-8")
            output = self.root / "build" / board
            result = self.cmake("configure.cmake", board, f"-DOUT_DIR={output}")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            header = (output / "config.hpp").read_text()
            self.assertIn("#define ENABLE_DR16 1", header)
            self.assertIn("#define ENABLE_VT03 1", header)
            self.assertIn("#define ENABLE_PS2 0", header)
            # Independent JSON bindings must drive the actual generated C++ aliases.
            changed = json.loads(json.dumps(data))
            bindings = changed["bindings"]
            bindings["remoter_uart"], bindings["vt03_uart"] = bindings["vt03_uart"], bindings["remoter_uart"]
            bindings["referee_uart"] = "uart7" if board == "h723_dmmc02" else "usart1"
            path.write_text(json.dumps(changed), encoding="utf-8")
            result = self.cmake("configure.cmake", board, f"-DOUT_DIR={output}")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            header = (output / "config.hpp").read_text()
            for role, key in (("dr16", "remoter_uart"), ("vt03", "vt03_uart"), ("referee", "referee_uart")):
                self.assertIn(f"inline constexpr bsp::usart::port {role} = {bindings[key]};", header)
            for failure in ("same_uart", "missing_uart", "referee", "zero_timeout", "fractional_timeout"):
                with self.subTest(board=board, failure=failure):
                    invalid = json.loads(json.dumps(data))
                    if failure == "same_uart":
                        invalid["bindings"]["vt03_uart"] = invalid["bindings"]["remoter_uart"]
                    elif failure == "missing_uart":
                        invalid["bindings"]["vt03_uart"] = "none"
                    elif failure == "referee":
                        invalid["bindings"]["referee_uart"] = invalid["bindings"]["vt03_uart"]
                    else:
                        invalid["remoter"]["offline_timeout_ticks"] = 0 if failure == "zero_timeout" else 1.5
                    path.write_text(json.dumps(invalid), encoding="utf-8")
                    result = self.cmake("configure.cmake", board, f"-DOUT_DIR={output}")
                    self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn("bindings.vt03_uart" if failure == "missing_uart" else
                                  "UART" if failure in ("same_uart", "referee") else "offline_timeout_ticks",
                                  result.stdout + result.stderr)

    def test_retired_serial_ps2_rejected(self):
        board = "h723_dmmc02"
        path = self.root / "configs/boards" / board / "params.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        data["remoter"] = {"source": "ps2_uart"}
        path.write_text(json.dumps(data), encoding="utf-8")
        result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/retired")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("params.remoter.source must be one of", result.stdout + result.stderr)

    def test_unknown_profile_rejected(self):
        for script, output in (("configure.cmake", "-DOUT_DIR=build/bad"),
                               ("export_editor_context.cmake", "-DPNX_EDITOR_OUTPUT=build/bad.json")):
            with self.subTest(script=script):
                self.assertNotEqual(self.cmake(script, "unknown_board", output).returncode, 0)
        self.assertFalse((self.root / "build/bad/config.hpp").exists())
        self.assertFalse((self.root / "build/bad.json").exists())

    def test_mismatched_identity_rejected(self):
        board = self.boards[0]
        path = self.root / "configs/boards" / board / "board.json"
        data = json.loads(path.read_text())
        data["name"] = "wrong_profile"
        path.write_text(json.dumps(data), encoding="utf-8")
        self.assertNotEqual(self.cmake("configure.cmake", board, "-DOUT_DIR=build/bad").returncode, 0)

    def test_fixed_binding_must_exist_in_ioc(self):
        board = self.boards[0]
        path = self.root / "configs/boards" / board / "board.json"
        data = json.loads(path.read_text())
        data["bindings"]["gpio_outputs"]["bmi088_acc_cs"]["pin"] = "invalid_pin"
        path.write_text(json.dumps(data), encoding="utf-8")
        result = self.cmake("configure.cmake", board, "-DOUT_DIR=build/bad")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.root / "build/bad/bsp_bindings.cpp").exists())


if __name__ == "__main__":
    unittest.main()
