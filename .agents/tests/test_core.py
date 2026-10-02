"""Compile production C++ with only CAN and RTOS boundaries replaced for host tests."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class CoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("g++")
        if not compiler:
            raise unittest.SkipTest("Host g++ is required; ARM compilation does not execute these tests")
        cls.temp = tempfile.TemporaryDirectory(prefix="pnx-core-")
        cls.addClassCleanup(cls.temp.cleanup)
        root = Path(cls.temp.name).resolve()
        params = json.loads((ROOT / "configs/boards/h723_dmmc02/params.json").read_text())
        params["referee"] = {"thread_priority": 11}
        params["remoter"] = {"source": "vt03_dr16"}
        params["bindings"]["referee_uart"] = "uart7"
        (root / "params.json").write_text(json.dumps(params), encoding="utf-8")
        profile = ROOT / "configs/boards/h723_dmmc02"
        generation = subprocess.run([
            "cmake", f"-DIOC={profile}/h723_dmmc02.ioc", f"-DBOARD_CONFIG={profile}/board.json",
            f"-DPARAMS={root}/params.json", f"-DROBOT_CONFIG={profile}/robot.json",
            "-DPNX_BOARD_FAMILY=stm32h7", f"-DOUT_DIR={root}/generated",
            "-P", str(ROOT / "configs/cmake/generate_config.cmake"),
        ], text=True, capture_output=True)
        if generation.returncode:
            raise RuntimeError(generation.stdout + generation.stderr)
        cls.executable = root / "core_tests.exe"
        includes = [ROOT / ".agents/tests/host", root / "generated"]
        for tree in ("libs", "bsp/h723_dmmc02", "devices", "modules"):
            includes.extend(sorted(p for p in (ROOT / tree).rglob("include") if p.is_dir()))
        sources = [ROOT / ".agents/tests/host/test_core.cpp", ROOT / "libs/control/src/pid.cpp",
                   ROOT / "libs/math/src/constrain.cpp",
                   ROOT / "devices/motors/motor/src/motorhandler.cpp"]
        for family in ("dji", "lk", "dm", "xv2"):
            sources.extend(sorted((ROOT / "devices/motors" / family / "src").glob("*.cpp")))
        command = [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-mno-ms-bitfields"]
        command.extend(f"-I{p}" for p in includes)
        command.extend(str(p) for p in sources)
        command.extend(["-o", str(cls.executable)])
        compilation = subprocess.run(command, text=True, capture_output=True)
        if compilation.returncode:
            raise RuntimeError(compilation.stdout + compilation.stderr)

    def check_case(self, name):
        result = subprocess.run([str(self.executable), name], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_bmi088_transactions_and_duration_stats(self):
        root = Path(self.temp.name).resolve()
        includes = [ROOT / ".agents/tests/host", root / "generated"]
        for tree in ("libs", "bsp/h723_dmmc02", "devices", "modules"):
            includes.extend(sorted(p for p in (ROOT / tree).rglob("include") if p.is_dir()))
        binary = root / "bmi088.exe"
        command = [shutil.which("g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror", "-mno-ms-bitfields"]
        command.extend(f"-I{path}" for path in includes)
        command.extend(str(ROOT / path) for path in (
            ".agents/tests/host/test_bmi088.cpp", "devices/imu/bmi088/src/bmi088.cpp"))
        command.extend(["-o", str(binary)])
        result = subprocess.run(command, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        result = subprocess.run([str(binary)], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def compile_and_run_device(self, name, *sources):
        root = Path(self.temp.name).resolve()
        includes = [ROOT / ".agents/tests/host", root / "generated"]
        for tree in ("libs", "bsp/h723_dmmc02", "devices", "modules"):
            includes.extend(sorted(p for p in (ROOT / tree).rglob("include") if p.is_dir()))
        if name == "dmimu":
            configured = root / "dmimu_enabled"
            configured.mkdir(exist_ok=True)
            config = (root / "generated/config.hpp").read_text()
            config = config.replace("#define HAS_AHRS 1", "#define HAS_AHRS 0")
            config = config.replace("#define HAS_DMIMU 0", "#define HAS_DMIMU 1")
            (configured / "config.hpp").write_text(config)
            includes.insert(0, configured)
        binary = root / f"{name}.exe"
        command = [shutil.which("g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror"]
        command.extend(f"-I{path}" for path in includes)
        if name in ("remotes", "dmimu", "referee"):
            command.append("-DPNX_TEST_THREAD_ENTRY")
        command.append(str(ROOT / f".agents/tests/host/test_{name}.cpp"))
        command.extend(str(ROOT / source) for source in sources)
        command.append("-mno-ms-bitfields")
        command.extend(["-o", str(binary)])
        result = subprocess.run(command, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        cases = [[], ["flow"], ["start_failure"], ["stop_failure"], ["resume_failure"],
                 ["thread_failure"], ["msg_failure"]] if name == "remotes" else [[]]
        if name == "dmimu":
            cases += [[case] for case in ("success", "semaphore", "topic", "thread", "cleanup")]
        for arguments in cases:
            with self.subTest(case=arguments):
                result = subprocess.run([str(binary), *arguments], text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_referee_bounded_receive(self):
        self.compile_and_run_device("referee", "modules/referee/src/referee.cpp", "libs/crc/src/crc.cpp")

    def test_ui_submission(self):
        self.compile_and_run_device("ui", "devices/ui/src/ui.cpp", "libs/crc/src/crc.cpp")

    def test_message_contention(self):
        self.compile_and_run_device("msg", "libs/msg/src/msg.cpp")

    def test_retained_log(self):
        self.compile_and_run_device("log", "libs/log/src/log.cpp")

    def test_led_frames_and_failures(self):
        self.compile_and_run_device("led", "devices/led/src/led.cpp")

    def test_filter_state(self):
        self.compile_and_run_device("filters", "libs/filter/src/iir.cpp", "libs/filter/src/kalman_1d.cpp")

    def test_spi_transfer_lengths(self):
        root = Path(self.temp.name).resolve()
        for board in ("h723_dmmc02", "f407_cboard"):
            with self.subTest(board=board):
                includes = [ROOT / ".agents/tests/host", root / "generated", ROOT / "libs/common/include",
                            ROOT / f"bsp/{board}/spi/include", ROOT / f"bsp/{board}/spi/src",
                            ROOT / f"bsp/{board}/bsp/include"]
                binary = root / f"spi_{board}.exe"
                command = [shutil.which("g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror"]
                command.extend(f"-I{p}" for p in includes)
                command.extend([str(ROOT / ".agents/tests/host/test_spi.cpp"),
                                str(ROOT / f"bsp/{board}/spi/src/bsp_spi.cpp"), "-o", str(binary)])
                result = subprocess.run(command, text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                result = subprocess.run([str(binary)], text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_dmimu_can_decode(self):
        self.compile_and_run_device("dmimu", "devices/imu/dmimu/src/dmimu.cpp",
                                    "modules/ahrs/src/ahrs.cpp", "libs/log/src/log.cpp")

    def test_gamepad_initialization(self):
        root = Path(self.temp.name).resolve() / "gamepad"
        root.mkdir()
        profile = ROOT / "configs/boards/h723_dmmc02"
        params = json.loads((profile / "params.json").read_text())
        params["remoter"] = {"source": "ps2"}
        params["ps2"] = {"backend": "gpio", "cmd": "pa0", "clk": "pa2", "cs": "pc0", "data": "pe9"}
        (root / "params.json").write_text(json.dumps(params))
        result = subprocess.run([
            "cmake", f"-DIOC={profile}/h723_dmmc02.ioc", f"-DBOARD_CONFIG={profile}/board.json",
            f"-DPARAMS={root}/params.json", f"-DROBOT_CONFIG={profile}/robot.json",
            "-DPNX_BOARD_FAMILY=stm32h7", f"-DOUT_DIR={root}/generated",
            "-P", str(ROOT / "configs/cmake/generate_config.cmake"),
        ], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        includes = [ROOT / ".agents/tests/host", root / "generated"]
        for tree in ("libs", "bsp/h723_dmmc02", "devices", "modules"):
            includes.extend(sorted(p for p in (ROOT / tree).rglob("include") if p.is_dir()))
        binary = root / "gamepad.exe"
        command = [shutil.which("g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
                   "-mno-ms-bitfields", "-DPNX_TEST_THREAD_ENTRY"]
        command.extend(f"-I{path}" for path in includes)
        command.extend(str(ROOT / path) for path in (
            ".agents/tests/host/test_gamepad.cpp", "modules/remoter/src/gamepad.cpp",
            "devices/remotes/src/ps2.cpp"))
        command.extend(["-o", str(binary)])
        result = subprocess.run(command, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for case in ("success", "device", "topic", "thread", "cleanup"):
            with self.subTest(case=case):
                result = subprocess.run([str(binary), case], text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_usart_receive_ownership(self):
        root = Path(self.temp.name).resolve()
        includes = [ROOT / ".agents/tests/host", root / "generated",
                    ROOT / "libs/common/include", ROOT / "bsp/f407_cboard/bsp/include",
                    ROOT / "bsp/f407_cboard/usart/include", ROOT / "bsp/f407_cboard/usart/src"]
        binary = root / "usart.exe"
        command = [shutil.which("g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror"]
        command.extend(f"-I{path}" for path in includes)
        command.extend([str(ROOT / ".agents/tests/host/test_usart.cpp"),
                        str(ROOT / "bsp/f407_cboard/usart/src/bsp_usart.cpp"), "-o", str(binary)])
        result = subprocess.run(command, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        result = subprocess.run([str(binary)], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_remote_protocols(self):
        self.compile_and_run_device("remotes", "modules/remoter/src/remoter.cpp", *(f"devices/remotes/src/{name}.cpp"
            for name in ("ps2", "dr16", "vt03")))

    def test_pid_modes(self):
        self.check_case("pid")

    def test_callback_bindings(self):
        self.check_case("callbacks")

    def test_referee_generated_priority(self):
        self.check_case("referee")

    def test_motor_packets_and_registration(self):
        self.check_case("motors")


class BxCanTests(unittest.TestCase):
    def test_f407_hal_adapter(self):
        compiler = shutil.which("g++")
        if not compiler:
            self.skipTest("Host g++ is required")
        with tempfile.TemporaryDirectory(prefix="pnx-bxcan-") as temporary:
            root = Path(temporary).resolve()
            profile = ROOT / "configs/boards/f407_cboard"
            params = json.loads((profile / "params.json").read_text(encoding="utf-8"))
            params["can"] = {"can2": {"id_type": "extended"}}
            (root / "params.json").write_text(json.dumps(params), encoding="utf-8")
            generation = subprocess.run([
                "cmake", f"-DIOC={profile}/f407_cboard.ioc",
                f"-DBOARD_CONFIG={profile}/board.json", f"-DPARAMS={root}/params.json",
                f"-DROBOT_CONFIG={profile}/robot.json", "-DPNX_BOARD_FAMILY=stm32f4",
                f"-DOUT_DIR={root}/generated", "-P", str(ROOT / "configs/cmake/generate_config.cmake"),
            ], text=True, capture_output=True)
            self.assertEqual(generation.returncode, 0, generation.stdout + generation.stderr)
            includes = [ROOT / ".agents/tests/host", root / "generated",
                        ROOT / "libs/common/include", ROOT / "bsp/f407_cboard/can/include",
                        ROOT / "bsp/f407_cboard/can/src", ROOT / "libs/log/include"]
            command = [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror"]
            command.extend(f"-I{path}" for path in includes)
            binary = root / "bxcan.exe"
            command.extend([str(ROOT / ".agents/tests/host/test_bxcan.cpp"),
                            str(ROOT / "bsp/f407_cboard/can/src/bsp_can.cpp"),
                            str(ROOT / "libs/log/src/log.cpp"), "-o", str(binary)])
            result = subprocess.run(command, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(binary)], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
