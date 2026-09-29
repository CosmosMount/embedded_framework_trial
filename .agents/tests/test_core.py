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
        for tree in ("libs", "bsp", "devices", "modules"):
            includes.extend(sorted(p for p in (ROOT / tree).rglob("include") if p.is_dir()))
        sources = [ROOT / ".agents/tests/host/test_core.cpp", ROOT / "libs/control/src/pid.cpp",
                   ROOT / "libs/math/src/constrain.cpp", ROOT / "libs/math/src/trans.cpp",
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

    def test_pid_modes(self):
        self.check_case("pid")

    def test_callback_bindings(self):
        self.check_case("callbacks")

    def test_referee_generated_priority(self):
        self.check_case("referee")

    def test_motor_packets_and_registration(self):
        self.check_case("motors")
