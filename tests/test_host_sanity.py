import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import host_sanity
import serial
from unittest import mock
import pytest


class DummySerial:
    def reset_input_buffer(self):
        pass

    def write(self, data):
        raise serial.SerialTimeoutException("timeout")

    def readline(self):
        return b""

    def close(self):
        pass


def test_run_device_exits_on_unresponsive_port():
    with mock.patch("host_sanity.serial.Serial", return_value=DummySerial()):
        with pytest.raises(SystemExit) as cm:
            host_sanity.run_device("/dev/ttyFAKE")
        assert "Timed out writing" in str(cm.value)
