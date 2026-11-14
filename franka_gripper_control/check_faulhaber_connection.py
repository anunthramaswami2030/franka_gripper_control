#!/usr/bin/env python3
"""Perform a non-moving health check of a FAULHABER CANopen controller.

The check combines Linux SocketCAN link statistics with CANopen node guarding,
expedited SDO uploads, CiA 402 state decoding, and passive observation of
boot-up, emergency, and CAN error frames.  Interface counters are compared
before and after the sample window so errors generated during the check are not
hidden by an initially healthy state.

The script is intentionally read-only with respect to the controller: it never
sends an NMT state command, Controlword, mode selection, PDO, SYNC, or motion
target.  Its only outbound frames are node-guard remote requests and SDO upload
(read) requests.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import re
import subprocess
import time

import can


@dataclass
class LinkState:
    """Relevant state parsed from ``ip -details -statistics link``.

    Attributes
    ----------
    text : str
        Complete command output retained for diagnostics.
    can_state : str | None
        Kernel CAN state, normally ``ERROR-ACTIVE`` for a healthy interface.
    tx_error, rx_error : int | None
        Live CAN transmit and receive error counters.
    bus_errors, bus_off : int | None
        Cumulative kernel bus-error and bus-off counters.

    ``None`` means the installed ``ip`` output did not expose the field.
    """

    text: str
    can_state: str | None
    tx_error: int | None
    rx_error: int | None
    bus_errors: int | None
    bus_off: int | None


class CheckError(RuntimeError):
    """Report an interface inspection or CANopen response failure."""

    pass


def read_link_state(interface: str) -> LinkState:
    """Inspect one SocketCAN network interface without changing it.

    Parameters
    ----------
    interface : str
        Linux SocketCAN interface name, for example ``can0``.

    Returns
    -------
    LinkState
        Parsed live and cumulative error state.

    Raises
    ------
    CheckError
        If the interface cannot be inspected.
    """
    result = subprocess.run(
        ["ip", "-details", "-statistics", "link", "show", interface],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise CheckError(result.stderr.strip() or f"Cannot inspect {interface}")

    text = result.stdout
    state_match = re.search(r"can state ([A-Z-]+)", text)
    counter_match = re.search(r"berr-counter tx (\d+) rx (\d+)", text)
    totals_match = re.search(
        r"re-started\s+bus-errors\s+arbit-lost\s+error-warn\s+"
        r"error-pass\s+bus-off\s*\n\s*(\d+)\s+(\d+)\s+(\d+)\s+"
        r"(\d+)\s+(\d+)\s+(\d+)",
        text,
    )
    return LinkState(
        text=text,
        can_state=state_match.group(1) if state_match else None,
        tx_error=int(counter_match.group(1)) if counter_match else None,
        rx_error=int(counter_match.group(2)) if counter_match else None,
        bus_errors=int(totals_match.group(2)) if totals_match else None,
        bus_off=int(totals_match.group(6)) if totals_match else None,
    )


class ReadOnlyCanopen:
    """Issue only CANopen node-guard and SDO-upload requests.

    Parameters
    ----------
    bus : can.BusABC
        Open python-can bus.
    node_id : int
        Target CANopen node ID.
    timeout : float
        Per-request response timeout in seconds.

    The class also records asynchronous boot-up, EMCY, and CAN error frames
    encountered while waiting for requested responses or draining the bus.
    """

    def __init__(self, bus: can.BusABC, node_id: int, timeout: float):
        """Derive standard node-specific COB-IDs and reset observations."""
        self.bus = bus
        self.node_id = node_id
        self.timeout = timeout
        self.sdo_request_id = 0x600 + node_id
        self.sdo_response_id = 0x580 + node_id
        self.heartbeat_id = 0x700 + node_id
        self.emergency_id = 0x080 + node_id
        self.boot_count = 0
        self.emergencies: list[tuple[int, bytes]] = []
        self.error_frames = 0

    def _send(self, arbitration_id: int, data: bytes = b"", *, remote: bool = False) -> None:
        """Send one standard data or remote CAN frame."""
        self.bus.send(
            can.Message(
                arbitration_id=arbitration_id,
                data=data,
                is_extended_id=False,
                is_remote_frame=remote,
            )
        )

    def _record_async(self, message: can.Message) -> None:
        """Record error, boot-up, and emergency frames for final assessment."""
        if message.is_error_frame:
            self.error_frames += 1
            return
        data = bytes(message.data)
        if message.arbitration_id == self.heartbeat_id and data == b"\x00":
            self.boot_count += 1
        elif message.arbitration_id == self.emergency_id and len(data) == 8:
            code = int.from_bytes(data[0:2], "little")
            self.emergencies.append((code, data))

    def _receive_until(self, predicate, deadline: float) -> can.Message:
        """Return the first matching frame while recording other traffic."""
        while time.monotonic() < deadline:
            message = self.bus.recv(max(0.0, deadline - time.monotonic()))
            if message is None:
                break
            self._record_async(message)
            if predicate(message):
                return message
        raise CheckError("Timed out waiting for CANopen response")

    def node_guard(self) -> tuple[int, float]:
        """Request the node's current NMT state with a guarding remote frame.

        Returns
        -------
        tuple[int, float]
            Seven-bit NMT state and round-trip latency in milliseconds.  The
            protocol's alternating toggle bit is masked out.
        """
        started = time.monotonic()
        self._send(self.heartbeat_id, remote=True)
        message = self._receive_until(
            lambda msg: (
                not msg.is_error_frame
                and msg.arbitration_id == self.heartbeat_id
                and not msg.is_remote_frame
                and len(msg.data) == 1
            ),
            started + self.timeout,
        )
        state = message.data[0] & 0x7F
        return state, (time.monotonic() - started) * 1000.0

    def sdo_read(
        self, index: int, subindex: int = 0, *, signed: bool = False
    ) -> tuple[int, float]:
        """Read a scalar object-dictionary entry with expedited SDO upload.

        Parameters
        ----------
        index : int
            16-bit CANopen object index.
        subindex : int, optional
            Eight-bit object subindex.
        signed : bool, optional
            Interpret the little-endian payload as signed.

        Returns
        -------
        tuple[int, float]
            Decoded value and round-trip latency in milliseconds.
        """
        request = bytes((0x40, index & 0xFF, index >> 8, subindex, 0, 0, 0, 0))
        started = time.monotonic()
        self._send(self.sdo_request_id, request)

        def matches(message: can.Message) -> bool:
            data = bytes(message.data)
            return (
                not message.is_error_frame
                and message.arbitration_id == self.sdo_response_id
                and len(data) == 8
                and data[1] == (index & 0xFF)
                and data[2] == (index >> 8)
                and data[3] == subindex
            )

        message = self._receive_until(matches, started + self.timeout)
        data = bytes(message.data)
        if data[0] == 0x80:
            abort = int.from_bytes(data[4:8], "little")
            raise CheckError(
                f"SDO abort 0x{abort:08X} reading 0x{index:04X}:{subindex:02X}"
            )
        sizes = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}
        size = sizes.get(data[0])
        if size is None:
            raise CheckError(f"Unexpected SDO response command 0x{data[0]:02X}")
        value = int.from_bytes(data[4 : 4 + size], "little", signed=signed)
        return value, (time.monotonic() - started) * 1000.0

    def drain(self, duration: float) -> None:
        """Passively observe asynchronous traffic for ``duration`` seconds."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            message = self.bus.recv(min(0.05, deadline - time.monotonic()))
            if message is not None:
                self._record_async(message)


NMT_NAMES = {
    0x00: "Boot-up",
    0x04: "Stopped",
    0x05: "Operational",
    0x7F: "Pre-operational",
}


def cia402_state(statusword: int) -> str:
    """Decode the finite-state-machine bits of a CiA 402 Statusword."""
    masked = statusword & 0x006F
    return {
        0x0000: "Not ready to switch on",
        0x0040: "Switch on disabled",
        0x0021: "Ready to switch on",
        0x0023: "Switched on",
        0x0027: "Operation enabled",
        0x0007: "Quick stop active",
        0x000F: "Fault reaction active",
        0x0008: "Fault",
    }.get(masked, f"Unknown state mask 0x{masked:04X}")


def main() -> int:
    """Run the read-only link/controller check and return a process status.

    Returns
    -------
    int
        Zero only if the interface, controller responses, drive status, and
        before/after CAN error counters all pass; otherwise one.
    """
    parser = argparse.ArgumentParser(description="Read-only FAULHABER CANopen health check")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--node-id", type=int, default=1)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--response-timeout", type=float, default=1.0)
    args = parser.parse_args()

    if not 1 <= args.node_id <= 127:
        parser.error("node ID must be from 1 through 127")
    if args.samples < 1:
        parser.error("--samples must be at least 1")

    failures: list[str] = []
    latencies: list[float] = []

    try:
        before = read_link_state(args.interface)
    except CheckError as error:
        print(f"FAIL: {error}")
        return 1

    print(
        f"Interface {args.interface}: state={before.can_state}, "
        f"tx_err={before.tx_error}, rx_err={before.rx_error}"
    )
    if before.can_state != "ERROR-ACTIVE":
        failures.append(f"CAN interface began in {before.can_state}")
    if (before.tx_error or 0) != 0 or (before.rx_error or 0) != 0:
        failures.append("CAN interface began with nonzero live error counters")

    try:
        with can.Bus(
            interface="socketcan",
            channel=args.interface,
            receive_own_messages=False,
        ) as bus:
            drive = ReadOnlyCanopen(bus, args.node_id, args.response_timeout)

            try:
                nmt_state, latency = drive.node_guard()
                latencies.append(latency)
                print(
                    f"Node {args.node_id}: {NMT_NAMES.get(nmt_state, f'0x{nmt_state:02X}')} "
                    f"({latency:.2f} ms)"
                )
            except CheckError as error:
                failures.append(f"Node guarding: {error}")

            # These standard and manufacturer-independent CiA 402 objects
            # characterize the controller without changing its state.
            reads = (
                ("Device type", 0x1000, 0, False),
                ("Statusword", 0x6041, 0, False),
                ("Active mode", 0x6061, 0, True),
                ("Position", 0x6064, 0, True),
                ("Velocity", 0x606C, 0, True),
                ("Motor current", 0x6078, 0, True),
            )
            values: dict[str, int] = {}
            for label, index, subindex, signed in reads:
                try:
                    value, latency = drive.sdo_read(index, subindex, signed=signed)
                    values[label] = value
                    latencies.append(latency)
                    suffix = " mA" if label == "Motor current" else ""
                    print(f"{label}: {value}{suffix} ({latency:.2f} ms)")
                except CheckError as error:
                    failures.append(f"{label}: {error}")

            if "Statusword" in values:
                status = values["Statusword"]
                print(f"CiA 402: {cia402_state(status)} (0x{status:04X})")
                if status & 0x0008:
                    failures.append(f"Drive statusword reports a fault: 0x{status:04X}")

            print(f"Monitoring {args.samples} read-only samples...")
            for sample in range(1, args.samples + 1):
                try:
                    status, latency = drive.sdo_read(0x6041)
                    latencies.append(latency)
                    if status & 0x0008:
                        failures.append(f"Sample {sample}: drive fault 0x{status:04X}")
                except CheckError as error:
                    failures.append(f"Sample {sample}: {error}")
                drive.drain(args.interval)

            if drive.boot_count:
                failures.append(f"Observed {drive.boot_count} controller boot/reset message(s)")
            nonzero_emergency = [item for item in drive.emergencies if item[0] != 0]
            if nonzero_emergency:
                codes = ", ".join(f"0x{code:04X}" for code, _ in nonzero_emergency)
                failures.append(f"Observed nonzero emergency code(s): {codes}")
            if drive.error_frames:
                failures.append(f"Observed {drive.error_frames} CAN error frame(s)")

    except (can.CanError, OSError) as error:
        failures.append(f"SocketCAN: {error}")

    try:
        after = read_link_state(args.interface)
        print(
            f"Final interface: state={after.can_state}, "
            f"tx_err={after.tx_error}, rx_err={after.rx_error}"
        )
        if after.can_state != "ERROR-ACTIVE":
            failures.append(f"CAN interface ended in {after.can_state}")
        if (after.tx_error or 0) != 0 or (after.rx_error or 0) != 0:
            failures.append("CAN interface ended with nonzero live error counters")
        if (
            before.bus_errors is not None
            and after.bus_errors is not None
            and after.bus_errors > before.bus_errors
        ):
            failures.append("Kernel bus-error counter increased during the test")
        if (
            before.bus_off is not None
            and after.bus_off is not None
            and after.bus_off > before.bus_off
        ):
            failures.append("Kernel bus-off counter increased during the test")
    except CheckError as error:
        failures.append(f"Final interface inspection: {error}")

    if latencies:
        print(
            f"Response latency: min={min(latencies):.2f} ms, "
            f"max={max(latencies):.2f} ms, "
            f"mean={sum(latencies) / len(latencies):.2f} ms"
        )

    if failures:
        print("\nRESULT: UNHEALTHY")
        for failure in failures:
            print(f"  - {failure}")
        print("No enable, mode-write, target, or movement command was sent.")
        return 1

    print("\nRESULT: HEALTHY")
    print("Node guarding, SDO reads, controller state, and CAN counters passed.")
    print("No enable, mode-write, target, or movement command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
