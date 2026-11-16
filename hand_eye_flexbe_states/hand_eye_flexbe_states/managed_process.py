# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Programs a state starts and stops, and whether a program is already running.

A program runs in its own process group, so that stopping it stops everything it started. What it
prints goes to the terminal of the behavior engine (or nowhere, for a program that prints a lot):
it is never kept in a pipe that nobody reads.
"""

import os
import signal
import subprocess

from flexbe_core import Logger
from hand_eye_flexbe_states.settings import processes
import psutil


class ManagedProcess:
    """
    A program started with a command line, that can be asked whether it ended and stopped.

    quiet: whether what it prints is thrown away (True) or goes to the terminal (False).
    """

    def __init__(self, command, quiet):
        self.command = [str(part) for part in command]
        self.output = subprocess.DEVNULL if quiet else None
        self.process = None

    def start(self):
        """Start the program."""
        self.process = subprocess.Popen(self.command, stdout=self.output, stderr=self.output,
                                        start_new_session=True)

    def started(self):
        """Return whether this object started the program."""
        return self.process is not None

    def ended(self):
        """Return whether the program was started and has ended by itself."""
        return self.process is not None and self.process.poll() is not None

    def stop(self):
        """Stop the program: interrupt it, then terminate it, then kill it."""
        if self.process is None or self.process.poll() is not None:
            self.process = None
            return
        waits = processes('stop')
        steps = ((lambda: os.killpg(os.getpgid(self.process.pid), signal.SIGINT),
                  waits['interrupt_wait_s']),
                 (self.process.terminate, waits['terminate_wait_s']))
        for ask, wait in steps:
            try:
                ask()
                self.process.wait(timeout=wait)
                break
            except (ProcessLookupError, subprocess.TimeoutExpired):
                continue
        else:
            self.process.kill()
        self.process = None


def executable_running(name):
    """Return whether a process whose command line names an executable is running."""
    for process in psutil.process_iter(['cmdline']):
        if any(name in part for part in process.info['cmdline'] or []):
            return True
    return False


def ros_lists(kind, names):
    """
    Return whether 'ros2 <kind> list' (node or service) shows one of the names.

    When the question cannot be asked, they are taken as not listed, and that is said.
    """
    try:
        listed = subprocess.run(['ros2', kind, 'list'], capture_output=True, text=True,
                                timeout=processes('ros_query_timeout_s')).stdout
    except (OSError, subprocess.TimeoutExpired) as error:
        Logger.logwarn(f"'ros2 {kind} list' gave no answer ({error}): "
                       f'{names} taken as not running')
        return False
    return any(name in listed for name in names)
