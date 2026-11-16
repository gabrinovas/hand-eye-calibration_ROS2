# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Configuration of the states: the files of the config folder of this package."""

from charuco_calibrator.config_loader import setting

PACKAGE = 'hand_eye_flexbe_states'


def processes(*keys):
    """Return a value of config/processes.yaml: the programs the states start."""
    return setting('processes', *keys, package=PACKAGE)


def capture_ui(*keys):
    """Return a value of config/capture_ui.yaml: the window the pictures are taken with."""
    return setting('capture_ui', *keys, package=PACKAGE)
