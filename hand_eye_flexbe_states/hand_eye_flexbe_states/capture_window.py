# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Window a capture is made with: the live picture of the camera, what has been taken and the keys.

Its look is a section of config/capture_ui.yaml (one per kind of capture).
"""

from charuco_calibrator.config_loader import ConfigError
import cv2
from hand_eye_flexbe_states.settings import capture_ui

CAPTURE, CANCEL = 'capture', 'cancel'
KEY_MASK = 0xFF
BLACK = (0, 0, 0)


class CaptureWindow:
    """Shows each picture with the texts of the capture over it and says what the operator asks."""

    def __init__(self, section, **names):
        self.look, self.keys = capture_ui(section), capture_ui('keys')
        self.title = self.look['window']['title'].format(**names)
        self.names = names
        self.opened = False

    def text(self, line, values):
        """Return the text of a line of the window; fail on a name it cannot fill in."""
        try:
            return line['text'].format(**self.names, **values)
        except KeyError as error:
            raise ConfigError(f"[Fail-Fast] The text '{line['text']}' of config/capture_ui.yaml "
                              f'names {error}, which this capture does not give') from error

    def show(self, picture, **values):
        """Show a picture with its texts; return CAPTURE, CANCEL or None (the key pressed)."""
        if not self.opened:
            cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
            size = self.look['window']
            cv2.resizeWindow(self.title, size['width'], size['height'])
            self.opened = True
        shown, band, overlay = picture.copy(), picture.copy(), self.look['overlay']
        margin, opacity = overlay['margin_px'], overlay['opacity']
        cv2.rectangle(band, (margin, margin), (shown.shape[1] - margin, overlay['bottom_px']),
                      BLACK, cv2.FILLED)
        cv2.addWeighted(band, opacity, shown, 1.0 - opacity, 0, shown)
        for line in self.look['lines']:
            cv2.putText(shown, self.text(line, values), tuple(line['origin']),
                        cv2.FONT_HERSHEY_SIMPLEX, line['scale'], tuple(line['color']),
                        line['thickness'])
        cv2.imshow(self.title, shown)
        key = cv2.waitKey(1) & KEY_MASK
        if key in self.keys['capture']:
            return CAPTURE
        return CANCEL if key in self.keys['cancel'] else None

    def close(self):
        """Close the window."""
        if self.opened:
            cv2.destroyAllWindows()
            for _ in range(self.look['close_wait_cycles']):
                cv2.waitKey(1)
        self.opened = False
