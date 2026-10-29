from html.parser import HTMLParser
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
PAGES = ('exts/edu.interactive.physics/assets/web_control/index.html', 'web_app/index.html', 'homepage_update/public/agv/control.html')
VOID = {'input', 'br', 'hr', 'img', 'meta', 'link', 'source', 'wbr', 'area', 'base', 'embed', 'param', 'track', 'col'}

class SensorMarkup(HTMLParser):
    def __init__(self):
        super().__init__()
        self.stack = []
        self.sensors = []

    def handle_starttag(self, tag, attributes):
        attrs = dict(attributes)
        if attrs.get('id') in ('missionUsbHumidity', 'missionUsbTemperature'):
            self.sensors.append((attrs['id'], list(self.stack)))
        if tag not in VOID:
            self.stack.append((tag, attrs))

    def handle_endtag(self, tag):
        for i in range(len(self.stack) - 1, -1, -1):
            if self.stack[i][0] == tag:
                self.stack = self.stack[:i]
                break

class SensorVisibilityTests(unittest.TestCase):
    def test_live_values_remain_visible_when_mission_is_hidden(self):
        for name in PAGES:
            with self.subTest(page=name):
                markup = SensorMarkup()
                markup.feed((ROOT / name).read_text())
                self.assertCountEqual([name for name, _ in markup.sensors], ['missionUsbHumidity', 'missionUsbTemperature'])
                for _, ancestors in markup.sensors:
                    for _, attrs in ancestors:
                        self.assertNotIn('temperature', attrs.get('class', '').split())
                        self.assertNotIn('hidden', attrs)
                        self.assertNotIn('inert', attrs)

if __name__ == '__main__':
    unittest.main()
