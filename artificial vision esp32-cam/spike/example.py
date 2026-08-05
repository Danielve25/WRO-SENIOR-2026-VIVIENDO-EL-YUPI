from pybricks.hubs import PrimeHub
from pybricks.tools import wait

from wro_vision_receiver import GridPatternReceiver


hub = PrimeHub()
receiver = GridPatternReceiver()

pattern = receiver.wait_for_pattern(10000)
if pattern is not None and pattern.valid:
    hub.display.text("OK")
    for row in range(3):
        for column in range(4):
            cell = row * 4 + column
            print(cell + 1, pattern.color(cell), pattern.confidence_at(cell))
else:
    hub.display.text("ERR")

while True:
    receiver.poll()
    wait(20)
