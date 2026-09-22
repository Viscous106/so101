"""Full-screen white window on the lab monitor = a lamp for the wrist camera. Kill the process to remove it."""
import time
from Xlib import X, display
d = display.Display(); s = d.screen()
w = s.root.create_window(0, 0, s.width_in_pixels, s.height_in_pixels, 0, s.root_depth, X.InputOutput, X.CopyFromParent,
                         background_pixel=s.white_pixel, override_redirect=True)
w.map(); w.configure(stack_mode=X.Above); d.sync()
while True:
    time.sleep(20); w.configure(stack_mode=X.Above); d.sync()
